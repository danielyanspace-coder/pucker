//! State of one packing place during search, candidate evaluation and placement.
//!
//! Boxes are placed by "dropping" them at a candidate (x, y): the box lands on the highest
//! top it overlaps, so there are never collisions from below or above. Every hard constraint
//! is checked before a box is committed (ТЗ §32, stage 4).

use crate::geometry::{hull_of_rects, overlap, strictly_inside_doubled, union_area, Rect};
use crate::model::{PackRule, PackingPlace};
use crate::prep::PrepItem;

const EPS: f64 = 1e-9;
/// Side gaps narrower than this are unlikely to be filled later.
const NARROW_GAP: i32 = 120;
const SIDE_NEG_X: usize = 0;
const SIDE_POS_X: usize = 1;
const SIDE_NEG_Y: usize = 2;
const SIDE_POS_Y: usize = 3;

#[derive(Clone, Debug)]
pub struct Placed {
    pub item: usize,
    pub orient: usize,
    pub x: i32,
    pub y: i32,
    pub z: i32,
    pub w: i32,
    pub d: i32,
    pub h: i32,
    /// (placed index, contact area mm²)
    pub supports: Vec<(usize, i64)>,
    pub support_area: i64,
    pub received: f64,
    /// Side contact area with neighbours or walls: -x, +x, -y, +y.
    pub side_contact: [i64; 4],
    /// Tilt reserve of the free-standing (unbraced) stack this box tops, mm:
    /// min over unbraced levels k below of `base_k / tan(tilt) - (top - bottom_k)`.
    /// Infinite when the box is braced by neighbours or walls.
    pub margin: f64,
}

impl Placed {
    fn top(&self) -> i32 {
        self.z + self.h
    }
}

/// Stage at which a candidate was rejected; later stage = closer to feasible.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Reject {
    Bounds,
    Height,
    Weight,
    Support,
    Fragility,
    Lateral,
    TopLoad,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Profile {
    /// Fill layer by layer (pallets): lowest Z first.
    Layer,
    /// Build walls from the back (vehicles): lowest Y first.
    Wall,
}

/// Scoring parameters of one search attempt.
#[derive(Clone, Copy, Debug)]
pub struct Weights {
    pub profile: Profile,
    /// Bonus (in mm of position) for full contact with neighbours / walls / support.
    pub contact: f64,
    /// Bonus (in mm) for the largest item volume, scaled by the cube root of the volume share.
    pub volume: f64,
    /// Bonus (in mm) when the top is flush with a neighbour's top.
    pub flush: f64,
    /// Penalty per mm of average empty gap trapped under the box.
    pub void: f64,
    /// Penalty (in mm) per side facing an unfillable narrow gap.
    pub gap: f64,
    /// Random additive noise on scores, in mm.
    pub jitter: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct Seed {
    /// Corner point the box is anchored to.
    pub x: i32,
    pub y: i32,
    /// The box extends toward -X / -Y from the point instead of +X / +Y.
    pub mx: bool,
    pub my: bool,
    /// Height of the surface next to the point, on the side the box extends to.
    pub z: i32,
    pub dead: bool,
}

impl Seed {
    /// Box minimum corner for rotated sizes `w × d`.
    pub fn anchor(&self, w: i32, d: i32) -> (i32, i32) {
        (if self.mx { self.x - w } else { self.x }, if self.my { self.y - d } else { self.y })
    }

    /// A point just inside the area the box would cover.
    fn sample(&self) -> (i32, i32) {
        (self.x - self.mx as i32, self.y - self.my as i32)
    }
}

#[derive(Clone, Debug)]
pub struct Candidate {
    pub orient: usize,
    pub x: i32,
    pub y: i32,
    pub z: i32,
    pub score: f64,
}

pub struct BinState<'a> {
    pub place: &'a PackingPlace,
    pub place_index: usize,
    rule: &'a PackRule,
    pub placed: Vec<Placed>,
    pub weight: f64,
    pub volume: i64,
    pub seeds: Vec<Seed>,
    pub base_z: i32,
    /// Height limit used by the search.
    pub max_h: i32,
    /// Smallest side of any item in the task: points where such a cube does not fit are useless.
    pub min_cube: i32,
    ox: i32,
    oy: i32,
    walls: bool,
    payload: f64,
    max_tilt_ratio: f64,
    // uniform grid of placed indices for spatial queries
    cell: i32,
    gx: i32,
    gy: i32,
    grid: Vec<Vec<u32>>,
    stamp: Vec<u32>,
    epoch: u32,
    buf: Vec<usize>,
    rects: Vec<Rect>,
    sup: Vec<(usize, i64)>,
}

impl<'a> BinState<'a> {
    pub fn new(place: &'a PackingPlace, place_index: usize, rule: &'a PackRule) -> Self {
        let (ox, oy) = place.overhang();
        let cell = 100;
        let gx = ((place.width + 2 * ox) / cell + 1).max(1);
        let gy = ((place.depth + 2 * oy) / cell + 1).max(1);
        let tan = rule.tilt_angle_deg.to_radians().tan();
        let base_z = place.base_z();
        BinState {
            place,
            place_index,
            rule,
            placed: Vec::new(),
            weight: 0.0,
            volume: 0,
            seeds: [(0, 0, false, false), (place.width, 0, true, false), (0, place.depth, false, true), (place.width, place.depth, true, true)]
                .into_iter()
                .map(|(x, y, mx, my)| Seed { x, y, mx, my, z: base_z, dead: false })
                .collect(),
            base_z,
            max_h: place.height,
            min_cube: 1,
            ox,
            oy,
            walls: place.has_walls(),
            payload: place.payload_limit(),
            max_tilt_ratio: if tan > 0.0 { 1.0 / tan } else { f64::INFINITY },
            cell,
            gx,
            gy,
            grid: vec![Vec::new(); (gx * gy) as usize],
            stamp: Vec::new(),
            epoch: 0,
            buf: Vec::new(),
            rects: Vec::new(),
            sup: Vec::new(),
        }
    }

    pub fn weight_fits(&self, w: f64) -> bool {
        self.weight + w <= self.payload + EPS
    }

    fn cells(&self, x0: i32, y0: i32, x1: i32, y1: i32) -> (i32, i32, i32, i32) {
        let c = self.cell;
        let cx0 = ((x0 + self.ox) / c).clamp(0, self.gx - 1);
        let cy0 = ((y0 + self.oy) / c).clamp(0, self.gy - 1);
        let cx1 = ((x1 + self.ox) / c).clamp(0, self.gx - 1);
        let cy1 = ((y1 + self.oy) / c).clamp(0, self.gy - 1);
        (cx0, cy0, cx1, cy1)
    }

    /// Collect indices of placed boxes whose cells intersect the given area into `self.buf`.
    fn query(&mut self, x0: i32, y0: i32, x1: i32, y1: i32) {
        self.epoch = self.epoch.wrapping_add(1);
        if self.epoch == 0 {
            self.stamp.iter_mut().for_each(|s| *s = 0);
            self.epoch = 1;
        }
        self.buf.clear();
        let (cx0, cy0, cx1, cy1) = self.cells(x0, y0, x1, y1);
        for cy in cy0..=cy1 {
            for cx in cx0..=cx1 {
                for &i in &self.grid[(cy * self.gx + cx) as usize] {
                    let i = i as usize;
                    if self.stamp[i] != self.epoch {
                        self.stamp[i] = self.epoch;
                        self.buf.push(i);
                    }
                }
            }
        }
    }

    fn point_height(&mut self, px: i32, py: i32) -> i32 {
        self.query(px, py, px, py);
        let mut z = self.base_z;
        for &j in &self.buf {
            let b = &self.placed[j];
            if b.x <= px && px < b.x + b.w && b.y <= py && py < b.y + b.d {
                z = z.max(b.top());
            }
        }
        z
    }

    /// Cheap part of the check: bounds and landing height.
    pub fn drop_z(&mut self, x: i32, y: i32, w: i32, d: i32, h: i32) -> Result<i32, Reject> {
        let p = self.place;
        let c = self.rule.clearance_mm;
        // Bounds (ТЗ §20) with allowed overhang (ТЗ §12).
        if x < -self.ox || y < -self.oy || x + w > p.width + self.ox || y + d > p.depth + self.oy {
            return Err(Reject::Bounds);
        }
        // Cells keep their boxes sorted by top, highest first: the first overlapping box of
        // a cell is that cell's answer, and boxes not above the current best end the scan.
        let mut z = self.base_z;
        let (cx0, cy0, cx1, cy1) = self.cells(x - c, y - c, x + w + c, y + d + c);
        for cy in cy0..=cy1 {
            for cx in cx0..=cx1 {
                for &j in &self.grid[(cy * self.gx + cx) as usize] {
                    let b = &self.placed[j as usize];
                    if b.top() <= z {
                        break;
                    }
                    if x < b.x + b.w + c && b.x < x + w + c && y < b.y + b.d + c && b.y < y + d + c {
                        z = b.top();
                        break;
                    }
                }
            }
        }
        if z + h > self.max_h {
            return Err(Reject::Height);
        }
        Ok(z)
    }

    /// Position part of the score (lower is better) without bonuses.
    pub fn position_score(&self, profile: Profile, x: i32, y: i32, z: i32, w: i32, d: i32, h: i32) -> f64 {
        let z_rel = (z - self.base_z) as f64;
        let top = (z + h - self.base_z) as f64;
        let edge_x = x.min(self.place.width - x - w).max(0) as f64;
        let edge_y = y.min(self.place.depth - y - d).max(0) as f64;
        let far_y = (y + d) as f64;
        match profile {
            Profile::Layer => z_rel * 4.0 + top * 1.0 + (edge_x + edge_y) * 0.05,
            Profile::Wall => far_y * 4.0 + z_rel * 1.0 + top * 0.5 + edge_x * 0.05,
        }
    }

    /// Remaining hard constraints for a box landing at `z` (found by `drop_z`).
    /// Returns the contact share, top flushness, trapped gap and narrow side gaps for scoring.
    #[allow(clippy::too_many_arguments)]
    pub fn check(
        &mut self,
        items: &[PrepItem],
        idx: usize,
        x: i32,
        y: i32,
        z: i32,
        w: i32,
        d: i32,
        h: i32,
    ) -> Result<(f64, f64, f64, f64), Reject> {
        let p = self.place;
        let it = &items[idx];
        let foot = Rect::new(x, y, w, d);
        let foot_area = foot.area();
        // Neighbours for supports, trapped gaps, side contacts and narrow gaps.
        let g = (self.rule.clearance_mm + self.rule.lateral_gap_mm).max(NARROW_GAP);
        self.query(x - g, y - g, x + w + g, y + d + g);

        // Support area (ТЗ §13) and centre of gravity inside the support polygon (ТЗ §14–15).
        self.rects.clear();
        self.sup.clear();
        let stol = self.rule.support_tolerance_mm;
        if z - self.base_z <= stol {
            if let Some(r) = foot.intersect(&Rect::new(0, 0, p.width, p.depth)) {
                self.rects.push(r);
            }
        }
        if z > self.base_z {
            for &j in &self.buf {
                let b = &self.placed[j];
                if b.top() <= z && b.top() >= z - stol {
                    if let Some(r) = foot.intersect(&Rect::new(b.x, b.y, b.w, b.d)) {
                        self.rects.push(r);
                        self.sup.push((j, r.area()));
                    }
                }
            }
        }
        let support = union_area(&self.rects);
        if (support as f64) < self.rule.required_support(w, d, h) * foot_area as f64 - EPS {
            return Err(Reject::Support);
        }
        if support < foot_area {
            let hull = hull_of_rects(&self.rects);
            if !strictly_inside_doubled(&hull, 2 * x as i64 + w as i64, 2 * y as i64 + d as i64) {
                return Err(Reject::Support);
            }
        }

        // Fragility (ТЗ §7): top.fragility <= bottom.fragility for touching boxes.
        if self.rule.use_fragility {
            for &(j, _) in &self.sup {
                if it.fragility > items[self.placed[j].item].fragility {
                    return Err(Reject::Fragility);
                }
            }
        }

        // Top load (ТЗ §8–9): push the new weight down through all supports.
        if self.rule.use_max_top_load && !self.sup.is_empty() && !self.load_fits(items, &self.sup, it.weight) {
            return Err(Reject::TopLoad);
        }

        let void = if support < foot_area { self.trapped_gap(&foot, z) } else { 0.0 };

        // Neighbours around the box: bracing (docs/DECISIONS.md §4) and scoring.
        let sup = std::mem::take(&mut self.sup);
        let (side, flush) = self.side_contacts(x, y, z, w, d, h, false);
        let result = (|| {
            if self.rule.use_lateral_stability
                && !self.is_braced(&side, w, d, h)
                && self.stack_margin(&sup, w, d, h) < -EPS
            {
                return Err(Reject::Lateral);
            }
            // A box standing on a narrow face must be held on both of its wide faces.
            if self.rule.use_lateral_stability && self.rule.is_standing(w, d, h) && !enclosed(&side, w, d, h, self.rule.standing_contact_ratio) {
                return Err(Reject::Lateral);
            }
            // For scoring, the pallet edges act like walls: aligning to them is good.
            let mut side_score = side;
            if !self.walls {
                let (fx, fy) = (d as i64 * h as i64, w as i64 * h as i64);
                if x <= 0 { side_score[SIDE_NEG_X] = fx; }
                if x + w >= p.width { side_score[SIDE_POS_X] = fx; }
                if y <= 0 { side_score[SIDE_NEG_Y] = fy; }
                if y + d >= p.depth { side_score[SIDE_POS_Y] = fy; }
            }
            let bottom = support as f64 / foot_area as f64;
            let side_total: i64 = side_score.iter().sum();
            let side_max = 2 * (w as i64 + d as i64) * h as i64;
            let contact = 0.4 * bottom + 0.6 * (side_total as f64 / side_max as f64).min(1.0);
            Ok((contact, flush, void, self.narrow_gaps(x, y, z, w, d, h)))
        })();
        self.sup = sup;
        result
    }

    /// Average empty gap (mm) left between the box bottom at `z` and the surface below it.
    /// Uses the neighbours from the last `drop_z` query.
    fn trapped_gap(&mut self, foot: &Rect, z: i32) -> f64 {
        let mut under: Vec<(i32, Rect)> = self
            .buf
            .iter()
            .filter_map(|&j| {
                let b = &self.placed[j];
                foot.intersect(&Rect::new(b.x, b.y, b.w, b.d)).map(|r| (b.top(), r))
            })
            .collect();
        under.sort_unstable_by(|a, b| b.0.cmp(&a.0));
        let area = foot.area();
        let mut covered = 0i64;
        let mut filled = 0.0f64;
        self.rects.clear();
        for (top, r) in under {
            self.rects.push(r);
            let now = union_area(&self.rects);
            filled += (now - covered) as f64 * (top - self.base_z) as f64;
            covered = now;
            if covered == area {
                break;
            }
        }
        let total = area as f64 * (z - self.base_z) as f64;
        (total - filled).max(0.0) / area as f64
    }

    /// How much of the box's sides face a gap too narrow to fill later (a future "shaft",
    /// ТЗ §24): per side, the gap width / NARROW_GAP when `0 < gap < NARROW_GAP`, summed.
    /// Uses the neighbours from the last query in `check`.
    fn narrow_gaps(&self, x: i32, y: i32, z: i32, w: i32, d: i32, h: i32) -> f64 {
        let p = self.place;
        // Distance to the nearest face on each side: walls or pallet edges first.
        let mut gap = [x, p.width - (x + w), y, p.depth - (y + d)];
        for &j in &self.buf {
            let b = &self.placed[j];
            if overlap(z, z + h, b.z, b.z + b.h) == 0 {
                continue;
            }
            if overlap(y, y + d, b.y, b.y + b.d) > 0 {
                if b.x + b.w <= x {
                    gap[SIDE_NEG_X] = gap[SIDE_NEG_X].min(x - (b.x + b.w));
                }
                if b.x >= x + w {
                    gap[SIDE_POS_X] = gap[SIDE_POS_X].min(b.x - (x + w));
                }
            }
            if overlap(x, x + w, b.x, b.x + b.w) > 0 {
                if b.y + b.d <= y {
                    gap[SIDE_NEG_Y] = gap[SIDE_NEG_Y].min(y - (b.y + b.d));
                }
                if b.y >= y + d {
                    gap[SIDE_POS_Y] = gap[SIDE_POS_Y].min(b.y - (y + d));
                }
            }
        }
        let slack = self.rule.lateral_gap_mm;
        gap.iter().filter(|&&g| g > slack && g < NARROW_GAP).map(|&g| g as f64 / NARROW_GAP as f64).sum()
    }

    /// Side contact area on each of the 4 sides (including rigid walls), and whether the top
    /// is flush with some touching neighbour's top.
    /// `fresh_query = false` reuses the neighbours found by the last `drop_z`.
    #[allow(clippy::too_many_arguments)]
    fn side_contacts(&mut self, x: i32, y: i32, z: i32, w: i32, d: i32, h: i32, fresh_query: bool) -> ([i64; 4], f64) {
        let p = self.place;
        let g = self.rule.lateral_gap_mm + self.rule.clearance_mm;
        let mut s = [0i64; 4];
        if self.walls {
            if x <= g {
                s[SIDE_NEG_X] = d as i64 * h as i64;
            }
            if x + w >= p.width - g {
                s[SIDE_POS_X] = d as i64 * h as i64;
            }
            if y <= g {
                s[SIDE_NEG_Y] = w as i64 * h as i64;
            }
            if y + d >= p.depth - g {
                s[SIDE_POS_Y] = w as i64 * h as i64;
            }
        }
        let mut flush = 0.0f64;
        if fresh_query {
            self.query(x - g, y - g, x + w + g, y + d + g);
        }
        for &j in &self.buf {
            let b = &self.placed[j];
            let oz = overlap(z, z + h, b.z, b.z + b.h);
            if oz == 0 {
                continue;
            }
            let oy = overlap(y, y + d, b.y, b.y + b.d);
            let ox = overlap(x, x + w, b.x, b.x + b.w);
            let mut touch = false;
            if oy > 0 {
                if (0..=g).contains(&(x - (b.x + b.w))) {
                    s[SIDE_NEG_X] += oy * oz;
                    touch = true;
                }
                if (0..=g).contains(&(b.x - (x + w))) {
                    s[SIDE_POS_X] += oy * oz;
                    touch = true;
                }
            }
            if ox > 0 {
                if (0..=g).contains(&(y - (b.y + b.d))) {
                    s[SIDE_NEG_Y] += ox * oz;
                    touch = true;
                }
                if (0..=g).contains(&(b.y - (y + d))) {
                    s[SIDE_POS_Y] += ox * oz;
                    touch = true;
                }
            }
            if touch {
                let diff = (b.top() - (z + h)).abs();
                if diff <= 5 {
                    flush = 1.0;
                } else if diff <= 30 {
                    flush = flush.max(0.5);
                }
            }
        }
        (s, flush)
    }

    /// Tilt reserve for an unbraced box of height `h` resting on `supports` (docs/DECISIONS.md §4):
    /// the box itself and every unbraced level below must not tip over at the tilt angle.
    fn stack_margin(&self, supports: &[(usize, i64)], w: i32, d: i32, h: i32) -> f64 {
        let own = w.min(d) as f64 * self.max_tilt_ratio - h as f64;
        supports.iter().fold(own, |m, &(j, _)| m.min(self.placed[j].margin - h as f64))
    }

    fn is_braced(&self, side: &[i64; 4], w: i32, d: i32, h: i32) -> bool {
        let r = self.rule.lateral_min_contact_ratio;
        let fx = d as f64 * h as f64 * r;
        let fy = w as f64 * h as f64 * r;
        let n = (side[SIDE_NEG_X] as f64 >= fx) as u8
            + (side[SIDE_POS_X] as f64 >= fx) as u8
            + (side[SIDE_NEG_Y] as f64 >= fy) as u8
            + (side[SIDE_POS_Y] as f64 >= fy) as u8;
        n >= self.rule.lateral_min_braced_sides
    }

    /// Would pushing `weight` down through `supports` overload any box below?
    fn load_fits(&self, items: &[PrepItem], supports: &[(usize, i64)], weight: f64) -> bool {
        let mut pending: Vec<(usize, f64)> = Vec::new();
        distribute(&mut pending, supports, weight);
        // Process from the highest box down so each box passes on its full increment once.
        while !pending.is_empty() {
            let (k, _) = pending.iter().enumerate().max_by_key(|(_, (j, _))| self.placed[*j].z).unwrap();
            let (j, delta) = pending.swap_remove(k);
            let b = &self.placed[j];
            if b.received + delta > items[b.item].max_top_load + EPS {
                return false;
            }
            distribute(&mut pending, &b.supports, delta);
        }
        true
    }

    /// Best feasible position among the `max_seeds` lowest live seeds for one item (sequence mode).
    pub fn best_candidate(
        &mut self,
        items: &[PrepItem],
        idx: usize,
        wts: &Weights,
        vol_share: f64,
        max_seeds: usize,
        rng: &mut crate::rng::Rng,
    ) -> Result<Candidate, Reject> {
        if !self.weight_fits(items[idx].weight) {
            return Err(Reject::Weight);
        }
        let mut best: Option<Candidate> = None;
        let mut furthest = Reject::Bounds;
        // Seeds come sorted by the profile's main key, so a lower bound of the score grows
        // along the list: once it cannot beat the best, the rest cannot either.
        let slack = wts.volume * vol_share.cbrt() + wts.contact + wts.flush + wts.jitter / 2.0;
        let min_h = items[idx].min_dim as f64;
        let mut gone: Vec<usize> = Vec::new();
        for s in self.lowest_seeds(wts.profile, max_seeds) {
            let seed = self.seeds[s];
            // No item fits where even the smallest possible cube does not.
            let m = self.min_cube;
            let (cx, cy) = seed.anchor(m, m);
            if self.drop_z(cx, cy, m, m, m).is_err() {
                gone.push(s);
                continue;
            }
            if let Some(b) = &best {
                let lower = match wts.profile {
                    Profile::Layer => 5.0 * (seed.z - self.base_z) as f64 + min_h,
                    Profile::Wall => 4.0 * (seed.y - seed.my as i32) as f64,
                } - slack;
                if lower >= b.score {
                    break;
                }
            }
            match self.try_seed(items, idx, &seed, wts, vol_share, best.as_ref().map(|b| b.score), false, rng) {
                Ok(Some(c)) => best = Some(c),
                Ok(None) => {}
                Err(r) => furthest = furthest.max(r),
            }
        }
        if !gone.is_empty() {
            gone.sort_unstable();
            for &g in gone.iter().rev() {
                self.seeds.swap_remove(g);
            }
        }
        best.ok_or(furthest)
    }

    /// Best orientation of the item at one seed. `Ok(None)` = feasible but not better than `bound`.
    #[allow(clippy::too_many_arguments)]
    pub fn try_seed(
        &mut self,
        items: &[PrepItem],
        idx: usize,
        seed: &Seed,
        wts: &Weights,
        vol_share: f64,
        bound: Option<f64>,
        need_feasible: bool,
        rng: &mut crate::rng::Rng,
    ) -> Result<Option<Candidate>, Reject> {
        let mut best: Option<Candidate> = None;
        let mut feasible = false;
        let mut furthest = Reject::Bounds;
        let max_bonus = wts.contact + wts.flush;
        let vol_bonus = wts.volume * vol_share.cbrt();
        let cur0 = bound;
        for oi in 0..items[idx].orients.len() {
            let [w, d, h] = items[idx].orients[oi].dims;
            let (sx, sy) = seed.anchor(w, d);
            // The box lands no lower than the surface at the seed, and the position score
            // only grows with height: a cheap bound before the landing search.
            let cur = best.as_ref().map(|b| b.score).or(cur0);
            if (feasible || !need_feasible) && cur.is_some_and(|c| {
                self.position_score(wts.profile, sx, sy, seed.z, w, d, h) - vol_bonus - max_bonus - wts.jitter / 2.0 >= c
            }) {
                continue;
            }
            let z = match self.drop_z(sx, sy, w, d, h) {
                Ok(z) => z,
                Err(r) => {
                    furthest = furthest.max(r);
                    continue;
                }
            };
            let pos = self.position_score(wts.profile, sx, sy, z, w, d, h) - vol_bonus;
            let cur = best.as_ref().map(|b| b.score).or(bound);
            let optimistic = pos - max_bonus - wts.jitter / 2.0;
            // Fill mode needs to know whether the seed works at all (to retire dead seeds),
            // so it checks until one orientation is feasible; sequence mode only needs the best.
            if cur.is_some_and(|c| optimistic >= c) && (feasible || !need_feasible) {
                continue;
            }
            match self.check(items, idx, sx, sy, z, w, d, h) {
                Ok((contact, flush, void, narrow)) => {
                    feasible = true;
                    let mut score = pos - wts.contact * contact - wts.flush * flush + wts.void * void + wts.gap * narrow;
                    if wts.jitter > 0.0 {
                        score += wts.jitter * (rng.next_f64() - 0.5);
                    }
                    if cur.map_or(true, |c| score < c) {
                        best = Some(Candidate { orient: oi, x: sx, y: sy, z, score });
                    }
                }
                Err(r) => furthest = furthest.max(r),
            }
        }
        if feasible {
            Ok(best)
        } else {
            Err(furthest)
        }
    }

    /// Feasibility of one exact position and orientation (layer mode).
    pub fn try_at(&mut self, items: &[PrepItem], idx: usize, orient: usize, x: i32, y: i32) -> Result<Candidate, Reject> {
        if !self.weight_fits(items[idx].weight) {
            return Err(Reject::Weight);
        }
        let [w, d, h] = items[idx].orients[orient].dims;
        let z = self.drop_z(x, y, w, d, h)?;
        self.check(items, idx, x, y, z, w, d, h)?;
        Ok(Candidate { orient, x, y, z, score: 0.0 })
    }

    /// Areas of the deck where no box top is within the support tolerance of `level`:
    /// a layer planned at `level` must not rely on them.
    pub fn unsupported_at(&self, level: i32) -> Vec<(i32, i32, i32, i32)> {
        let (pw, pd) = (self.place.width, self.place.depth);
        if level == self.base_z {
            return Vec::new();
        }
        let stol = self.rule.support_tolerance_mm;
        let tops: Vec<Rect> = self
            .placed
            .iter()
            .filter(|b| b.top() <= level && b.top() >= level - stol)
            .filter_map(|b| Rect::new(b.x, b.y, b.w, b.d).intersect(&Rect::new(0, 0, pw, pd)))
            .collect();
        let mut xs: Vec<i64> = tops.iter().flat_map(|r| [r.x0, r.x1]).chain([0, pw as i64]).collect();
        let mut ys: Vec<i64> = tops.iter().flat_map(|r| [r.y0, r.y1]).chain([0, pd as i64]).collect();
        xs.sort_unstable();
        xs.dedup();
        ys.sort_unstable();
        ys.dedup();
        let mut out = Vec::new();
        for yw in ys.windows(2) {
            let mut run: Option<i64> = None;
            for xw in xs.windows(2) {
                let covered = tops.iter().any(|r| r.x0 <= xw[0] && r.x1 >= xw[1] && r.y0 <= yw[0] && r.y1 >= yw[1]);
                match (covered, run) {
                    (false, None) => run = Some(xw[0]),
                    (true, Some(x0)) => {
                        out.push((x0 as i32, yw[0] as i32, (xw[0] - x0) as i32, (yw[1] - yw[0]) as i32));
                        run = None;
                    }
                    _ => {}
                }
            }
            if let Some(x0) = run {
                out.push((x0 as i32, yw[0] as i32, (pw as i64 - x0) as i32, (yw[1] - yw[0]) as i32));
            }
        }
        out
    }

    /// Indices of up to `k` best live seeds for the profile.
    pub fn lowest_seeds(&self, profile: Profile, k: usize) -> Vec<usize> {
        let mut v: Vec<usize> = (0..self.seeds.len()).filter(|&i| !self.seeds[i].dead).collect();
        let (pw, pd) = (self.place.width, self.place.depth);
        let key = |i: &usize| {
            let s = &self.seeds[*i];
            let (px, py) = s.sample();
            let ex = px.min(pw - 1 - px);
            let ey = py.min(pd - 1 - py);
            match profile {
                Profile::Layer => (s.z, ex + ey, px),
                Profile::Wall => (py, s.z, ex),
            }
        };
        if v.len() > k {
            v.select_nth_unstable_by_key(k, key);
            v.truncate(k);
        }
        v.sort_unstable_by_key(key);
        v
    }

    pub fn revive_seeds(&mut self) {
        self.seeds.iter_mut().for_each(|s| s.dead = false);
    }

    /// Commit a feasible candidate.
    pub fn place(&mut self, items: &[PrepItem], idx: usize, cand: &Candidate) {
        let it = &items[idx];
        let [w, d, h] = it.orients[cand.orient].dims;
        let (x, y, z) = (cand.x, cand.y, cand.z);
        let c = self.rule.clearance_mm;
        let foot = Rect::new(x, y, w, d);

        self.query(x - c, y - c, x + w + c, y + d + c);
        let mut supports: Vec<(usize, i64)> = Vec::new();
        let stol = self.rule.support_tolerance_mm;
        let mut rects = Vec::new();
        if z - self.base_z <= stol {
            rects.extend(foot.intersect(&Rect::new(0, 0, self.place.width, self.place.depth)));
        }
        if z > self.base_z {
            for &j in &self.buf {
                let b = &self.placed[j];
                if b.top() <= z && b.top() >= z - stol {
                    if let Some(r) = foot.intersect(&Rect::new(b.x, b.y, b.w, b.d)) {
                        supports.push((j, r.area()));
                        rects.push(r);
                    }
                }
            }
        }
        let support_area = union_area(&rects);

        let (side, _) = self.side_contacts(x, y, z, w, d, h, true);
        let neighbours: Vec<usize> = self.buf.clone();
        let braced = self.is_braced(&side, w, d, h);
        let margin = if braced { f64::INFINITY } else { self.stack_margin(&supports, w, d, h) };

        // Push the load down.
        let mut pending: Vec<(usize, f64)> = Vec::new();
        distribute(&mut pending, &supports, it.weight);
        while !pending.is_empty() {
            let (k, _) = pending.iter().enumerate().max_by_key(|(_, (j, _))| self.placed[*j].z).unwrap();
            let (j, delta) = pending.swap_remove(k);
            self.placed[j].received += delta;
            let sup = self.placed[j].supports.clone();
            distribute(&mut pending, &sup, delta);
        }

        // The new box also braces its neighbours.
        let g = self.rule.lateral_gap_mm + c;
        for j in neighbours {
            let b = &self.placed[j];
            let oz = overlap(z, z + h, b.z, b.z + b.h);
            if oz == 0 {
                continue;
            }
            let oy = overlap(y, y + d, b.y, b.y + b.d);
            let ox = overlap(x, x + w, b.x, b.x + b.w);
            let mut add = [0i64; 4];
            if oy > 0 && (0..=g).contains(&(x - (b.x + b.w))) {
                add[SIDE_POS_X] += oy * oz;
            }
            if oy > 0 && (0..=g).contains(&(b.x - (x + w))) {
                add[SIDE_NEG_X] += oy * oz;
            }
            if ox > 0 && (0..=g).contains(&(y - (b.y + b.d))) {
                add[SIDE_POS_Y] += ox * oz;
            }
            if ox > 0 && (0..=g).contains(&(b.y - (y + d))) {
                add[SIDE_NEG_Y] += ox * oz;
            }
            if add.iter().any(|&a| a > 0) {
                let (bw, bd, bh) = (b.w, b.d, b.h);
                let mut s = b.side_contact;
                for k in 0..4 {
                    s[k] += add[k];
                }
                let now_braced = self.is_braced(&s, bw, bd, bh);
                let b = &mut self.placed[j];
                b.side_contact = s;
                if now_braced {
                    b.margin = f64::INFINITY;
                }
            }
        }

        let new_index = self.placed.len();
        self.placed.push(Placed {
            item: idx,
            orient: cand.orient,
            x,
            y,
            z,
            w,
            d,
            h,
            supports,
            support_area,
            received: 0.0,
            side_contact: side,
            margin,
        });
        self.stamp.push(0);
        let (cx0, cy0, cx1, cy1) = self.cells(x - c, y - c, x + w + c, y + d + c);
        for cy in cy0..=cy1 {
            for cx in cx0..=cx1 {
                let top = z + h;
                let cell = &mut self.grid[(cy * self.gx + cx) as usize];
                let at = cell.partition_point(|&j| self.placed[j as usize].top() >= top);
                cell.insert(at, new_index as u32);
            }
        }
        self.weight += it.weight;
        self.volume += w as i64 * d as i64 * h as i64;

        // Candidate points: drop the ones whose surface changed, add this box's corners.
        self.seeds.retain(|s| {
            let (px, py) = s.sample();
            !(px >= x - c && px < x + w + c && py >= y - c && py < y + d + c)
        });
        let py = self.project_y(x + w + c, y, z, h);
        let px = self.project_x(x, y + d + c, z, h);
        let (f, t) = (false, true);
        let fresh = [
            // on top of the box
            (x, y, f, f),
            (x + w, y, t, f),
            (x, y + d, f, t),
            (x + w, y + d, t, t),
            // beside the box, aligned with its edges
            (x + w + c, y, f, f),
            (x + w + c, y + d, f, t),
            (x - c, y, t, f),
            (x - c, y + d, t, t),
            (x, y + d + c, f, f),
            (x + w, y + d + c, t, f),
            (x, y - c, f, t),
            (x + w, y - c, t, t),
            // projected toward the walls
            (x + w + c, py, f, f),
            (px, y + d + c, f, f),
        ];
        for (sx, sy, mx, my) in fresh {
            let mut seed = Seed { x: sx, y: sy, mx, my, z: 0, dead: false };
            let (qx, qy) = seed.sample();
            if qx < -self.ox || qy < -self.oy || qx >= self.place.width + self.ox || qy >= self.place.depth + self.oy {
                continue;
            }
            if self.seeds.iter().any(|s| s.x == sx && s.y == sy && s.mx == mx && s.my == my) {
                continue;
            }
            seed.z = self.point_height(qx, qy);
            if seed.z < self.max_h {
                self.seeds.push(seed);
            }
        }
    }

    /// Slide a point toward -Y until it meets a box face at the same height band or the wall.
    fn project_y(&mut self, px: i32, py: i32, z: i32, h: i32) -> i32 {
        let c = self.rule.clearance_mm;
        self.query(px, 0, px + 1, py);
        let mut best = 0;
        for &j in &self.buf {
            let b = &self.placed[j];
            if b.x <= px && px < b.x + b.w + c && b.y + b.d <= py && overlap(z, z + h, b.z, b.z + b.h) > 0 {
                best = best.max(b.y + b.d + c);
            }
        }
        best.min(py)
    }

    fn project_x(&mut self, px: i32, py: i32, z: i32, h: i32) -> i32 {
        let c = self.rule.clearance_mm;
        self.query(0, py, px, py + 1);
        let mut best = 0;
        for &j in &self.buf {
            let b = &self.placed[j];
            if b.y <= py && py < b.y + b.d + c && b.x + b.w <= px && overlap(z, z + h, b.z, b.z + b.h) > 0 {
                best = best.max(b.x + b.w + c);
            }
        }
        best.min(px)
    }
}

/// Is a standing box held on both sides across its thin direction (docs/DECISIONS.md §4b)?
/// `side` is contact area per side: -x, +x, -y, +y.
pub fn enclosed(side: &[i64; 4], w: i32, d: i32, h: i32, ratio: f64) -> bool {
    let fx = d as f64 * h as f64 * ratio;
    let fy = w as f64 * h as f64 * ratio;
    let across_x = side[SIDE_NEG_X] as f64 >= fx && side[SIDE_POS_X] as f64 >= fx;
    let across_y = side[SIDE_NEG_Y] as f64 >= fy && side[SIDE_POS_Y] as f64 >= fy;
    // Thin along x when w < d: it would tip over the x sides.
    if w < d {
        across_x
    } else if d < w {
        across_y
    } else {
        across_x || across_y
    }
}

/// Split `weight` between supports proportionally to contact area (ТЗ §9).
fn distribute(pending: &mut Vec<(usize, f64)>, supports: &[(usize, i64)], weight: f64) {
    let total: i64 = supports.iter().map(|s| s.1).sum();
    if total == 0 || weight == 0.0 {
        return;
    }
    for &(j, a) in supports {
        let delta = weight * a as f64 / total as f64;
        match pending.iter_mut().find(|(k, _)| *k == j) {
            Some(e) => e.1 += delta,
            None => pending.push((j, delta)),
        }
    }
}
