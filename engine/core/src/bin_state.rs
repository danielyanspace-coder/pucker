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
/// Tipping reserve (mm) that counts as plenty when scoring positions.
const RESERVE_MM: f64 = 600.0;
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
    /// Tipping reserve (mm) per direction (−x, +x, −y, +y) of the stack this box tops:
    /// how much taller it could be before tipping at the transport acceleration.
    /// Infinite in a direction where the box leans on a neighbour or wall.
    pub margin: [f64; 4],
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
    /// Accelerations (g) boxes must withstand without tipping, per direction.
    acc: [f64; 4],
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
            walls: place.holds_sides(),
            payload: place.payload_limit(),
            acc: tip_accels(place, rule),
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
            // Tipping reserve left for boxes that will stand on this one, 0..1.
            let mut reserve = 1.0;
            if self.rule.use_lateral_stability {
                let m = self.tip_margins(&sup, &side, x, y, w, d, h);
                if m.iter().any(|&v| v < -EPS) {
                    return Err(Reject::Lateral);
                }
                reserve = m.iter().fold(f64::INFINITY, |a, &v| a.min(v)).min(RESERVE_MM) / RESERVE_MM;
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
            // A box exactly on top of one of the same footprint builds a column: columns of
            // equal boxes stand side by side and hold each other all the way up.
            let column = sup.len() == 1 && {
                let b = &self.placed[sup[0].0];
                b.x == x && b.y == y && b.w == w && b.d == d
            };
            let flush = if column { 1.0 } else { flush };
            let side_part = (side_total as f64 / side_max as f64).min(1.0);
            let contact = 0.3 * bottom + 0.45 * side_part + 0.25 * reserve;
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

    /// Tipping reserve in each of the four directions (docs/DECISIONS.md §4): unless the box
    /// leans on a neighbour or wall on that side, the box and every free level below must
    /// stay upright at that direction's transport acceleration. A box resting on a firm
    /// support ties the other stacks under it (bonding), as in `final_tip_margins`.
    #[allow(clippy::too_many_arguments)]
    fn tip_margins(&self, supports: &[(usize, i64)], side: &[i64; 4], x: i32, y: i32, w: i32, d: i32, h: i32) -> [f64; 4] {
        if self.tied(x, y, w, d) {
            return [f64::INFINITY; 4];
        }
        let own = own_margins(w, d, h, self.acc);
        let held = held_dirs(side, w, d, h, own, self.rule);
        let foot = w as i64 * d as i64;
        let mut m = [f64::INFINITY; 4];
        for a in 0..4 {
            if held[a] {
                continue;
            }
            m[a] = supports.iter().fold(own[a], |acc, &(j, area)| {
                let b = &self.placed[j];
                let bridged = area * 5 >= foot.min(b.w as i64 * b.d as i64)
                    && supports.iter().any(|&(k, ka)| k != j && ka * 4 >= foot && self.placed[k].margin[a] >= -EPS);
                if bridged { acc } else { acc.min(b.margin[a] - h as f64) }
            });
        }
        m
    }

    /// Recompute every box's tipping reserve after a change (new contacts, new bonds).
    fn refresh_margins(&mut self) {
        let n = self.placed.len();
        let boxes: Vec<TipBox> = self
            .placed
            .iter()
            .map(|p| TipBox { w: p.w, d: p.d, h: p.h, side: p.side_contact, supports: &p.supports, tied: [self.tied(p.x, p.y, p.w, p.d); 4] })
            .collect();
        let order: Vec<usize> = (0..n).collect();
        let margin = final_tip_margins(&boxes, &order, self.acc, self.rule);
        drop(boxes);
        for (p, m) in self.placed.iter_mut().zip(margin) {
            p.margin = m;
        }
    }

    /// On the perimeter of a stretch-wrapped pallet (touching the film).
    fn tied(&self, x: i32, y: i32, w: i32, d: i32) -> bool {
        let g = self.rule.lateral_gap_mm + self.rule.clearance_mm;
        self.place.wrap_ties() && (x <= g || y <= g || x + w >= self.place.width - g || y + d >= self.place.depth - g)
    }

    /// Final tipping check of the finished load (docs/DECISIONS.md §4), including the load
    /// securing the result asks for (`tied_dirs`). Boxes that neither are held nor stand on
    /// their own in some direction are taken out together with everything resting on them,
    /// and the rest is rebuilt; repeated until stable. Returns the removed items.
    pub fn settle(&mut self, items: &[PrepItem]) -> Vec<usize> {
        let mut removed = Vec::new();
        if !self.rule.use_lateral_stability {
            return removed;
        }
        loop {
            let n = self.placed.len();
            let dims: Vec<[i32; 6]> = self.placed.iter().map(|p| [p.x, p.y, p.z, p.w, p.d, p.h]).collect();
            let tied = tied_dirs(self.place, self.rule, &dims);
            let boxes: Vec<TipBox> = self
                .placed
                .iter()
                .zip(tied)
                .map(|(p, t)| TipBox { w: p.w, d: p.d, h: p.h, side: p.side_contact, supports: &p.supports, tied: t })
                .collect();
            // Supports are always placed earlier, so index order is bottom-up.
            let order: Vec<usize> = (0..n).collect();
            let margin = final_tip_margins(&boxes, &order, self.acc, self.rule);
            drop(boxes);
            let mut drop = vec![false; n];
            for i in 0..n {
                drop[i] = self.placed[i].supports.iter().any(|&(j, _)| drop[j]) || margin[i].iter().any(|&m| m < -EPS);
            }
            if !drop.contains(&true) {
                break;
            }
            let mut nb = BinState::new(self.place, self.place_index, self.rule);
            nb.min_cube = self.min_cube;
            nb.max_h = self.max_h;
            for (i, p) in std::mem::take(&mut self.placed).into_iter().enumerate() {
                if drop[i] {
                    removed.push(p.item);
                } else {
                    nb.place(items, p.item, &Candidate { orient: p.orient, x: p.x, y: p.y, z: p.z, score: 0.0 });
                }
            }
            *self = nb;
        }
        removed
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
        let margin = self.tip_margins(&supports, &side, x, y, w, d, h);

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
                let mut s = b.side_contact;
                for k in 0..4 {
                    s[k] += add[k];
                }
                self.placed[j].side_contact = s;
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
        if self.rule.use_lateral_stability {
            self.refresh_margins();
        }
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

/// Accelerations (in g) a box must withstand without tipping, per direction
/// (−x, +x, −y, +y). Road transport (EN 12195-1): 0.5 g sideways and rearwards, 0.8 g
/// forwards when braking. In vehicles and containers the front wall is at y = 0 (loading
/// starts there, doors at the far end), so braking throws the cargo towards −y. A pallet
/// may be loaded either way round, so it gets 0.5 g in every direction (EUMOS 40509).
pub fn tip_accels(place: &crate::model::PackingPlace, rule: &PackRule) -> [f64; 4] {
    let lat = rule.accel_lateral_g.max(0.01);
    if place.place_type == crate::model::PackingPlaceType::Pallet {
        [lat; 4]
    } else {
        [lat, lat, rule.accel_longitudinal_g.max(lat), lat]
    }
}

/// Reserve (mm) of a free-standing box against tipping in each direction: a uniform box tips
/// when a·h/2 > b/2, so it stays upright while h ≤ b / a.
pub fn own_margins(w: i32, d: i32, h: i32, acc: [f64; 4]) -> [f64; 4] {
    let (w, d, h) = (w as f64, d as f64, h as f64);
    [w / acc[0] - h, w / acc[1] - h, d / acc[2] - h, d / acc[3] - h]
}

/// Is the box blocked from tipping in each direction, i.e. does it lean on a neighbour, wall
/// or stretch wrap on that side? A box that could not stand on its own in that direction
/// needs `standing_contact_ratio` of the face touched, others `lateral_min_contact_ratio`.
pub fn held_dirs(side: &[i64; 4], w: i32, d: i32, h: i32, own: [f64; 4], rule: &PackRule) -> [bool; 4] {
    let face = [d, d, w, w];
    let mut held = [false; 4];
    for k in 0..4 {
        let ratio = if own[k] < 0.0 { rule.standing_contact_ratio } else { rule.lateral_min_contact_ratio };
        held[k] = side[k] as f64 >= face[k] as f64 * h as f64 * ratio;
    }
    held
}

/// One box for the final tipping check: size, side contacts (−x, +x, −y, +y) and the boxes
/// it rests on with contact areas.
pub struct TipBox<'b> {
    pub w: i32,
    pub d: i32,
    pub h: i32,
    pub side: [i64; 4],
    pub supports: &'b [(usize, i64)],
    /// Directions held by securing rather than by neighbours: the stretch wrap for boxes on
    /// the perimeter of a pallet (every way), a strap or load bar across the rear face of
    /// the load in a vehicle (+y).
    pub tied: [bool; 4],
}

/// Directions each box ([x, y, z, w, d, h]) is held in by load securing (see `TipBox::tied`).
pub fn tied_dirs(place: &crate::model::PackingPlace, rule: &PackRule, boxes: &[[i32; 6]]) -> Vec<[bool; 4]> {
    let g = rule.lateral_gap_mm + rule.clearance_mm;
    if place.wrap_ties() {
        return boxes
            .iter()
            .map(|&[x, y, _, w, d, _]| {
                let t = x <= g || y <= g || x + w >= place.width - g || y + d >= place.depth - g;
                [t; 4]
            })
            .collect();
    }
    if !place.has_walls() || !rule.secure_rear_face {
        return vec![[false; 4]; boxes.len()];
    }
    // The strap across the rear face reaches a box when nothing stands behind it.
    boxes
        .iter()
        .map(|&[x, y, z, w, d, h]| {
            let hidden = boxes.iter().any(|&[bx, by, bz, bw, _, bh]| {
                by >= y + d && overlap(x, x + w, bx, bx + bw) > 0 && overlap(z, z + h, bz, bz + bh) > 0
            });
            [false, false, false, !hidden]
        })
        .collect()
}

/// Tipping reserve per box and direction in the finished load (docs/DECISIONS.md §4);
/// negative = the box (with the stack under it) tips at the transport acceleration.
/// `order` lists the boxes bottom-up. A box is held in a direction when
/// - it is tied by securing (`TipBox::tied`), or
/// - it leans on a neighbour or wall on that side (`held_dirs`), or
/// - a box resting on it cannot move that way: that box is itself held, or also rests on
///   another box that stands firm (bonding, «перевязка»: the bridge ties the stacks and
///   friction under it keeps the top of this stack in place); held tops pass this down
///   the column.
///
/// Everything else must stand on its own: the box and every free level under it.
/// Holding is only derived from boxes already known to be firm, so boxes never hold
/// each other up in a circle.
pub fn final_tip_margins(boxes: &[TipBox], order: &[usize], acc: [f64; 4], rule: &PackRule) -> Vec<[f64; 4]> {
    let n = boxes.len();
    let foot = |i: usize| boxes[i].w as i64 * boxes[i].d as i64;
    let mut above: Vec<Vec<(usize, i64)>> = vec![Vec::new(); n];
    for (u, b) in boxes.iter().enumerate() {
        for &(j, a) in b.supports {
            above[j].push((u, a));
        }
    }
    let own: Vec<[f64; 4]> = boxes.iter().map(|b| own_margins(b.w, b.d, b.h, acc)).collect();
    let mut fixed: Vec<[bool; 4]> = boxes
        .iter()
        .zip(&own)
        .map(|(b, o)| {
            let f = held_dirs(&b.side, b.w, b.d, b.h, *o, rule);
            std::array::from_fn(|k| f[k] || b.tied[k])
        })
        .collect();
    let mut margin = vec![[f64::INFINITY; 4]; n];
    loop {
        for &i in order {
            let b = &boxes[i];
            for a in 0..4 {
                margin[i][a] = if fixed[i][a] {
                    f64::INFINITY
                } else {
                    b.supports.iter().fold(own[i][a], |m, &(j, _)| m.min(margin[j][a] - b.h as f64))
                };
            }
        }
        let mut changed = false;
        for i in 0..n {
            for a in 0..4 {
                if fixed[i][a] {
                    continue;
                }
                let tied = above[i].iter().any(|&(u, area)| {
                    area * 5 >= foot(i).min(foot(u))
                        && (fixed[u][a]
                            || boxes[u].supports.iter().any(|&(k, ka)| k != i && ka * 4 >= foot(u) && margin[k][a] >= -EPS))
                });
                if tied {
                    fixed[i][a] = true;
                    changed = true;
                }
            }
        }
        if !changed {
            return margin;
        }
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
