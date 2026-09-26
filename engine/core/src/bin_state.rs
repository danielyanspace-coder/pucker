//! State of one packing place during search, candidate evaluation and placement.
//!
//! Boxes are placed by "dropping" them at a candidate (x, y): the box lands on the highest
//! top it overlaps, so there are never collisions from below or above. Every hard constraint
//! is checked before a box is committed (ТЗ §32, stage 4).

use crate::geometry::{hull_of_rects, overlap, strictly_inside_doubled, support_levers, union_area, Rect};
use crate::model::{PackRule, PackingPlace};
use crate::prep::PrepItem;

const EPS: f64 = 1e-9;
/// Side gaps narrower than this are unlikely to be filled later.
const NARROW_GAP: i32 = 120;
/// Tipping reserve (mm of lever) that counts as plenty when scoring positions.
const RESERVE_MM: f64 = 100.0;
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
    /// Contact area with walls (or stretch wrap) per face.
    pub wall: [i64; 4],
    /// Neighbours touching a face: (placed index, face of this box, area).
    pub touch: Vec<(usize, u8, i64)>,
    /// Tipping reserve (mm) per direction (−x, +x, −y, +y) of this box with everything
    /// resting on it (`stack_check`). Infinite where it is held.
    pub margin: [f64; 4],
    /// Distance from the centre to the edge of the support, per direction (`support_levers`).
    pub lever: [f64; 4],
    pub mass: f64,
    /// Loads on the box (`Loads`, `stack_check`).
    pub loads: Loads,
    /// Held against tipping per direction (`stack_check`).
    pub pinned: [bool; 4],
    /// Push (kg) and moment (kg·mm) it can still take from boxes leaning on it (`Stacks::slack`).
    pub slack: [[f64; 2]; 4],
}

impl Placed {
    fn top(&self) -> i32 {
        self.z + self.h
    }

    fn tip_box<'b>(&'b self, supports: &'b [(usize, i64)]) -> TipBox<'b> {
        TipBox {
            x: self.x,
            y: self.y,
            z: self.z,
            w: self.w,
            d: self.d,
            h: self.h,
            mass: self.mass,
            wall: self.wall,
            touch: &self.touch,
            supports,
            lever: self.lever,
            tied: [false; 4],
        }
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

#[derive(Clone)]
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
        let lever = if support >= foot_area { full_levers(w, d) } else { support_levers(&self.rects, x as f64 + w as f64 / 2.0, y as f64 + d as f64 / 2.0) };
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
        let (side, anch, touch, flush) = self.side_contacts(x, y, z, w, d, h, false);
        let result = (|| {
            // Tipping reserve left for boxes that will stand on this one, 0..1.
            let mut reserve = 1.0;
            if self.rule.use_lateral_stability {
                match self.tip_check(&sup, &anch, &touch, lever, x, y, z, w, d, h, it.weight) {
                    Some(r) => reserve = r.min(RESERVE_MM) / RESERVE_MM,
                    None => return Err(Reject::Lateral),
                }
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
    /// Contact with walls (or stretch wrap) per face.
    fn wall_contact(&self, x: i32, y: i32, w: i32, d: i32, h: i32) -> [i64; 4] {
        let p = self.place;
        let mut s = [0i64; 4];
        if self.walls {
            let gaps = [x, p.width - (x + w), y, p.depth - (y + d)];
            let face = [d, d, w, w];
            for k in 0..4 {
                if lean_gap_ok(gaps[k] - self.rule.clearance_mm, h) {
                    s[k] = face[k] as i64 * h as i64;
                }
            }
        }
        s
    }

    /// Side contacts of a box at this position: total per face (walls and neighbours), the part
    /// that holds it (walls and neighbours that are themselves held that way), the touching
    /// neighbours (index, face, area) and how flush its top is with theirs.
    #[allow(clippy::type_complexity)]
    #[allow(clippy::too_many_arguments)]
    fn side_contacts(&mut self, x: i32, y: i32, z: i32, w: i32, d: i32, h: i32, fresh_query: bool) -> ([i64; 4], [i64; 4], Vec<(usize, u8, i64)>, f64) {
        let g = self.rule.lateral_gap_mm + self.rule.clearance_mm;
        let mut s = self.wall_contact(x, y, w, d, h);
        let mut anch = s;
        let mut touch = Vec::new();
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
            let mut faces: [i64; 4] = [0; 4];
            if oy > 0 {
                if (0..=g).contains(&(x - (b.x + b.w))) {
                    faces[SIDE_NEG_X] = oy * oz;
                }
                if (0..=g).contains(&(b.x - (x + w))) {
                    faces[SIDE_POS_X] = oy * oz;
                }
            }
            if ox > 0 {
                if (0..=g).contains(&(y - (b.y + b.d))) {
                    faces[SIDE_NEG_Y] = ox * oz;
                }
                if (0..=g).contains(&(b.y - (y + d))) {
                    faces[SIDE_POS_Y] = ox * oz;
                }
            }
            let gaps = [x - (b.x + b.w), b.x - (x + w), y - (b.y + b.d), b.y - (y + d)];
            let reach = (z + h).min(b.top()) - z;
            for k in 0..4 {
                if faces[k] > 0 {
                    s[k] += faces[k];
                    if b.pinned[k] && lean_gap_ok(gaps[k].max(0), reach) {
                        anch[k] += faces[k];
                    }
                    touch.push((j, k as u8, faces[k]));
                }
            }
            if faces.iter().any(|&f| f > 0) {
                let diff = (b.top() - (z + h)).abs();
                if diff <= 5 {
                    flush = 1.0;
                } else if diff <= 30 {
                    flush = flush.max(0.5);
                }
            }
        }
        (s, anch, touch, flush)
    }

    /// Can the box go here without tipping over or toppling anything under it? The box with
    /// everything already on its supports must keep the line of gravity plus transport force
    /// inside each support (`stack_check`). Returns the smallest reserve (mm), or None.
    #[allow(clippy::too_many_arguments)]
    fn tip_check(&self, sup: &[(usize, i64)], anch: &[i64; 4], touch: &[(usize, u8, i64)], lever: [f64; 4], x: i32, y: i32, z: i32, w: i32, d: i32, h: i32, mass: f64) -> Option<f64> {
        let wall = self.wall_contact(x, y, w, d, h);
        let cand = TipBox { x, y, z, w, d, h, mass, wall, touch, supports: sup, lever, tied: [false; 4] };
        let own = own_margins(lever, h, self.acc);
        let held = held_dirs(anch, w, d, h, own, self.rule);
        let add = own_loads(&cand);
        let mut worst = f64::INFINITY;
        let acc_nom = nominal(self.acc, self.rule);
        let mu = self.rule.friction;
        for k in 0..4 {
            if !held[k] {
                let bal = balance(&cand, &add, self.acc[k], k);
                let slide = (acc_nom[k] - mu) * mass;
                if (bal < -EPS || slide > EPS) && !self.can_lean(&cand, k, -bal, slide, own[k]) {
                    return None;
                }
                worst = worst.min((bal / mass.max(1e-9)).max(0.0));
            }
        }
        // Push the new weight down: every box under it now carries a share, riding with it or
        // pressing on it (`add_loads`).
        let foot = w as i64 * d as i64;
        let mut down: std::collections::BTreeMap<usize, (Loads, [bool; 4])> = Default::default();
        let total: i64 = sup.iter().map(|s| s.1).sum::<i64>().max(1);
        for &(j, a) in sup {
            let q = &self.placed[j];
            let fj = q.w as i64 * q.d as i64;
            let e = down.entry(j).or_insert(([0.0; 8], [false; 4]));
            add_loads(&mut e.0, &add, a as f64 / total as f64, &cand, &q.tip_box(&q.supports));
            let p = pins(a, foot, fj);
            for (e1, h1) in e.1.iter_mut().zip(held) {
                *e1 |= h1 && p;
            }
        }
        while let Some((j, (delta, pin))) = down.pop_last() {
            let p = &self.placed[j];
            let tb = p.tip_box(&p.supports);
            let new: Loads = std::array::from_fn(|q| p.loads[q] + delta[q]);
            for k in 0..4 {
                if p.pinned[k] || pin[k] {
                    continue;
                }
                // Change of its moment balance; what it had to spare covers it first.
                let bal = balance(&tb, &new, self.acc[k], k);
                let left = p.slack[k][1] + (bal - balance(&tb, &p.loads, self.acc[k], k));
                let left_f = p.slack[k][0] + (mu - acc_nom[k]) * weight_of(&delta);
                if left < -EPS || left_f < -EPS {
                    let own = own_margins(p.lever, p.h, self.acc)[k];
                    if !self.can_lean(&tb, k, -left, -left_f, own) {
                        return None;
                    }
                }
                worst = worst.min((bal / weight_of(&new).max(1e-9)).max(0.0));
            }
            let tot: i64 = p.supports.iter().map(|s| s.1).sum::<i64>().max(1);
            let fp = p.w as i64 * p.d as i64;
            for &(s2, a) in &p.supports {
                let q = &self.placed[s2];
                let fs = q.w as i64 * q.d as i64;
                let e = down.entry(s2).or_insert(([0.0; 8], [false; 4]));
                add_loads(&mut e.0, &delta, a as f64 / tot as f64, &tb, &q.tip_box(&q.supports));
                let ps = pins(a, fp, fs);
                for (e1, p1) in e.1.iter_mut().zip(pin) {
                    *e1 |= p1 && ps;
                }
            }
        }
        Some(worst)
    }

    /// Can box `b` pass a tipping deficit (kg·mm) and a sliding excess (kg) to what it touches
    /// on side `k`? Walls and held boxes take anything, others what they have left
    /// (`Stacks::slack`); one step only, the final check follows whole rows.
    fn can_lean(&self, b: &TipBox, k: usize, moment: f64, force: f64, own: f64) -> bool {
        let holding: Vec<(usize, i64)> = b
            .touch
            .iter()
            .filter(|t| {
                let q = &self.placed[t.0];
                let (g, r) = gap_reach(b, &q.tip_box(&q.supports), k);
                t.1 as usize == k && lean_gap_ok(g, r)
            })
            .map(|t| (t.0, t.2))
            .collect();
        let contact = b.wall[k] + holding.iter().map(|t| t.1).sum::<i64>();
        if !lean_contact_ok(b, k, contact, own, self.rule) {
            return false;
        }
        holding.iter().all(|&(j, a)| {
            let q = &self.placed[j];
            if q.pinned[k] {
                return true;
            }
            let qb = q.tip_box(&q.supports);
            let zc = contact_mid(b, &qb);
            let f = a as f64 / contact as f64 * force.max(moment / (zc - b.z as f64).max(1.0)).max(0.0);
            q.slack[k][0] >= f - EPS && q.slack[k][1] >= f * (zc - q.z as f64).max(1.0) - EPS
        })
    }

    /// Recompute bodies, holds and reserves after a change (new box, new contacts).
    fn refresh_margins(&mut self) {
        let n = self.placed.len();
        let st = {
            let boxes: Vec<TipBox> = self.placed.iter().map(|p| p.tip_box(&p.supports)).collect();
            let order: Vec<usize> = (0..n).collect();
            stack_check(&boxes, &order, self.acc, self.rule)
        };
        for (i, p) in self.placed.iter_mut().enumerate() {
            p.margin = st.reserve[i];
            p.loads = st.loads[i];
            p.pinned = st.pinned[i];
            p.slack = st.slack[i];
        }
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
                .map(|(p, t)| TipBox { tied: t, ..p.tip_box(&p.supports) })
                .collect();
            // Supports are always placed earlier, so index order is bottom-up.
            let order: Vec<usize> = (0..n).collect();
            let margin = stack_check(&boxes, &order, self.acc, self.rule).reserve;
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

        let (side, _, touch, _) = self.side_contacts(x, y, z, w, d, h, true);
        let wall = self.wall_contact(x, y, w, d, h);
        let new_index = self.placed.len();
        let neighbours: Vec<usize> = self.buf.clone();
        let lever = box_levers(&rects, x, y, w, d);
        let m = it.weight;
        let loads = [m, m * (x as f64 + w as f64 / 2.0), m * (y as f64 + d as f64 / 2.0), m * (z as f64 + h as f64 / 2.0), 0.0, 0.0, 0.0, 0.0];

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
                let b = &mut self.placed[j];
                for (k, &a) in add.iter().enumerate() {
                    b.side_contact[k] += a;
                    if a > 0 {
                        b.touch.push((new_index, k as u8, a));
                    }
                }
            }
        }

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
            wall,
            touch,
            margin: [f64::INFINITY; 4],
            lever,
            mass: m,
            loads,
            pinned: [false; 4],
            slack: [[f64::INFINITY; 2]; 4],
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
/// All are multiplied by `tip_safety_factor`.
pub fn tip_accels(place: &crate::model::PackingPlace, rule: &PackRule) -> [f64; 4] {
    let k = rule.tip_safety_factor.max(1.0);
    let lat = rule.accel_lateral_g.max(0.01) * k;
    if place.place_type == crate::model::PackingPlaceType::Pallet {
        [lat; 4]
    } else {
        [lat, lat, (rule.accel_longitudinal_g * k).max(lat), lat]
    }
}

/// Reserve (mm) of a free-standing box against tipping in each direction: a uniform box tips
/// when a·h/2 exceeds the lever to the edge of its support, so it stays upright while
/// h ≤ 2·lever / a. Standing fully on something, the lever is half its base.
pub fn own_margins(lever: [f64; 4], h: i32, acc: [f64; 4]) -> [f64; 4] {
    std::array::from_fn(|k| 2.0 * lever[k] / acc[k] - h as f64)
}

/// Levers of a box whose whole base is supported.
pub fn full_levers(w: i32, d: i32) -> [f64; 4] {
    let (hw, hd) = (w as f64 / 2.0, d as f64 / 2.0);
    [hw, hw, hd, hd]
}

/// Levers of a box at (x, y) of size w × d standing on `rects` (parts of its footprint).
pub fn box_levers(rects: &[Rect], x: i32, y: i32, w: i32, d: i32) -> [f64; 4] {
    if union_area(rects) >= w as i64 * d as i64 {
        return full_levers(w, d);
    }
    support_levers(rects, x as f64 + w as f64 / 2.0, y as f64 + d as f64 / 2.0)
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

/// One box for the tipping check: position, size, mass, walls and neighbours at its sides,
/// the boxes it rests on with contact areas and its levers (`support_levers`).
#[derive(Clone)]
pub struct TipBox<'b> {
    pub x: i32,
    pub y: i32,
    pub z: i32,
    pub w: i32,
    pub d: i32,
    pub h: i32,
    pub mass: f64,
    /// Contact with walls (or stretch wrap) per face.
    pub wall: [i64; 4],
    /// Touching neighbours: (index, face of this box, area).
    pub touch: &'b [(usize, u8, i64)],
    pub supports: &'b [(usize, i64)],
    pub lever: [f64; 4],
    /// Directions held by securing rather than by neighbours: a strap or load bar across
    /// the rear face of the load in a vehicle (+y). Stretch wrap on a pallet only keeps
    /// boxes from falling outwards (it acts as a wall); it does not stop a box tipping
    /// inwards into a gap.
    pub tied: [bool; 4],
}

/// Directions each box ([x, y, z, w, d, h]) is held in by load securing (see `TipBox::tied`).
pub fn tied_dirs(place: &crate::model::PackingPlace, rule: &PackRule, boxes: &[[i32; 6]]) -> Vec<[bool; 4]> {
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

/// Loads on a box for the tipping check. What rides with it as one body: the box itself and
/// boxes standing mostly on it (`RIDE`): mass M and moments M·x, M·y, M·z. And what only
/// presses on it from boxes bonded over it and other boxes: weight N at the contact
/// (N·x, N·y), whose inertia pushes at its top (N·z). A free column tips as one body; a box
/// under a bonded layer carries the layer's weight but does not tip it over with itself.
pub type Loads = [f64; 8];

/// Share of its support a box must have on one box to ride with it.
const RIDE: f64 = 0.8;

/// Loads of a box alone, uniform density.
pub fn own_loads(b: &TipBox) -> Loads {
    let m = b.mass.max(1e-6);
    [m, m * (b.x as f64 + b.w as f64 / 2.0), m * (b.y as f64 + b.d as f64 / 2.0), m * (b.z as f64 + b.h as f64 / 2.0), 0.0, 0.0, 0.0, 0.0]
}

/// Total weight carried (kg).
fn weight_of(l: &Loads) -> f64 {
    l[0] + l[4]
}

/// Add the loads `l` of box `upper` (a `share` of it rests on `lower`) to `acc`.
fn add_loads(acc: &mut Loads, l: &Loads, share: f64, upper: &TipBox, lower: &TipBox) {
    if share >= RIDE {
        for q in 0..8 {
            acc[q] += share * l[q];
        }
    } else {
        let n = share * weight_of(l);
        let x0 = upper.x.max(lower.x) as f64;
        let x1 = (upper.x + upper.w).min(lower.x + lower.w) as f64;
        let y0 = upper.y.max(lower.y) as f64;
        let y1 = (upper.y + upper.d).min(lower.y + lower.d) as f64;
        acc[4] += n;
        acc[5] += n * (x0 + x1) / 2.0;
        acc[6] += n * (y0 + y1) / 2.0;
        acc[7] += n * (lower.z + lower.h) as f64;
    }
}

/// Moment balance (kg·mm) of box `b` with loads `l` against tipping in direction `k` at
/// acceleration `a`: weight times the distance to the edge of the support, minus inertia
/// times its height over the support. Negative = it tips.
pub fn balance(b: &TipBox, l: &Loads, a: f64, k: usize) -> f64 {
    let cx = b.x as f64 + b.w as f64 / 2.0;
    let cy = b.y as f64 + b.d as f64 / 2.0;
    let dist = |px: f64, py: f64| match k {
        0 => px - (cx - b.lever[0]),
        1 => cx + b.lever[1] - px,
        2 => py - (cy - b.lever[2]),
        _ => cy + b.lever[3] - py,
    };
    let mut bal = 0.0;
    for (m, o) in [(l[0], 1), (l[4], 5)] {
        if m > 1e-9 {
            let (px, py, pz) = (l[o] / m, l[o + 1] / m, l[o + 2] / m);
            bal += m * (dist(px, py) - a * (pz - b.z as f64));
        }
    }
    bal
}

/// Reserve (mm): the balance per kg carried.
pub fn reserve_of(b: &TipBox, l: &Loads, a: f64, k: usize) -> f64 {
    balance(b, l, a, k) / weight_of(l).max(1e-9)
}

/// A box resting on another with this much contact passes its hold down to it: if the upper
/// box cannot move that way, friction under it keeps the lower one from tipping.
fn pins(area: i64, foot_upper: i64, foot_lower: i64) -> bool {
    area * 5 >= foot_upper.min(foot_lower)
}

/// Stability of a finished load (docs/DECISIONS.md §4). For every box and direction, the box
/// with what rides on it and what presses on it (`Loads`) must not tip over the edge of its
/// support (`balance`). A box is held in a
/// direction, and then not checked, when it leans on a wall, securing or a neighbour that is
/// itself held that way (a tight row up to the wall), or when a box resting on it is held
/// that way. A neighbour that could itself fall over holds nothing. `order` lists the boxes
/// bottom-up.
pub struct Stacks {
    pub pinned: Vec<[bool; 4]>,
    pub loads: Vec<Loads>,
    /// Reserve (mm) per direction, infinite where held (`reserve_of`); negative = tips.
    /// A box that leans on neighbours able to carry it gets 0.
    pub reserve: Vec<[f64; 4]>,
    /// What the box can still take from boxes leaning on it, per direction, after what
    /// already leans on it: push force (kg) before it slides, and moment (kg·mm) before it
    /// tips. Infinite where held.
    pub slack: Vec<[[f64; 2]; 4]>,
    /// Fails by sliding (pushed or thrown harder than friction holds, nothing to lean on).
    pub slides: Vec<[bool; 4]>,
}

/// Contact centre height of two touching boxes, for the lever of a push between them.
fn contact_mid(a: &TipBox, b: &TipBox) -> f64 {
    let lo = a.z.max(b.z) as f64;
    let hi = (a.z + a.h).min(b.z + b.h) as f64;
    (lo + hi) / 2.0
}

/// Largest tilt (rad) at which a box may still close a gap to what it leans on. A gap low
/// down near the pivot takes a large tilt to close: the box topples over a low neighbour
/// like over a kerb.
const LEAN_TILT: f64 = 0.035;

/// Does a gap of `gap` mm to a neighbour whose contact reaches `reach` mm above the box's
/// bottom still hold it (closes within `LEAN_TILT`)?
pub fn lean_gap_ok(gap: i32, reach: i32) -> bool {
    gap as f64 <= 2.0 + LEAN_TILT * reach.max(0) as f64
}

/// Gap along face `k` of `a` to `b`, and how high above `a`'s bottom their contact reaches.
fn gap_reach(a: &TipBox, b: &TipBox, k: usize) -> (i32, i32) {
    let gap = match k {
        0 => a.x - (b.x + b.w),
        1 => b.x - (a.x + a.w),
        2 => a.y - (b.y + b.d),
        _ => b.y - (a.y + a.d),
    };
    (gap.max(0), (a.z + a.h).min(b.z + b.h) - a.z)
}

/// Touches on face `k` of box `i` that can hold it (`lean_gap_ok`).
fn holding_touches<'a>(boxes: &'a [TipBox], i: usize, k: usize) -> impl Iterator<Item = (usize, i64)> + 'a {
    boxes[i].touch.iter().filter(move |t| {
        if t.1 as usize != k {
            return false;
        }
        let (g, r) = gap_reach(&boxes[i], &boxes[t.0], k);
        lean_gap_ok(g, r)
    }).map(|t| (t.0, t.2))
}

/// Contact needed on a face to lean on it: half the face for a box that could not stand
/// on its own, a fifth otherwise.
fn lean_contact_ok(b: &TipBox, k: usize, contact: i64, own: f64, rule: &PackRule) -> bool {
    let face = if k < 2 { b.d } else { b.w } as f64 * b.h as f64;
    let ratio = if own < 0.0 { rule.standing_contact_ratio } else { rule.lateral_min_contact_ratio };
    contact > 0 && contact as f64 >= face * ratio
}

pub fn stack_check(boxes: &[TipBox], order: &[usize], acc: [f64; 4], rule: &PackRule) -> Stacks {
    let acc_nom = nominal(acc, rule);
    let n = boxes.len();
    let foot = |i: usize| boxes[i].w as i64 * boxes[i].d as i64;
    let mut above: Vec<Vec<(usize, i64)>> = vec![Vec::new(); n];
    for (u, b) in boxes.iter().enumerate() {
        for &(j, a) in b.supports {
            above[j].push((u, a));
        }
    }
    let mut loads: Vec<Loads> = boxes.iter().map(own_loads).collect();
    for &i in order.iter().rev() {
        let mut acc = loads[i];
        for &(u, a) in &above[i] {
            let tot: i64 = boxes[u].supports.iter().map(|s| s.1).sum::<i64>().max(1);
            add_loads(&mut acc, &loads[u], a as f64 / tot as f64, &boxes[u], &boxes[i]);
        }
        loads[i] = acc;
    }
    // Held: grows from the walls inwards through tight contacts, and down from held boxes to
    // what they stand on. Only from boxes already known to be held, never in a circle.
    let own: Vec<[f64; 4]> = boxes.iter().map(|b| own_margins(b.lever, b.h, acc)).collect();
    let mut pinned: Vec<[bool; 4]> = boxes.iter().map(|b| b.tied).collect();
    loop {
        let mut changed = false;
        for i in 0..n {
            let b = &boxes[i];
            let mut anch = b.wall;
            for k in 0..4 {
                for (j, a) in holding_touches(boxes, i, k) {
                    if pinned[j][k] {
                        anch[k] += a;
                    }
                }
            }
            let held = held_dirs(&anch, b.w, b.d, b.h, own[i], rule);
            for k in 0..4 {
                if !pinned[i][k] && held[k] {
                    pinned[i][k] = true;
                    changed = true;
                }
            }
        }
        for &i in order.iter().rev() {
            let from_above: [bool; 4] = std::array::from_fn(|k| above[i].iter().any(|&(u, a)| pinned[u][k] && pins(a, foot(u), foot(i))));
            for (p, f) in pinned[i].iter_mut().zip(from_above) {
                if !*p && f {
                    *p = true;
                    changed = true;
                }
            }
        }
        if !changed {
            break;
        }
    }
    let mut reserve: Vec<[f64; 4]> = (0..n)
        .map(|i| std::array::from_fn(|k| if pinned[i][k] { f64::INFINITY } else { reserve_of(&boxes[i], &loads[i], acc[k], k) }))
        .collect();
    // Leaning (CTU Code: rows leaning on each other load the front one) and bonding. A box
    // that would tip, or is pushed harder than friction holds, needs the excess taken off:
    // - by the neighbours it touches on that side (shared by contact area): walls and held
    //   boxes take anything, others carry it from their own slack or pass it on;
    // - failing that, by the boxes resting on it that also rest on other boxes (a bonded
    //   layer, as bricks in a wall): friction under such a box on its other supports holds
    //   the top of this one, up to (friction − acceleration) × the weight it puts there,
    //   and those supports take the push at their top.
    // Pushes go to boxes already handled too; those are handled again (bounded).
    let mut slack = vec![[[f64::INFINITY; 2]; 4]; n];
    let mut slides = vec![[false; 4]; n];
    for k in 0..4 {
        let mut idx: Vec<usize> = (0..n).collect();
        idx.sort_by_key(|&i| {
            let b = &boxes[i];
            match k {
                0 => -(b.x + b.w),
                1 => b.x,
                2 => -(b.y + b.d),
                _ => b.y,
            }
        });
        let mut push_f = vec![0.0f64; n];
        let mut push_m = vec![0.0f64; n];
        // Pushes each box has passed on: (target, force, moment), undone when handled again.
        let mut passed: Vec<Vec<(usize, f64, f64)>> = vec![Vec::new(); n];
        let mut failed = vec![false; n];
        let mut queue: std::collections::VecDeque<usize> = idx.iter().copied().collect();
        let mut queued = vec![true; n];
        let mut budget = 20 * n + 100;
        while let Some(i) = queue.pop_front() {
            queued[i] = false;
            if pinned[i][k] {
                continue;
            }
            if budget == 0 {
                failed[i] = true;
                continue;
            }
            budget -= 1;
            for (j, f, m) in std::mem::take(&mut passed[i]) {
                push_f[j] -= f;
                push_m[j] -= m;
            }
            failed[i] = false;
            let b = &boxes[i];
            let mass = weight_of(&loads[i]);
            let sf = (rule.friction - acc_nom[k]) * mass - push_f[i];
            let sm = balance(&boxes[i], &loads[i], acc[k], k) - push_m[i];
            if sf >= -EPS && sm >= -EPS {
                continue;
            }
            let (ex_f, ex_m) = ((-sf).max(0.0), (-sm).max(0.0));
            let mut out: Vec<(usize, f64, f64)> = Vec::new();
            let contact = b.wall[k] + holding_touches(boxes, i, k).map(|t| t.1).sum::<i64>();
            if lean_contact_ok(b, k, contact, own[i][k], rule) {
                for (j, a) in holding_touches(boxes, i, k) {
                    let share = a as f64 / contact as f64;
                    let zc = contact_mid(b, &boxes[j]);
                    let f = share * ex_f.max(ex_m / (zc - b.z as f64).max(1.0));
                    out.push((j, f, f * (zc - boxes[j].z as f64).max(1.0)));
                }
            } else {
                // Bonding from above.
                let need = ex_f.max(ex_m / (b.h as f64).max(1.0));
                let mut caps: Vec<(usize, f64)> = Vec::new();
                for &(u, _) in &above[i] {
                    let tot: i64 = boxes[u].supports.iter().map(|s| s.1).sum::<i64>().max(1);
                    for &(k2, a2) in boxes[u].supports {
                        if k2 == i || failed[k2] {
                            continue;
                        }
                        let c = (rule.friction - acc_nom[k]).max(0.0) * weight_of(&loads[u]) * a2 as f64 / tot as f64;
                        if c > 0.0 {
                            caps.push((k2, c));
                        }
                    }
                }
                let cap: f64 = caps.iter().map(|c| c.1).sum();
                if cap + EPS < need {
                    failed[i] = true;
                    continue;
                }
                for (k2, c) in caps {
                    let f = need * c / cap;
                    out.push((k2, f, f * boxes[k2].h as f64));
                }
            }
            for &(j, f, m) in &out {
                if pinned[j][k] {
                    continue;
                }
                push_f[j] += f;
                push_m[j] += m;
                if !queued[j] {
                    queued[j] = true;
                    queue.push_back(j);
                }
            }
            passed[i] = out.into_iter().filter(|o| !pinned[o.0][k]).collect();
        }
        for i in 0..n {
            if pinned[i][k] {
                continue;
            }
            let mass = weight_of(&loads[i]);
            let sf = (rule.friction - acc_nom[k]) * mass - push_f[i];
            let sm = balance(&boxes[i], &loads[i], acc[k], k) - push_m[i];
            if failed[i] {
                slides[i][k] = sm >= -EPS;
                reserve[i][k] = reserve[i][k].min(-1.0);
                slack[i][k] = [0.0, 0.0];
            } else if sf >= -EPS && sm >= -EPS {
                slack[i][k] = [sf, sm];
            } else {
                slack[i][k] = [0.0, 0.0];
                reserve[i][k] = reserve[i][k].max(0.0);
            }
        }
    }
    Stacks { pinned, loads, reserve, slack, slides }
}

/// Transport accelerations without the tipping safety factor (for sliding).
pub fn nominal(acc: [f64; 4], rule: &PackRule) -> [f64; 4] {
    let k = rule.tip_safety_factor.max(1.0);
    std::array::from_fn(|q| acc[q] / k)
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
