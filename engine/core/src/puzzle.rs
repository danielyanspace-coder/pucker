//! «Puzzle» layers (docs/DECISIONS.md §4c): full, flat layers of boxes of one height, laid
//! in rows that run from wall to wall with no gap wider than a box can close by leaning
//! (`lean_gap_ok`). In such a layer every box is held on all four sides through its row and
//! the neighbouring rows up to the walls or the stretch wrap, so it can neither tip nor
//! slide, and the next layer stands on a flat top.
//!
//! A layer of height `H` takes boxes that have a side within `support_tolerance_mm` below
//! `H`. It is built as strips across the place: every strip has one depth, its boxes are
//! chosen by an exact subset sum so their lengths fill the width, and the strip depths add
//! up to the depth of the place. Each planned layer goes through the full checks of
//! `BinState`; a layer that fails anywhere is dropped as a whole.

use crate::bin_state::BinState;
use crate::model::PackRule;
use crate::prep::PrepItem;
use crate::rng::Rng;

/// Attempts to plan one layer at a given height and direction of the strips.
const ATTEMPTS: usize = 6;
/// Layer heights tried per layer.
const HEIGHTS: usize = 6;
/// Strip depths tried at each step of a plan.
const DEPTHS: usize = 4;

/// Widest gap (mm) a box of height `h` may leave to what holds it: it closes within the tilt
/// allowed by `lean_gap_ok`.
fn tol(h: i32) -> i32 {
    (1.0 + 0.03 * h.max(0) as f64).floor() as i32
}

/// A box that can stand in a layer: its upright orientation and footprint.
#[derive(Clone, Copy)]
struct Piece {
    idx: usize,
    /// Orientation for footprint `(w, d)`; `turned` for `(d, w)`.
    orient: usize,
    turned: Option<usize>,
    w: i32,
    d: i32,
}

/// Upright orientation with height in `[lo, hi]` (the tallest one).
fn upright(it: &PrepItem, lo: i32, hi: i32) -> Option<Piece> {
    let (o, dims) = it
        .orients
        .iter()
        .enumerate()
        .filter(|(_, o)| (lo..=hi).contains(&o.dims[2]))
        .max_by_key(|(_, o)| o.dims[2])
        .map(|(k, o)| (k, o.dims))?;
    let [w, d, h] = dims;
    let turned = it.orients.iter().position(|q| q.dims == [d, w, h]);
    Some(Piece { idx: 0, orient: o, turned, w, d })
}

/// Exact subset sum: pieces whose lengths add up to `[len - slack, len]`, longest total
/// first. Returns indices into `lens`. Earlier entries are preferred.
fn subset_fill(lens: &[i32], len: i32, slack: i32) -> Option<Vec<usize>> {
    let n = len.max(0) as usize;
    let mut who = vec![u32::MAX; n + 1];
    let mut reach = vec![false; n + 1];
    reach[0] = true;
    for (k, &a) in lens.iter().enumerate() {
        let a = a as usize;
        if a == 0 || a > n {
            continue;
        }
        for s in (a..=n).rev() {
            if !reach[s] && reach[s - a] {
                reach[s] = true;
                who[s] = k as u32;
            }
        }
        if reach[n] {
            break;
        }
    }
    let lo = (len - slack).max(1) as usize;
    let mut s = (lo..=n).rev().find(|&s| reach[s])?;
    let mut out = Vec::new();
    while s > 0 {
        let k = who[s] as usize;
        out.push(k);
        s -= lens[k] as usize;
    }
    Some(out)
}

/// A layer plan: (item, orientation, x, y) for every box.
type Plan = Vec<(usize, usize, i32, i32)>;

/// Plan one layer of height `h` in a `width × depth` area, strips running along x.
fn plan_layer(pieces: &[Piece], h: i32, width: i32, depth: i32, rng: &mut Rng) -> Option<Plan> {
    let t = tol(h);
    let mut best: Option<(i64, Plan)> = None;
    for attempt in 0..ATTEMPTS {
        let mut used = vec![false; pieces.len()];
        let mut plan = Vec::new();
        let mut y = 0;
        let mut ok = false;
        loop {
            let rest = depth - y;
            if rest <= t {
                ok = true;
                break;
            }
            // Strip depths on offer: a side of an unused piece that still fits.
            let mut depths: Vec<i32> = pieces
                .iter()
                .zip(&used)
                .filter(|(_, u)| !**u)
                .flat_map(|(p, _)| [p.d, if p.turned.is_some() { p.w } else { 0 }])
                .filter(|&s| s > 0 && s <= rest)
                .collect();
            depths.sort_unstable();
            depths.dedup();
            if depths.is_empty() {
                break;
            }
            // A strip that closes the layer comes first; then deep strips (fewer strips,
            // fewer joints), with some randomness between attempts.
            let mut order: Vec<(f64, i32)> = depths
                .iter()
                .map(|&s| {
                    let closes = rest - s <= t;
                    let noise = if attempt == 0 { 0.0 } else { rng.next_f64() * 0.5 };
                    (if closes { 10.0 } else { 0.0 } + s as f64 / depth as f64 + noise, s)
                })
                .collect();
            order.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
            let mut placed_strip = false;
            for &(_, s) in order.iter().take(DEPTHS) {
                // Pieces for this strip: depth within [s - t, s], length along x.
                let mut cand: Vec<(usize, i32, usize)> = Vec::new(); // (piece, length, orient)
                for (k, p) in pieces.iter().enumerate() {
                    if used[k] {
                        continue;
                    }
                    if (s - t..=s).contains(&p.d) {
                        cand.push((k, p.w, p.orient));
                    } else if let Some(o) = p.turned.filter(|_| (s - t..=s).contains(&p.w)) {
                        cand.push((k, p.d, o));
                    }
                }
                if cand.is_empty() {
                    continue;
                }
                if attempt > 0 {
                    // Keep the preference order (strong first) only roughly.
                    for i in (1..cand.len()).rev() {
                        if rng.next_f64() < 0.3 {
                            let j = rng.below(i + 1);
                            cand.swap(i, j);
                        }
                    }
                }
                let lens: Vec<i32> = cand.iter().map(|c| c.1).collect();
                if let Some(pick) = subset_fill(&lens, width, t) {
                    let mut x = 0;
                    for k in pick {
                        let (pk, l, o) = cand[k];
                        used[pk] = true;
                        plan.push((pieces[pk].idx, o, x, y));
                        x += l;
                    }
                    y += s;
                    placed_strip = true;
                    break;
                }
            }
            if !placed_strip {
                break;
            }
        }
        if ok {
            let vol: i64 = plan.len() as i64;
            if best.as_ref().is_none_or(|b| vol > b.0) {
                best = Some((vol, plan));
            }
            break;
        }
    }
    best.map(|b| b.1)
}

/// Build as many full puzzle layers as possible from the bottom of `bin`. Items used are
/// removed from `types`. Stops at the first height where no full layer passes the checks.
pub fn puzzle_bin(bin: &mut BinState, items: &[PrepItem], types: &mut [Vec<usize>], rule: &PackRule, rng: &mut Rng) {
    let place = bin.place;
    let (pw, pd) = (place.width, place.depth);
    let stol = rule.support_tolerance_mm.max(0);
    let mut level = bin.base_z;
    loop {
        let avail = bin.max_h - level;
        if avail <= 0 {
            return;
        }
        let pool: Vec<usize> = types
            .iter()
            .flatten()
            .copied()
            .collect();
        if pool.is_empty() {
            return;
        }
        // Heights with enough boxes to cover the floor at least once.
        let mut heights: Vec<i32> = pool.iter().flat_map(|&i| items[i].orients.iter().map(|o| o.dims[2])).filter(|&h| h <= avail).collect();
        heights.sort_unstable();
        heights.dedup();
        let area = pw as f64 * pd as f64;
        let mut scored: Vec<(f64, i32)> = heights
            .iter()
            .filter_map(|&h| {
                let foot: f64 = pool
                    .iter()
                    .filter_map(|&i| upright(&items[i], h - stol, h))
                    .map(|p| p.w as f64 * p.d as f64)
                    .sum();
                (foot >= area).then_some((foot, h))
            })
            .collect();
        // Strong, heavy boxes belong low: prefer heights whose boxes are strong on average.
        scored.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
        scored.truncate(HEIGHTS);
        let mut done = false;
        for &(_, h) in &scored {
            let mut pieces: Vec<Piece> = pool
                .iter()
                .filter_map(|&i| upright(&items[i], h - stol, h).map(|p| Piece { idx: i, ..p }))
                .collect();
            // Strong and heavy first: the subset sum prefers earlier pieces.
            pieces.sort_by(|a, b| {
                let (ia, ib) = (&items[a.idx], &items[b.idx]);
                ib.fragility.cmp(&ia.fragility).then(ib.weight.partial_cmp(&ia.weight).unwrap())
            });
            for along_x in [true, false] {
                let (w, d) = if along_x { (pw, pd) } else { (pd, pw) };
                let ps: Vec<Piece> = if along_x {
                    pieces.clone()
                } else {
                    // Strips along y: swap the roles of the footprint sides.
                    pieces
                        .iter()
                        .filter_map(|p| p.turned.map(|t| Piece { orient: t, turned: Some(p.orient), w: p.d, d: p.w, ..*p }))
                        .collect()
                };
                let Some(plan) = plan_layer(&ps, h, w, d, rng) else { continue };
                // Try the whole layer on a copy; keep it only if every box passes, checked as a
                // whole once the layer is complete (`BinState::batch`).
                let mut trial = bin.clone();
                trial.batch = true;
                let mut top = level;
                let mut weakest = u8::MAX;
                let mut ok = true;
                for &(idx, o, a, b) in &plan {
                    let (x, y) = if along_x { (a, b) } else { (b, a) };
                    match trial.try_at(items, idx, o, x, y) {
                        Ok(c) if c.z <= level + stol => {
                            top = top.max(c.z + items[idx].orients[o].dims[2]);
                            weakest = weakest.min(items[idx].fragility);
                            trial.place(items, idx, &c);
                        }
                        _ => {
                            ok = false;
                            break;
                        }
                    }
                }
                trial.batch = false;
                if !ok || !trial.settle(items).is_empty() {
                    continue;
                }
                *bin = trial;
                let used: std::collections::HashSet<usize> = plan.iter().map(|p| p.0).collect();
                for t in types.iter_mut() {
                    t.retain(|i| !used.contains(i));
                }
                level = top;
                done = true;
                break;
            }
            if done {
                break;
            }
        }
        if !done {
            return;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subset_fill_finds_exact_rows() {
        let lens = [300, 250, 250, 200, 120];
        let pick = subset_fill(&lens, 800, 3).unwrap();
        let sum: i32 = pick.iter().map(|&k| lens[k]).sum();
        assert!((797..=800).contains(&sum), "{}", sum);
        assert!(subset_fill(&[300, 300], 800, 3).is_none());
    }
}
