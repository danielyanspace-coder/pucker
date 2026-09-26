//! Search: constructive placement + parallel multi-start with a time budget (ТЗ §31–32).
//!
//! Each attempt orders the items (several sort keys, then randomised variants) and places them
//! one by one at the best feasible candidate point, opening a new place when needed.
//! The best valid solution is kept at all times, so stopping early never loses the result.

use std::time::Instant;

use rayon::prelude::*;

use crate::bin_state::{BinState, Placed, Profile, Reject, Weights};
use crate::metrics::{bin_metrics, BinMetrics, BoxView};
use crate::model::*;
use crate::precheck::{capacity_warnings, fits_empty, fmt_kg, fmt_m3};
use crate::prep::{prepare, PrepItem};
use crate::rng::Rng;
use crate::validator::validate;

#[derive(Clone, Copy, Debug)]
enum OrderKey {
    FragVolume,
    FragBaseArea,
    FragWeight,
    FragHeight,
    Volume,
    Weight,
}

const ORDER_KEYS: [OrderKey; 6] = [
    OrderKey::FragVolume,
    OrderKey::FragBaseArea,
    OrderKey::FragWeight,
    OrderKey::FragHeight,
    OrderKey::Volume,
    OrderKey::Weight,
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mode {
    /// Take items in a fixed order, put each at its best position.
    Sequence,
    /// Build flat layers of similar-height items, then fill what is left.
    Layers,
    /// Take the lowest free positions, put the best-fitting item there.
    Fill,
}

#[derive(Clone, Copy, Debug)]
struct Start {
    mode: Mode,
    /// Layer mode: how much shorter than the layer an item may be, mm.
    tol: i32,
    /// Layer mode: stop layering below this layer density.
    min_density: f64,
    /// Fill/layer modes: split the load evenly across the expected number of places first.
    balance: bool,
    key: OrderKey,
    wts: Weights,
    /// Seeds examined per step in fill mode.
    k: usize,
    noise: f64,
    seed: u64,
}

#[derive(Clone)]
struct BinSol {
    place_index: usize,
    placed: Vec<Placed>,
    metrics: BinMetrics,
}

#[derive(Clone)]
struct Solution {
    /// Parameters that produced this solution, for local search around it.
    start: Option<Start>,
    bins: Vec<BinSol>,
    unplaced: Vec<(usize, Reject)>,
    placed_volume: i64,
    placed_count: usize,
    capacity_volume: i64,
    stability: f64,
    compactness: f64,
    overhang: i32,
    height: i32,
}

impl Solution {
    /// Lexicographic comparison (ТЗ §3, §10A.6, §21): place everything, fewest places,
    /// smallest places, stability (in 0.05 steps), density, less overhang, lower load.
    fn better_than(&self, o: &Solution) -> bool {
        use std::cmp::Ordering::*;
        let bucket = |s: f64| (s * 20.0).round() as i64;
        let ord = self
            .placed_volume
            .cmp(&o.placed_volume)
            .then(self.placed_count.cmp(&o.placed_count))
            .then(o.bins.len().cmp(&self.bins.len()))
            .then(o.capacity_volume.cmp(&self.capacity_volume))
            .then(bucket(self.stability).cmp(&bucket(o.stability)))
            .then(self.compactness.partial_cmp(&o.compactness).unwrap_or(Equal))
            .then(o.overhang.cmp(&self.overhang))
            .then(o.height.cmp(&self.height));
        ord == Greater
    }
}

fn order_items(items: &[PrepItem], ids: &[usize], s: &Start) -> Vec<usize> {
    let mut rng = Rng::new(s.seed ^ 0xA5A5);
    let keyed: Vec<(i32, u8, f64, usize)> = ids
        .iter()
        .map(|&i| {
            let it = &items[i];
            let noise = 1.0 + s.noise * (rng.next_f64() - 0.5);
            let (frag, k) = match s.key {
                OrderKey::FragVolume => (it.fragility, it.volume as f64),
                OrderKey::FragBaseArea => (it.fragility, it.base_area as f64),
                OrderKey::FragWeight => (it.fragility, it.weight),
                OrderKey::FragHeight => (it.fragility, it.max_dim as f64),
                OrderKey::Volume => (0, it.volume as f64),
                OrderKey::Weight => (0, it.weight),
            };
            (it.priority, frag, k * noise, i)
        })
        .collect();
    let mut keyed = keyed;
    keyed.sort_by(|a, b| {
        b.0.cmp(&a.0)
            .then(b.1.cmp(&a.1))
            .then(b.2.partial_cmp(&a.2).unwrap_or(std::cmp::Ordering::Equal))
            .then(a.3.cmp(&b.3))
    });
    keyed.into_iter().map(|k| k.3).collect()
}

fn construct(items: &[PrepItem], ids: &[usize], pool: &[(usize, &PackingPlace)], rule: &PackRule, s: &Start) -> Solution {
    if s.mode != Mode::Sequence {
        return construct_fill(items, ids, pool, rule, s);
    }
    let order = order_items(items, ids, s);
    let wts = s.wts;
    let max_vol = ids.iter().map(|&i| items[i].volume).max().unwrap_or(1).max(1) as f64;
    let mut rng = Rng::new(s.seed);
    let mut bins: Vec<BinState> = Vec::new();
    let mut used = vec![0u32; pool.len()];
    let mut unplaced = Vec::new();
    for idx in order {
        let mut furthest = Reject::Bounds;
        let mut done = false;
        let share = items[idx].volume as f64 / max_vol;
        // Scanning every point is best on small tasks and too slow on big ones.
        let max_seeds = if ids.len() <= 600 { usize::MAX } else { 64 };
        for b in bins.iter_mut() {
            match b.best_candidate(items, idx, &wts, share, max_seeds, &mut rng) {
                Ok(c) => {
                    b.place(items, idx, &c);
                    done = true;
                    break;
                }
                Err(r) => furthest = furthest.max(r),
            }
        }
        if !done {
            for (k, &(pi, place)) in pool.iter().enumerate() {
                if used[k] >= place.quantity || !fits_empty(&items[idx], place) {
                    continue;
                }
                let mut nb = BinState::new(place, pi, rule);
                match nb.best_candidate(items, idx, &wts, share, max_seeds, &mut rng) {
                    Ok(c) => {
                        nb.place(items, idx, &c);
                        bins.push(nb);
                        used[k] += 1;
                        done = true;
                        break;
                    }
                    Err(r) => furthest = furthest.max(r),
                }
            }
        }
        if !done {
            unplaced.push((idx, furthest));
        }
    }
    let bins: Vec<BinSol> = bins
        .into_iter()
        .map(|b| {
            let metrics = bin_metrics(b.place, &views(items, &b.placed, b.place.base_z()));
            BinSol { place_index: b.place_index, placed: b.placed, metrics }
        })
        .collect();
    summarize(bins, unplaced, pool)
}

/// Identical physical units share one type so each position is tried once per type.
fn group_types(items: &[PrepItem], ids: &[usize]) -> Vec<Vec<usize>> {
    let mut map: std::collections::HashMap<(Vec<[i32; 3]>, u64, u8, u64, i32), usize> = Default::default();
    let mut types: Vec<Vec<usize>> = Vec::new();
    for &i in ids {
        let it = &items[i];
        let key = (
            it.orients.iter().map(|o| o.dims).collect::<Vec<_>>(),
            it.weight.to_bits(),
            it.fragility,
            it.max_top_load.to_bits(),
            it.priority,
        );
        let t = *map.entry(key).or_insert_with(|| {
            types.push(Vec::new());
            types.len() - 1
        });
        types[t].push(i);
    }
    // Pop order inside a type: keep the input order.
    types.iter_mut().for_each(|v| v.reverse());
    types
}

fn construct_fill(items: &[PrepItem], ids: &[usize], pool: &[(usize, &PackingPlace)], rule: &PackRule, s: &Start) -> Solution {
    let mut rng = Rng::new(s.seed);
    let mut types = group_types(items, ids);
    let max_vol = ids.iter().map(|&i| items[i].volume).max().unwrap_or(1).max(1) as f64;
    let mut furthest = vec![Reject::Bounds; types.len()];
    let mut used = vec![0u32; pool.len()];
    let mut bins: Vec<BinState> = Vec::new();

    // Balanced mode: split every type across the expected number of places first, so each
    // place gets its own strong base and light top, then pack the places one by one.
    if s.balance {
        let &(pi, place) = &pool[0];
        let total_vol: f64 = ids.iter().map(|&i| items[i].volume as f64).sum();
        let n = ((total_vol / (place.usable_volume() as f64 * 0.8)).ceil() as usize).min(place.quantity as usize);
        if n >= 2 {
            let mut parts: Vec<Vec<Vec<usize>>> = vec![vec![Vec::new(); types.len()]; n];
            let mut load = vec![0.0f64; n];
            let mut order: Vec<usize> = (0..types.len()).collect();
            order.sort_by(|&a, &b| {
                let (ia, ib) = (&items[types[a][0]], &items[types[b][0]]);
                ib.fragility.cmp(&ia.fragility).then(ib.volume.cmp(&ia.volume))
            });
            for t in order {
                for i in std::mem::take(&mut types[t]) {
                    let b = (0..n).min_by(|&x, &y| load[x].partial_cmp(&load[y]).unwrap()).unwrap();
                    load[b] += items[i].volume as f64;
                    parts[b][t].push(i);
                }
            }
            for part in parts.iter_mut() {
                let mut bin = BinState::new(place, pi, rule);
                if s.mode == Mode::Layers {
                    layer_bin(&mut bin, items, part, rule, s, &mut rng);
                }
                fill_bin(&mut bin, items, part, &mut furthest, max_vol, s, &mut rng);
                for (t, rest) in part.iter_mut().enumerate() {
                    types[t].append(rest);
                }
                if !bin.placed.is_empty() {
                    bins.push(bin);
                    used[0] += 1;
                }
            }
            // Leftovers first try the places already started.
            for bin in bins.iter_mut() {
                fill_bin(bin, items, &mut types, &mut furthest, max_vol, s, &mut rng);
            }
        }
    }

    loop {
        if types.iter().all(|t| t.is_empty()) {
            break;
        }
        let mut progressed = false;
        for (k, &(pi, place)) in pool.iter().enumerate() {
            if used[k] >= place.quantity
                || !types.iter().any(|t| t.last().is_some_and(|&i| fits_empty(&items[i], place)))
            {
                continue;
            }
            let mut bin = BinState::new(place, pi, rule);
            if s.mode == Mode::Layers {
                layer_bin(&mut bin, items, &mut types, rule, s, &mut rng);
            }
            fill_bin(&mut bin, items, &mut types, &mut furthest, max_vol, s, &mut rng);
            if bin.placed.is_empty() {
                continue;
            }
            used[k] += 1;
            bins.push(bin);
            progressed = true;
            break;
        }
        if !progressed {
            break;
        }
    }
    let unplaced = types
        .iter()
        .enumerate()
        .flat_map(|(t, v)| v.iter().map(move |&i| (i, t)))
        .map(|(i, t)| (i, furthest[t]))
        .collect();
    let done: Vec<BinSol> = bins
        .into_iter()
        .map(|b| {
            let metrics = bin_metrics(b.place, &views(items, &b.placed, b.base_z));
            BinSol { place_index: b.place_index, placed: b.placed, metrics }
        })
        .collect();
    summarize(done, unplaced, pool)
}

/// Build flat layers bottom-up: pick the layer height whose 2D plan is densest,
/// then place the plan through the exact checks (items that fail stay for later).
fn layer_bin(bin: &mut BinState, items: &[PrepItem], types: &mut [Vec<usize>], rule: &PackRule, s: &Start, rng: &mut Rng) {
    let place = bin.place;
    let (pw, pd) = (place.width, place.depth);
    let area = pw as f64 * pd as f64;
    let mut level = bin.base_z;
    // Nothing in a layer may be stronger than the weakest item of the layer below it.
    let mut cap = u8::MAX;
    // Strong classes build the lower layers first (like tiers in `fill_bin`).
    let mut tiers: Vec<u8> = if rule.use_fragility {
        types.iter().filter_map(|t| t.last().map(|&i| items[i].fragility)).collect()
    } else {
        vec![0]
    };
    tiers.sort_unstable_by(|a, b| b.cmp(a));
    tiers.dedup();
    for tier in tiers {
    loop {
        let avail = bin.max_h - level;
        if avail <= 0 {
            return;
        }
        let live: Vec<usize> = (0..types.len())
            .filter(|&t| types[t].last().is_some_and(|&i| !rule.use_fragility || items[i].fragility <= cap))
            .collect();
        if live.is_empty() {
            return;
        }
        let floors = [tier];

        let blocked = bin.unsupported_at(level);
        let blocked_area: f64 = blocked.iter().map(|b| b.2 as f64 * b.3 as f64).sum();
        let usable = (area - blocked_area).max(1.0);
        let mut best: Option<(f64, i32, u8, Vec<(usize, usize, i32, i32)>)> = None;
        for &floor in &floors {
            let pool: Vec<usize> = live
                .iter()
                .copied()
                .filter(|&t| !rule.use_fragility || items[*types[t].last().unwrap()].fragility >= floor)
                .collect();
            let mut heights: Vec<i32> = pool
                .iter()
                .flat_map(|&t| items[*types[t].last().unwrap()].orients.iter().map(|o| o.dims[2]))
                .filter(|&h| h <= avail)
                .collect();
            heights.sort_unstable();
            heights.dedup();
            let mut screened: Vec<(f64, i32)> = heights
                .iter()
                .map(|&v| {
                    let (mut foot, mut vol) = (0.0, 0.0);
                    for &t in &pool {
                        let i = *types[t].last().unwrap();
                        if let Some(o) = best_vertical(&items[i], v, s.tol) {
                            let [w, d, h] = items[i].orients[o].dims;
                            foot += w as f64 * d as f64 * types[t].len() as f64;
                            vol += w as f64 * d as f64 * h as f64 * types[t].len() as f64;
                        }
                    }
                    let cover = (foot / usable).min(1.0) * usable / area;
                    let fill = if foot > 0.0 { vol / (foot * v as f64) } else { 0.0 };
                    (cover * fill, v)
                })
                .filter(|&(sc, _)| sc >= s.min_density)
                .collect();
            screened.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap().then(b.1.cmp(&a.1)));
            screened.truncate(3);
            for &(_, v) in &screened {
                let mut groups = Vec::new();
                let mut orient_of = Vec::new();
                for &t in &pool {
                    let i = *types[t].last().unwrap();
                    if let Some(o) = best_vertical(&items[i], v, s.tol) {
                        let [w, d, _] = items[i].orients[o].dims;
                        let tier = if rule.use_fragility { items[i].fragility as i32 } else { 0 };
                        groups.push(crate::rect2d::RectGroup { id: t, count: types[t].len(), w, h: d, tier });
                        orient_of.push((t, o));
                    }
                }
                let plan = crate::rect2d::pack_maxrects(pw, pd, &blocked, &groups, rule.clearance_mm, rng, s.wts.jitter / 100.0);
                let mut vol = 0.0;
                let mut out = Vec::with_capacity(plan.len());
                for p in plan {
                    let o = orient_of.iter().find(|(t, _)| *t == p.id).unwrap().1;
                    let i = *types[p.id].last().unwrap();
                    // The packer may have turned the footprint by 90°.
                    let o = if items[i].orients[o].dims[0] == p.w {
                        o
                    } else {
                        let [w, d, h] = items[i].orients[o].dims;
                        items[i].orients.iter().position(|q| q.dims == [d, w, h]).unwrap_or(o)
                    };
                    let [w, d, h] = items[i].orients[o].dims;
                    vol += w as f64 * d as f64 * h as f64;
                    out.push((p.id, o, p.x, p.y));
                }
                // Prefer dense layers; among similar ones, stronger items lower.
                let score = vol / (area * v as f64) + 0.004 * floor as f64;
                if best.as_ref().map_or(true, |b| score > b.0) {
                    best = Some((score, v, floor, out));
                }
            }
        }
        // No dense layer for this class: let weaker classes join.
        let Some((_, _, _, plan)) = best else { break };
        let planned = plan.len();
        let mut placed = 0;
        let mut top = level;
        let mut weakest = u8::MAX;
        for (t, o, x, y) in plan {
            let Some(&idx) = types[t].last() else { continue };
            if let Ok(c) = bin.try_at(items, idx, o, x, y) {
                types[t].pop();
                bin.place(items, idx, &c);
                placed += 1;
                top = top.max(c.z + items[idx].orients[o].dims[2]);
                weakest = weakest.min(items[idx].fragility);
            }
        }
        // A layer that mostly failed leaves an uneven surface: hand over to the gap filler.
        if placed == 0 || (placed as f64) < 0.8 * planned as f64 {
            return;
        }
        cap = cap.min(weakest);
        level = top.max(level + 1);
    }
    }
}

/// Orientation whose height is the largest one within `[v - tol, v]`.
fn best_vertical(it: &PrepItem, v: i32, tol: i32) -> Option<usize> {
    it.orients
        .iter()
        .enumerate()
        .filter(|(_, o)| o.dims[2] <= v && o.dims[2] >= v - tol)
        .max_by_key(|(_, o)| (o.dims[2], o.dims[0] as i64 * o.dims[1] as i64))
        .map(|(i, _)| i)
}

fn fill_bin(
    bin: &mut BinState,
    items: &[PrepItem],
    types: &mut [Vec<usize>],
    furthest: &mut [Reject],
    max_vol: f64,
    s: &Start,
    rng: &mut Rng,
) {
    // Tiers: higher priority first, then stronger items first (they must end up below).
    let mut tiers: Vec<(i32, u8)> = types
        .iter()
        .filter_map(|t| t.last().map(|&i| (items[i].priority, items[i].fragility)))
        .collect();
    tiers.sort_unstable_by(|a, b| b.cmp(a));
    tiers.dedup();
    let mut failed = vec![false; types.len()];
    let mut allowed: Vec<usize> = Vec::new();
    for tier in tiers {
        bin.revive_seeds();
        loop {
            allowed.clear();
            for (t, v) in types.iter().enumerate() {
                let Some(&i) = v.last() else { continue };
                if failed[t] || (items[i].priority, items[i].fragility) < tier {
                    continue;
                }
                if !bin.weight_fits(items[i].weight) {
                    failed[t] = true;
                    furthest[t] = furthest[t].max(Reject::Weight);
                    continue;
                }
                allowed.push(t);
            }
            if allowed.is_empty() {
                break;
            }
            let seeds = bin.lowest_seeds(s.wts.profile, s.k);
            if seeds.is_empty() {
                allowed.iter().for_each(|&t| failed[t] = true);
                break;
            }
            let mut best: Option<(crate::bin_state::Candidate, usize)> = None;
            for si in seeds {
                let seed = bin.seeds[si];
                let mut any = false;
                for &t in &allowed {
                    let idx = *types[t].last().unwrap();
                    let share = items[idx].volume as f64 / max_vol;
                    let bound = best.as_ref().map(|b| b.0.score);
                    match bin.try_seed(items, idx, &seed, &s.wts, share, bound, rng) {
                        Ok(Some(c)) => {
                            best = Some((c, t));
                            any = true;
                        }
                        Ok(None) => any = true,
                        Err(r) => furthest[t] = furthest[t].max(r),
                    }
                }
                if !any {
                    bin.seeds[si].dead = true;
                }
            }
            if let Some((c, t)) = best {
                let idx = types[t].pop().unwrap();
                bin.place(items, idx, &c);
            }
        }
    }
}

fn views(items: &[PrepItem], placed: &[Placed], base: i32) -> Vec<BoxView> {
    placed
        .iter()
        .map(|p| BoxView {
            x: p.x,
            y: p.y,
            z: p.z,
            w: p.w,
            d: p.d,
            h: p.h,
            weight: items[p.item].weight,
            support_ratio: (p.z != base).then(|| p.support_area as f64 / (p.w as i64 * p.d as i64) as f64),
        })
        .collect()
}

fn summarize(bins: Vec<BinSol>, unplaced: Vec<(usize, Reject)>, pool: &[(usize, &PackingPlace)]) -> Solution {
    let placed_volume = bins.iter().map(|b| b.metrics.item_volume).sum();
    let placed_count = bins.iter().map(|b| b.placed.len()).sum();
    let capacity_volume = bins
        .iter()
        .map(|b| pool.iter().find(|(pi, _)| *pi == b.place_index).map_or(0, |(_, p)| p.usable_volume()))
        .sum();
    let total_w: f64 = bins.iter().map(|b| b.metrics.item_volume as f64).sum::<f64>().max(1.0);
    let stability = bins.iter().map(|b| b.metrics.stability_score * b.metrics.item_volume as f64).sum::<f64>() / total_w;
    let bbox: i64 = bins.iter().map(|b| b.metrics.bbox_volume).sum();
    Solution {
        start: None,
        compactness: placed_volume as f64 / bbox.max(1) as f64,
        overhang: bins.iter().map(|b| b.metrics.max_overhang_mm).max().unwrap_or(0),
        height: bins.iter().map(|b| b.metrics.used_height).max().unwrap_or(0),
        bins,
        unplaced,
        placed_volume,
        placed_count,
        capacity_volume,
        stability,
    }
}

fn deterministic_starts(seed: u64) -> Vec<Start> {
    let mut v = Vec::new();
    let mut n = 0u64;
    let mut next = || {
        n += 1;
        seed ^ n.wrapping_mul(0x9E37_79B9)
    };
    for (tol, min_density) in [(0, 0.75), (10, 0.75), (20, 0.7), (40, 0.7), (10, 0.6)] {
        let wts = Weights { profile: Profile::Layer, contact: 400.0, volume: 300.0, flush: 150.0, void: 4.0, jitter: 0.0 };
        v.push(Start { mode: Mode::Layers, tol, min_density, balance: false, key: OrderKey::FragVolume, wts, k: 6, noise: 0.0, seed: next() });
    }
    for tol in [0, 10, 20] {
        let wts = Weights { profile: Profile::Layer, contact: 400.0, volume: 300.0, flush: 150.0, void: 4.0, jitter: 0.0 };
        v.push(Start { mode: Mode::Layers, tol, min_density: 0.7, balance: true, key: OrderKey::FragVolume, wts, k: 6, noise: 0.0, seed: next() });
        v.push(Start { mode: Mode::Fill, tol, min_density: 1.0, balance: true, key: OrderKey::FragVolume, wts, k: 6 + tol as usize / 10, noise: 0.0, seed: next() });
    }
    for profile in [Profile::Layer, Profile::Wall] {
        for (contact, volume, flush) in [(400.0, 300.0, 150.0), (800.0, 150.0, 300.0), (250.0, 600.0, 100.0)] {
            let wts = Weights { profile, contact, volume, flush, void: 4.0, jitter: 0.0 };
            v.push(Start { mode: Mode::Fill, tol: 0, min_density: 1.0, balance: false, key: OrderKey::FragVolume, wts, k: 6, noise: 0.0, seed: next() });
        }
    }
    for &key in &ORDER_KEYS {
        for profile in [Profile::Layer, Profile::Wall] {
            let wts = Weights { profile, contact: 400.0, volume: 0.0, flush: 150.0, void: 4.0, jitter: 0.0 };
            v.push(Start { mode: Mode::Sequence, tol: 0, min_density: 1.0, balance: false, key, wts, k: 0, noise: 0.0, seed: next() });
        }
    }
    v
}

fn random_start(rng: &mut Rng) -> Start {
    let profile = if rng.next_f64() < 0.5 { Profile::Layer } else { Profile::Wall };
    let wts = Weights {
        profile,
        contact: 100.0 + rng.next_f64() * 900.0,
        volume: rng.next_f64() * 800.0,
        flush: rng.next_f64() * 400.0,
        void: rng.next_f64() * 10.0,
        jitter: if rng.next_f64() < 0.5 { rng.next_f64() * 60.0 } else { 0.0 },
    };
    let r = rng.next_f64();
    Start {
        mode: if r < 0.5 { Mode::Layers } else if r < 0.85 { Mode::Fill } else { Mode::Sequence },
        tol: [0, 5, 10, 20, 30, 50][rng.below(6)],
        min_density: 0.55 + rng.next_f64() * 0.3,
        balance: rng.next_f64() < 0.2,
        key: ORDER_KEYS[rng.below(ORDER_KEYS.len())],
        wts,
        k: 2 + rng.below(10),
        noise: rng.next_f64() * 0.4,
        seed: rng.next_u64(),
    }
}

fn mutate(b: &Start, rng: &mut Rng) -> Start {
    let mut f = |x: f64| x * (0.75 + rng.next_f64() * 0.5);
    let wts = Weights {
        profile: b.wts.profile,
        contact: f(b.wts.contact),
        volume: f(b.wts.volume),
        flush: f(b.wts.flush),
        void: f(b.wts.void),
        jitter: f(b.wts.jitter.max(5.0)),
    };
    Start {
        mode: b.mode,
        tol: (b.tol + [-10, -5, 0, 5, 10][rng.below(5)]).max(0),
        min_density: (b.min_density + (rng.next_f64() - 0.5) * 0.1).clamp(0.4, 0.95),
        balance: b.balance,
        key: if rng.next_f64() < 0.2 { ORDER_KEYS[rng.below(ORDER_KEYS.len())] } else { b.key },
        wts,
        k: (b.k as i64 + rng.below(5) as i64 - 2).clamp(2, 16) as usize,
        noise: (b.noise + (rng.next_f64() - 0.5) * 0.1).clamp(0.0, 0.5),
        seed: rng.next_u64(),
    }
}

struct SearchOutcome {
    best: Solution,
    iterations: u64,
    best_at: u64,
    first_at: Instant,
}

fn search(
    items: &[PrepItem],
    ids: &[usize],
    pool: &[(usize, &PackingPlace)],
    rule: &PackRule,
    seed: u64,
    deadline: Instant,
) -> SearchOutcome {
    let mut rng = Rng::new(seed);
    let first = deterministic_starts(seed);
    let threads = rayon::current_num_threads().max(1);
    let mut best: Option<Solution> = None;
    let mut iterations = 0u64;
    let mut best_at = 0u64;
    let mut no_improve = 0u32;
    let mut queue = first;
    let mut first_at = None;
    loop {
        let batch: Vec<Start> = if queue.is_empty() {
            // Half exploration, half small changes around the best solution so far.
            let around = best.as_ref().and_then(|b| b.start);
            (0..threads.max(2))
                .map(|i| match around {
                    Some(b) if i % 2 == 0 => mutate(&b, &mut rng),
                    _ => random_start(&mut rng),
                })
                .collect()
        } else {
            queue.drain(..threads.min(queue.len())).collect()
        };
        let sols: Vec<Solution> = batch
            .par_iter()
            .map(|s| {
                let t = Instant::now();
                let mut sol = construct(items, ids, pool, rule, s);
                sol.start = Some(*s);
                // Per-attempt profiling: PUCKER_TRACE=1 pucker pack …
                if std::env::var("PUCKER_TRACE").is_ok() {
                    eprintln!("{:?} tol={} k={} balance={} {:.2}s placed={} bins={} compact={:.3}", s.mode, s.tol, s.k, s.balance, t.elapsed().as_secs_f64(), sol.placed_count, sol.bins.len(), sol.compactness);
                }
                sol
            })
            .collect();
        for s in sols {
            iterations += 1;
            if best.as_ref().map_or(true, |b| s.better_than(b)) {
                best = Some(s);
                best_at = iterations;
                no_improve = 0;
            } else {
                no_improve += 1;
            }
        }
        first_at.get_or_insert_with(Instant::now);
        let b = best.as_ref().unwrap();
        let perfect = b.unplaced.is_empty() && b.bins.len() <= 1 && b.compactness >= 0.999;
        if perfect || Instant::now() >= deadline || no_improve >= rule.max_no_improve_starts || ids.is_empty() {
            break;
        }
    }
    SearchOutcome { best: best.unwrap(), iterations, best_at, first_at: first_at.unwrap() }
}

/// Run the packing engine end to end: prechecks, search, metrics, validation.
pub fn pack(req: &PackingRequest) -> Result<PackingResult, String> {
    let t0 = Instant::now();
    let rule = &req.pack_rule;
    if req.available_packing_places.is_empty() {
        return Err("Не задано ни одного места погрузки".into());
    }
    for p in &req.available_packing_places {
        if p.width <= 0 || p.depth <= 0 || p.height <= p.base_z() {
            return Err(format!("Место «{}»: некорректные размеры", p.id));
        }
    }
    let items = prepare(&req.items, rule)?;
    let seed = rule.random_seed.unwrap_or(42);
    let places = &req.available_packing_places;
    let mut warnings = Vec::new();

    // Items that no place can take at all are reported right away (ТЗ §36).
    let mut pre_unplaced: Vec<(usize, UnplacedReason)> = Vec::new();
    let mut ids = Vec::new();
    for (i, it) in items.iter().enumerate() {
        if !places.iter().any(|p| fits_empty(it, p)) {
            pre_unplaced.push((i, UnplacedReason::ItemTooLarge));
        } else if !places.iter().any(|p| it.weight <= p.payload_limit() + 1e-9) {
            pre_unplaced.push((i, UnplacedReason::ItemTooHeavy));
        } else {
            ids.push(i);
        }
    }

    // Candidate pools: the whole list when fixed, each place type on its own when auto.
    let pools: Vec<Vec<(usize, &PackingPlace)>> = match req.packing_place_mode {
        PackingPlaceMode::FixedPackingPlace => vec![places.iter().enumerate().collect()],
        PackingPlaceMode::AutoSelectPackingPlace => places.iter().enumerate().map(|p| vec![p]).collect(),
    };
    if req.packing_place_mode == PackingPlaceMode::FixedPackingPlace {
        let pool: Vec<&PackingPlace> = places.iter().collect();
        let sub: Vec<PrepItem> = ids.iter().map(|&i| items[i].clone()).collect();
        warnings.extend(capacity_warnings(&sub, &pool));
    }

    let total_budget = rule.time_limit_seconds.max(0.1);
    let mut best: Option<Solution> = None;
    let mut iterations = 0;
    let mut best_at = 0;
    let mut first_solution = None;
    for (k, pool) in pools.iter().enumerate() {
        let left = total_budget - t0.elapsed().as_secs_f64();
        let share = left / (pools.len() - k) as f64;
        let deadline = Instant::now() + std::time::Duration::from_secs_f64(share.max(0.05));
        let out = search(&items, &ids, pool, rule, seed.wrapping_add(k as u64), deadline);
        iterations += out.iterations;
        first_solution.get_or_insert((out.first_at - t0).as_secs_f64());
        if best.as_ref().map_or(true, |b| out.best.better_than(b)) {
            best_at = iterations - out.iterations + out.best_at;
            best = Some(out.best);
        }
    }
    let best = best.unwrap();

    let mut result = build_result(req, &items, best, pre_unplaced, warnings, t0, iterations, best_at, seed);
    result.diagnostics.first_solution_time = first_solution.unwrap_or(0.0);
    Ok(result)
}

#[allow(clippy::too_many_arguments)]
fn build_result(
    req: &PackingRequest,
    items: &[PrepItem],
    sol: Solution,
    pre_unplaced: Vec<(usize, UnplacedReason)>,
    mut warnings: Vec<Warning>,
    t0: Instant,
    iterations: u64,
    best_at: u64,
    seed: u64,
) -> PackingResult {
    let src = &req.items;
    let mut counters: std::collections::HashMap<usize, u32> = Default::default();
    let mut bins_out = Vec::new();
    for b in &sol.bins {
        let place = &req.available_packing_places[b.place_index];
        let n = counters.entry(b.place_index).or_insert(0);
        *n += 1;
        let bin_id = format!("{}-{}", place.id, n);
        let mut supported_items: Vec<Vec<String>> = vec![Vec::new(); b.placed.len()];
        for p in &b.placed {
            for &(j, _) in &p.supports {
                supported_items[j].push(src[items[p.item].src].id.clone());
            }
        }
        let placed_items = b
            .placed
            .iter()
            .enumerate()
            .map(|(k, p)| {
                let it = &items[p.item];
                let s = &src[it.src];
                PlacedItem {
                    item_id: s.id.clone(),
                    sku: s.sku.clone(),
                    bin_id: bin_id.clone(),
                    x: p.x,
                    y: p.y,
                    z: p.z,
                    width: p.w,
                    depth: p.d,
                    height: p.h,
                    rotation: it.orients[p.orient].code.to_string(),
                    weight: it.weight,
                    fragility: it.fragility,
                    max_top_load: it.max_top_load,
                    support_ratio: p.support_area as f64 / (p.w as i64 * p.d as i64) as f64,
                    received_top_load: p.received,
                    supported_by: p.supports.iter().map(|&(j, _)| src[items[b.placed[j].item].src].id.clone()).collect(),
                    supported_items: std::mem::take(&mut supported_items[k]),
                }
            })
            .collect();
        let m = &b.metrics;
        bins_out.push(PackedBin {
            bin_id,
            place_id: place.id.clone(),
            place_name: place.name.clone(),
            place_type: place.place_type,
            width: place.width,
            depth: place.depth,
            height: place.height,
            base_z: place.base_z(),
            total_weight: m.item_weight + place.pallet_weight(),
            item_weight: m.item_weight,
            pallet_weight: place.pallet_weight(),
            used_height: m.used_height,
            utilization: m.utilization,
            compactness: m.compactness,
            center_of_mass: m.center_of_mass,
            cg_offset_mm: m.cg_offset_mm,
            stability_score: m.stability_score,
            max_overhang_mm: m.max_overhang_mm,
            placed_items,
        });
    }

    let reason_of = |r: Reject| match r {
        Reject::Bounds | Reject::Height => UnplacedReason::InsufficientSpace,
        Reject::Weight => UnplacedReason::BinWeightLimit,
        Reject::Support | Reject::Lateral => UnplacedReason::NoStablePosition,
        Reject::Fragility => UnplacedReason::FragilityConflict,
        Reject::TopLoad => UnplacedReason::MaxTopLoadConflict,
    };
    let mut unplaced: Vec<UnplacedItem> = pre_unplaced
        .into_iter()
        .chain(sol.unplaced.iter().map(|&(i, r)| (i, reason_of(r))))
        .map(|(i, reason)| {
            let s = &src[items[i].src];
            UnplacedItem { item_id: s.id.clone(), sku: s.sku.clone(), reason, message: reason.message_ru().to_string() }
        })
        .collect();
    unplaced.sort_by(|a, b| a.sku.cmp(&b.sku).then(a.item_id.cmp(&b.item_id)));

    let unplaced_idx: Vec<usize> = unplaced
        .iter()
        .filter_map(|u| items.iter().position(|it| src[it.src].id == u.item_id))
        .collect();
    let un_vol: i64 = unplaced_idx.iter().map(|&i| items[i].volume).sum();
    let un_wt: f64 = unplaced_idx.iter().map(|&i| items[i].weight).sum();
    let mut skus: Vec<&str> = unplaced.iter().map(|u| u.sku.as_str()).collect();
    skus.sort_unstable();
    skus.dedup();
    let placed_count: usize = bins_out.iter().map(|b| b.placed_items.len()).sum();
    if !unplaced.is_empty() {
        warnings.push(Warning {
            code: "INSUFFICIENT_SPACE".into(),
            severity: Severity::Error,
            message: format!(
                "Размер пространства не позволяет разместить весь груз. Размещено {} из {} коробок; \
                 не поместились {} коробок ({} артикулов), {} м³, {} кг. \
                 Добавьте ещё место погрузки, выберите место больше или включите автоподбор.",
                placed_count,
                src.len(),
                unplaced.len(),
                skus.len(),
                fmt_m3(un_vol),
                fmt_kg(un_wt)
            ),
        });
    }
    if req.available_packing_places.iter().any(|p| p.use_pallet_base && p.place_type != PackingPlaceType::Pallet) {
        warnings.push(Warning {
            code: "PALLET_BASE_IGNORED".into(),
            severity: Severity::Info,
            message: "Укладка «палеты внутри машины» будет в следующей версии; груз размещён прямо в кузове.".into(),
        });
    }

    let item_vol: i64 = bins_out.iter().flat_map(|b| &b.placed_items).map(|p| p.width as i64 * p.depth as i64 * p.height as i64).sum();
    let cap: i64 = sol.capacity_volume.max(1);
    let summary = Summary {
        items_total: src.len(),
        items_placed: placed_count,
        items_unplaced: unplaced.len(),
        skus_unplaced: skus.len(),
        unplaced_volume_m3: un_vol as f64 / 1e9,
        unplaced_weight_kg: un_wt,
        bins_used: bins_out.len(),
        utilization: item_vol as f64 / cap as f64,
        compactness: sol.compactness,
    };

    let mut result = PackingResult {
        bins: bins_out,
        unplaced,
        warnings,
        summary,
        validation: ValidationReport::default(),
        diagnostics: Diagnostics {
            calculation_time: 0.0,
            first_solution_time: 0.0,
            iterations,
            solutions_checked: iterations,
            valid_solution_found: false,
            random_seed: seed,
            best_found_at_iteration: best_at,
        },
    };
    result.validation = validate(req, &result);
    result.diagnostics.valid_solution_found = result.validation.valid;
    result.diagnostics.calculation_time = t0.elapsed().as_secs_f64();
    result
}
