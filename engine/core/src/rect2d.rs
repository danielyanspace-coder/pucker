//! 2D rectangle packing (MaxRects, best short side fit) used to plan one flat layer.

#[derive(Clone, Copy, Debug)]
struct Free {
    x: i32,
    y: i32,
    w: i32,
    h: i32,
}

/// A group of identical rectangles offered to the packer.
#[derive(Clone, Debug)]
pub struct RectGroup {
    pub id: usize,
    pub count: usize,
    pub w: i32,
    pub h: i32,
    /// Groups with a higher tier are placed first; lower tiers fill what is left.
    pub tier: i32,
}

#[derive(Clone, Copy, Debug)]
pub struct Placed2 {
    pub id: usize,
    pub x: i32,
    pub y: i32,
    pub w: i32,
}

/// Pack rectangles into `width × height`. `gap` is added to each rectangle's sides
/// (clearance), `noise` perturbs choices for diversification.
/// `blocked` areas (x, y, w, h) are unavailable.
pub fn pack_maxrects(
    width: i32,
    height: i32,
    blocked: &[(i32, i32, i32, i32)],
    groups: &[RectGroup],
    gap: i32,
    rng: &mut crate::rng::Rng,
    noise: f64,
) -> Vec<Placed2> {
    let mut free = vec![Free { x: 0, y: 0, w: width + gap, h: height + gap }];
    for &(x, y, w, h) in blocked {
        split(&mut free, x, y, w, h);
    }
    let mut left: Vec<usize> = groups.iter().map(|g| g.count).collect();
    let mut out = Vec::new();
    let mut tiers: Vec<i32> = groups.iter().map(|g| g.tier).collect();
    tiers.sort_unstable_by(|a, b| b.cmp(a));
    tiers.dedup();
    for tier in tiers {
        loop {
            // Best (group, rotation, free rect) by short side fit, then long side fit.
            let mut best: Option<(f64, i32, usize, bool, usize)> = None;
            for (gi, g) in groups.iter().enumerate() {
                if left[gi] == 0 || g.tier != tier {
                    continue;
                }
                for rot in [false, true] {
                    if rot && g.w == g.h {
                        continue;
                    }
                    let (w, h) = if rot { (g.h + gap, g.w + gap) } else { (g.w + gap, g.h + gap) };
                    for (fi, f) in free.iter().enumerate() {
                        if w > f.w || h > f.h {
                            continue;
                        }
                        let short = (f.w - w).min(f.h - h) as f64;
                        let long = (f.w - w).max(f.h - h);
                        let short = if noise > 0.0 { short * (1.0 + noise * rng.next_f64()) } else { short };
                        if best.map_or(true, |b| short < b.0 || (short == b.0 && long < b.1)) {
                            best = Some((short, long, gi, rot, fi));
                        }
                    }
                }
            }
            let Some((_, _, gi, rot, fi)) = best else { break };
            let g = &groups[gi];
            let (w, h) = if rot { (g.h, g.w) } else { (g.w, g.h) };
            let f = free[fi];
            let p = Placed2 { id: g.id, x: f.x, y: f.y, w };
            left[gi] -= 1;
            out.push(p);
            split(&mut free, p.x, p.y, w + gap, h + gap);
        }
    }
    out
}

fn split(free: &mut Vec<Free>, x: i32, y: i32, w: i32, h: i32) {
    let mut next: Vec<Free> = Vec::with_capacity(free.len() + 4);
    for f in free.drain(..) {
        if x >= f.x + f.w || x + w <= f.x || y >= f.y + f.h || y + h <= f.y {
            next.push(f);
            continue;
        }
        if x > f.x {
            next.push(Free { x: f.x, y: f.y, w: x - f.x, h: f.h });
        }
        if x + w < f.x + f.w {
            next.push(Free { x: x + w, y: f.y, w: f.x + f.w - (x + w), h: f.h });
        }
        if y > f.y {
            next.push(Free { x: f.x, y: f.y, w: f.w, h: y - f.y });
        }
        if y + h < f.y + f.h {
            next.push(Free { x: f.x, y: y + h, w: f.w, h: f.y + f.h - (y + h) });
        }
    }
    // Drop rectangles contained in others.
    let n = next.len();
    let mut keep = vec![true; n];
    for i in 0..n {
        if !keep[i] {
            continue;
        }
        for j in 0..n {
            if i == j || !keep[j] {
                continue;
            }
            let (a, b) = (next[i], next[j]);
            if a.x >= b.x && a.y >= b.y && a.x + a.w <= b.x + b.w && a.y + a.h <= b.y + b.h {
                keep[i] = false;
                break;
            }
        }
    }
    *free = next.into_iter().zip(keep).filter_map(|(f, k)| k.then_some(f)).collect();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fills_exact_grid() {
        let g = [RectGroup { id: 0, count: 6, w: 400, h: 400, tier: 0 }];
        let mut rng = crate::rng::Rng::new(1);
        let p = pack_maxrects(800, 1200, &[], &g, 0, &mut rng, 0.0);
        assert_eq!(p.len(), 6);
        for (i, a) in p.iter().enumerate() {
            for b in &p[i + 1..] {
                let sep = a.x + a.w <= b.x || b.x + b.w <= a.x || a.y + 400 <= b.y || b.y + 400 <= a.y;
                assert!(sep);
            }
        }
    }

    #[test]
    fn respects_blocked_area() {
        let g = [RectGroup { id: 0, count: 10, w: 400, h: 400, tier: 0 }];
        let mut rng = crate::rng::Rng::new(1);
        let p = pack_maxrects(800, 800, &[(0, 0, 400, 800)], &g, 0, &mut rng, 0.0);
        assert_eq!(p.len(), 2);
        assert!(p.iter().all(|q| q.x == 400));
    }
}
