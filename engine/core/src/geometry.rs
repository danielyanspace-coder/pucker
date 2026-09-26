//! Small exact integer geometry helpers shared by the search engine and the validator.

/// Axis-aligned rectangle `[x0, x1) × [y0, y1)` in millimetres.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rect {
    pub x0: i64,
    pub y0: i64,
    pub x1: i64,
    pub y1: i64,
}

impl Rect {
    pub fn new(x: i32, y: i32, w: i32, d: i32) -> Rect {
        Rect { x0: x as i64, y0: y as i64, x1: (x + w) as i64, y1: (y + d) as i64 }
    }

    pub fn area(&self) -> i64 {
        (self.x1 - self.x0).max(0) * (self.y1 - self.y0).max(0)
    }

    /// Intersection with positive area, if any.
    pub fn intersect(&self, o: &Rect) -> Option<Rect> {
        let r = Rect {
            x0: self.x0.max(o.x0),
            y0: self.y0.max(o.y0),
            x1: self.x1.min(o.x1),
            y1: self.y1.min(o.y1),
        };
        (r.x0 < r.x1 && r.y0 < r.y1).then_some(r)
    }
}

/// Length of the overlap of `[a0, a1)` and `[b0, b1)`.
pub fn overlap(a0: i32, a1: i32, b0: i32, b1: i32) -> i64 {
    (a1.min(b1) - a0.max(b0)).max(0) as i64
}

/// Area of the union of rectangles, each shared region counted once (ТЗ §13).
pub fn union_area(rects: &[Rect]) -> i64 {
    match rects.len() {
        0 => return 0,
        1 => return rects[0].area(),
        _ => {}
    }
    // Contacts under a box are usually disjoint (placed boxes never overlap): plain sum.
    let disjoint = rects.iter().enumerate().all(|(i, a)| rects[i + 1..].iter().all(|b| a.intersect(b).is_none()));
    if disjoint {
        return rects.iter().map(Rect::area).sum();
    }
    let mut xs: Vec<i64> = rects.iter().flat_map(|r| [r.x0, r.x1]).collect();
    xs.sort_unstable();
    xs.dedup();
    let mut spans: Vec<(i64, i64)> = Vec::with_capacity(rects.len());
    let mut total = 0;
    for w in xs.windows(2) {
        let (a, b) = (w[0], w[1]);
        spans.clear();
        spans.extend(rects.iter().filter(|r| r.x0 <= a && r.x1 >= b).map(|r| (r.y0, r.y1)));
        if spans.is_empty() {
            continue;
        }
        spans.sort_unstable();
        let mut covered = 0;
        let (mut s, mut e) = spans[0];
        for &(y0, y1) in &spans[1..] {
            if y0 > e {
                covered += e - s;
                s = y0;
                e = y1;
            } else {
                e = e.max(y1);
            }
        }
        covered += e - s;
        total += covered * (b - a);
    }
    total
}

fn cross(o: (i64, i64), a: (i64, i64), b: (i64, i64)) -> i64 {
    (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0)
}

/// Convex hull (counter-clockwise, no collinear points) of the corners of the rectangles.
pub fn hull_of_rects(rects: &[Rect]) -> Vec<(i64, i64)> {
    let mut pts: Vec<(i64, i64)> =
        rects.iter().flat_map(|r| [(r.x0, r.y0), (r.x1, r.y0), (r.x1, r.y1), (r.x0, r.y1)]).collect();
    pts.sort_unstable();
    pts.dedup();
    if pts.len() < 3 {
        return pts;
    }
    // Andrew's monotone chain: lower hull, then upper hull.
    let mut hull: Vec<(i64, i64)> = Vec::with_capacity(pts.len() + 1);
    for &p in &pts {
        while hull.len() >= 2 && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0 {
            hull.pop();
        }
        hull.push(p);
    }
    let lower = hull.len() + 1;
    for &p in pts.iter().rev().skip(1) {
        while hull.len() >= lower && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0 {
            hull.pop();
        }
        hull.push(p);
    }
    hull.pop();
    hull
}

/// Is the point `(px2 / 2, py2 / 2)` strictly inside the convex polygon?
/// Coordinates are passed doubled so that box centres stay integer.
pub fn strictly_inside_doubled(hull: &[(i64, i64)], px2: i64, py2: i64) -> bool {
    if hull.len() < 3 {
        return false;
    }
    let p = (px2, py2);
    (0..hull.len()).all(|i| {
        let a = (hull[i].0 * 2, hull[i].1 * 2);
        let b = (hull[(i + 1) % hull.len()].0 * 2, hull[(i + 1) % hull.len()].1 * 2);
        cross(a, b, p) > 0
    })
}

/// The six axis permutations of an item (ТЗ §6). Code letters name the original side on X, Y, Z.
pub fn orientations(w: i32, d: i32, h: i32) -> Vec<([i32; 3], &'static str)> {
    let all = [
        ([w, d, h], "WDH"),
        ([w, h, d], "WHD"),
        ([d, w, h], "DWH"),
        ([d, h, w], "DHW"),
        ([h, w, d], "HWD"),
        ([h, d, w], "HDW"),
    ];
    let mut out: Vec<([i32; 3], &'static str)> = Vec::with_capacity(6);
    for (dims, code) in all {
        if !out.iter().any(|(o, _)| *o == dims) {
            out.push((dims, code));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn union_counts_overlap_once() {
        let a = Rect::new(0, 0, 10, 10);
        let b = Rect::new(5, 5, 10, 10);
        assert_eq!(union_area(&[a, b]), 175);
        assert_eq!(union_area(&[a, a]), 100);
    }

    #[test]
    fn hull_and_inside() {
        let hull = hull_of_rects(&[Rect::new(0, 0, 10, 10)]);
        assert_eq!(hull.len(), 4);
        assert!(strictly_inside_doubled(&hull, 10, 10));
        assert!(!strictly_inside_doubled(&hull, 20, 10)); // on the edge
        assert!(!strictly_inside_doubled(&hull, 30, 10));
        // L-shape of two rectangles: hull is a pentagon.
        let hull = hull_of_rects(&[Rect::new(0, 0, 10, 2), Rect::new(0, 0, 2, 10)]);
        assert_eq!(hull.len(), 5);
        assert!(strictly_inside_doubled(&hull, 8, 8));
        assert!(!strictly_inside_doubled(&hull, 16, 16));
    }

    #[test]
    fn orientations_dedup() {
        assert_eq!(orientations(1, 2, 3).len(), 6);
        assert_eq!(orientations(2, 2, 3).len(), 3);
        assert_eq!(orientations(2, 2, 2).len(), 1);
    }
}
