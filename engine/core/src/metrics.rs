//! Density and stability metrics (ТЗ §16, §22, §26).

use crate::model::PackingPlace;

pub struct BoxView {
    pub x: i32,
    pub y: i32,
    pub z: i32,
    pub w: i32,
    pub d: i32,
    pub h: i32,
    pub weight: f64,
    /// `None` for boxes standing on the base.
    pub support_ratio: Option<f64>,
}

#[derive(Clone, Debug, Default)]
pub struct BinMetrics {
    pub item_volume: i64,
    pub item_weight: f64,
    pub used_height: i32,
    pub utilization: f64,
    pub compactness: f64,
    pub bbox_volume: i64,
    pub center_of_mass: [f64; 3],
    pub cg_offset_mm: f64,
    pub stability_score: f64,
    pub max_overhang_mm: i32,
}

pub fn bin_metrics(place: &PackingPlace, boxes: &[BoxView]) -> BinMetrics {
    let base = place.base_z();
    let mut m = BinMetrics { used_height: base, ..Default::default() };
    if boxes.is_empty() {
        return m;
    }
    let (mut x0, mut y0, mut x1, mut y1, mut z1) = (i32::MAX, i32::MAX, i32::MIN, i32::MIN, base);
    let pw = place.pallet_weight();
    let mut mass = pw;
    let mut mom = [pw * place.width as f64 / 2.0, pw * place.depth as f64 / 2.0, pw * base as f64 / 2.0];
    let mut supp_sum = 0.0;
    let mut supp_n = 0;
    for b in boxes {
        m.item_volume += b.w as i64 * b.d as i64 * b.h as i64;
        m.item_weight += b.weight;
        x0 = x0.min(b.x);
        y0 = y0.min(b.y);
        x1 = x1.max(b.x + b.w);
        y1 = y1.max(b.y + b.d);
        z1 = z1.max(b.z + b.h);
        mass += b.weight;
        mom[0] += b.weight * (b.x as f64 + b.w as f64 / 2.0);
        mom[1] += b.weight * (b.y as f64 + b.d as f64 / 2.0);
        mom[2] += b.weight * (b.z as f64 + b.h as f64 / 2.0);
        if let Some(s) = b.support_ratio {
            supp_sum += s;
            supp_n += 1;
        }
        let over = (-b.x).max(-b.y).max(b.x + b.w - place.width).max(b.y + b.d - place.depth).max(0);
        m.max_overhang_mm = m.max_overhang_mm.max(over);
    }
    m.used_height = z1;
    m.bbox_volume = (x1 - x0) as i64 * (y1 - y0) as i64 * (z1 - base) as i64;
    m.utilization = m.item_volume as f64 / place.usable_volume().max(1) as f64;
    m.compactness = m.item_volume as f64 / m.bbox_volume.max(1) as f64;
    if mass > 0.0 {
        m.center_of_mass = [mom[0] / mass, mom[1] / mass, mom[2] / mass];
    }
    let (cx, cy) = (place.width as f64 / 2.0, place.depth as f64 / 2.0);
    m.cg_offset_mm = ((m.center_of_mass[0] - cx).powi(2) + (m.center_of_mass[1] - cy).powi(2)).sqrt();

    // Stability score in [0, 1]: centred CG, low CG, good support, flat top.
    let half_diag = (cx * cx + cy * cy).sqrt().max(1.0);
    let s_center = 1.0 - (m.cg_offset_mm / half_diag).min(1.0);
    let load_h = (z1 - base).max(1) as f64;
    let s_low = 1.0 - ((m.center_of_mass[2] - base as f64) / load_h).clamp(0.0, 1.0);
    let s_support = if supp_n > 0 { supp_sum / supp_n as f64 } else { 1.0 };
    let s_flat = 1.0 - top_irregularity(boxes, x0, y0, x1, y1, base, z1);
    m.stability_score = 0.35 * s_center + 0.2 * s_low + 0.25 * s_support + 0.2 * s_flat;
    m
}

/// Mean gap between the top surface and the highest point, sampled on a grid, in `[0, 1]`.
fn top_irregularity(boxes: &[BoxView], x0: i32, y0: i32, x1: i32, y1: i32, base: i32, top: i32) -> f64 {
    const N: i32 = 24;
    let span = (top - base).max(1) as f64;
    let mut sum = 0.0;
    for i in 0..N {
        for j in 0..N {
            let px = x0 + ((x1 - x0) as i64 * (2 * i as i64 + 1) / (2 * N as i64)) as i32;
            let py = y0 + ((y1 - y0) as i64 * (2 * j as i64 + 1) / (2 * N as i64)) as i32;
            let h = boxes
                .iter()
                .filter(|b| b.x <= px && px < b.x + b.w && b.y <= py && py < b.y + b.d)
                .map(|b| b.z + b.h)
                .max()
                .unwrap_or(base);
            sum += (top - h) as f64 / span;
        }
    }
    sum / (N * N) as f64
}
