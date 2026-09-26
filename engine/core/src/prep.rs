//! Preprocessing (ТЗ §32, stage 1): validation, orientations, resolved top-load limits.

use crate::geometry::orientations;
use crate::model::{Item, PackRule};

#[derive(Clone, Debug)]
pub struct Orient {
    pub dims: [i32; 3],
    pub code: &'static str,
}

#[derive(Clone, Debug)]
pub struct PrepItem {
    pub src: usize,
    pub weight: f64,
    pub fragility: u8,
    /// Resolved limit in kg; `f64::INFINITY` when the check is disabled.
    pub max_top_load: f64,
    pub volume: i64,
    pub orients: Vec<Orient>,
    pub min_dim: i32,
    pub max_dim: i32,
    pub base_area: i64,
    pub priority: i32,
}

/// Default limit when the input has none: top face area × pressure of the fragility class.
/// Uses the largest face, since the item may be rotated to rest on it.
pub fn default_max_top_load(item: &Item, rule: &PackRule) -> f64 {
    let table = &rule.top_load_pressure_kg_m2;
    if table.is_empty() {
        return f64::INFINITY;
    }
    let k = (item.fragility.max(1) as usize - 1).min(table.len() - 1);
    let mut s = [item.width as f64, item.depth as f64, item.height as f64];
    s.sort_by(|a, b| a.partial_cmp(b).unwrap());
    // Area of the face opposite the smallest dimension, in m².
    let area_m2 = s[1] * s[2] / 1e6;
    area_m2 * table[k]
}

pub fn prepare(items: &[Item], rule: &PackRule) -> Result<Vec<PrepItem>, String> {
    let mut ids = std::collections::HashSet::new();
    if let Some(dup) = items.iter().find(|it| !ids.insert(it.id.as_str())) {
        return Err(format!("Повторяющийся id коробки: {}", dup.id));
    }
    items
        .iter()
        .enumerate()
        .map(|(i, it)| {
            if it.width <= 0 || it.depth <= 0 || it.height <= 0 {
                return Err(format!("Коробка {}: размеры должны быть больше нуля", it.id));
            }
            if !(it.weight >= 0.0) {
                return Err(format!("Коробка {}: некорректный вес", it.id));
            }
            if it.fragility == 0 {
                return Err(format!("Коробка {}: хрупкость должна быть от 1", it.id));
            }
            let max_top_load = if rule.use_max_top_load {
                it.max_top_load.unwrap_or_else(|| default_max_top_load(it, rule))
            } else {
                f64::INFINITY
            };
            // No standing on a narrow face (docs/DECISIONS.md §4b). Lying on the largest face
            // always passes, so every item keeps at least one orientation.
            let orients: Vec<Orient> = orientations(it.width, it.depth, it.height)
                .into_iter()
                .filter(|(d, _)| d[2] as f64 <= rule.max_item_slenderness * d[0].min(d[1]) as f64 + 1e-9)
                .map(|(dims, code)| Orient { dims, code })
                .collect();
            let mut s = [it.width, it.depth, it.height];
            s.sort_unstable();
            Ok(PrepItem {
                src: i,
                weight: it.weight,
                fragility: it.fragility,
                max_top_load,
                volume: it.width as i64 * it.depth as i64 * it.height as i64,
                orients,
                min_dim: s[0],
                max_dim: s[2],
                base_area: s[1] as i64 * s[2] as i64,
                priority: it.priority,
            })
        })
        .collect()
}
