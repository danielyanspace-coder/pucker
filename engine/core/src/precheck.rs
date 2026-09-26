//! Instant checks before the search (docs/DECISIONS.md §5).

use crate::model::{PackingPlace, Severity, Warning};
use crate::prep::PrepItem;

/// Would the item fit into an empty place in at least one orientation?
pub fn fits_empty(it: &PrepItem, place: &PackingPlace) -> bool {
    let (ox, oy) = place.overhang();
    let hmax = place.height - place.base_z();
    it.orients.iter().any(|o| {
        let [w, d, h] = o.dims;
        w <= place.width + 2 * ox && d <= place.depth + 2 * oy && h <= hmax
    })
}

pub fn fmt_m3(mm3: i64) -> String {
    format!("{:.2}", mm3 as f64 / 1e9).replace('.', ",")
}

pub fn fmt_kg(kg: f64) -> String {
    format!("{:.0}", kg)
}

/// Capacity warnings for a pool of places (quantities included).
pub fn capacity_warnings(items: &[PrepItem], pool: &[&PackingPlace]) -> Vec<Warning> {
    let mut out = Vec::new();
    let vol: i64 = items.iter().map(|i| i.volume).sum();
    let wt: f64 = items.iter().map(|i| i.weight).sum();
    let cap_vol: i64 = pool.iter().map(|p| p.usable_volume() * p.quantity as i64).sum();
    let cap_wt: f64 = pool.iter().map(|p| p.payload_limit() * p.quantity as f64).sum();
    let places: u32 = pool.iter().map(|p| p.quantity).sum();

    if vol > cap_vol {
        let need = pool
            .first()
            .map(|p| {
                let by_vol = (vol as f64 / p.usable_volume().max(1) as f64).ceil();
                let by_wt = (wt / p.payload_limit().max(1e-9)).ceil();
                by_vol.max(by_wt) as u64
            })
            .unwrap_or(0);
        out.push(Warning {
            code: "VOLUME_EXCEEDED".into(),
            severity: Severity::Error,
            message: format!(
                "Размер пространства не позволяет разместить такое количество коробок: груз занимает {} м³, \
                 а доступно {} м³ ({} мест). Потребуется минимум {} мест.",
                fmt_m3(vol),
                fmt_m3(cap_vol),
                places,
                need
            ),
        });
    } else if vol as f64 > 0.9 * cap_vol as f64 {
        out.push(Warning {
            code: "HIGH_FILL".into(),
            severity: Severity::Warning,
            message: format!(
                "Груз занимает {:.0}% доступного объёма. Реально достижимое заполнение обычно 85–90%, \
                 скорее всего поместится не всё.",
                100.0 * vol as f64 / cap_vol.max(1) as f64
            ),
        });
    }
    if wt > cap_wt + 1e-9 {
        out.push(Warning {
            code: "WEIGHT_EXCEEDED".into(),
            severity: Severity::Error,
            message: format!(
                "Вес груза {} кг превышает допустимую нагрузку {} кг ({} мест).",
                fmt_kg(wt),
                fmt_kg(cap_wt),
                places
            ),
        });
    }
    out
}
