//! Independent validator (ТЗ §38). It re-derives every hard constraint from the request and
//! the final coordinates only, without any state from the search engine.

use std::collections::HashMap;

use crate::geometry::{hull_of_rects, orientations, overlap, strictly_inside_doubled, union_area, Rect};
use crate::model::*;
use crate::prep::default_max_top_load;

const EPS: f64 = 1e-6;

pub fn validate(req: &PackingRequest, res: &PackingResult) -> ValidationReport {
    let mut v: Vec<Violation> = Vec::new();
    let rule = &req.pack_rule;
    let by_id: HashMap<&str, &Item> = req.items.iter().map(|i| (i.id.as_str(), i)).collect();
    let mut seen: HashMap<&str, u32> = HashMap::new();

    let mut push = |code: &str, bin: &str, item: &str, msg: String| {
        v.push(Violation { code: code.into(), bin_id: bin.into(), item_id: item.into(), message: msg })
    };

    for u in &res.unplaced {
        *seen.entry(u.item_id.as_str()).or_default() += 1;
    }

    for bin in &res.bins {
        let Some(place) = req.available_packing_places.iter().find(|p| p.id == bin.place_id) else {
            push("UNKNOWN_PLACE", &bin.bin_id, "", format!("Неизвестное место {}", bin.place_id));
            continue;
        };
        let base = place.base_z();
        let (ox, oy) = place.overhang();
        let items = &bin.placed_items;
        let n = items.len();
        let mut src: Vec<Option<&Item>> = Vec::with_capacity(n);

        // Identity, orientation, bounds, overhang, height (ТЗ §20, §12).
        let mut weight = 0.0;
        for p in items {
            *seen.entry(p.item_id.as_str()).or_default() += 1;
            let it = by_id.get(p.item_id.as_str()).copied();
            src.push(it);
            let Some(it) = it else {
                push("UNKNOWN_ITEM", &bin.bin_id, &p.item_id, "Коробки нет во входных данных".into());
                continue;
            };
            weight += it.weight;
            if !orientations(it.width, it.depth, it.height).iter().any(|(d, _)| *d == [p.width, p.depth, p.height]) {
                push("BAD_ROTATION", &bin.bin_id, &p.item_id, "Размеры не совпадают ни с одним поворотом".into());
            }
            if p.height as f64 > rule.max_item_slenderness * p.width.min(p.depth) as f64 + 1e-9 {
                push("ON_EDGE", &bin.bin_id, &p.item_id, format!("Коробка стоит на узкой грани: высота {} мм при основании {} мм", p.height, p.width.min(p.depth)));
            }
            if p.x < -ox || p.y < -oy || p.x + p.width > place.width + ox || p.y + p.depth > place.depth + oy {
                push("OUT_OF_BOUNDS", &bin.bin_id, &p.item_id, "Выход за границы или превышен свес".into());
            }
            if p.z < base {
                push("BELOW_BASE", &bin.bin_id, &p.item_id, "Коробка ниже основания".into());
            }
            if p.z + p.height > place.height {
                push("HEIGHT_EXCEEDED", &bin.bin_id, &p.item_id, "Превышена максимальная высота".into());
            }
        }
        // Weight and pallet payload (ТЗ §11).
        if weight > place.max_payload + EPS {
            push("WEIGHT_EXCEEDED", &bin.bin_id, "", format!("Вес {:.1} кг > {:.1} кг", weight, place.max_payload));
        }
        if let (Some(pal), PackingPlaceType::Pallet) = (&place.pallet, place.place_type) {
            if weight > pal.max_payload + EPS {
                push("PALLET_PAYLOAD_EXCEEDED", &bin.bin_id, "", format!("Вес {:.1} кг > {:.1} кг", weight, pal.max_payload));
            }
        }

        // Collisions with clearance (ТЗ §19).
        let c = rule.clearance_mm;
        let mut order: Vec<usize> = (0..n).collect();
        order.sort_by_key(|&i| items[i].x);
        for a in 0..n {
            let pa = &items[order[a]];
            for &ib in &order[a + 1..] {
                let pb = &items[ib];
                if pb.x >= pa.x + pa.width + c {
                    break;
                }
                let sep_x = pa.x + pa.width + c <= pb.x || pb.x + pb.width + c <= pa.x;
                let sep_y = pa.y + pa.depth + c <= pb.y || pb.y + pb.depth + c <= pa.y;
                let sep_z = pa.z + pa.height <= pb.z || pb.z + pb.height <= pa.z;
                if !(sep_x || sep_y || sep_z) {
                    push("COLLISION", &bin.bin_id, &pa.item_id, format!("Пересечение с {}", pb.item_id));
                }
            }
        }

        // Supports (ТЗ §13–15).
        // A top up to `support_tolerance_mm` below the bottom still carries the box.
        let stol = rule.support_tolerance_mm;
        let mut supports: Vec<Vec<(usize, i64)>> = vec![Vec::new(); n];
        for (i, p) in items.iter().enumerate() {
            let foot = Rect::new(p.x, p.y, p.width, p.depth);
            let mut rects = Vec::new();
            if p.z - base <= stol {
                rects.extend(foot.intersect(&Rect::new(0, 0, place.width, place.depth)));
            }
            if p.z > base {
                for (j, q) in items.iter().enumerate() {
                    let top = q.z + q.height;
                    if j == i || top > p.z || top < p.z - stol {
                        continue;
                    }
                    if let Some(r) = foot.intersect(&Rect::new(q.x, q.y, q.width, q.depth)) {
                        rects.push(r);
                        supports[i].push((j, r.area()));
                    }
                }
            }
            let area = union_area(&rects);
            let ratio = area as f64 / foot.area() as f64;
            let need = rule.required_support(p.width, p.depth, p.height);
            if ratio + 1e-9 < need {
                push("SUPPORT", &bin.bin_id, &p.item_id, format!("Опора {:.0}% < {:.0}%", ratio * 100.0, need * 100.0));
            } else if area < foot.area() {
                let hull = hull_of_rects(&rects);
                if !strictly_inside_doubled(&hull, 2 * p.x as i64 + p.width as i64, 2 * p.y as i64 + p.depth as i64) {
                    push("CENTER_OF_GRAVITY", &bin.bin_id, &p.item_id, "Центр тяжести вне опоры".into());
                }
            }
            // Fragility (ТЗ §7).
            if rule.use_fragility {
                for &(j, _) in &supports[i] {
                    if p.fragility > items[j].fragility {
                        push("FRAGILITY", &bin.bin_id, &p.item_id, format!("Класс {} стоит на классе {} ({})", p.fragility, items[j].fragility, items[j].item_id));
                    }
                }
            }
        }

        // Top load, recomputed from the top down (ТЗ §8–9).
        if rule.use_max_top_load {
            let mut received = vec![0.0f64; n];
            let mut order: Vec<usize> = (0..n).collect();
            order.sort_by_key(|&i| std::cmp::Reverse(items[i].z));
            for &i in &order {
                let total_area: i64 = supports[i].iter().map(|s| s.1).sum();
                let w = src[i].map_or(0.0, |it| it.weight);
                let load = w + received[i];
                if total_area > 0 {
                    for &(j, a) in &supports[i] {
                        received[j] += load * a as f64 / total_area as f64;
                    }
                }
            }
            for i in 0..n {
                let Some(it) = src[i] else { continue };
                let limit = it.max_top_load.unwrap_or_else(|| default_max_top_load(it, rule));
                if received[i] > limit + EPS {
                    push("TOP_LOAD", &bin.bin_id, &items[i].item_id, format!("Нагрузка {:.1} кг > {:.1} кг", received[i], limit));
                }
            }
        }

        // Lateral stability: no free-standing towers (docs/DECISIONS.md §4).
        if rule.use_lateral_stability {
            let g = rule.lateral_gap_mm + c;
            let walls = place.holds_sides();
            let mut side = vec![[0i64; 4]; n];
            for (i, p) in items.iter().enumerate() {
                let (fx, fy) = (p.depth as i64 * p.height as i64, p.width as i64 * p.height as i64);
                if walls {
                    if p.x <= g { side[i][0] = fx; }
                    if p.x + p.width >= place.width - g { side[i][1] = fx; }
                    if p.y <= g { side[i][2] = fy; }
                    if p.y + p.depth >= place.depth - g { side[i][3] = fy; }
                }
            }
            for a in 0..n {
                let pa = &items[order[a]];
                for &ib in &order[a + 1..] {
                    let pb = &items[ib];
                    if pb.x > pa.x + pa.width + g {
                        break;
                    }
                    let oz = overlap(pa.z, pa.z + pa.height, pb.z, pb.z + pb.height);
                    if oz == 0 {
                        continue;
                    }
                    let (ia, ib2) = (order[a], ib);
                    let oy = overlap(pa.y, pa.y + pa.depth, pb.y, pb.y + pb.depth);
                    let ox = overlap(pa.x, pa.x + pa.width, pb.x, pb.x + pb.width);
                    if oy > 0 && (0..=g).contains(&(pb.x - (pa.x + pa.width))) {
                        side[ia][1] += oy * oz;
                        side[ib2][0] += oy * oz;
                    }
                    if oy > 0 && (0..=g).contains(&(pa.x - (pb.x + pb.width))) {
                        side[ia][0] += oy * oz;
                        side[ib2][1] += oy * oz;
                    }
                    if ox > 0 && (0..=g).contains(&(pb.y - (pa.y + pa.depth))) {
                        side[ia][3] += ox * oz;
                        side[ib2][2] += ox * oz;
                    }
                    if ox > 0 && (0..=g).contains(&(pa.y - (pb.y + pb.depth))) {
                        side[ia][2] += ox * oz;
                        side[ib2][3] += ox * oz;
                    }
                }
            }
            for (i, p) in items.iter().enumerate() {
                if rule.is_standing(p.width, p.depth, p.height)
                    && !crate::bin_state::enclosed(&side[i], p.width, p.depth, p.height, rule.standing_contact_ratio)
                {
                    push("STANDING_FREE", &bin.bin_id, &p.item_id, "Коробка стоит на ребре без соседей с обеих сторон".into());
                }
            }
            let tan = rule.tilt_angle_deg.to_radians().tan();
            let ratio = if tan > 0.0 { 1.0 / tan } else { f64::INFINITY };
            let mut margin = vec![f64::INFINITY; n];
            let mut up: Vec<usize> = (0..n).collect();
            up.sort_by_key(|&i| items[i].z);
            for &i in &up {
                let p = &items[i];
                let r = rule.lateral_min_contact_ratio;
                let fx = p.depth as f64 * p.height as f64 * r;
                let fy = p.width as f64 * p.height as f64 * r;
                let s = side[i];
                let braced = (s[0] as f64 >= fx) as u8 + (s[1] as f64 >= fx) as u8 + (s[2] as f64 >= fy) as u8 + (s[3] as f64 >= fy) as u8 >= rule.lateral_min_braced_sides;
                if braced {
                    continue;
                }
                let h = p.height as f64;
                let own = p.width.min(p.depth) as f64 * ratio - h;
                margin[i] = supports[i].iter().fold(own, |m, &(j, _)| m.min(margin[j] - h));
                if margin[i] < -EPS {
                    push("TOWER", &bin.bin_id, &p.item_id, format!("Неустойчивая стопка: не хватает {:.0} мм запаса на опрокидывание", -margin[i]));
                }
            }
        }
    }

    // Every input item is either placed or reported as unplaced, exactly once (ТЗ §36).
    for it in &req.items {
        match seen.get(it.id.as_str()).copied().unwrap_or(0) {
            1 => {}
            0 => v.push(Violation { code: "MISSING_ITEM".into(), bin_id: String::new(), item_id: it.id.clone(), message: "Коробка пропала из результата".into() }),
            k => v.push(Violation { code: "DUPLICATE_ITEM".into(), bin_id: String::new(), item_id: it.id.clone(), message: format!("Коробка встречается {} раз", k) }),
        }
    }

    ValidationReport { valid: v.is_empty(), violations: v }
}
