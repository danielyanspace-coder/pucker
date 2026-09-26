//! Fixed benchmark scenarios (ТЗ §39): geometry, scale and the customer's sample file.

use pucker_core::rng::Rng;
use pucker_core::*;

pub struct Scenario {
    pub name: &'static str,
    /// Whether ≥85% compactness is geometrically reachable (ТЗ §23).
    pub dense_expected: bool,
    pub request: PackingRequest,
}

fn item(id: String, sku: &str, dims: [i32; 3], weight: f64, fragility: u8) -> Item {
    Item {
        id,
        sku: sku.to_string(),
        name: String::new(),
        width: dims[0],
        depth: dims[1],
        height: dims[2],
        weight,
        fragility,
        max_top_load: None,
        priority: 0,
    }
}

/// `count` copies of each type: (dims, weight, fragility).
fn items_of(types: &[([i32; 3], f64, u8, usize)]) -> Vec<Item> {
    let mut v = Vec::new();
    for (t, &(dims, w, f, n)) in types.iter().enumerate() {
        let sku = format!("T{:02}", t + 1);
        for k in 0..n {
            v.push(item(format!("{}#{}", sku, k + 1), &sku, dims, w, f));
        }
    }
    v
}

fn random_types(rng: &mut Rng, n_types: usize, min: i32, max: i32, total: usize) -> Vec<([i32; 3], f64, u8, usize)> {
    let per = (total / n_types).max(1);
    (0..n_types)
        .map(|_| {
            let mut d = || min + rng.below((max - min) as usize + 1) as i32;
            let dims = [d(), d(), d()];
            let vol = dims.iter().map(|&x| x as f64).product::<f64>() / 1e9;
            // Density 100–300 kg/m³, typical for boxed goods.
            let weight = (vol * (100.0 + rng.next_f64() * 200.0) * 10.0).round() / 10.0;
            let frag = 1 + rng.below(10) as u8;
            (dims, weight.max(0.1), frag, per)
        })
        .collect()
}

pub fn scenarios(presets: &[PackingPlace], example: Option<Vec<Item>>, time_limit: f64) -> Vec<Scenario> {
    let place = |id: &str, q: u32| {
        let mut p = presets.iter().find(|p| p.id == id).unwrap().clone();
        p.quantity = q;
        p
    };
    let rule = PackRule { time_limit_seconds: time_limit, random_seed: Some(7), ..PackRule::default() };
    let req = |items: Vec<Item>, places: Vec<PackingPlace>| PackingRequest {
        items,
        packing_place_mode: PackingPlaceMode::FixedPackingPlace,
        available_packing_places: places,
        pack_rule: rule.clone(),
    };
    let mut rng = Rng::new(2024);
    let mut v = vec![
        Scenario {
            name: "одинаковые коробки 400×300×250",
            dense_expected: true,
            request: req(items_of(&[([400, 300, 250], 12.0, 8, 48)]), vec![place("PALLET_EUR", 2)]),
        },
        Scenario {
            name: "3 размера коробок",
            dense_expected: true,
            request: req(
                items_of(&[([400, 300, 200], 10.0, 8, 30), ([300, 200, 200], 6.0, 6, 40), ([200, 200, 100], 3.0, 4, 60)]),
                vec![place("PALLET_EUR", 3)],
            ),
        },
        Scenario {
            name: "много мелких (1000 шт, 5 размеров)",
            dense_expected: true,
            request: req(
                items_of(&[
                    ([200, 150, 100], 2.0, 6, 200),
                    ([150, 100, 100], 1.2, 5, 200),
                    ([100, 100, 100], 0.8, 5, 200),
                    ([200, 100, 50], 0.8, 3, 200),
                    ([100, 50, 50], 0.2, 2, 200),
                ]),
                vec![place("PALLET_EUR", 3)],
            ),
        },
        Scenario {
            name: "крупные + мелкие",
            dense_expected: true,
            request: req(
                items_of(&[([600, 400, 400], 25.0, 9, 12), ([400, 200, 200], 6.0, 6, 40), ([200, 200, 200], 3.0, 4, 60)]),
                vec![place("PALLET_EUR", 3)],
            ),
        },
        Scenario {
            name: "почти размер палеты 1190×790×300",
            dense_expected: true,
            request: req(items_of(&[([790, 1190, 300], 60.0, 9, 5)]), vec![place("PALLET_EUR", 1)]),
        },
        Scenario {
            name: "разнородные 30 типов, 500 шт → Газель ×2",
            dense_expected: false,
            request: req(items_of(&random_types(&mut rng, 30, 150, 500, 500)), vec![place("GAZELLE_STANDARD", 2)]),
        },
        Scenario {
            name: "масштаб: 1000 шт, 40 типов → фура",
            dense_expected: false,
            request: req(items_of(&random_types(&mut rng, 40, 200, 600, 1000)), vec![place("TRUCK_20T", 2)]),
        },
        Scenario {
            name: "масштаб: 2000 шт, 40 типов → фура",
            dense_expected: false,
            request: req(items_of(&random_types(&mut rng, 40, 150, 500, 2000)), vec![place("TRUCK_20T", 2)]),
        },
    ];
    if let Some(items) = example {
        v.push(Scenario { name: "«Пример» заказчика → EUR-палеты", dense_expected: false, request: req(items, vec![place("PALLET_EUR", 3)]) });
    }
    v
}
