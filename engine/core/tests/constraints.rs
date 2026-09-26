//! Hard-constraint tests from ТЗ §39: the validator on hand-made layouts, and the engine
//! on small scenarios (its results must always pass the validator).

use pucker_core::validator::validate;
use pucker_core::*;

fn item(id: &str, dims: [i32; 3], weight: f64, fragility: u8) -> Item {
    Item {
        id: id.into(),
        sku: id.split('#').next().unwrap().into(),
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

fn place(kind: PackingPlaceType, dims: [i32; 3]) -> PackingPlace {
    PackingPlace {
        id: "P".into(),
        place_type: kind,
        preset_id: None,
        name: "test".into(),
        width: dims[0],
        depth: dims[1],
        height: dims[2],
        max_payload: 10_000.0,
        tare_weight: 0.0,
        quantity: 1,
        is_open_top: false,
        allow_overhang: false,
        max_overhang_x_mm: 0,
        max_overhang_y_mm: 0,
        use_pallet_base: false,
        pallet: None,
        door_width: None,
        door_height: None,
    }
}

fn request(items: Vec<Item>, p: PackingPlace) -> PackingRequest {
    PackingRequest {
        items,
        packing_place_mode: PackingPlaceMode::FixedPackingPlace,
        available_packing_places: vec![p],
        pack_rule: PackRule { time_limit_seconds: 2.0, random_seed: Some(1), ..PackRule::default() },
    }
}

/// A placed box at (x, y, z) with the item's own orientation.
fn at(req: &PackingRequest, id: &str, x: i32, y: i32, z: i32) -> PlacedItem {
    let it = req.items.iter().find(|i| i.id == id).unwrap();
    PlacedItem {
        item_id: id.into(),
        sku: it.sku.clone(),
        bin_id: "P-1".into(),
        x,
        y,
        z,
        width: it.width,
        depth: it.depth,
        height: it.height,
        rotation: "WDH".into(),
        weight: it.weight,
        fragility: it.fragility,
        max_top_load: 0.0,
        support_ratio: 0.0,
        received_top_load: 0.0,
        supported_by: vec![],
        supported_items: vec![],
    }
}

fn result(req: &PackingRequest, placed: Vec<PlacedItem>) -> PackingResult {
    let p = &req.available_packing_places[0];
    PackingResult {
        bins: vec![PackedBin {
            bin_id: "P-1".into(),
            place_id: p.id.clone(),
            place_name: p.name.clone(),
            place_type: p.place_type,
            width: p.width,
            depth: p.depth,
            height: p.height,
            base_z: p.base_z(),
            total_weight: 0.0,
            item_weight: 0.0,
            pallet_weight: 0.0,
            used_height: 0,
            utilization: 0.0,
            compactness: 0.0,
            center_of_mass: [0.0; 3],
            cg_offset_mm: 0.0,
            stability_score: 0.0,
            max_overhang_mm: 0,
            placed_items: placed,
        }],
        unplaced: vec![],
        warnings: vec![],
        summary: Summary::default(),
        validation: ValidationReport::default(),
        diagnostics: Diagnostics::default(),
    }
}

fn codes(req: &PackingRequest, placed: Vec<PlacedItem>) -> Vec<String> {
    validate(req, &result(req, placed)).violations.into_iter().map(|v| v.code).collect()
}

/// A closed 1000×1000×1000 box with walls, no load limits unless set by the test.
fn room() -> PackingPlace {
    place(PackingPlaceType::Custom, [1000, 1000, 1000])
}

#[test]
fn fragility_weaker_on_stronger_only() {
    let mut req = request(vec![item("a", [500, 500, 100], 5.0, 1), item("b", [500, 500, 100], 5.0, 2), item("c", [500, 500, 100], 5.0, 5)], room());
    req.pack_rule.use_max_top_load = false;
    // 2 on 1 is forbidden.
    assert!(codes(&req, vec![at(&req, "a", 0, 0, 0), at(&req, "b", 0, 0, 100), at(&req, "c", 500, 0, 0)]).contains(&"FRAGILITY".to_string()));
    // 2 on 5 and 1 on 2 are fine.
    assert!(codes(&req, vec![at(&req, "c", 0, 0, 0), at(&req, "b", 0, 0, 100), at(&req, "a", 0, 0, 200)]).is_empty());
}

#[test]
fn support_ratio_70_passes_69_fails() {
    let req = request(vec![item("base", [700, 1000, 100], 5.0, 5), item("top", [1000, 100, 100], 1.0, 5)], room());
    // 700 of 1000 mm supported = 70%.
    assert!(codes(&req, vec![at(&req, "base", 0, 0, 0), at(&req, "top", 0, 0, 100)]).is_empty());
    let req = request(vec![item("base", [690, 1000, 100], 5.0, 5), item("top", [1000, 100, 100], 1.0, 5)], room());
    assert!(codes(&req, vec![at(&req, "base", 0, 0, 0), at(&req, "top", 0, 0, 100)]).contains(&"SUPPORT".to_string()));
}

#[test]
fn centre_of_gravity_must_be_over_support() {
    let mut req = request(vec![item("base", [400, 1000, 100], 5.0, 5), item("top", [1000, 100, 100], 1.0, 5)], room());
    req.pack_rule.min_support_ratio = 0.3;
    // 40% support, but the centre (x = 500) is outside the base (0..400).
    assert!(codes(&req, vec![at(&req, "base", 0, 0, 0), at(&req, "top", 0, 0, 100)]).contains(&"CENTER_OF_GRAVITY".to_string()));
}

#[test]
fn top_load_goes_through_levels_and_splits_by_area() {
    // C carries B (10 kg) and A (10 kg) through B: 20 kg.
    let mk = |limit: f64| {
        let mut c = item("c", [500, 500, 100], 1.0, 10);
        c.max_top_load = Some(limit);
        request(vec![item("a", [500, 500, 100], 10.0, 10), item("b", [500, 500, 100], 10.0, 10), c], room())
    };
    let stack = |req: &PackingRequest| vec![at(req, "c", 0, 0, 0), at(req, "b", 0, 0, 100), at(req, "a", 0, 0, 200)];
    let req = mk(20.0);
    assert!(codes(&req, stack(&req)).is_empty(), "exactly at the limit is allowed");
    let req = mk(19.9);
    assert!(codes(&req, stack(&req)).contains(&"TOP_LOAD".to_string()));

    // A 30 kg plate over two supports: 3/4 of its area on L, 1/4 on R.
    let mut l = item("l", [750, 500, 100], 1.0, 10);
    l.max_top_load = Some(22.5);
    let mut r = item("r", [250, 500, 100], 1.0, 10);
    r.max_top_load = Some(7.4);
    let req = request(vec![l, r, item("p", [1000, 500, 50], 30.0, 10)], room());
    let got = codes(&req, vec![at(&req, "l", 0, 0, 0), at(&req, "r", 750, 0, 0), at(&req, "p", 0, 0, 100)]);
    assert_eq!(got, vec!["TOP_LOAD".to_string()], "only R (7.5 kg > 7.4 kg) is overloaded");
}

#[test]
fn overhang_limits_on_pallet() {
    let mut p = place(PackingPlaceType::Pallet, [800, 1200, 1800]);
    p.allow_overhang = true;
    p.max_overhang_x_mm = 30;
    p.max_overhang_y_mm = 30;
    let req = request(vec![item("a", [830, 600, 200], 5.0, 5)], p.clone());
    assert!(codes(&req, vec![at(&req, "a", 0, 0, 0)]).is_empty(), "30 mm is at the limit");
    let req = request(vec![item("a", [831, 600, 200], 5.0, 5)], p.clone());
    assert!(codes(&req, vec![at(&req, "a", 0, 0, 0)]).contains(&"OUT_OF_BOUNDS".to_string()));
    p.allow_overhang = false;
    let req = request(vec![item("a", [801, 600, 200], 5.0, 5)], p);
    assert!(codes(&req, vec![at(&req, "a", 0, 0, 0)]).contains(&"OUT_OF_BOUNDS".to_string()));
}

#[test]
fn collisions_and_missing_items_are_caught() {
    let req = request(vec![item("a", [500, 500, 100], 5.0, 5), item("b", [500, 500, 100], 5.0, 5), item("c", [100, 100, 100], 1.0, 5)], room());
    let got = codes(&req, vec![at(&req, "a", 0, 0, 0), at(&req, "b", 499, 0, 0)]);
    assert!(got.contains(&"COLLISION".to_string()));
    assert!(got.contains(&"MISSING_ITEM".to_string()));
}

#[test]
fn lone_tower_is_rejected_but_braced_stack_is_fine() {
    // Three 250×250×300 boxes stacked alone in the middle of a pallet: 900 mm on a 250 mm base.
    let p = place(PackingPlaceType::Pallet, [800, 1200, 1800]);
    let tower: Vec<Item> = (0..3).map(|i| item(&format!("t#{}", i), [250, 250, 300], 2.0, 5)).collect();
    let req = request(tower.clone(), p.clone());
    let layout = |req: &PackingRequest| (0..3).map(|i| at(req, &format!("t#{}", i), 50, 500, 300 * i)).collect::<Vec<_>>();
    assert!(codes(&req, layout(&req)).contains(&"TOWER".to_string()));
    // The same stack leaning on a wide block next to it is braced.
    let mut items = tower;
    items.push(item("wall", [500, 600, 900], 20.0, 9));
    let req = request(items, p);
    let mut l = layout(&req);
    l.push(at(&req, "wall", 300, 300, 0));
    let got = codes(&req, l);
    assert!(got.is_empty(), "{:?}", got);
}

#[test]
fn standing_on_edge_needs_neighbours_on_both_sides() {
    // 100×400×300 standing on its 100 mm side: height 3× the thin side.
    let p = place(PackingPlaceType::Pallet, [800, 1200, 1800]);
    let mk = |extra: bool| {
        let mut v = vec![item("thin", [100, 400, 300], 3.0, 5)];
        if extra {
            v.push(item("l", [300, 400, 300], 5.0, 5));
            v.push(item("r", [300, 400, 300], 5.0, 5));
        }
        request(v, p.clone())
    };
    let req = mk(false);
    assert!(codes(&req, vec![at(&req, "thin", 300, 300, 0)]).contains(&"STANDING_FREE".to_string()));
    // Squeezed between two boxes it is fine.
    let req = mk(true);
    assert!(codes(&req, vec![at(&req, "l", 0, 300, 0), at(&req, "thin", 300, 300, 0), at(&req, "r", 400, 300, 0)]).is_empty());
    // Taller than 3× its thin side is never allowed.
    let req = request(vec![item("pole", [100, 400, 310], 3.0, 5)], p.clone());
    assert!(codes(&req, vec![at(&req, "pole", 0, 0, 0)]).contains(&"ON_EDGE".to_string()));
}

#[test]
fn engine_results_always_pass_validation() {
    let mut rng = pucker_core::rng::Rng::new(3);
    for case in 0..6 {
        let items: Vec<Item> = (0..120)
            .map(|i| {
                let mut d = || 80 + rng.below(320) as i32;
                let dims = [d(), d(), d()];
                item(&format!("s{}#{}", i % 15, i), dims, 0.5 + rng.next_f64() * 15.0, 1 + rng.below(10) as u8)
            })
            .collect();
        let kind = if case % 2 == 0 { PackingPlaceType::Pallet } else { PackingPlaceType::Van };
        let mut p = place(kind, [800, 1200, 1800]);
        p.quantity = 3;
        let mut req = request(items, p);
        req.pack_rule.time_limit_seconds = 1.0;
        let res = pack(&req).unwrap();
        assert!(res.validation.valid, "case {}: {:?}", case, &res.validation.violations[..res.validation.violations.len().min(5)]);
        assert_eq!(res.summary.items_placed + res.summary.items_unplaced, 120);
    }
}

#[test]
fn identical_boxes_pack_densely() {
    let items: Vec<Item> = (0..48).map(|i| item(&format!("b#{}", i), [400, 300, 250], 12.0, 8)).collect();
    let mut p = place(PackingPlaceType::Pallet, [800, 1200, 1644]);
    p.quantity = 2;
    let res = pack(&request(items, p)).unwrap();
    assert!(res.validation.valid);
    assert_eq!(res.summary.bins_used, 1);
    assert!(res.summary.compactness >= 0.85, "compactness {}", res.summary.compactness);
}

#[test]
fn unplaceable_items_are_reported_with_reasons() {
    let items = vec![item("huge", [2000, 100, 100], 1.0, 5), item("heavy", [100, 100, 100], 20_000.0, 5), item("ok", [100, 100, 100], 1.0, 5)];
    let res = pack(&request(items, room())).unwrap();
    let reason = |id: &str| res.unplaced.iter().find(|u| u.item_id == id).map(|u| u.reason);
    assert_eq!(reason("huge"), Some(UnplacedReason::ItemTooLarge));
    assert_eq!(reason("heavy"), Some(UnplacedReason::ItemTooHeavy));
    assert_eq!(reason("ok"), None);
    assert!(res.validation.valid);
}

#[test]
fn too_much_cargo_triggers_capacity_warnings() {
    let items: Vec<Item> = (0..30).map(|i| item(&format!("b#{}", i), [500, 500, 500], 5.0, 5)).collect();
    let res = pack(&request(items, room())).unwrap();
    let has = |c: &str| res.warnings.iter().any(|w| w.code == c);
    assert!(has("VOLUME_EXCEEDED"), "precheck before search");
    assert!(has("INSUFFICIENT_SPACE"), "summary after search");
    assert_eq!(res.summary.items_placed, 8);
    assert!(res.unplaced.iter().all(|u| u.reason == UnplacedReason::InsufficientSpace));
    assert!(res.validation.valid);
}
