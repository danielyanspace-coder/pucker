//! `pucker` — command line runner: import items, pack, print a report, write JSON / 3D HTML.

mod bench;
mod html;
mod import;

use clap::{Parser, Subcommand};
use pucker_core::*;

use import::{FragilityScale, ImportOptions};

const PRESETS: &str = include_str!("../../../presets/places.json");

#[derive(Parser)]
#[command(name = "pucker", about = "Расчёт 3D-укладки коробок")]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Упаковать коробки из Excel/JSON
    Pack(PackArgs),
    /// Перепроверить готовый результат независимым валидатором
    Validate {
        #[arg(long)]
        request: String,
        #[arg(long)]
        result: String,
    },
    /// Упаковать по готовому JSON-запросу (формат API)
    Run {
        #[arg(long)]
        request: String,
        #[arg(long)]
        out: Option<String>,
        #[arg(long)]
        html: Option<String>,
    },
    /// Прогнать эталонный набор задач (ТЗ §39)
    Bench {
        /// Время на задачу, с. По умолчанию — по размеру задачи, не больше 3 минут
        #[arg(long)]
        time_limit: Option<f64>,
        /// Добавить «Пример» заказчика
        #[arg(long)]
        example: Option<String>,
        /// Только сценарии, в названии которых есть эта строка
        #[arg(long)]
        filter: Option<String>,
    },
    /// Показать доступные пресеты мест погрузки
    Presets,
}

#[derive(clap::Args)]
struct PackArgs {
    /// Файл с коробками: .xlsx или .json (массив Item)
    #[arg(long)]
    items: String,
    /// Место погрузки: ID пресета, можно с количеством (PALLET_EUR:3). Повторяемый
    #[arg(long = "place")]
    places: Vec<String>,
    /// Своё место: ШИРИНАxГЛУБИНАxВЫСОТА[:ГРУЗОПОДЪЁМНОСТЬ[:КОЛ-ВО]] в мм и кг
    #[arg(long = "custom")]
    customs: Vec<String>,
    /// Автоподбор среди указанных мест вместо фиксированного выбора
    #[arg(long)]
    auto: bool,
    #[arg(long, value_enum, default_value = "tz")]
    fragility_scale: FragilityScale,
    #[arg(long, default_value_t = 10)]
    fragility_levels: u8,
    /// Единица длины в Excel (mm|cm|m), по умолчанию из заголовков
    #[arg(long)]
    unit: Option<String>,
    /// Время поиска, с. По умолчанию — по размеру задачи, не больше 3 минут
    #[arg(long)]
    time_limit: Option<f64>,
    #[arg(long)]
    seed: Option<u64>,
    #[arg(long, default_value_t = 0)]
    clearance: i32,
    /// Разрешить свес на палете, мм
    #[arg(long)]
    overhang: Option<i32>,
    /// Максимальное отношение высоты коробки к меньшей стороне основания
    #[arg(long)]
    max_slenderness: Option<f64>,
    /// Отключить проверку боковой устойчивости (башен)
    #[arg(long)]
    no_lateral: bool,
    /// Отключить правило хрупкости (для диагностики)
    #[arg(long)]
    no_fragility: bool,
    /// Отключить проверку нагрузки сверху (для диагностики)
    #[arg(long)]
    no_top_load: bool,
    #[arg(long)]
    out: Option<String>,
    #[arg(long)]
    html: Option<String>,
    /// Сохранить собранный запрос (для validate и API)
    #[arg(long)]
    request_out: Option<String>,
}

fn presets() -> Vec<PackingPlace> {
    serde_json::from_str(PRESETS).expect("presets/places.json is valid")
}

fn parse_place(spec: &str) -> Result<PackingPlace, String> {
    let (id, qty) = match spec.split_once(':') {
        Some((id, q)) => (id, q.parse::<u32>().map_err(|_| format!("неверное количество в «{}»", spec))?),
        None => (spec, 1),
    };
    let mut p = presets()
        .into_iter()
        .find(|p| p.id.eq_ignore_ascii_case(id))
        .ok_or_else(|| format!("нет пресета «{}» (см. pucker presets)", id))?;
    p.quantity = qty;
    Ok(p)
}

fn parse_custom(spec: &str, n: usize) -> Result<PackingPlace, String> {
    let mut parts = spec.split(':');
    let dims: Vec<i32> = parts
        .next()
        .unwrap_or("")
        .split(['x', 'х', '*'])
        .map(|s| s.trim().parse::<i32>())
        .collect::<Result<_, _>>()
        .map_err(|_| format!("неверные размеры «{}»", spec))?;
    if dims.len() != 3 {
        return Err(format!("нужно ШИРИНАxГЛУБИНАxВЫСОТА: «{}»", spec));
    }
    let payload = parts.next().map(|s| s.parse::<f64>()).transpose().map_err(|_| "неверная грузоподъёмность")?;
    let qty = parts.next().map(|s| s.parse::<u32>()).transpose().map_err(|_| "неверное количество")?;
    Ok(PackingPlace {
        id: format!("CUSTOM_{}", n),
        place_type: PackingPlaceType::Custom,
        preset_id: None,
        name: format!("Своё место {}×{}×{}", dims[0], dims[1], dims[2]),
        width: dims[0],
        depth: dims[1],
        height: dims[2],
        max_payload: payload.unwrap_or(f64::MAX),
        tare_weight: 0.0,
        quantity: qty.unwrap_or(1),
        is_open_top: false,
        allow_overhang: false,
        max_overhang_x_mm: 0,
        max_overhang_y_mm: 0,
        use_pallet_base: false,
        pallet: None,
        stretch_wrapped: None,
        door_width: None,
        door_height: None,
    })
}

fn pct(x: f64) -> String {
    format!("{:.1}%", x * 100.0)
}

fn print_report(res: &PackingResult) {
    let s = &res.summary;
    println!();
    println!("== Результат ==");
    println!("Коробок: {} · размещено {} · не поместилось {}", s.items_total, s.items_placed, s.items_unplaced);
    println!("Мест погрузки: {} · заполнение {} · плотность блока {}", s.bins_used, pct(s.utilization), pct(s.compactness));
    for b in &res.bins {
        println!(
            "  {} ({}): {} кор., {:.1} кг, высота {} мм, заполнение {}, плотность {}, устойчивость {:.2}, смещение ЦМ {:.0} мм",
            b.bin_id,
            b.place_name,
            b.placed_items.len(),
            b.total_weight,
            b.used_height,
            pct(b.utilization),
            pct(b.compactness),
            b.stability_score,
            b.cg_offset_mm
        );
    }
    for w in &res.warnings {
        let tag = match w.severity {
            Severity::Error => "⛔",
            Severity::Warning => "⚠️",
            Severity::Info => "ℹ️",
        };
        println!("{} {}", tag, w.message);
    }
    if !res.unplaced.is_empty() {
        let mut by: std::collections::BTreeMap<(String, &str), u32> = Default::default();
        for u in &res.unplaced {
            *by.entry((u.sku.clone(), u.reason.message_ru())).or_default() += 1;
        }
        println!("Не поместились:");
        for ((sku, why), n) in by {
            println!("  {} × {} — {}", sku, n, why);
        }
    }
    let d = &res.diagnostics;
    println!(
        "Проверка: {} · время {:.2} с · попыток {} (лучшая № {}) · seed {}",
        if res.validation.valid { "OK, нарушений нет".to_string() } else { format!("{} нарушений", res.validation.violations.len()) },
        d.calculation_time,
        d.iterations,
        d.best_found_at_iteration,
        d.random_seed
    );
    for v in res.validation.violations.iter().take(20) {
        println!("  ✗ {} {} {}: {}", v.code, v.bin_id, v.item_id, v.message);
    }
}

fn run_pack(a: PackArgs) -> Result<(), String> {
    let unit_mm = match a.unit.as_deref() {
        None => None,
        Some("mm") => Some(1.0),
        Some("cm") => Some(10.0),
        Some("m") => Some(1000.0),
        Some(u) => return Err(format!("неизвестная единица «{}»", u)),
    };
    let opt = ImportOptions { scale: a.fragility_scale, levels: a.fragility_levels, unit_mm };
    let items = import::read_items(&a.items, &opt)?;
    let mut places: Vec<PackingPlace> = a.places.iter().map(|s| parse_place(s)).collect::<Result<_, _>>()?;
    for (i, c) in a.customs.iter().enumerate() {
        places.push(parse_custom(c, i + 1)?);
    }
    if places.is_empty() {
        places.push(parse_place("PALLET_EUR")?);
    }
    if let Some(o) = a.overhang {
        for p in &mut places {
            p.allow_overhang = o > 0;
            p.max_overhang_x_mm = o;
            p.max_overhang_y_mm = o;
        }
    }
    let rule = PackRule {
        time_limit_seconds: a.time_limit.unwrap_or(180.0),
        auto_time: a.time_limit.is_none(),
        random_seed: a.seed,
        clearance_mm: a.clearance,
        use_lateral_stability: !a.no_lateral,
        use_fragility: !a.no_fragility,
        max_item_slenderness: a.max_slenderness.unwrap_or(PackRule::default().max_item_slenderness),
        use_max_top_load: !a.no_top_load,
        ..PackRule::default()
    };
    let req = PackingRequest {
        items,
        packing_place_mode: if a.auto { PackingPlaceMode::AutoSelectPackingPlace } else { PackingPlaceMode::FixedPackingPlace },
        available_packing_places: places,
        pack_rule: rule,
    };
    let vol: i64 = req.items.iter().map(|i| i.width as i64 * i.depth as i64 * i.height as i64).sum();
    let wt: f64 = req.items.iter().map(|i| i.weight).sum();
    println!(
        "Загружено {} коробок, {:.3} м³, {:.1} кг. Места: {}",
        req.items.len(),
        vol as f64 / 1e9,
        wt,
        req.available_packing_places.iter().map(|p| format!("{} ×{}", p.name, p.quantity)).collect::<Vec<_>>().join(", ")
    );
    if let Some(path) = &a.request_out {
        std::fs::write(path, serde_json::to_string_pretty(&req).unwrap()).map_err(|e| e.to_string())?;
    }
    let res = pack(&req)?;
    print_report(&res);
    if let Some(path) = &a.out {
        std::fs::write(path, serde_json::to_string_pretty(&res).unwrap()).map_err(|e| e.to_string())?;
        println!("JSON: {}", path);
    }
    if let Some(path) = &a.html {
        std::fs::write(path, html::render(&req, &res)).map_err(|e| e.to_string())?;
        println!("3D: {}", path);
    }
    Ok(())
}

fn main() {
    let cli = Cli::parse();
    let r = match cli.cmd {
        Cmd::Pack(a) => run_pack(a),
        Cmd::Run { request, out, html } => (|| {
            let req: PackingRequest = serde_json::from_str(&std::fs::read_to_string(request).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?;
            let res = pack(&req)?;
            print_report(&res);
            if let Some(path) = out {
                std::fs::write(&path, serde_json::to_string_pretty(&res).unwrap()).map_err(|e| e.to_string())?;
            }
            if let Some(path) = html {
                std::fs::write(&path, html::render(&req, &res)).map_err(|e| e.to_string())?;
            }
            Ok(())
        })(),
        Cmd::Bench { time_limit, example, filter } => (|| {
            let ex = match example {
                Some(p) => Some(import::read_items(&p, &ImportOptions { scale: FragilityScale::Tz, levels: 10, unit_mm: None })?),
                None => None,
            };
            println!("{:<44} {:>6} {:>6} {:>5} {:>8} {:>8} {:>7} {:>7} {}", "сценарий", "кор.", "разм.", "мест", "плотн.", "запол.", "1-е, с", "всего", "проверка");
            let mut ok = true;
            for sc in bench::scenarios(&presets(), ex, time_limit) {
                if filter.as_ref().is_some_and(|f| !sc.name.contains(f.as_str())) {
                    continue;
                }
                let r = pack(&sc.request)?;
                let s = &r.summary;
                let dense = if sc.dense_expected { if s.compactness >= 0.85 { " ≥85% ✓" } else { " <85% ✗" } } else { "" };
                ok &= r.validation.valid && (!sc.dense_expected || s.compactness >= 0.85);
                println!(
                    "{:<44} {:>6} {:>6} {:>5} {:>8} {:>8} {:>7.2} {:>7.1} {}{}",
                    sc.name,
                    s.items_total,
                    s.items_placed,
                    s.bins_used,
                    pct(s.compactness),
                    pct(s.utilization),
                    r.diagnostics.first_solution_time,
                    r.diagnostics.calculation_time,
                    if r.validation.valid { "OK" } else { "ОШИБКИ" },
                    dense
                );
            }
            if ok { Ok(()) } else { Err("есть проваленные сценарии".into()) }
        })(),
        Cmd::Presets => {
            for p in presets() {
                println!("{:<26} {:<34} {}×{}×{} мм, {} кг", p.id, p.name, p.width, p.depth, p.height, p.max_payload);
            }
            Ok(())
        }
        Cmd::Validate { request, result } => (|| {
            let req: PackingRequest = serde_json::from_str(&std::fs::read_to_string(request).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?;
            let res: PackingResult = serde_json::from_str(&std::fs::read_to_string(result).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?;
            let v = validator::validate(&req, &res);
            if v.valid {
                println!("OK: нарушений нет");
            } else {
                for x in &v.violations {
                    println!("✗ {} {} {}: {}", x.code, x.bin_id, x.item_id, x.message);
                }
                return Err(format!("{} нарушений", v.violations.len()));
            }
            Ok(())
        })(),
    };
    if let Err(e) = r {
        eprintln!("Ошибка: {}", e);
        std::process::exit(1);
    }
}
