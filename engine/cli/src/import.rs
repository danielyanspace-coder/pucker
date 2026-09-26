//! Excel / JSON import of the item list.

use calamine::{open_workbook_auto, Data, Reader};
use pucker_core::Item;

#[derive(Clone, Copy, Debug, PartialEq, Eq, clap::ValueEnum)]
pub enum FragilityScale {
    /// 1 = most fragile, higher = stronger (as in the customer spec).
    Tz,
    /// 1 = strongest; converted with `levels + 1 - value`.
    Inverted,
}

pub struct ImportOptions {
    pub scale: FragilityScale,
    pub levels: u8,
    /// Length unit override in mm per unit (10 = centimetres); `None` = from headers.
    pub unit_mm: Option<f64>,
}

#[derive(Default)]
struct Columns {
    sku: Option<usize>,
    name: Option<usize>,
    barcode: Option<usize>,
    weight: Option<usize>,
    length: Option<usize>,
    width: Option<usize>,
    height: Option<usize>,
    fragility: Option<usize>,
    quantity: Option<usize>,
    top_load: Option<usize>,
    priority: Option<usize>,
}

fn cell_str(c: &Data) -> String {
    match c {
        Data::String(s) => s.trim().to_string(),
        Data::Float(f) => {
            if f.fract() == 0.0 { format!("{}", *f as i64) } else { f.to_string() }
        }
        Data::Int(i) => i.to_string(),
        Data::Bool(b) => b.to_string(),
        _ => String::new(),
    }
}

fn cell_num(c: &Data) -> Option<f64> {
    match c {
        Data::Float(f) => Some(*f),
        Data::Int(i) => Some(*i as f64),
        Data::String(s) => s.trim().replace(',', ".").replace(' ', "").parse().ok(),
        _ => None,
    }
}

/// mm per unit from a header like "Длина, см".
fn unit_of(header: &str) -> Option<f64> {
    let h = header.to_lowercase();
    if h.contains("мм") || h.contains("mm") {
        Some(1.0)
    } else if h.contains("см") || h.contains("cm") {
        Some(10.0)
    } else if h.ends_with(", м") || h.ends_with("(м)") || h.ends_with(" m") {
        Some(1000.0)
    } else {
        None
    }
}

/// Convert a length to integer millimetres, rounding up after trimming float noise
/// (20.200000000000003 cm must become 202 mm, not 203).
pub fn to_mm(value: f64, mm_per_unit: f64) -> i32 {
    let mm = value * mm_per_unit;
    let trimmed = (mm * 100.0).round() / 100.0;
    trimmed.ceil() as i32
}

fn parse_fragility(c: &Data, opt: &ImportOptions) -> Result<u8, String> {
    let s = cell_str(c).to_lowercase();
    let yes = ["да", "yes", "true", "хрупкий", "хрупкое", "+"];
    let no = ["нет", "no", "false", "не хрупкий", "нехрупкий", "-", ""];
    if yes.contains(&s.as_str()) {
        return Ok(1);
    }
    if no.contains(&s.as_str()) {
        return Ok(opt.levels.max(2));
    }
    let v = cell_num(c).ok_or_else(|| format!("не удалось прочитать хрупкость «{}»", s))?;
    let v = v.round() as i64;
    if v < 1 || v > opt.levels as i64 {
        return Err(format!("хрупкость {} вне диапазона 1..{}", v, opt.levels));
    }
    Ok(match opt.scale {
        FragilityScale::Tz => v as u8,
        FragilityScale::Inverted => (opt.levels as i64 + 1 - v) as u8,
    })
}

pub fn read_items(path: &str, opt: &ImportOptions) -> Result<Vec<Item>, String> {
    if path.ends_with(".json") {
        let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
        return serde_json::from_str(&text).map_err(|e| e.to_string());
    }
    let mut wb = open_workbook_auto(path).map_err(|e| format!("не удалось открыть {}: {}", path, e))?;
    let range = wb
        .worksheet_range_at(0)
        .ok_or("в файле нет листов")?
        .map_err(|e| e.to_string())?;
    let mut rows = range.rows();
    let header: Vec<String> = rows.next().ok_or("пустой лист")?.iter().map(cell_str).collect();

    let mut col = Columns::default();
    let mut len_unit = None;
    let mut weight_grams = false;
    for (i, h) in header.iter().enumerate() {
        let l = h.to_lowercase();
        let set = |slot: &mut Option<usize>| {
            if slot.is_none() {
                *slot = Some(i)
            }
        };
        if l.contains("артикул") || l == "sku" || l.contains("код") && !l.contains("штрих") {
            set(&mut col.sku);
        } else if l.contains("наимен") || l.contains("назван") || l.contains("товар") {
            set(&mut col.name);
        } else if l.contains("штрих") || l.contains("barcode") || l.contains("ean") {
            set(&mut col.barcode);
        } else if l.contains("масса") || l.contains("вес") || l.contains("weight") {
            weight_grams = l.contains(", г") || l.contains("(г)") || l.ends_with(" г");
            set(&mut col.weight);
        } else if l.contains("длин") || l.contains("length") {
            len_unit = len_unit.or(unit_of(&l));
            set(&mut col.length);
        } else if l.contains("ширин") || l.contains("width") {
            len_unit = len_unit.or(unit_of(&l));
            set(&mut col.width);
        } else if l.contains("высот") || l.contains("height") {
            len_unit = len_unit.or(unit_of(&l));
            set(&mut col.height);
        } else if l.contains("хруп") || l.contains("fragil") {
            set(&mut col.fragility);
        } else if l.contains("кол") || l.contains("qty") || l.contains("quantity") {
            set(&mut col.quantity);
        } else if l.contains("нагруз") || l.contains("top_load") {
            set(&mut col.top_load);
        } else if l.contains("приоритет") || l.contains("priority") {
            set(&mut col.priority);
        }
    }
    let need = |c: Option<usize>, name: &str| c.ok_or_else(|| format!("не найдена колонка «{}»", name));
    let (cl, cw, ch) = (need(col.length, "Длина")?, need(col.width, "Ширина")?, need(col.height, "Высота")?);
    let cwt = need(col.weight, "Масса")?;
    let mm = opt.unit_mm.or(len_unit).unwrap_or(10.0);

    let mut items = Vec::new();
    let mut per_sku: std::collections::HashMap<String, u32> = Default::default();
    for (r, row) in rows.enumerate() {
        let line = r + 2;
        if row.iter().all(|c| matches!(c, Data::Empty)) {
            continue;
        }
        let get = |c: Option<usize>| c.and_then(|i| row.get(i));
        let num = |c: usize, what: &str| -> Result<f64, String> {
            row.get(c).and_then(cell_num).ok_or_else(|| format!("строка {}: не прочитано «{}»", line, what))
        };
        let sku = get(col.sku).map(cell_str).filter(|s| !s.is_empty()).unwrap_or_else(|| format!("row{}", line));
        // In the sample file the "Штрихкод" column actually holds product names.
        let name = get(col.name)
            .map(cell_str)
            .or_else(|| get(col.barcode).map(cell_str).filter(|s| s.parse::<f64>().is_err()))
            .unwrap_or_default();
        let (l, w, h) = (num(cl, "Длина")?, num(cw, "Ширина")?, num(ch, "Высота")?);
        let mut weight = num(cwt, "Масса")?;
        if weight_grams {
            weight /= 1000.0;
        }
        let fragility = match get(col.fragility) {
            Some(c) => parse_fragility(c, opt).map_err(|e| format!("строка {}: {}", line, e))?,
            None => opt.levels,
        };
        let qty = match col.quantity {
            Some(c) => num(c, "Количество")?.round() as i64,
            None => 1,
        };
        let top_load = get(col.top_load).and_then(cell_num);
        let priority = get(col.priority).and_then(cell_num).map_or(0, |p| p.round() as i32);
        let (wm, dm, hm) = (to_mm(l, mm), to_mm(w, mm), to_mm(h, mm));
        if wm <= 0 || dm <= 0 || hm <= 0 {
            return Err(format!("строка {}: размеры должны быть больше нуля", line));
        }
        for _ in 0..qty.max(0) {
            let k = per_sku.entry(sku.clone()).or_default();
            *k += 1;
            items.push(Item {
                id: format!("{}#{}", sku, k),
                sku: sku.clone(),
                name: name.clone(),
                width: wm,
                depth: dm,
                height: hm,
                weight,
                fragility,
                max_top_load: top_load,
                priority,
            });
        }
    }
    Ok(items)
}

#[cfg(test)]
mod tests {
    use super::to_mm;

    #[test]
    fn mm_conversion_trims_float_noise() {
        assert_eq!(to_mm(20.200000000000003, 10.0), 202);
        assert_eq!(to_mm(28.499999999999996, 10.0), 285);
        assert_eq!(to_mm(24.81, 10.0), 249);
        assert_eq!(to_mm(1.3, 10.0), 13);
    }
}
