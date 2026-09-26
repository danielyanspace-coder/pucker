//! Input and output data model (ТЗ §5, §10, §10A, §29, §34–§37).
//!
//! Linear sizes are integer millimetres, weights are kilograms.
//! Coordinates: X — width, Y — depth, Z — height (ТЗ §4).

use serde::{Deserialize, Serialize};

/// One physical unit of cargo (ТЗ §5).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Item {
    pub id: String,
    pub sku: String,
    #[serde(default)]
    pub name: String,
    pub width: i32,
    pub depth: i32,
    pub height: i32,
    pub weight: f64,
    /// 1 = most fragile, higher = stronger (ТЗ §7).
    pub fragility: u8,
    /// Max external vertical load, kg. `None` = derived from fragility (docs/DECISIONS.md §2).
    #[serde(default)]
    pub max_top_load: Option<f64>,
    /// Higher priority items are kept when not everything fits.
    #[serde(default)]
    pub priority: i32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum PackingPlaceType {
    Pallet,
    Container,
    Truck,
    Van,
    RailWagon,
    Custom,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Pallet {
    pub width: i32,
    pub depth: i32,
    pub height: i32,
    pub weight: f64,
    pub max_payload: f64,
}

/// A packing place: pallet, container, truck body… approximated by a box (ТЗ §10A).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PackingPlace {
    pub id: String,
    #[serde(rename = "type")]
    pub place_type: PackingPlaceType,
    #[serde(default)]
    pub preset_id: Option<String>,
    #[serde(default)]
    pub name: String,
    /// Usable inner width / depth. For a pallet — the pallet deck size.
    pub width: i32,
    pub depth: i32,
    /// Max total height from the floor (for a pallet — including the pallet itself).
    pub height: i32,
    /// Max cargo weight, kg.
    pub max_payload: f64,
    #[serde(default)]
    pub tare_weight: f64,
    #[serde(default = "one")]
    pub quantity: u32,
    #[serde(default)]
    pub is_open_top: bool,
    #[serde(default)]
    pub allow_overhang: bool,
    #[serde(default)]
    pub max_overhang_x_mm: i32,
    #[serde(default)]
    pub max_overhang_y_mm: i32,
    #[serde(default)]
    pub use_pallet_base: bool,
    #[serde(default)]
    pub pallet: Option<Pallet>,
    #[serde(default)]
    pub door_width: Option<i32>,
    #[serde(default)]
    pub door_height: Option<i32>,
}

fn one() -> u32 {
    1
}

impl PackingPlace {
    /// Rigid walls exist in vehicles and containers, not on a pallet.
    pub fn has_walls(&self) -> bool {
        self.place_type != PackingPlaceType::Pallet
    }

    /// Z where cargo starts (top of the pallet deck, or the floor).
    pub fn base_z(&self) -> i32 {
        match (&self.pallet, self.place_type) {
            (Some(p), PackingPlaceType::Pallet) => p.height,
            _ => 0,
        }
    }

    pub fn pallet_weight(&self) -> f64 {
        match (&self.pallet, self.place_type) {
            (Some(p), PackingPlaceType::Pallet) => p.weight,
            _ => 0.0,
        }
    }

    /// Effective cargo weight limit.
    pub fn payload_limit(&self) -> f64 {
        match (&self.pallet, self.place_type) {
            (Some(p), PackingPlaceType::Pallet) => self.max_payload.min(p.max_payload),
            _ => self.max_payload,
        }
    }

    /// Overhang is only allowed where there are no walls (ТЗ §12).
    pub fn overhang(&self) -> (i32, i32) {
        if self.allow_overhang && !self.has_walls() {
            (self.max_overhang_x_mm.max(0), self.max_overhang_y_mm.max(0))
        } else {
            (0, 0)
        }
    }

    pub fn usable_volume(&self) -> i64 {
        self.width as i64 * self.depth as i64 * (self.height - self.base_z()).max(0) as i64
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum PackingPlaceMode {
    FixedPackingPlace,
    AutoSelectPackingPlace,
}

/// Algorithm settings (ТЗ §29 plus lateral stability from docs/DECISIONS.md §4).
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct PackRule {
    pub min_support_ratio: f64,
    /// A top this much lower than a box bottom still counts as support (cardboard compresses).
    pub support_tolerance_mm: i32,
    pub clearance_mm: i32,
    pub use_fragility: bool,
    pub use_max_top_load: bool,
    /// Allowed load, kg per m² of the top face, for fragility classes 1..=10.
    pub top_load_pressure_kg_m2: Vec<f64>,
    pub use_lateral_stability: bool,
    /// Tilt angle a free-standing stack must survive.
    pub tilt_angle_deg: f64,
    /// Side counts as braced when contact covers at least this share of the face.
    pub lateral_min_contact_ratio: f64,
    /// Neighbours closer than this still count as touching.
    pub lateral_gap_mm: i32,
    /// A box counts as braced when this many of its 4 sides are in contact.
    pub lateral_min_braced_sides: u8,
    pub target_compactness: f64,
    /// Hard limit on search time.
    pub time_limit_seconds: f64,
    /// Scale the time with the task: base + per_type × distinct box types + per_item × items,
    /// capped by the hard limit. Mixed loads are harder than many identical boxes.
    pub auto_time: bool,
    pub auto_time_base_seconds: f64,
    pub auto_time_per_type_seconds: f64,
    pub auto_time_per_item_seconds: f64,
    /// Stop when the best has not improved for this share of the budget…
    pub stagnation_fraction: f64,
    /// …but never sooner than this.
    pub min_stagnation_seconds: f64,
    /// Share of the time reserved for repacking each place on its own at the end.
    pub polish_fraction: f64,
    pub random_seed: Option<u64>,
}

impl Default for PackRule {
    fn default() -> Self {
        PackRule {
            min_support_ratio: 0.70,
            support_tolerance_mm: 5,
            clearance_mm: 0,
            use_fragility: true,
            use_max_top_load: true,
            top_load_pressure_kg_m2: default_pressure_table(),
            use_lateral_stability: true,
            tilt_angle_deg: 20.0,
            lateral_min_contact_ratio: 0.20,
            lateral_gap_mm: 20,
            lateral_min_braced_sides: 1,
            target_compactness: 0.85,
            time_limit_seconds: 180.0,
            auto_time: true,
            auto_time_base_seconds: 20.0,
            auto_time_per_type_seconds: 0.5,
            auto_time_per_item_seconds: 0.05,
            stagnation_fraction: 0.35,
            min_stagnation_seconds: 5.0,
            polish_fraction: 0.25,
            random_seed: None,
        }
    }
}

/// 200 kg/m² for class 1 rising linearly to 3000 kg/m² for class 10.
pub fn default_pressure_table() -> Vec<f64> {
    (0..10).map(|i| 200.0 + i as f64 * (3000.0 - 200.0) / 9.0).collect()
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PackingRequest {
    pub items: Vec<Item>,
    pub packing_place_mode: PackingPlaceMode,
    pub available_packing_places: Vec<PackingPlace>,
    #[serde(default)]
    pub pack_rule: PackRule,
}

// ---------------------------------------------------------------- output

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PlacedItem {
    pub item_id: String,
    pub sku: String,
    pub bin_id: String,
    pub x: i32,
    pub y: i32,
    pub z: i32,
    /// Sizes after rotation.
    pub width: i32,
    pub depth: i32,
    pub height: i32,
    /// Which original sides map to X, Y, Z, e.g. "DWH".
    pub rotation: String,
    pub weight: f64,
    pub fragility: u8,
    pub max_top_load: f64,
    pub support_ratio: f64,
    pub received_top_load: f64,
    pub supported_by: Vec<String>,
    pub supported_items: Vec<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PackedBin {
    pub bin_id: String,
    pub place_id: String,
    pub place_name: String,
    pub place_type: PackingPlaceType,
    pub width: i32,
    pub depth: i32,
    pub height: i32,
    pub base_z: i32,
    pub total_weight: f64,
    pub item_weight: f64,
    pub pallet_weight: f64,
    pub used_height: i32,
    pub utilization: f64,
    pub compactness: f64,
    pub center_of_mass: [f64; 3],
    pub cg_offset_mm: f64,
    pub stability_score: f64,
    pub max_overhang_mm: i32,
    pub placed_items: Vec<PlacedItem>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum UnplacedReason {
    ItemTooLarge,
    ItemTooHeavy,
    InsufficientSpace,
    NoStablePosition,
    MaxTopLoadConflict,
    FragilityConflict,
    BinWeightLimit,
    HeightLimit,
    TimeLimit,
    NoFeasibleBin,
}

impl UnplacedReason {
    pub fn message_ru(self) -> &'static str {
        match self {
            UnplacedReason::ItemTooLarge => "коробка больше места погрузки",
            UnplacedReason::ItemTooHeavy => "коробка тяжелее допустимой нагрузки места",
            UnplacedReason::InsufficientSpace => "не хватило свободного места",
            UnplacedReason::NoStablePosition => "нет устойчивого положения",
            UnplacedReason::MaxTopLoadConflict => "нижние коробки не выдержат нагрузку",
            UnplacedReason::FragilityConflict => "некуда поставить по правилу хрупкости",
            UnplacedReason::BinWeightLimit => "превышен допустимый вес",
            UnplacedReason::HeightLimit => "не хватает высоты",
            UnplacedReason::TimeLimit => "не хватило времени расчёта",
            UnplacedReason::NoFeasibleBin => "нет подходящего места погрузки",
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct UnplacedItem {
    pub item_id: String,
    pub sku: String,
    pub reason: UnplacedReason,
    pub message: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Severity {
    Info,
    Warning,
    Error,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Warning {
    pub code: String,
    pub severity: Severity,
    pub message: String,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Diagnostics {
    pub calculation_time: f64,
    /// Seconds until the first valid solution existed (anytime behaviour, ТЗ §31).
    pub first_solution_time: f64,
    pub iterations: u64,
    pub solutions_checked: u64,
    pub valid_solution_found: bool,
    pub random_seed: u64,
    pub best_found_at_iteration: u64,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Summary {
    pub items_total: usize,
    pub items_placed: usize,
    pub items_unplaced: usize,
    pub skus_unplaced: usize,
    pub unplaced_volume_m3: f64,
    pub unplaced_weight_kg: f64,
    pub bins_used: usize,
    pub utilization: f64,
    pub compactness: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Violation {
    pub code: String,
    pub bin_id: String,
    pub item_id: String,
    pub message: String,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct ValidationReport {
    pub valid: bool,
    pub violations: Vec<Violation>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PackingResult {
    pub bins: Vec<PackedBin>,
    pub unplaced: Vec<UnplacedItem>,
    pub warnings: Vec<Warning>,
    pub summary: Summary,
    pub validation: ValidationReport,
    pub diagnostics: Diagnostics,
}
