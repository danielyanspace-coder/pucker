//! Pucker packing engine.
//!
//! Entry point: [`pack`]. The search engine and the [`validator`] are separate on purpose (ТЗ §38).

pub mod bin_state;
pub mod geometry;
pub mod metrics;
pub mod model;
pub mod precheck;
pub mod prep;
mod packer;
mod rect2d;
pub mod rng;
pub mod validator;

pub use model::*;
pub use packer::pack;
