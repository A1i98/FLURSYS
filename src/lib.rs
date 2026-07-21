//! FLURSYS is a Rust-native scientific simulation system.
//!
//! This crate contains the initial numerical foundation and fluid-simulation
//! capabilities. Its public contracts will evolve as the broader system is
//! designed and validated.

pub mod cases;
pub mod field;
pub mod grid;
pub mod output;
pub mod solver;

pub use cases::{Case, CaseKind};
pub use solver::{
    ConvectionScheme, IncompressibleSolver, PressureSolverKind, PressureVelocityCoupling,
    RunSummary, SimulationConfig,
};
