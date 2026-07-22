//! FLURSYS is a Rust-native scientific simulation system.
//!
//! This crate contains the initial numerical foundation and fluid-simulation
//! capabilities. Its public contracts will evolve as the broader system is
//! designed and validated.

pub mod cases;
pub mod field;
pub mod grid;
pub mod output;
pub mod preprocess;
pub mod project;
pub mod runtime;
pub mod solver;

pub use cases::{Case, CaseKind};
pub use preprocess::{
    BoundaryCondition, BoundaryConditionKind, BoundaryFace, GeometryDimension, GeometryModel,
    GeometryPart, GeometryPartKind, MeshSettings, MeshTopology, PreprocessingModel,
    SolverBoundaryOverrides,
};
pub use project::{Project, ProjectCase, ProjectCoupling, ProjectPressureSolver, ProjectSolver};
pub use solver::{
    ConvectionScheme, FieldUpdate, IncompressibleSolver, PressureSolverKind,
    PressureVelocityCoupling, RunSummary, SimulationConfig, SolverStep,
};
