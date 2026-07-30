//! FLURSYS is a Rust-native scientific simulation system.
//!
//! This crate contains the initial numerical foundation and fluid-simulation
//! capabilities. Its public contracts will evolve as the broader system is
//! designed and validated.

pub mod cases;
pub mod field;
pub mod grid;
pub mod mesh;
pub mod output;
pub mod physics;
pub mod preprocess;
pub mod project;
pub mod runtime;
pub mod solver;

pub use cases::{Case, CaseKind};
pub use mesh::{ExtrudedMesh3D, StructuredMesh2D};
pub use physics::{
    BuoyancyModel, EnergyModel, PhysicsSettings, ThermalBoundaryCondition, ThermalSettings,
};
pub use preprocess::{
    BoundaryCondition, BoundaryConditionKind, BoundaryFace, GeometryDimension, GeometryFeature,
    GeometryFeatureKind, GeometryModel, GeometryPart, GeometryPartKind, GeometrySketch,
    MeshSettings, MeshTopology, PreprocessingModel, SketchAxis, SketchDimension,
    SketchDimensionKind, SketchEntity, SketchEntityKind, SketchPlane, SketchProfileKind,
    SolverBoundaryOverrides,
};
pub use project::{Project, ProjectCase, ProjectCoupling, ProjectPressureSolver, ProjectSolver};
pub use solver::{
    ConvectionScheme, FieldUpdate, IncompressibleSolver, PressureSolverKind,
    PressureVelocityCoupling, RunSummary, SimulationConfig, SolverStep,
};
