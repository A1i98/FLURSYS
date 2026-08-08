mod incompressible;
mod incompressible_3d;

pub use incompressible::{
    ConvectionScheme, FieldUpdate, IncompressibleSolver, PressureSolverKind,
    PressureVelocityCoupling, RunSummary, SimulationConfig, SolverStep, TimeStepSettings,
};
pub use incompressible_3d::{LidDrivenCavity3DConfig, LidDrivenCavity3DSolver, RunSummary3D};
