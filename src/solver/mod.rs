mod incompressible;

pub use incompressible::{
    ConvectionScheme, IncompressibleSolver, PressureSolverKind, PressureVelocityCoupling,
    RunSummary, SimulationConfig,
};
