//! FLURSYS is a Rust-native scientific simulation system.
//!
//! This crate contains the initial numerical foundation and fluid-simulation
//! capabilities. Its public contracts will evolve as the broader system is
//! designed and validated.

pub mod cases;
pub mod field;
pub mod fields;
pub mod grid;
pub mod io;
pub mod linear;
pub mod mesh;
pub mod mesh_artifact;
pub mod momentum;
pub mod numerics;
pub mod output;
pub mod physics;
pub mod poisson;
pub mod preprocess;
pub mod project;
pub mod runtime;
pub mod solver;
pub mod workbench;

pub use cases::{Case, CaseKind};
pub use fields::{CellField, FaceField, FieldError};
pub use io::gmsh::{load_gmsh, parse_gmsh, GmshError};
pub use linear::{
    axpy, bicgstab, cg, copy_into, dot, norm_inf, norm_l2, pcg, residual, residual_into, scale,
    CsrBuilder, CsrMatrix, JacobiPreconditioner, LinearAlgebraError, LinearSolveReport,
    LinearSolverOptions, LinearSolverStatus,
};
pub use mesh::{
    BoundaryPatch, BoundaryType, Cell, CellDefinition, ExtrudedMesh3D, Face, MeshDimension,
    MeshError, MeshId, MeshQualityReport as UnstructuredMeshQualityReport, MeshStatistics, Point,
    StructuredMesh2D, UnstructuredMesh, Vec3,
};
pub use mesh_artifact::{MeshQualityReport, StructuredMeshArtifact};
pub use momentum::{
    assemble_momentum_component, constant_body_force, momentum_component_field,
    pressure_gradient_source, solve_momentum_component, solve_momentum_velocity, MomentumComponent,
    MomentumError, MomentumOptions, MomentumSystem, ResolvedVelocityBoundaryConditions,
    VelocityBoundaryCondition,
};
pub use numerics::{
    divergence, divergence_into, face_flux, face_flux_into, gauss_gradient_from_faces,
    gauss_gradient_from_faces_into, integrated_diffusion, integrated_diffusion_into,
    integrated_diffusion_with_stencil, integrated_diffusion_with_stencil_into,
    integrated_divergence, integrated_divergence_into, interpolate_diffusivity,
    interpolate_diffusivity_into, interpolate_scalar, interpolate_scalar_into,
    interpolate_vector_into, laplacian, laplacian_into, laplacian_with_stencil,
    laplacian_with_stencil_into, least_squares_gradient, least_squares_gradient_into,
    non_orthogonal_decomposition, orthogonal_laplacian_into,
    projection_non_orthogonal_decomposition, DiffusionOptions, Diffusivity,
    DiffusivityInterpolation, GradientScheme, LeastSquaresGradientStencil, NonOrthogonalCorrection,
    NonOrthogonalDecomposition, NumericsError, ResolvedScalarBoundaryConditions,
    ScalarBoundaryCondition, ScalarBoundaryValue,
};
pub use physics::{
    BuoyancyModel, EnergyModel, PhysicsSettings, ThermalBoundaryCondition, ThermalSettings,
};
pub use poisson::{
    assemble_poisson, solve_poisson, solve_poisson_correction_sweeps, PoissonError,
    PoissonLinearSolver, PoissonOptions, PoissonReference, PoissonSystem,
};
pub use preprocess::{
    BoundaryCondition, BoundaryConditionKind, BoundaryFace, GeometryDimension, GeometryFeature,
    GeometryFeatureKind, GeometryModel, GeometryPart, GeometryPartKind, GeometryRegion,
    GeometryRegionKind, GeometrySketch, MeshSettings, MeshTopology, PreprocessingModel, SketchAxis,
    SketchConstraint, SketchConstraintKind, SketchDimension, SketchDimensionKind, SketchEntity,
    SketchEntityKind, SketchPlane, SketchProfileKind, SolverBoundaryOverrides,
};
pub use project::{
    Project, ProjectCase, ProjectConvectionScheme, ProjectCoupling, ProjectPressureSolver,
    ProjectSolver, ProjectTimeStepSettings,
};
pub use solver::{
    ConvectionScheme, FieldUpdate, IncompressibleSolver, LidDrivenCavity3DConfig,
    LidDrivenCavity3DSolver, PressureSolverKind, PressureVelocityCoupling, RunSummary,
    RunSummary3D, SimulationConfig, SolverStep, TimeStepSettings,
};
pub use workbench::{
    AnalysisDimension, AnalysisKind, ExecutionPlan, SolverBackend, WorkbenchAnalysis,
};
