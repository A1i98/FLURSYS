//! FLURSYS is a Rust-native scientific simulation system.
//!
//! This crate contains the initial numerical foundation and fluid-simulation
//! capabilities. Its public contracts will evolve as the broader system is
//! designed and validated.

pub mod cases;
pub mod field;
pub mod fields;
pub mod geometry;
pub mod grid;
pub mod io;
pub mod linear;
pub mod mesh;
pub mod mesh_artifact;
pub mod meshing;
pub mod momentum;
pub mod numerics;
pub mod output;
pub mod physics;
pub mod poisson;
pub mod preprocess;
pub mod project;
pub mod runtime;
pub mod simple;
pub mod solver;
pub mod unstructured_incompressible;
pub mod workbench;

pub use cases::{Case, CaseKind};
pub use fields::{CellField, FaceField, FieldError};
pub use geometry::{
    BodyId, BoxEntities, CircleHoleEntities, EdgeGeometry, EdgeId, FaceId, GeometryBody,
    GeometryBodyRepresentation, GeometryEdge, GeometryError, GeometryFace,
    GeometryFaceRepresentation, GeometryRevision, GeometryTopology, GeometryVertex, OrientedEdge,
    RectangleEntities, VertexId,
};
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
pub use meshing::{
    GeneratedMesh, GeometryGmshExport, GeometryToGmshMap, GmshExecutable, GmshGeoDocument,
    GmshGeometryExporter, GmshMeshOptions, GmshMesher, GmshMeshingReport, GmshVersion,
    MeshingError,
};
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
pub use simple::{
    assemble_pressure_correction, continuity_rms, correct_cell_velocity_from_gradient,
    correct_face_flux, initial_face_flux, momentum_inverse_diagonal, pressure_face_coefficients,
    rhie_chow_predicted_flux, solve_pressure_correction, solve_simple, PressureCorrectionSystem,
    SimpleError, SimpleOptions, SimpleReport, SimpleState,
};
pub use solver::{
    ConvectionScheme, FieldUpdate, IncompressibleSolver, LidDrivenCavity3DConfig,
    LidDrivenCavity3DSolver, PressureSolverKind, PressureVelocityCoupling, RunSummary,
    RunSummary3D, SimulationConfig, SolverStep, TimeStepSettings,
};
pub use unstructured_incompressible::{
    patch_flux, solve_incompressible, IncompressibleBoundaryCondition, IncompressibleCase,
    IncompressibleCaseError, IncompressibleInitialConditions, IncompressibleMaterial,
    IncompressibleSolution, IncompressibleSolveError, IncompressibleSolveReport,
    IncompressibleSolveStatus, IncompressibleSolverOptions, ResolvedIncompressibleBoundaries,
};
pub use workbench::{
    build_example, example_descriptor, example_descriptors, expectations, verify_solution,
    AnalysisDimension, AnalysisKind, ExampleCheck, ExampleExpectations, ExampleProjectDescriptor,
    ExampleProjectError, ExampleProjectId, ExampleVerificationReport, ExecutionPlan,
    GeometryEditorState, GeometrySelectionTarget, GeometryTool, MeshQualityMetric,
    MeshQualityValues, MeshRenderCache, MeshSelection, MeshSelectionTarget, NamedSelection,
    NamedSelectionError, NamedSelectionStore, PreviewPrimitive, RenderRange, SolveStatus,
    SolverBackend, ViewTransform, WorkbenchAnalysis, WorkbenchError, WorkbenchSession,
};
