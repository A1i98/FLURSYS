//! Canonical project, meshing, solver orchestration and results shared by GUI and headless clients.

pub mod boundary_reports;
pub mod budgets;
pub mod case_format;
pub mod catalogue;
pub mod checkpoint;
pub mod dfg;
pub mod diagnostics;
pub mod editor;
pub mod engineering;
pub mod examples;
pub mod field_mapping;
pub mod force_analysis;
pub mod initialization;
pub mod mesh_report;
pub mod mesh_storage;
pub mod mesh_viewport;
pub mod picking;
pub mod plane_integrals;
pub mod plane_monitor;
pub mod plane_statistics;
pub mod project;
mod result_slice;
pub mod results;
pub mod selection;
pub mod series_plot;
pub mod session;
pub mod solve_review;
pub mod wall_reports;

pub use editor::{GeometryEditorState, GeometryTool, PreviewPrimitive, ViewTransform};
pub use examples::{
    build_example, descriptor as example_descriptor, example_descriptors, expectations,
    verify_solution, ExampleCheck, ExampleExpectations, ExampleProjectDescriptor,
    ExampleProjectError, ExampleProjectId, ExampleVerificationReport,
};
pub use mesh_viewport::{
    MeshQualityMetric, MeshQualityValues, MeshRenderCache, MeshSelection, MeshSelectionTarget,
    RenderRange,
};
pub use picking::{CadPickMode, CadSelectionState};
pub use project::{
    autosave_workspace, delete_workspace_run, discard_workspace_recovery, discover_templates,
    discover_templates_from_roots, finalize_workspace_run, finalize_workspace_run_continuing,
    load_recent_projects, load_workspace, load_workspace_result, recovery_is_newer,
    save_case_template, save_recent_projects, save_workspace, BoundaryAssignment,
    BoundaryLayerControl, BoxRefinement, CaseTemplateManifest, EdgeDivisionControl, LocalMeshSize,
    MeshControlTarget, MeshMethod, MeshRecipe, MeshTuning, PhysicalBoundaryCondition,
    ProjectDocumentError, RecentProjectEntry, RecentProjects, RunRecord, RunStatus,
    TemplateExpectations, ThresholdRefinement, WorkbenchMaterial, WorkbenchMeshSettings,
    WorkbenchProject, WorkbenchSolverSettings, PROJECT_DOCUMENT_FILE, RECENT_PROJECTS_FILE,
    WORKBENCH_PROJECT_FORMAT_VERSION,
};
pub use results::streamlines::{
    StreamlineDirection, StreamlineField, StreamlineOptions, StreamlinePath,
};
pub use results::{
    load_legacy_vtk_result, parse_legacy_vtk_result, ResultDataError, ResultDataset,
    ResultFieldKind, ResultProbe, ResultRenderCache,
};
pub use selection::{
    GeometrySelectionTarget, NamedSelection, NamedSelectionError, NamedSelectionStore,
};
pub use session::{SolveStatus, WorkbenchError, WorkbenchSession};
