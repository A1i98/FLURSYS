//! Solver-independent analysis intent and executable solver plans.
//!
//! A project records what the engineer wants to analyse. `ExecutionPlan` is
//! created only after project data has been validated against the capabilities
//! of a concrete numerical backend.

pub mod editor;
pub mod examples;
pub mod mesh_viewport;
pub mod picking;
pub mod project;
pub mod results;
pub mod selection;
pub mod session;

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
    discover_templates_from_roots, finalize_workspace_run, load_recent_projects, load_workspace,
    load_workspace_result, recovery_is_newer, save_case_template, save_recent_projects,
    save_workspace, BoundaryAssignment, CaseTemplateManifest, PhysicalBoundaryCondition,
    ProjectDocumentError, RecentProjectEntry, RecentProjects, RunRecord, RunStatus,
    TemplateExpectations, WorkbenchMaterial, WorkbenchMeshSettings, WorkbenchProject,
    WorkbenchSolverSettings, PROJECT_DOCUMENT_FILE, RECENT_PROJECTS_FILE,
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

use crate::{LidDrivenCavity3DConfig, SimulationConfig};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum AnalysisDimension {
    #[default]
    TwoD,
    ThreeD,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum AnalysisKind {
    #[default]
    IncompressibleFlow,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct WorkbenchAnalysis {
    pub dimension: AnalysisDimension,
    pub kind: AnalysisKind,
}

impl WorkbenchAnalysis {
    pub fn validate(&self) -> Result<(), String> {
        match self.kind {
            AnalysisKind::IncompressibleFlow => Ok(()),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SolverBackend {
    StructuredIncompressible2D,
    StructuredCavity3D,
}

#[derive(Clone, Debug)]
pub enum ExecutionPlan {
    StructuredIncompressible2D(Box<SimulationConfig>),
    StructuredCavity3D(LidDrivenCavity3DConfig),
}

impl ExecutionPlan {
    pub fn backend(&self) -> SolverBackend {
        match self {
            Self::StructuredIncompressible2D(_) => SolverBackend::StructuredIncompressible2D,
            Self::StructuredCavity3D(_) => SolverBackend::StructuredCavity3D,
        }
    }

    pub fn capability_summary(&self) -> &'static str {
        match self {
            Self::StructuredIncompressible2D(_) => {
                "structured 2D incompressible flow with the selected case and supported boundaries"
            }
            Self::StructuredCavity3D(_) => {
                "structured 3D lid-driven cavity with no CAD solids and case-default walls"
            }
        }
    }
}
