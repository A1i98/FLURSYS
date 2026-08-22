//! Solver-independent analysis intent and executable solver plans.
//!
//! A project records what the engineer wants to analyse. `ExecutionPlan` is
//! created only after project data has been validated against the capabilities
//! of a concrete numerical backend.

pub mod editor;
pub mod examples;
pub mod mesh_viewport;
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
