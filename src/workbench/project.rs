//! Versioned, portable project-workspace documents for the unstructured workbench.
//!
//! `WorkbenchProject` intentionally contains user intent and persistent run
//! metadata only. Meshes, VTK fields, renderer caches, and worker handles are
//! project-local derived artifacts rather than JSON payloads.

use super::{
    load_legacy_vtk_result, GeometrySelectionTarget, NamedSelection, ResultDataset, SolveStatus,
    WorkbenchSession,
};
use crate::{GeometryTopology, MeshDimension, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fs::{self, File};
use std::io::Write;
use std::path::{Component, Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

pub const WORKBENCH_PROJECT_FORMAT_VERSION: u32 = 2;
pub const PROJECT_DOCUMENT_FILE: &str = "project.json";
pub const RECENT_PROJECTS_FILE: &str = "recent-projects.json";

/// Small application-owned index for workspace shortcuts. It contains only a
/// canonical path, display name, and last-open timestamp; portable project
/// intent remains exclusively in each workspace's `project.json`.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct RecentProjects {
    pub entries: Vec<RecentProjectEntry>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecentProjectEntry {
    pub workspace: PathBuf,
    pub display_name: String,
    pub last_open_unix_ms: u128,
}

impl RecentProjects {
    pub fn record(&mut self, workspace: &Path, display_name: impl Into<String>) {
        let workspace = fs::canonicalize(workspace).unwrap_or_else(|_| workspace.to_path_buf());
        self.entries.retain(|entry| entry.workspace != workspace);
        self.entries.insert(
            0,
            RecentProjectEntry {
                workspace,
                display_name: display_name.into(),
                last_open_unix_ms: now_ms(),
            },
        );
        self.entries.truncate(10);
    }

    pub fn remove_missing(&mut self) {
        self.entries.retain(|entry| entry.workspace.is_dir());
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum PhysicalBoundaryCondition {
    TotalPressure {
        pressure: f64,
    },
    ParabolicInlet {
        velocity: Vec3,
        axis: usize,
        center: f64,
        half_width: f64,
    },
    FlowRateInlet {
        rate: f64,
        mass: bool,
    },
    PressureOutletBackflow {
        pressure: f64,
        velocity: Vec3,
    },
    NoSlipWall,
    Slip,
    Symmetry,
    FarField {
        velocity: Vec3,
        pressure: f64,
    },
    MovingWall {
        velocity: Vec3,
    },
    VelocityInlet {
        velocity: Vec3,
    },
    PressureOutlet {
        pressure: f64,
    },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BoundaryAssignment {
    pub selection: String,
    pub condition: PhysicalBoundaryCondition,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct WorkbenchMeshSettings {
    #[serde(default = "default_cpu_threads")]
    pub threads: usize,
    pub dimension: MeshDimension,
    pub global_size: f64,
    pub min_size: f64,
    pub max_size: f64,
    pub element_order: u8,
}

impl Default for WorkbenchMeshSettings {
    fn default() -> Self {
        Self {
            threads: 1,
            dimension: MeshDimension::TwoD,
            global_size: 0.1,
            min_size: 0.05,
            max_size: 0.2,
            element_order: 1,
        }
    }
}

/// Persisted advanced meshing intent. Generated meshes remain disposable
/// artifacts; changing this recipe invalidates only the current mesh/solution.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct MeshRecipe {
    pub tuning: MeshTuning,
    pub edge_divisions: Vec<EdgeDivisionControl>,
    pub regions: Vec<BoxRefinement>,
    pub local_sizes: Vec<LocalMeshSize>,
    pub refinements: Vec<ThresholdRefinement>,
    pub boundary_layers: Vec<BoundaryLayerControl>,
}

/// Analytic 2D boundary curves: exact element count and optional grading.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EdgeDivisionControl {
    pub id: u32,
    pub name: String,
    pub targets: Vec<MeshControlTarget>,
    pub cells: u32,
    pub progression: f64,
    pub reverse: bool,
    #[serde(default)]
    pub symmetric: bool,
    pub enabled: bool,
}
impl EdgeDivisionControl {
    pub fn validate(&self) -> Result<(), String> {
        if self.name.trim().is_empty()
            || self.targets.is_empty()
            || !(1..=10000).contains(&self.cells)
            || !self.progression.is_finite()
            || if self.symmetric {
                !(0.01..=1.0).contains(&self.progression)
            } else {
                !(0.5..=2.0).contains(&self.progression)
                    || self.progression.ln().abs() * f64::from(self.cells.saturating_sub(1))
                        > 1000.0_f64.ln()
            }
        {
            return Err("Edge divisions require a scope, 1–10000 cells; progression 0.5–2 with end ratio ≤1000, or symmetric Bump coefficient 0.01–1".into());
        }
        Ok(())
    }
}

/// Axis-aligned volume sizing; independent of boundary naming.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BoxRefinement {
    pub id: u32,
    pub name: String,
    pub minimum: crate::Vec3,
    pub maximum: crate::Vec3,
    pub size: f64,
    pub transition: f64,
    pub enabled: bool,
}
impl BoxRefinement {
    pub fn validate(&self) -> Result<(), String> {
        let a = self.minimum;
        let b = self.maximum;
        if ![a.x, a.y, a.z, b.x, b.y, b.z, self.size, self.transition]
            .iter()
            .all(|v| v.is_finite())
            || a.x >= b.x
            || a.y >= b.y
            || a.z >= b.z
            || self.size <= 0.0
            || self.transition < 0.0
            || self.name.trim().is_empty()
        {
            return Err("Box refinement requires finite min < max on every axis, positive size and nonnegative transition".into());
        }
        Ok(())
    }
}

/// Explicit topology method: counts are cells, not Gmsh node counts.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum MeshMethod {
    #[default]
    Unstructured,
    QuadDominant,
    MappedQuad,
    OGrid,
    HexBlock,
    SweepPrism,
    SweepHex,
}

/// Explicit Gmsh controls shared by the UI, persistence and geometry export.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct MeshTuning {
    pub method: MeshMethod,
    pub max_size_growth: Option<f64>,
    pub divisions: [u32; 3],
    /// Cell-to-cell growth along mapped I/J or O-grid radial direction.
    pub progression: [f64; 2],

    pub algorithm_2d: u8,
    pub algorithm_3d: u8,
    pub curvature_points: u32,
    pub smoothing_steps: u32,
    pub optimize: bool,
    pub optimize_netgen: bool,
    pub optimize_threshold: f64,
    pub extend_boundary_sizes: bool,
}
impl Default for MeshTuning {
    fn default() -> Self {
        Self {
            max_size_growth: None,
            method: MeshMethod::Unstructured,
            divisions: [40, 40, 10],
            progression: [1.0, 1.0],
            algorithm_2d: 6,
            algorithm_3d: 1,
            curvature_points: 32,
            smoothing_steps: 10,
            optimize: true,
            optimize_netgen: false,
            optimize_threshold: 0.3,
            extend_boundary_sizes: true,
        }
    }
}
impl MeshTuning {
    pub fn validate(&self) -> Result<(), String> {
        if self
            .max_size_growth
            .is_some_and(|g| !g.is_finite() || !(1.01..=2.0).contains(&g))
        {
            return Err("Distance-field growth limit must be 1.01–2, or left empty".into());
        }
        if self.divisions.iter().any(|n| !(1..=10000).contains(n))
            || self
                .progression
                .iter()
                .any(|r| !r.is_finite() || !(0.5..=2.0).contains(r))
        {
            return Err(
                "Mapped divisions must be 1–10000; progression must be finite and 0.5–2".into(),
            );
        }
        let cells = self
            .divisions
            .iter()
            .map(|n| u64::from(*n))
            .product::<u64>();
        let estimate = match self.method {
            MeshMethod::HexBlock | MeshMethod::SweepHex => cells,
            MeshMethod::OGrid => 4 * u64::from(self.divisions[0]) * u64::from(self.divisions[1]),
            _ => u64::from(self.divisions[0]) * u64::from(self.divisions[1]),
        };
        if matches!(
            self.method,
            MeshMethod::MappedQuad
                | MeshMethod::OGrid
                | MeshMethod::HexBlock
                | MeshMethod::SweepHex
        ) && estimate > 20_000_000
        {
            return Err(
                "Requested structured mesh exceeds 20 million cells; reduce divisions".into(),
            );
        }
        for axis in 0..2 {
            if self.progression[axis].ln().abs() * f64::from(self.divisions[axis].saturating_sub(1))
                > 1000.0_f64.ln()
            {
                return Err("Mapped progression creates an end-to-end size ratio above 1000; reduce growth or divisions".into());
            }
        }
        if ![1, 5, 6].contains(&self.algorithm_2d) || ![1, 4, 10].contains(&self.algorithm_3d) {
            return Err("Unsupported triangle/tetrahedron meshing algorithm".into());
        }
        if self.curvature_points > 1000 || self.smoothing_steps > 100 {
            return Err("Curvature sampling must be 0–1000; smoothing steps 0–100".into());
        }
        if !self.optimize_threshold.is_finite() || !(0.0..=1.0).contains(&self.optimize_threshold) {
            return Err("Gmsh optimization quality threshold must be between 0 and 1".into());
        }
        Ok(())
    }
}

/// Stable project targets; backend Gmsh entity tags are resolved only while a
/// deterministic geometry export is being generated.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum MeshControlTarget {
    Geometry(GeometrySelectionTarget),
    NamedSelection(String),
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LocalMeshSize {
    pub id: u32,
    pub name: String,
    pub targets: Vec<MeshControlTarget>,
    pub size: f64,
    pub enabled: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ThresholdRefinement {
    pub id: u32,
    pub name: String,
    pub targets: Vec<MeshControlTarget>,
    pub sampling: u32,
    pub size_min: f64,
    pub size_max: f64,
    pub distance_min: f64,
    pub distance_max: f64,
    pub enabled: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BoundaryLayerControl {
    pub id: u32,
    pub name: String,
    pub targets: Vec<MeshControlTarget>,
    pub first_layer_height: f64,
    pub growth_ratio: f64,
    pub layer_count: u32,
    pub enabled: bool,
}

impl BoundaryLayerControl {
    /// Requested geometric stack; Gmsh may truncate layers near collisions.
    pub fn layer_dimensions(&self) -> Result<(f64, f64), String> {
        if !self.first_layer_height.is_finite()
            || self.first_layer_height <= 0.0
            || !self.growth_ratio.is_finite()
            || !(1.0..=3.0).contains(&self.growth_ratio)
            || !(1..=64).contains(&self.layer_count)
        {
            return Err(
                "Inflation requires positive first height, growth 1–3 and 1–64 target layers"
                    .into(),
            );
        }
        let mut height = self.first_layer_height;
        let mut total = 0.0;
        for _ in 0..self.layer_count {
            total += height;
            height *= self.growth_ratio;
        }
        let last = height / self.growth_ratio;
        if !total.is_finite() || !last.is_finite() {
            return Err(
                "Inflation thickness overflow; reduce height, growth or layer count".into(),
            );
        }
        Ok((total, last))
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct WorkbenchMaterial {
    #[serde(default)]
    pub catalog: Option<crate::materials::MaterialSnapshot>,
    pub density: f64,
    pub kinematic_viscosity: f64,
}

impl Default for WorkbenchMaterial {
    fn default() -> Self {
        Self {
            catalog: None,
            density: 1.0,
            kinematic_viscosity: 0.01,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct WorkbenchSolverSettings {
    #[serde(default)]
    pub compute: crate::compute::ComputeOptions,
    #[serde(default)]
    pub pressure_coupling: crate::simple::PressureCoupling,
    #[serde(default)]
    pub volume_models: crate::volume_models::VolumeModels,
    #[serde(default)]
    pub cyclic_pairs: Vec<crate::periodic::TranslationalCyclicPair>,
    #[serde(default)]
    pub initial_profile: Option<super::initialization::InitialProfile>,
    #[serde(default)]
    pub compressible: Option<crate::compressible::Settings>,
    #[serde(default)]
    pub equations: Option<crate::equations::EquationSolvers>,
    #[serde(default)]
    pub boundary_profiles: crate::unstructured_incompressible::BoundaryProfiles,
    #[serde(default)]
    pub scripting: crate::scripting::Settings,
    #[serde(default)]
    pub thermal: crate::thermal::Options,
    #[serde(default)]
    pub time: crate::unstructured_incompressible::TimeIntegration,
    #[serde(default)]
    pub physical_end_time: Option<f64>,
    #[serde(default)]
    pub force_analysis: crate::workbench::force_analysis::Settings,
    #[serde(default)]
    pub turbulence: crate::turbulence::TurbulenceOptions,
    #[serde(default)]
    pub convection: crate::momentum::MomentumConvection,
    #[serde(default = "default_cpu_threads")]
    pub threads: usize,
    pub max_outer_iterations: usize,
    pub continuity_absolute_tolerance: f64,
    #[serde(default = "default_coupling_tolerance")]
    pub coupling_relative_tolerance: f64,
    pub velocity_relaxation: f64,
    pub pressure_relaxation: f64,
}

impl Default for WorkbenchSolverSettings {
    fn default() -> Self {
        Self {
            compute: Default::default(),
            equations: None,
            pressure_coupling: Default::default(),
            volume_models: Default::default(),
            cyclic_pairs: Vec::new(),
            initial_profile: None,
            compressible: None,
            boundary_profiles: Default::default(),
            scripting: Default::default(),
            thermal: crate::thermal::Options::default(),
            time: Default::default(),
            physical_end_time: None,
            force_analysis: Default::default(),
            turbulence: Default::default(),
            convection: crate::momentum::MomentumConvection::Upwind,
            threads: 0,
            max_outer_iterations: 1000,
            continuity_absolute_tolerance: 1.0e-10,
            coupling_relative_tolerance: default_coupling_tolerance(),
            velocity_relaxation: 0.7,
            pressure_relaxation: 0.3,
        }
    }
}

fn default_cpu_threads() -> usize {
    1
}

fn default_coupling_tolerance() -> f64 {
    1.0e-6
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum RunStatus {
    Converged,
    MaxIterations,
    Failed,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RunRecord {
    #[serde(default)]
    pub physical_time: Option<f64>,
    #[serde(default)]
    pub time_steps: usize,
    pub id: String,
    pub ordinal: u32,
    pub created_unix_ms: u128,
    pub status: RunStatus,
    pub mesh_identity: Option<u64>,
    pub iterations: Option<usize>,
    pub continuity_residual: Option<f64>,
    pub total_inflow: Option<f64>,
    pub total_outflow: Option<f64>,
    pub net_boundary_flux: Option<f64>,
    #[serde(default)]
    pub failure_diagnostic: Option<String>,
    pub report_path: PathBuf,
    pub solution_path: Option<PathBuf>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct WorkbenchProject {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mesh_asset: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub case_context: Option<super::case_format::CaseContext>,

    pub format_version: u32,
    pub project_id: String,
    pub name: String,
    /// Applied catalogue provenance history; current physical values remain explicit.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub catalog_history: Vec<serde_json::Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub scalar_transport: Option<crate::multiphysics::Options>,
    pub geometry: GeometryTopology,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cad_document: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cad_geometry_hash: Option<String>,
    #[serde(default)]
    pub plane_monitors: Vec<super::plane_monitor::FlowPlaneMonitor>,
    #[serde(default)]
    pub fluid_face: Option<crate::FaceId>,
    pub named_selections: Vec<NamedSelection>,
    pub mesh: WorkbenchMeshSettings,
    #[serde(default)]
    pub mesh_recipe: MeshRecipe,
    pub boundaries: Vec<BoundaryAssignment>,
    pub material: WorkbenchMaterial,
    pub solver: WorkbenchSolverSettings,
    #[serde(default)]
    pub runs: Vec<RunRecord>,
}

impl Default for WorkbenchProject {
    fn default() -> Self {
        Self::blank("Untitled Project")
    }
}

impl WorkbenchProject {
    pub fn blank(name: impl Into<String>) -> Self {
        Self {
            mesh_asset: None,
            case_context: None,
            format_version: WORKBENCH_PROJECT_FORMAT_VERSION,
            project_id: unique_id("project"),
            name: name.into(),
            catalog_history: Vec::new(),
            scalar_transport: None,
            geometry: GeometryTopology::new(),
            cad_document: None,
            cad_geometry_hash: None,
            plane_monitors: Vec::new(),
            fluid_face: None,
            named_selections: Vec::new(),
            mesh: WorkbenchMeshSettings::default(),
            mesh_recipe: MeshRecipe::default(),
            boundaries: Vec::new(),
            material: WorkbenchMaterial::default(),
            solver: WorkbenchSolverSettings::default(),
            runs: Vec::new(),
        }
    }

    pub fn validate(&self) -> Result<(), ProjectDocumentError> {
        self.solver
            .time
            .validate()
            .map_err(ProjectDocumentError::Invalid)?;
        self.solver
            .volume_models
            .validate()
            .map_err(ProjectDocumentError::Invalid)?;
        self.solver
            .pressure_coupling
            .validate(self.solver.time.is_transient())
            .map_err(ProjectDocumentError::Invalid)?;
        self.solver
            .turbulence
            .validate()
            .map_err(ProjectDocumentError::Invalid)?;
        if self
            .mesh_asset
            .as_ref()
            .is_some_and(|s| s.len() != 64 || !s.bytes().all(|b| b.is_ascii_hexdigit()))
        {
            return Err(ProjectDocumentError::Invalid(
                "Invalid mesh asset identifier".into(),
            ));
        }

        let mut plane_ids = BTreeSet::new();
        for plane in &self.plane_monitors {
            plane.validate().map_err(ProjectDocumentError::Invalid)?;
            if !plane_ids.insert(&plane.id) {
                return Err(ProjectDocumentError::Invalid(
                    "Duplicate flow monitor plane ID".into(),
                ));
            }
        }
        if self.format_version != WORKBENCH_PROJECT_FORMAT_VERSION {
            return Err(ProjectDocumentError::UnsupportedVersion {
                found: self.format_version,
                supported: WORKBENCH_PROJECT_FORMAT_VERSION,
            });
        }
        if self.project_id.trim().is_empty() || self.name.trim().is_empty() {
            return Err(ProjectDocumentError::Invalid(
                "project ID and name must be non-empty".into(),
            ));
        }
        if let Some(context) = &self.case_context {
            context
                .metadata
                .validate()
                .map_err(ProjectDocumentError::Invalid)?;
        }
        if let Some(profile) = &self.solver.initial_profile {
            profile.validate().map_err(ProjectDocumentError::Invalid)?;
        }
        self.geometry
            .validate()
            .map_err(|error| ProjectDocumentError::Invalid(error.to_string()))?;
        if !self.solver.coupling_relative_tolerance.is_finite()
            || self.solver.coupling_relative_tolerance <= 0.0
        {
            return Err(ProjectDocumentError::Invalid(
                "Momentum/velocity relative tolerance must be finite and positive".into(),
            ));
        }
        if self.fluid_face.is_some_and(|id| {
            self.geometry.face(id).is_none() || self.geometry.cad_owner(id).is_some()
        }) {
            return Err(ProjectDocumentError::Invalid(
                "The selected 2D fluid domain is missing or belongs to a solid".into(),
            ));
        }
        if !(self.mesh.global_size.is_finite()
            && self.mesh.global_size > 0.0
            && self.mesh.min_size.is_finite()
            && self.mesh.min_size > 0.0
            && self.mesh.max_size.is_finite()
            && self.mesh.max_size >= self.mesh.min_size
            && self.mesh.element_order == 1)
        {
            return Err(ProjectDocumentError::Invalid(
                "invalid Gmsh mesh settings".into(),
            ));
        }
        if !(self.material.density.is_finite()
            && self.material.density > 0.0
            && self.material.kinematic_viscosity.is_finite()
            && self.material.kinematic_viscosity >= 0.0)
        {
            return Err(ProjectDocumentError::Invalid(
                "invalid material settings".into(),
            ));
        }
        if self.solver.max_outer_iterations == 0
            || !self.solver.continuity_absolute_tolerance.is_finite()
            || self.solver.continuity_absolute_tolerance <= 0.0
            || !(0.0 < self.solver.velocity_relaxation && self.solver.velocity_relaxation <= 1.0)
            || !(0.0 < self.solver.pressure_relaxation && self.solver.pressure_relaxation <= 1.0)
        {
            return Err(ProjectDocumentError::Invalid(
                "invalid SIMPLE settings".into(),
            ));
        }
        self.solver
            .scripting
            .validate(self.solver.time.is_transient())
            .map_err(ProjectDocumentError::Invalid)?;
        for p in self.solver.boundary_profiles.values() {
            p.validate().map_err(ProjectDocumentError::Invalid)?;
        }
        if let Some(e) = &self.solver.equations {
            e.validate().map_err(ProjectDocumentError::Invalid)?;
        }
        self.solver
            .thermal
            .validate()
            .map_err(ProjectDocumentError::Invalid)?;
        if let Some(material) = &self.material.catalog {
            material.validate().map_err(ProjectDocumentError::Invalid)?;
        }
        self.mesh_recipe
            .tuning
            .validate()
            .map_err(ProjectDocumentError::Invalid)?;
        let mut names = BTreeSet::new();
        for selection in &self.named_selections {
            if selection.name.trim().is_empty()
                || !names.insert(&selection.name)
                || selection.targets.is_empty()
            {
                return Err(ProjectDocumentError::Invalid(
                    "invalid Named Selection".into(),
                ));
            }
            for target in &selection.targets {
                if !target_exists(&self.geometry, *target) {
                    return Err(ProjectDocumentError::DanglingSelection {
                        selection: selection.name.clone(),
                    });
                }
            }
        }
        validate_mesh_recipe(&self.mesh_recipe, &self.geometry, &names)?;
        for assignment in &self.boundaries {
            if !names.contains(&assignment.selection) {
                return Err(ProjectDocumentError::Invalid(format!(
                    "boundary assignment references unknown Named Selection {:?}",
                    assignment.selection
                )));
            }
            validate_boundary(&assignment.condition)?;
        }
        for run in &self.runs {
            validate_relative_artifact_path(&run.report_path)?;
            if let Some(path) = &run.solution_path {
                validate_relative_artifact_path(path)?;
            }
        }
        Ok(())
    }

    pub fn next_run(&self, status: RunStatus) -> RunRecord {
        let ordinal = self.runs.iter().map(|run| run.ordinal).max().unwrap_or(0) + 1;
        let directory = format!("runs/run-{ordinal:04}");
        RunRecord {
            physical_time: None,
            time_steps: 0,
            id: format!("run-{ordinal:04}"),
            ordinal,
            created_unix_ms: now_ms(),
            status,
            mesh_identity: None,
            iterations: None,
            continuity_residual: None,
            total_inflow: None,
            total_outflow: None,
            net_boundary_flux: None,
            failure_diagnostic: None,
            report_path: PathBuf::from(&directory).join("report.json"),
            solution_path: None,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CaseTemplateManifest {
    pub format_version: u32,
    pub id: String,
    pub name: String,
    pub category: String,
    pub description: String,
    #[serde(default)]
    pub capabilities: Vec<String>,
    #[serde(default)]
    pub expectations: TemplateExpectations,
    #[serde(default)]
    pub metadata: super::case_format::CaseMetadata,
    pub project_template: WorkbenchProject,
}

/// Human-readable, data-driven validation hints for a reusable case template.
/// These are not solver inputs and never affect execution.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct TemplateExpectations {
    pub closed_domain: bool,
    pub positive_streamwise_flow: bool,
    pub curved_boundary: bool,
    pub non_ideal_mesh_quality: bool,
    pub notes: String,
}

impl CaseTemplateManifest {
    pub fn validate(&self) -> Result<(), ProjectDocumentError> {
        if self.format_version != super::case_format::CASE_FORMAT_VERSION
            || self.id.trim().is_empty()
            || self.name.trim().is_empty()
        {
            return Err(ProjectDocumentError::Invalid(
                "invalid case template manifest".into(),
            ));
        }
        self.metadata
            .validate()
            .map_err(ProjectDocumentError::Invalid)?;
        self.project_template.validate()
    }
}

#[derive(Debug)]
pub enum ProjectDocumentError {
    Io(String),
    Json(String),
    UnsupportedVersion { found: u32, supported: u32 },
    Invalid(String),
    DanglingSelection { selection: String },
    UnsafeArtifactPath(PathBuf),
    Artifact(String),
}

impl std::fmt::Display for ProjectDocumentError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(error) => write!(f, "project I/O error: {error}"),
            Self::Json(error) => write!(f, "invalid project JSON: {error}"),
            Self::UnsupportedVersion { found, supported } => write!(
                f,
                "unsupported workbench project format {found}; supported version is {supported}"
            ),
            Self::Invalid(error) => write!(f, "invalid workbench project: {error}"),
            Self::DanglingSelection { selection } => write!(
                f,
                "Named Selection {selection:?} references geometry that does not exist"
            ),
            Self::UnsafeArtifactPath(path) => write!(
                f,
                "project artifact path must be relative and contained in the workspace: {}",
                path.display()
            ),
            Self::Artifact(error) => write!(f, "workbench artifact error: {error}"),
        }
    }
}
impl std::error::Error for ProjectDocumentError {}

pub fn save_workspace(
    workspace: &Path,
    project: &WorkbenchProject,
) -> Result<(), ProjectDocumentError> {
    project.validate()?;
    fs::create_dir_all(workspace).map_err(io_error)?;
    fs::create_dir_all(workspace.join("runs")).map_err(io_error)?;
    if let Some(context) = &project.case_context {
        super::case_format::write_assets(&workspace.join("case-assets"), &context.metadata.assets)
            .map_err(ProjectDocumentError::Invalid)?;
    }
    let destination = workspace.join(PROJECT_DOCUMENT_FILE);
    atomic_json_write(&destination, project)?;
    // JSON is portable and authoritative. Index failure must state that the
    // project was already saved rather than implying the user's data was lost.
    crate::database::Database::open(&workspace.join("workspace.sqlite3"))
        .and_then(|mut db| db.index_project(workspace, project))
        .map_err(|e| {
            ProjectDocumentError::Io(format!(
                "Project JSON saved, but SQLite metadata indexing failed: {e}"
            ))
        })?;
    let recovery = workspace.join("autosave").join(PROJECT_DOCUMENT_FILE);
    if recovery.exists() {
        fs::remove_file(recovery).map_err(io_error)?;
    }
    Ok(())
}

fn migrate_project_json(value: &mut serde_json::Value) -> Result<(), ProjectDocumentError> {
    let found = value
        .get("format_version")
        .and_then(serde_json::Value::as_u64)
        .ok_or_else(|| ProjectDocumentError::Json("missing project format_version".into()))?
        as u32;
    match found {
        WORKBENCH_PROJECT_FORMAT_VERSION => Ok(()),
        1 => {
            let object = value.as_object_mut().ok_or_else(|| {
                ProjectDocumentError::Json("project document must be a JSON object".into())
            })?;
            object.insert(
                "mesh_recipe".into(),
                serde_json::to_value(MeshRecipe::default())
                    .expect("default mesh recipe serializes"),
            );
            object.insert(
                "format_version".into(),
                serde_json::json!(WORKBENCH_PROJECT_FORMAT_VERSION),
            );
            Ok(())
        }
        _ => Err(ProjectDocumentError::UnsupportedVersion {
            found,
            supported: WORKBENCH_PROJECT_FORMAT_VERSION,
        }),
    }
}

pub fn load_workspace(workspace: &Path) -> Result<WorkbenchProject, ProjectDocumentError> {
    let path = workspace.join(PROJECT_DOCUMENT_FILE);
    let text = fs::read_to_string(&path).map_err(io_error)?;
    let mut value: serde_json::Value = serde_json::from_str(&text)
        .map_err(|error| ProjectDocumentError::Json(error.to_string()))?;
    migrate_project_json(&mut value)?;
    let project: WorkbenchProject = serde_json::from_value(value)
        .map_err(|error| ProjectDocumentError::Json(error.to_string()))?;
    project.validate()?;
    Ok(project)
}

/// Loads one persisted successful run from its canonical project-relative VTK
/// artifact. The dataset owns its historical topology and never borrows the
/// current in-memory mesh.
pub fn load_workspace_result(
    workspace: &Path,
    project: &WorkbenchProject,
    run_id: &str,
) -> Result<ResultDataset, ProjectDocumentError> {
    project.validate()?;
    let run = project
        .runs
        .iter()
        .find(|run| run.id == run_id)
        .ok_or_else(|| ProjectDocumentError::Invalid(format!("run {run_id:?} does not exist")))?;
    if matches!(run.status, RunStatus::Failed) {
        return Err(ProjectDocumentError::Artifact(format!(
            "run {run_id:?} failed and has no result fields"
        )));
    }
    let solution = run.solution_path.as_ref().ok_or_else(|| {
        ProjectDocumentError::Artifact(format!("run {run_id:?} has no solution artifact"))
    })?;
    validate_relative_artifact_path(solution)?;
    load_legacy_vtk_result(&workspace.join(solution), &run.id)
        .map_err(|error| ProjectDocumentError::Artifact(error.to_string()))
}

/// Persists a terminal solve as a project-local run directory, complete report,
/// and optional VTK solution. Idle and in-flight sessions have no terminal
/// record and return `Ok(None)`.
pub fn finalize_workspace_run(
    workspace: &Path,
    project: &mut WorkbenchProject,
    session: &WorkbenchSession,
) -> Result<Option<RunRecord>, ProjectDocumentError> {
    finalize_workspace_run_continuing(workspace, project, session, None)
}

/// Update the same persisted run after a continuation, or create its first record.
/// Callers explicitly choose the continuation ID; historical runs are never guessed.
pub fn finalize_workspace_run_continuing(
    workspace: &Path,
    project: &mut WorkbenchProject,
    session: &WorkbenchSession,
    continuation: Option<&str>,
) -> Result<Option<RunRecord>, ProjectDocumentError> {
    let status = match session.status() {
        SolveStatus::Converged => RunStatus::Converged,
        SolveStatus::MaxIterations => RunStatus::MaxIterations,
        SolveStatus::Failed(_) => RunStatus::Failed,
        SolveStatus::Idle | SolveStatus::Solving => return Ok(None),
    };
    let mut record = match continuation {
        Some(id) => project
            .runs
            .iter()
            .find(|r| r.id == id)
            .cloned()
            .ok_or_else(|| {
                ProjectDocumentError::Invalid(format!("Continuation run {id:?} does not exist"))
            })?,
        None => project.next_run(status.clone()),
    };
    record.status = status;
    record.failure_diagnostic = None;
    if let SolveStatus::Failed(diagnostic) = session.status() {
        record.failure_diagnostic = Some(diagnostic.clone());
    }
    let run_directory = workspace.join(format!("runs/run-{:04}", record.ordinal));
    fs::create_dir_all(&run_directory).map_err(io_error)?;
    if let (Some(mesh), Some(solution)) = (session.mesh(), session.solution()) {
        record.physical_time = solution.physical_time;
        record.time_steps = solution.completed_time_steps;
        record.mesh_identity = Some(mesh.mesh.id().get());
        record.iterations = Some(solution.report.outer_iterations);
        record.continuity_residual = Some(solution.report.final_continuity_rms);
        record.total_inflow = Some(solution.report.total_inflow);
        record.total_outflow = Some(solution.report.total_outflow);
        record.net_boundary_flux = Some(solution.report.net_boundary_flux);
        let solution_path = run_directory.join("solution.vtk");
        session
            .export_vtk(&solution_path)
            .map_err(ProjectDocumentError::Artifact)?;
        record.solution_path = Some(PathBuf::from(format!(
            "runs/run-{:04}/solution.vtk",
            record.ordinal
        )));
    }
    let report = serde_json::json!({
        "report_format": 2,
        "solver_version": env!("CARGO_PKG_VERSION"),
        "run": &record,
        "mesh_settings": &project.mesh,
        "mesh_recipe": &project.mesh_recipe,
        "material": &project.material,
        "solver_settings": &project.solver,
        "last_equation_linear_solves":session.solution().map(|s|&s.report.simple.equation_linear_reports),
        "physical_time_s":session.solution().and_then(|s|s.physical_time),
        "surface_budgets":session.solution().map(|s| &s.budgets),
        "completed_time_steps":session.solution().map(|s|s.completed_time_steps),
        "nonlinear_iterations":session.solution().map(|s|s.nonlinear_iterations),
        "max_diffusion_number_last_step":session.solution().and_then(|s|s.max_diffusion_number),
        "max_courant_last_step":session.solution().and_then(|s|s.max_courant),
        "last_time_step_s":session.solution().and_then(|s|s.last_time_step),
        "rejected_time_steps":session.solution().map(|s|s.rejected_time_steps),
        "wall_reports":session.solution().and_then(|s|session.prepare_case().ok().and_then(|c|super::wall_reports::reports_with_turbulence(&c,&s.velocity,&s.pressure,s.turbulence.as_ref(),true).ok())),
        "turbulence":session.solution().and_then(|s|s.turbulence.as_ref()).map(|t|serde_json::json!({
            "model":t.model,
            "nut_max":t.nut.values().iter().copied().fold(0.0,f64::max),
            "nu_tilda_max":t.nu_tilda.as_ref().map(|v|v.values().iter().copied().fold(0.0,f64::max)),
            "k_max":(t.model!=crate::turbulence::TurbulenceModel::SpalartAllmaras).then(||t.k.values().iter().copied().fold(0.0,f64::max)),
            "omega_max":matches!(t.model,crate::turbulence::TurbulenceModel::KOmega|crate::turbulence::TurbulenceModel::KOmegaSst).then(||t.omega.values().iter().copied().fold(0.0,f64::max)),
            "epsilon_max":t.epsilon.as_ref().map(|e|e.values().iter().copied().fold(0.0,f64::max)),
            "wall_treatment":session.solver_options().turbulence.wall_treatment,
            "validation":"experimental; not benchmark validated"
        })),
        "cpu_worker_threads": session.solution().map(|s| s.report.cpu_threads),
        "performance": session.performance(),
        "boundaries": &project.boundaries,
        "spatial_methods": session.mesh().map(|m| if m.mesh.dimension()==MeshDimension::ThreeD {serde_json::json!({"pressure_gradient":"Gauss with bounded face-pressure interpolation","momentum_nonorthogonal":"limited correction: abs(explicit flux) <= abs(two-point flux)","rhie_chow":"interpolated cell pressure response; response-gradient covariance retained"})}else{serde_json::json!({"pressure_gradient":"least squares","momentum_nonorthogonal":"explicit Gauss correction","rhie_chow":"interpolated cell pressure response; response-gradient covariance retained"})}),
        "mesh_preflight": session.mesh().map(|m|super::diagnostics::mesh_preflight(&m.mesh)),
        "conservation": session.mesh().zip(session.solution()).map(|(m,s)|super::boundary_reports::conservation_report(&m.mesh,s,session.material().density)),
        "boundary_flow_reports": session.mesh().zip(session.solution()).map(|(mesh,solution)|super::boundary_reports::boundary_flow_reports(&mesh.mesh,solution,session.material().density)),
        "continuity_history": session.solution().map(|solution| &solution.report.continuity_history),
        "convergence_histories": session.solution().map(|s|serde_json::json!({"momentum":s.report.simple.momentum_relative_history,"velocity":s.report.simple.velocity_change_history,"pressure_linear":s.report.simple.pressure_linear_history})),
        "coupling_convergence": session.solution().map(|solution| serde_json::json!({
            "momentum_relative_residual": solution.report.simple.final_momentum_relative_residual,
            "velocity_relative_change": solution.report.simple.final_velocity_relative_change,
        })),
    });
    atomic_json_write(&run_directory.join("report.json"), &report)?;
    if let Some(existing) = project.runs.iter_mut().find(|r| r.id == record.id) {
        *existing = record.clone();
    } else {
        project.runs.push(record.clone());
    }
    save_workspace(workspace, project)?;
    Ok(Some(record))
}

pub fn autosave_workspace(
    workspace: &Path,
    project: &WorkbenchProject,
) -> Result<(), ProjectDocumentError> {
    project.validate()?;
    let autosave = workspace.join("autosave").join(PROJECT_DOCUMENT_FILE);
    atomic_json_write(&autosave, project)
}

/// Discards the autosaved document without touching the canonical workspace
/// project or any other workspace artifacts.
pub fn discard_workspace_recovery(workspace: &Path) -> Result<(), ProjectDocumentError> {
    let recovery = workspace.join("autosave").join(PROJECT_DOCUMENT_FILE);
    match fs::remove_file(recovery) {
        Ok(()) => Ok(()),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(io_error(error)),
    }
}

/// Deletes one persisted run record and only its canonical `runs/run-NNNN`
/// artifact directory. The directory is first moved into a hidden staging path
/// so a project-save failure can restore it before the in-memory document is
/// changed.
pub fn delete_workspace_run(
    workspace: &Path,
    project: &mut WorkbenchProject,
    run_id: &str,
) -> Result<(), ProjectDocumentError> {
    let index = project
        .runs
        .iter()
        .position(|run| run.id == run_id)
        .ok_or_else(|| ProjectDocumentError::Invalid(format!("run {run_id:?} does not exist")))?;
    let run = &project.runs[index];
    let directory = canonical_run_directory(run)?;
    let source = workspace.join(&directory);
    let staged =
        workspace
            .join("runs")
            .join(format!(".deleting-{}-{}", run.ordinal, unique_id("run")));
    let staged_artifacts = if source.exists() {
        fs::rename(&source, &staged).map_err(io_error)?;
        true
    } else {
        false
    };

    let mut updated = project.clone();
    updated.runs.remove(index);
    if let Err(error) = save_workspace(workspace, &updated) {
        if staged_artifacts {
            let _ = fs::rename(&staged, &source);
        }
        return Err(error);
    }
    *project = updated;
    if staged_artifacts {
        fs::remove_dir_all(&staged).map_err(io_error)?;
    }
    Ok(())
}

fn canonical_run_directory(run: &RunRecord) -> Result<PathBuf, ProjectDocumentError> {
    let directory = PathBuf::from("runs").join(format!("run-{:04}", run.ordinal));
    let expected_id = format!("run-{:04}", run.ordinal);
    let expected_report = directory.join("report.json");
    let expected_solution = directory.join("solution.vtk");
    if run.id != expected_id
        || run.report_path != expected_report
        || run
            .solution_path
            .as_ref()
            .is_some_and(|path| path != &expected_solution)
    {
        return Err(ProjectDocumentError::Invalid(format!(
            "run {:?} does not use canonical workspace artifacts",
            run.id
        )));
    }
    Ok(directory)
}

pub fn recovery_is_newer(workspace: &Path) -> Result<bool, ProjectDocumentError> {
    let project = workspace.join(PROJECT_DOCUMENT_FILE);
    let recovery = workspace.join("autosave").join(PROJECT_DOCUMENT_FILE);
    if !recovery.exists() {
        return Ok(false);
    }
    if !project.exists() {
        return Ok(true);
    }
    Ok(fs::metadata(&recovery)
        .map_err(io_error)?
        .modified()
        .map_err(io_error)?
        >= fs::metadata(&project)
            .map_err(io_error)?
            .modified()
            .map_err(io_error)?)
}

/// Saves a validated template at `<template_root>/<template-id>/case.json`.
///
/// The template ID must be a single safe directory name so saving a user
/// template cannot escape the chosen template root. The atomic write makes a
/// completed save immediately discoverable by [`discover_templates`].
pub fn save_case_template(
    template_root: &Path,
    manifest: &CaseTemplateManifest,
) -> Result<PathBuf, ProjectDocumentError> {
    manifest.validate()?;
    validate_template_id(&manifest.id)?;
    let destination = template_root.join(&manifest.id).join("case.json");
    if destination.exists() {
        return Err(ProjectDocumentError::Invalid(format!(
            "Case Template {:?} already exists; choose a different template ID",
            manifest.id
        )));
    }
    // A template is reusable initial intent, never a solved-artifact history.
    // Keep the caller's document intact while enforcing that storage boundary.
    let mut portable_manifest = manifest.clone();
    portable_manifest.project_template.runs.clear();
    portable_manifest.project_template.mesh_asset = None;
    portable_manifest.project_template.case_context = None;
    super::case_format::write_assets(
        destination
            .parent()
            .ok_or_else(|| ProjectDocumentError::Invalid("Case directory missing".into()))?,
        &portable_manifest.metadata.assets,
    )
    .map_err(ProjectDocumentError::Invalid)?;
    atomic_json_write(&destination, &portable_manifest)?;
    Ok(destination)
}

/// Discovers templates below one root, retaining the original one-root API.
pub fn discover_templates(root: &Path) -> Vec<Result<CaseTemplateManifest, ProjectDocumentError>> {
    discover_templates_from_roots([root])
}

/// Discovers templates from built-in and user roots in the supplied order.
///
/// Existing roots are canonicalized before comparison so supplying the same
/// root (including through a symlink) does not create duplicate results. A
/// template ID is accepted only once across all roots; later copies are
/// returned as duplicate-ID errors rather than silently replacing the earlier
/// template.
pub fn discover_templates_from_roots<I, P>(
    roots: I,
) -> Vec<Result<CaseTemplateManifest, ProjectDocumentError>>
where
    I: IntoIterator<Item = P>,
    P: AsRef<Path>,
{
    let mut discovered = Vec::new();
    let mut ids = BTreeSet::new();
    let mut visited_roots = BTreeSet::new();
    for root in roots {
        let root = root.as_ref();
        let identity = fs::canonicalize(root).unwrap_or_else(|_| root.to_path_buf());
        if !visited_roots.insert(identity) {
            continue;
        }
        discovered.extend(discover_templates_in_root(root, &mut ids));
    }
    discovered
}

/// Loads the local recent-workspace index. A missing settings file means there
/// are no recent projects; malformed settings are reported rather than silently
/// discarded.
pub fn load_recent_projects(
    settings_directory: &Path,
) -> Result<RecentProjects, ProjectDocumentError> {
    let path = settings_directory.join(RECENT_PROJECTS_FILE);
    match fs::read_to_string(&path) {
        Ok(text) => serde_json::from_str(&text)
            .map_err(|error| ProjectDocumentError::Json(error.to_string())),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(RecentProjects::default()),
        Err(error) => Err(io_error(error)),
    }
}

/// Persists the local recent-workspace index atomically.
pub fn save_recent_projects(
    settings_directory: &Path,
    recent_projects: &RecentProjects,
) -> Result<(), ProjectDocumentError> {
    atomic_json_write(
        &settings_directory.join(RECENT_PROJECTS_FILE),
        recent_projects,
    )
}

fn discover_templates_in_root(
    root: &Path,
    ids: &mut BTreeSet<String>,
) -> Vec<Result<CaseTemplateManifest, ProjectDocumentError>> {
    let Ok(entries) = fs::read_dir(root) else {
        return Vec::new();
    };
    let mut entries = entries.filter_map(Result::ok).collect::<Vec<_>>();
    entries.sort_by_key(|entry| entry.file_name());
    let mut discovered = Vec::new();
    for entry in entries {
        let path = entry.path().join("case.json");
        if !path.is_file() {
            continue;
        }
        let manifest = fs::read_to_string(&path)
            .map_err(io_error)
            .and_then(|text| {
                let manifest: CaseTemplateManifest = serde_json::from_str(&text)
                    .map_err(|error| ProjectDocumentError::Json(error.to_string()))?;
                manifest.validate()?;
                Ok(manifest)
            });
        match manifest {
            Ok(manifest) if ids.insert(manifest.id.clone()) => discovered.push(Ok(manifest)),
            Ok(manifest) => discovered.push(Err(ProjectDocumentError::Invalid(format!(
                "duplicate Case Template ID {:?}",
                manifest.id
            )))),
            Err(error) => discovered.push(Err(error)),
        }
    }
    discovered
}

fn validate_template_id(id: &str) -> Result<(), ProjectDocumentError> {
    let mut components = Path::new(id).components();
    let safe_component = matches!(components.next(), Some(Component::Normal(_)))
        && components.next().is_none()
        && !id.contains(['/', '\\']);
    safe_component.then_some(()).ok_or_else(|| {
        ProjectDocumentError::Invalid("template ID must be a single directory name".into())
    })
}

fn atomic_json_write<T: Serialize>(
    destination: &Path,
    value: &T,
) -> Result<(), ProjectDocumentError> {
    let parent = destination
        .parent()
        .ok_or_else(|| ProjectDocumentError::Io("workspace path has no parent".into()))?;
    fs::create_dir_all(parent).map_err(io_error)?;
    let temporary = parent.join(format!(
        ".{}.{}.tmp",
        destination
            .file_name()
            .and_then(|name| name.to_str())
            .unwrap_or("project"),
        unique_id("save")
    ));
    let text = serde_json::to_string_pretty(value)
        .map_err(|error| ProjectDocumentError::Json(error.to_string()))?;
    let mut file = File::create(&temporary).map_err(io_error)?;
    file.write_all(text.as_bytes()).map_err(io_error)?;
    file.write_all(b"\n").map_err(io_error)?;
    file.sync_all().map_err(io_error)?;
    fs::rename(&temporary, destination).map_err(io_error)
}

fn target_exists(geometry: &GeometryTopology, target: GeometrySelectionTarget) -> bool {
    match target {
        GeometrySelectionTarget::Vertex(id) => geometry.vertex(id).is_some(),
        GeometrySelectionTarget::Edge(id) => geometry.edge(id).is_some(),
        GeometrySelectionTarget::Face(id) => geometry.face(id).is_some(),
        GeometrySelectionTarget::Body(id) => geometry.body(id).is_some(),
    }
}

fn validate_mesh_recipe(
    recipe: &MeshRecipe,
    geometry: &GeometryTopology,
    named_selections: &BTreeSet<&String>,
) -> Result<(), ProjectDocumentError> {
    let mut ids = BTreeSet::new();
    for control in &recipe.edge_divisions {
        if !ids.insert(control.id) {
            return Err(ProjectDocumentError::Invalid(
                "Duplicate mesh-control ID".into(),
            ));
        }
        control.validate().map_err(ProjectDocumentError::Invalid)?;
        validate_mesh_control_targets(&control.targets, geometry, named_selections)?;
    }
    for control in &recipe.regions {
        if !ids.insert(control.id) {
            return Err(ProjectDocumentError::Invalid(
                "Duplicate mesh-control ID".into(),
            ));
        }
        control.validate().map_err(ProjectDocumentError::Invalid)?;
    }

    for control in &recipe.local_sizes {
        if !ids.insert(control.id)
            || control.name.trim().is_empty()
            || control.targets.is_empty()
            || !control.size.is_finite()
            || control.size <= 0.0
        {
            return Err(ProjectDocumentError::Invalid(
                "invalid local mesh size control".into(),
            ));
        }
        validate_mesh_control_targets(&control.targets, geometry, named_selections)?;
    }
    for control in &recipe.refinements {
        if !ids.insert(control.id)
            || control.name.trim().is_empty()
            || control.targets.is_empty()
            || control.sampling == 0
            || !control.size_min.is_finite()
            || !control.size_max.is_finite()
            || !control.distance_min.is_finite()
            || !control.distance_max.is_finite()
            || control.size_min <= 0.0
            || control.size_min > control.size_max
            || control.distance_min < 0.0
            || control.distance_min >= control.distance_max
        {
            return Err(ProjectDocumentError::Invalid(
                "invalid threshold refinement control".into(),
            ));
        }
        validate_mesh_control_targets(&control.targets, geometry, named_selections)?;
    }
    for control in &recipe.boundary_layers {
        if !ids.insert(control.id)
            || control.name.trim().is_empty()
            || control.targets.is_empty()
            || !control.first_layer_height.is_finite()
            || !control.growth_ratio.is_finite()
            || control.first_layer_height <= 0.0
            || control.growth_ratio < 1.0
            || control.layer_count == 0
        {
            return Err(ProjectDocumentError::Invalid(
                "invalid boundary layer control".into(),
            ));
        }
        control
            .layer_dimensions()
            .map_err(ProjectDocumentError::Invalid)?;
        validate_mesh_control_targets(&control.targets, geometry, named_selections)?;
    }
    Ok(())
}

fn validate_mesh_control_targets(
    targets: &[MeshControlTarget],
    geometry: &GeometryTopology,
    named_selections: &BTreeSet<&String>,
) -> Result<(), ProjectDocumentError> {
    for target in targets {
        match target {
            MeshControlTarget::Geometry(target) if target_exists(geometry, *target) => {}
            MeshControlTarget::NamedSelection(name) if named_selections.contains(name) => {}
            _ => {
                return Err(ProjectDocumentError::Invalid(
                    "mesh control references a missing geometry ID or Named Selection".into(),
                ));
            }
        }
    }
    Ok(())
}

fn validate_boundary(condition: &PhysicalBoundaryCondition) -> Result<(), ProjectDocumentError> {
    let values: &[f64] = match condition {
        PhysicalBoundaryCondition::ParabolicInlet {
            velocity,
            axis,
            center,
            half_width,
        } => {
            if *axis > 2 || !half_width.is_finite() || *half_width <= 0.0 {
                return Err(ProjectDocumentError::Invalid(
                    "Profile axis must be X/Y/Z and half-width positive".into(),
                ));
            }
            &[velocity.x, velocity.y, velocity.z, *center, *half_width]
        }

        PhysicalBoundaryCondition::NoSlipWall
        | PhysicalBoundaryCondition::Slip
        | PhysicalBoundaryCondition::Symmetry => &[],
        PhysicalBoundaryCondition::FlowRateInlet { rate, .. } => {
            if !rate.is_finite() || *rate < 0.0 {
                return Err(ProjectDocumentError::Invalid(
                    "Flow rate must be finite and nonnegative".into(),
                ));
            }
            std::slice::from_ref(rate)
        }
        PhysicalBoundaryCondition::PressureOutletBackflow { velocity, pressure }
        | PhysicalBoundaryCondition::FarField { velocity, pressure } => {
            &[velocity.x, velocity.y, velocity.z, *pressure]
        }
        PhysicalBoundaryCondition::MovingWall { velocity }
        | PhysicalBoundaryCondition::VelocityInlet { velocity } => {
            &[velocity.x, velocity.y, velocity.z]
        }
        PhysicalBoundaryCondition::TotalPressure { pressure }
        | PhysicalBoundaryCondition::PressureOutlet { pressure } => std::slice::from_ref(pressure),
    };
    values
        .iter()
        .all(|value| value.is_finite())
        .then_some(())
        .ok_or_else(|| ProjectDocumentError::Invalid("boundary values must be finite".into()))
}

fn validate_relative_artifact_path(path: &Path) -> Result<(), ProjectDocumentError> {
    if path.is_absolute()
        || path.components().any(|component| {
            matches!(
                component,
                Component::ParentDir | Component::RootDir | Component::Prefix(_)
            )
        })
    {
        return Err(ProjectDocumentError::UnsafeArtifactPath(path.to_path_buf()));
    }
    Ok(())
}

fn now_ms() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis()
}
fn unique_id(prefix: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    format!("{prefix}-{nanos}-{}", std::process::id())
}
fn io_error(error: std::io::Error) -> ProjectDocumentError {
    ProjectDocumentError::Io(error.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::workbench::examples::{build_example, ExampleProjectId};
    use crate::{
        pressure_face_coefficients, solve_incompressible, CellField,
        IncompressibleBoundaryCondition, ResultFieldKind, ResultRenderCache, StreamlineDirection,
        StreamlineField, StreamlineOptions, Vec3, WorkbenchSession,
    };
    use tempfile::tempdir;

    #[test]
    fn workspace_round_trip_preserves_mixed_boundaries_and_cpu_workers() {
        let directory = tempdir().unwrap();
        let mut project = WorkbenchProject::blank("mixed boundaries");
        let rectangle = project.geometry.add_rectangle(2.0, 1.0).unwrap();
        for (name, edge, condition) in [
            ("left", rectangle.left, PhysicalBoundaryCondition::Slip),
            (
                "right",
                rectangle.right,
                PhysicalBoundaryCondition::Symmetry,
            ),
            (
                "top",
                rectangle.top,
                PhysicalBoundaryCondition::FarField {
                    velocity: Vec3::new(1.0, 0.0, 0.0),
                    pressure: 0.25,
                },
            ),
            (
                "bottom",
                rectangle.bottom,
                PhysicalBoundaryCondition::NoSlipWall,
            ),
        ] {
            project.named_selections.push(NamedSelection {
                name: name.into(),
                targets: vec![GeometrySelectionTarget::Edge(edge)],
            });
            project.boundaries.push(BoundaryAssignment {
                selection: name.into(),
                condition,
            });
        }
        project.solver.threads = 2;
        project.mesh.threads = 3;
        save_workspace(directory.path(), &project).unwrap();
        let restored = load_workspace(directory.path()).unwrap();
        assert_eq!(restored.boundaries, project.boundaries);
        assert_eq!(restored.solver.threads, 2);
        assert_eq!(restored.mesh.threads, 3);
        let session = WorkbenchSession::from_project(&restored).unwrap();
        let round_trip = session.to_project("restored");
        assert_eq!(round_trip.boundaries.len(), project.boundaries.len());
        for assignment in &project.boundaries {
            assert!(round_trip.boundaries.contains(assignment));
        }
        assert_eq!(round_trip.solver.threads, 2);
        assert_eq!(round_trip.mesh.threads, 3);
        assert_eq!(session.mesh_cpu_threads(), 3);
        let mut old_settings = serde_json::to_value(&project.solver).unwrap();
        old_settings.as_object_mut().unwrap().remove("threads");
        let old_settings: WorkbenchSolverSettings = serde_json::from_value(old_settings).unwrap();
        assert_eq!(old_settings.threads, 1);
    }

    #[test]
    fn workspace_round_trip_preserves_geometry_ids_and_allocator_state() {
        let directory = tempdir().unwrap();
        let mut project = WorkbenchProject::blank("round trip");
        let rectangle = project.geometry.add_rectangle(2.0, 1.0).unwrap();
        project.named_selections.push(NamedSelection {
            name: "inlet".into(),
            targets: vec![GeometrySelectionTarget::Edge(rectangle.left)],
        });
        project.boundaries.push(BoundaryAssignment {
            selection: "inlet".into(),
            condition: PhysicalBoundaryCondition::VelocityInlet {
                velocity: Vec3::new(1.0, 0.0, 0.0),
            },
        });
        save_workspace(directory.path(), &project).unwrap();
        let mut restored = load_workspace(directory.path()).unwrap();
        assert_eq!(
            restored.geometry.face(rectangle.face).unwrap().id,
            rectangle.face
        );
        assert_eq!(restored.named_selections, project.named_selections);
        let next = restored
            .geometry
            .add_vertex(Vec3::new(9.0, 0.0, 0.0))
            .unwrap();
        assert!(next.get() > rectangle.vertices[3].get());
    }

    #[test]
    fn mesh_recipe_rejects_invalid_size_and_stale_targets() {
        let mut project = WorkbenchProject::blank("recipe validation");
        project.mesh_recipe.local_sizes.push(LocalMeshSize {
            id: 1,
            name: "invalid".into(),
            targets: vec![MeshControlTarget::NamedSelection("missing".into())],
            size: 0.0,
            enabled: true,
        });

        assert!(matches!(
            project.validate(),
            Err(ProjectDocumentError::Invalid(message)) if message.contains("local mesh size")
        ));
    }

    #[test]
    fn mesh_recipe_emits_deterministic_local_and_threshold_gmsh_fields() {
        let mut session = WorkbenchSession::new();
        let rectangle = session.add_rectangle(2.0, 1.0).unwrap();
        session
            .create_named_selection("inlet", vec![GeometrySelectionTarget::Edge(rectangle.left)])
            .unwrap();
        session
            .create_named_selection(
                "other",
                [rectangle.bottom, rectangle.right, rectangle.top]
                    .into_iter()
                    .map(GeometrySelectionTarget::Edge)
                    .collect(),
            )
            .unwrap();
        session
            .set_mesh_configuration(MeshDimension::TwoD, 0.1, 0.01, 0.2, 1)
            .unwrap();
        session.set_mesh_recipe(MeshRecipe {
            local_sizes: vec![LocalMeshSize {
                id: 1,
                name: "inlet sizing".into(),
                targets: vec![MeshControlTarget::NamedSelection("inlet".into())],
                size: 0.025,
                enabled: true,
            }],
            refinements: vec![ThresholdRefinement {
                id: 2,
                name: "inlet distance".into(),
                targets: vec![MeshControlTarget::NamedSelection("inlet".into())],
                sampling: 64,
                size_min: 0.01,
                size_max: 0.1,
                distance_min: 0.05,
                distance_max: 0.5,
                enabled: true,
            }],
            ..MeshRecipe::default()
        });

        let (export, _) = session.mesh_generation_inputs().unwrap();
        let geo = export.document.to_geo_string().unwrap();

        assert!(geo.contains("MeshSize { PointsOf{ Curve{2003}; } } = 0.025"));
        assert!(geo.contains("Field[1] = Distance"));
        assert!(geo.contains("Field[2] = Threshold"));
        assert!(geo.contains("Background Field = 2"));
    }

    #[test]
    fn mesh_recipe_emits_real_2d_boundary_layer_field() {
        let mut session = WorkbenchSession::new();
        let rectangle = session.add_rectangle(2.0, 1.0).unwrap();
        session
            .create_named_selection(
                "wall",
                vec![GeometrySelectionTarget::Edge(rectangle.bottom)],
            )
            .unwrap();
        session
            .create_named_selection(
                "other",
                [rectangle.left, rectangle.right, rectangle.top]
                    .into_iter()
                    .map(GeometrySelectionTarget::Edge)
                    .collect(),
            )
            .unwrap();
        session.set_mesh_recipe(MeshRecipe {
            boundary_layers: vec![BoundaryLayerControl {
                id: 3,
                name: "wall layer".into(),
                targets: vec![MeshControlTarget::NamedSelection("wall".into())],
                first_layer_height: 0.005,
                growth_ratio: 1.2,
                layer_count: 5,
                enabled: true,
            }],
            ..MeshRecipe::default()
        });

        let (export, _) = session.mesh_generation_inputs().unwrap();
        let geo = export.document.to_geo_string().unwrap();

        assert!(geo.contains("Field[1] = BoundaryLayer"));
        assert!(geo.contains("Field[1].CurvesList = {2000}"));
        assert!(geo.contains("BoundaryLayer Field = 1"));
    }

    #[test]
    fn session_preserves_mesh_recipe_as_canonical_project_intent() {
        let mut session = WorkbenchSession::new();
        let recipe = MeshRecipe {
            local_sizes: vec![LocalMeshSize {
                id: 7,
                name: "inlet sizing".into(),
                targets: vec![MeshControlTarget::NamedSelection("inlet".into())],
                size: 0.025,
                enabled: true,
            }],
            ..MeshRecipe::default()
        };

        session.set_mesh_recipe(recipe.clone());

        assert_eq!(session.to_project("recipe persistence").mesh_recipe, recipe);
    }

    #[test]
    fn legacy_project_migrates_to_an_empty_mesh_recipe() {
        let directory = tempdir().unwrap();
        let project = WorkbenchProject::blank("legacy recipe migration");
        save_workspace(directory.path(), &project).unwrap();
        let path = directory.path().join(PROJECT_DOCUMENT_FILE);
        let mut value: serde_json::Value =
            serde_json::from_str(&fs::read_to_string(&path).unwrap()).unwrap();
        value["format_version"] = serde_json::json!(1);
        value.as_object_mut().unwrap().remove("mesh_recipe");
        fs::write(&path, serde_json::to_string(&value).unwrap()).unwrap();

        let restored = load_workspace(directory.path()).unwrap();

        assert_eq!(restored.format_version, WORKBENCH_PROJECT_FORMAT_VERSION);
        assert_eq!(restored.mesh_recipe, MeshRecipe::default());
    }

    #[test]
    fn session_document_adapter_preserves_intent_without_runtime_artifacts() {
        let mut session = WorkbenchSession::new();
        let rectangle = session.add_rectangle(2.0, 1.0).unwrap();
        session
            .create_named_selection("inlet", vec![GeometrySelectionTarget::Edge(rectangle.left)])
            .unwrap();
        session
            .configure_named_boundary(
                "inlet",
                IncompressibleBoundaryCondition::VelocityInlet {
                    velocity: Vec3::new(0.2, 0.0, 0.0),
                },
            )
            .unwrap();
        session
            .set_mesh_configuration(MeshDimension::TwoD, 0.2, 0.1, 0.2, 1)
            .unwrap();

        let document = session.to_project("adapter");
        let restored = WorkbenchSession::from_project(&document).unwrap();

        assert_eq!(
            restored.geometry().face(rectangle.face).unwrap().id,
            rectangle.face
        );
        assert_eq!(
            restored
                .named_selections()
                .get("inlet")
                .unwrap()
                .targets
                .len(),
            1
        );
        assert!(matches!(
            restored.boundary_assignment("inlet"),
            Some(IncompressibleBoundaryCondition::VelocityInlet { .. })
        ));
        assert!(!restored.has_mesh());
        assert!(restored.solution().is_none());
    }

    #[test]
    fn discarding_recovery_removes_only_the_recovery_document() {
        let directory = tempdir().unwrap();
        let project = WorkbenchProject::blank("recovery");
        autosave_workspace(directory.path(), &project).unwrap();

        discard_workspace_recovery(directory.path()).unwrap();

        assert!(!directory
            .path()
            .join("autosave")
            .join(PROJECT_DOCUMENT_FILE)
            .exists());
    }

    #[test]
    fn recovery_written_after_a_canonical_save_is_detected_even_on_coarse_filesystems() {
        let directory = tempdir().unwrap();
        let project = WorkbenchProject::blank("recovery");
        save_workspace(directory.path(), &project).unwrap();
        autosave_workspace(directory.path(), &project).unwrap();

        assert!(recovery_is_newer(directory.path()).unwrap());
    }

    #[test]
    fn recovery_is_separate_from_canonical_project() {
        let directory = tempdir().unwrap();
        let project = WorkbenchProject::blank("recovery");
        autosave_workspace(directory.path(), &project).unwrap();
        assert!(recovery_is_newer(directory.path()).unwrap());
        save_workspace(directory.path(), &project).unwrap();
        assert!(!directory
            .path()
            .join("autosave")
            .join(PROJECT_DOCUMENT_FILE)
            .exists());
        assert!(!recovery_is_newer(directory.path()).unwrap());
    }

    #[test]
    fn terminal_failure_is_finalized_as_a_project_local_record_and_report() {
        let directory = tempdir().unwrap();
        let mut project = WorkbenchProject::blank("failed run");
        let mut session = WorkbenchSession::new();
        session.complete_solve(Err(crate::IncompressibleSolveError::Case(
            crate::IncompressibleCaseError::InvalidInitialConditions,
        )));

        let record = finalize_workspace_run(directory.path(), &mut project, &session)
            .unwrap()
            .unwrap();

        assert_eq!(record.status, RunStatus::Failed);
        assert!(record.solution_path.is_none());
        assert!(record
            .failure_diagnostic
            .as_deref()
            .is_some_and(|message| message.contains("InvalidInitialConditions")));
        assert_eq!(
            load_workspace(directory.path()).unwrap().runs,
            vec![record.clone()]
        );
        let report = fs::read_to_string(directory.path().join(record.report_path)).unwrap();
        assert!(report.contains("created_unix_ms"));
        assert!(report.contains("solver_settings"));
        assert!(!directory.path().join("runs/run-0001/solution.vtk").exists());
    }

    #[test]
    fn historical_result_loader_uses_the_run_artifact_not_current_runtime_mesh() {
        let directory = tempdir().unwrap();
        let mut project = WorkbenchProject::blank("archived result");
        let mut run = project.next_run(RunStatus::Converged);
        run.solution_path = Some(PathBuf::from("runs/run-0001/solution.vtk"));
        let artifact = directory.path().join(run.solution_path.as_ref().unwrap());
        fs::create_dir_all(artifact.parent().unwrap()).unwrap();
        fs::write(
            &artifact,
            "# vtk DataFile Version 3.0\nrun\nASCII\nDATASET UNSTRUCTURED_GRID\nPOINTS 3 double\n0 0 0\n1 0 0\n0 1 0\nCELLS 1 4\n3 0 1 2\nCELL_TYPES 1\n7\nCELL_DATA 1\nSCALARS pressure double 1\nLOOKUP_TABLE default\n2\nSCALARS velocity_magnitude double 1\nLOOKUP_TABLE default\n5\nVECTORS velocity double\n3 4 0\n",
        )
        .unwrap();
        project.runs.push(run);

        let dataset = load_workspace_result(directory.path(), &project, "run-0001").unwrap();

        assert_eq!(dataset.run_id(), "run-0001");
        assert_eq!(dataset.mesh().cell_count(), 1);
        assert_eq!(dataset.pressure(), &[2.0]);
    }

    #[test]
    fn pressure_response_is_positive_for_the_real_boundary_layer_face_316() {
        let session = build_example(ExampleProjectId::LaminarChannel2D).unwrap();
        let (export, options) = session.mesh_generation_inputs().unwrap();
        let generated = session
            .mesher()
            .generate(&export.document, &options)
            .unwrap();
        let mesh = &generated.mesh;
        let non_positive_projection_count = mesh
            .faces()
            .iter()
            .filter_map(|face| {
                face.neighbour.map(|neighbour| {
                    face.area_vector
                        .dot(mesh.cells()[neighbour].center - mesh.cells()[face.owner].center)
                })
            })
            .filter(|projection| *projection <= 0.0)
            .count();
        assert_eq!(non_positive_projection_count, 0);
        let face = &mesh.faces()[316];
        let neighbour = face.neighbour.unwrap();
        let mut r_au = CellField::filled(mesh, 1.0);
        r_au.values_mut()[face.owner] = 1.9144529309776508;
        r_au.values_mut()[neighbour] = 1.243027278055116;

        let coefficients = pressure_face_coefficients(mesh, &r_au).unwrap();
        assert!(coefficients[316].is_finite() && coefficients[316] > 0.0);
    }

    #[test]
    fn cylinder_and_channel_showcases_include_persisted_advanced_mesh_recipes() {
        let cylinder = build_example(ExampleProjectId::CylinderFlow2D).unwrap();
        let channel = build_example(ExampleProjectId::LaminarChannel2D).unwrap();
        let channel_3d = build_example(ExampleProjectId::Channel3D).unwrap();

        assert!(!cylinder.mesh_recipe().refinements.is_empty());
        assert!(!channel.mesh_recipe().boundary_layers.is_empty());
        assert!(!channel_3d.mesh_recipe().refinements.is_empty());
    }

    #[test]
    fn real_gmsh_2d_boundary_layer_recipe_generates_mesh() {
        let mut session = build_example(ExampleProjectId::LaminarChannel2D).unwrap();
        session.set_mesh_recipe(MeshRecipe {
            boundary_layers: vec![BoundaryLayerControl {
                id: 1,
                name: "wall layers".into(),
                targets: vec![
                    MeshControlTarget::NamedSelection("top_wall".into()),
                    MeshControlTarget::NamedSelection("bottom_wall".into()),
                ],
                first_layer_height: 0.01,
                growth_ratio: 1.2,
                layer_count: 4,
                enabled: true,
            }],
            ..MeshRecipe::default()
        });

        let (export, options) = session.mesh_generation_inputs().unwrap();
        let generated = session
            .mesher()
            .generate(&export.document, &options)
            .unwrap();

        assert!(generated.mesh.cell_count() > 0);
        assert!(generated.mesh.points().len() > 4);
    }

    #[test]
    fn real_gmsh_3d_local_face_size_recipe_generates_mesh() {
        let mut session = build_example(ExampleProjectId::Channel3D).unwrap();
        session.set_mesh_recipe(MeshRecipe {
            local_sizes: vec![LocalMeshSize {
                id: 1,
                name: "inlet sizing".into(),
                targets: vec![MeshControlTarget::NamedSelection("inlet".into())],
                size: 0.1,
                enabled: true,
            }],
            ..MeshRecipe::default()
        });

        let (export, options) = session.mesh_generation_inputs().unwrap();
        let geo = export.document.to_geo_string().unwrap();
        let generated = session
            .mesher()
            .generate(&export.document, &options)
            .unwrap();

        assert!(geo.contains("MeshSize { PointsOf{ Surface{1}; } } = 0.1"));
        assert!(generated.mesh.cell_count() > 0);
    }

    #[test]
    fn real_gmsh_3d_distance_threshold_recipe_generates_mesh() {
        let mut session = build_example(ExampleProjectId::Channel3D).unwrap();
        session.set_mesh_recipe(MeshRecipe {
            refinements: vec![ThresholdRefinement {
                id: 2,
                name: "inlet refinement".into(),
                targets: vec![MeshControlTarget::NamedSelection("inlet".into())],
                sampling: 48,
                size_min: 0.1,
                size_max: 0.2,
                distance_min: 0.0,
                distance_max: 0.4,
                enabled: true,
            }],
            ..MeshRecipe::default()
        });

        let (export, options) = session.mesh_generation_inputs().unwrap();
        let generated = session
            .mesher()
            .generate(&export.document, &options)
            .unwrap();

        assert!(generated.mesh.cell_count() > 0);
    }

    #[test]
    fn mesh_recipe_remesh_preserves_historical_run_result_topology() {
        let directory = tempdir().unwrap();
        let mut session = build_example(ExampleProjectId::LaminarChannel2D).unwrap();
        let (export, options) = session.mesh_generation_inputs().unwrap();
        session.install_mesh(
            session
                .mesher()
                .generate(&export.document, &options)
                .unwrap(),
        );
        session.complete_solve(solve_incompressible(&session.prepare_case().unwrap()));
        let mut project = session.to_project("remesh history");
        let first = finalize_workspace_run(directory.path(), &mut project, &session)
            .unwrap()
            .unwrap();

        session.set_mesh_recipe(MeshRecipe {
            local_sizes: vec![LocalMeshSize {
                id: 1,
                name: "inlet refinement".into(),
                targets: vec![MeshControlTarget::NamedSelection("inlet".into())],
                size: 0.1,
                enabled: true,
            }],
            ..MeshRecipe::default()
        });
        assert!(!session.has_mesh());
        assert!(session.solution().is_none());
        project.mesh_recipe = session.mesh_recipe().clone();

        let (export, options) = session.mesh_generation_inputs().unwrap();
        session.install_mesh(
            session
                .mesher()
                .generate(&export.document, &options)
                .unwrap(),
        );
        session.complete_solve(solve_incompressible(&session.prepare_case().unwrap()));
        let second = finalize_workspace_run(directory.path(), &mut project, &session)
            .unwrap()
            .unwrap();
        save_workspace(directory.path(), &project).unwrap();
        let restored = load_workspace(directory.path()).unwrap();
        let first_result = load_workspace_result(directory.path(), &restored, &first.id).unwrap();
        let second_result = load_workspace_result(directory.path(), &restored, &second.id).unwrap();

        assert_eq!(first_result.run_id(), "run-0001");
        assert_eq!(second_result.run_id(), "run-0002");
        assert_ne!(
            ResultRenderCache::build(&first_result)
                .unwrap()
                .artifact_fingerprint(),
            ResultRenderCache::build(&second_result)
                .unwrap()
                .artifact_fingerprint()
        );
    }

    #[test]
    fn real_gmsh_channel_run_survives_save_reload_and_postprocess_without_resolve() {
        let directory = tempdir().unwrap();
        let mut session = build_example(ExampleProjectId::LaminarChannel2D).unwrap();
        let (export, options) = session.mesh_generation_inputs().unwrap();
        let generated = session
            .mesher()
            .generate(&export.document, &options)
            .unwrap();
        session.install_mesh(generated);
        let solution = solve_incompressible(&session.prepare_case().unwrap());
        assert!(solution.is_ok(), "Channel solve failed: {solution:?}");
        session.complete_solve(solution);

        let mut project = session.to_project("channel postprocess acceptance");
        let run = finalize_workspace_run(directory.path(), &mut project, &session)
            .unwrap()
            .unwrap();
        drop(session);
        let restored = load_workspace(directory.path()).unwrap();
        let dataset = load_workspace_result(directory.path(), &restored, &run.id).unwrap();
        let cache = ResultRenderCache::build(&dataset).unwrap();
        let probe = dataset.probe_cell(0).unwrap();
        let field = StreamlineField::from_dataset(&dataset).unwrap();
        let path = field
            .integrate(
                probe.center,
                StreamlineDirection::Forward,
                StreamlineOptions {
                    step_size: 0.05,
                    max_steps: 16,
                    max_length: 2.0,
                    stagnation_speed: 1.0e-12,
                },
            )
            .unwrap();

        assert_eq!(dataset.run_id(), run.id);
        assert_eq!(dataset.mesh().cell_count(), cache.cell_count());
        assert!(dataset.pressure().iter().all(|value| value.is_finite()));
        assert!(dataset
            .velocity_magnitude()
            .iter()
            .all(|value| value.is_finite()));
        assert!(cache.velocity_for_cell(probe.cell_index).is_some());
        assert!(!path.points.is_empty());
    }

    #[test]
    fn cylinder_streamline_locator_excludes_the_real_gmsh_hole() {
        let session = build_example(ExampleProjectId::CylinderFlow2D).unwrap();
        let (export, options) = session.mesh_generation_inputs().unwrap();
        let generated = session
            .mesher()
            .generate(&export.document, &options)
            .unwrap();
        let cell_count = generated.mesh.cell_count();
        let dataset = ResultDataset::from_values(
            "cylinder-run",
            &generated.mesh,
            vec![0.0; cell_count],
            vec![Vec3::new(1.0, 0.0, 0.0); cell_count],
        )
        .unwrap();
        let field = StreamlineField::from_dataset(&dataset).unwrap();

        assert!(field.locate_cell(Vec3::new(2.0, 1.0, 0.0)).is_none());
        assert!(field
            .integrate(
                Vec3::new(2.0, 1.0, 0.0),
                StreamlineDirection::Forward,
                StreamlineOptions {
                    step_size: 0.05,
                    max_steps: 8,
                    max_length: 1.0,
                    stagnation_speed: 1.0e-12,
                },
            )
            .is_err());
    }

    #[test]
    fn real_gmsh_3d_channel_run_reloads_exterior_result_surface() {
        let directory = tempdir().unwrap();
        let mut session = build_example(ExampleProjectId::Channel3D).unwrap();
        let (export, options) = session.mesh_generation_inputs().unwrap();
        let generated = session
            .mesher()
            .generate(&export.document, &options)
            .unwrap();
        session.install_mesh(generated);
        let solution = solve_incompressible(&session.prepare_case().unwrap());
        session.complete_solve(solution);

        let mut project = session.to_project("3d channel postprocess acceptance");
        let run = finalize_workspace_run(directory.path(), &mut project, &session)
            .unwrap()
            .unwrap();
        drop(session);
        let restored = load_workspace(directory.path()).unwrap();
        let dataset = load_workspace_result(directory.path(), &restored, &run.id).unwrap();
        let cache = ResultRenderCache::build(&dataset).unwrap();

        assert_eq!(dataset.mesh().dimension(), MeshDimension::ThreeD);
        assert!(!cache.mesh_cache().surface_triangles().is_empty());
        assert!(cache.mesh_cache().triangle_faces().iter().all(|&face| {
            cache
                .scalar_for_face(ResultFieldKind::Pressure, face)
                .is_some()
        }));
    }

    #[test]
    fn real_gmsh_cavity_run_reloads_pressure_and_velocity_result_fields() {
        let directory = tempdir().unwrap();
        let mut session = build_example(ExampleProjectId::LidDrivenCavity2D).unwrap();
        let (export, options) = session.mesh_generation_inputs().unwrap();
        let generated = session
            .mesher()
            .generate(&export.document, &options)
            .unwrap();
        session.install_mesh(generated);
        let solution = solve_incompressible(&session.prepare_case().unwrap());
        session.complete_solve(solution);

        let mut project = session.to_project("cavity postprocess acceptance");
        let run = finalize_workspace_run(directory.path(), &mut project, &session)
            .unwrap()
            .unwrap();
        drop(session);
        let restored = load_workspace(directory.path()).unwrap();
        let dataset = load_workspace_result(directory.path(), &restored, &run.id).unwrap();
        let cache = ResultRenderCache::build(&dataset).unwrap();

        assert!(dataset.pressure().iter().all(|value| value.is_finite()));
        assert!(dataset
            .velocity()
            .iter()
            .all(|value| value.norm().is_finite()));
        assert!(cache
            .scalar_for_cell(ResultFieldKind::Pressure, 0)
            .is_some());
        assert!(cache
            .scalar_for_cell(ResultFieldKind::VelocityMagnitude, 0)
            .is_some());
    }

    #[test]
    fn recent_workspace_index_is_atomic_deduplicated_and_prunes_missing_paths() {
        let directory = tempdir().unwrap();
        let first = directory.path().join("first");
        let second = directory.path().join("second");
        fs::create_dir_all(&first).unwrap();
        fs::create_dir_all(&second).unwrap();
        let mut recent = RecentProjects::default();
        recent.record(&first, "First");
        recent.record(&second, "Second");
        recent.record(&first, "First renamed");
        save_recent_projects(directory.path(), &recent).unwrap();

        let mut restored = load_recent_projects(directory.path()).unwrap();
        assert_eq!(restored.entries.len(), 2);
        assert_eq!(restored.entries[0].display_name, "First renamed");
        fs::remove_dir_all(&second).unwrap();
        restored.remove_missing();
        assert_eq!(restored.entries.len(), 1);
    }

    #[test]
    fn deleting_a_persisted_run_removes_only_its_record_and_artifact_directory() {
        let directory = tempdir().unwrap();
        let mut project = WorkbenchProject::blank("run history");
        let first = project.next_run(RunStatus::Converged);
        project.runs.push(first.clone());
        let second = project.next_run(RunStatus::Failed);
        project.runs.push(second.clone());
        save_workspace(directory.path(), &project).unwrap();
        for run in [&first, &second] {
            let run_dir = directory.path().join(
                run.report_path
                    .parent()
                    .expect("report has a run directory"),
            );
            fs::create_dir_all(&run_dir).unwrap();
            fs::write(run_dir.join("report.json"), "{}\n").unwrap();
        }

        delete_workspace_run(directory.path(), &mut project, &first.id).unwrap();

        assert_eq!(project.runs, vec![second.clone()]);
        assert!(!directory.path().join("runs/run-0001").exists());
        assert!(directory.path().join("runs/run-0002/report.json").exists());
        assert_eq!(load_workspace(directory.path()).unwrap().runs, vec![second]);
    }

    #[test]
    fn unsupported_future_version_is_rejected() {
        let directory = tempdir().unwrap();
        let mut project = WorkbenchProject::blank("future");
        project.format_version += 1;
        let path = directory.path().join(PROJECT_DOCUMENT_FILE);
        fs::write(&path, serde_json::to_string(&project).unwrap()).unwrap();
        assert!(matches!(
            load_workspace(directory.path()),
            Err(ProjectDocumentError::UnsupportedVersion { .. })
        ));
    }

    #[test]
    fn template_discovery_isolated_from_malformed_and_duplicate_entries() {
        let directory = tempdir().unwrap();
        for name in ["first", "second"] {
            let template_dir = directory.path().join(name);
            fs::create_dir_all(&template_dir).unwrap();
            let manifest = CaseTemplateManifest {
                format_version: WORKBENCH_PROJECT_FORMAT_VERSION,
                id: "duplicate".into(),
                name: name.into(),
                category: "test".into(),
                description: "test template".into(),
                capabilities: Vec::new(),
                expectations: TemplateExpectations::default(),
                metadata: Default::default(),
                project_template: WorkbenchProject::blank(name),
            };
            fs::write(
                template_dir.join("case.json"),
                serde_json::to_string(&manifest).unwrap(),
            )
            .unwrap();
        }
        let broken = directory.path().join("broken");
        fs::create_dir_all(&broken).unwrap();
        fs::write(broken.join("case.json"), "not JSON").unwrap();

        let discovered = discover_templates(directory.path());
        assert_eq!(discovered.iter().filter(|entry| entry.is_ok()).count(), 1);
        assert_eq!(discovered.iter().filter(|entry| entry.is_err()).count(), 2);
    }

    #[test]
    fn built_in_templates_use_the_shared_catalogue() {
        let templates = crate::workbench::catalogue::builtin_templates().unwrap();
        assert!(templates.len() >= 7);
        for id in [
            "blank-2d",
            "blank-3d",
            "lid-driven-cavity",
            "laminar-channel",
            "cylinder-flow",
            "skewed-mesh-verification",
            "channel-3d",
        ] {
            assert!(templates.iter().any(|template| template.id == id));
        }
        assert!(templates
            .iter()
            .all(|template| !template.expectations.notes.is_empty()));
        let channel = templates
            .iter()
            .find(|template| template.id == "laminar-channel")
            .unwrap();
        let cylinder = templates
            .iter()
            .find(|template| template.id == "cylinder-flow")
            .unwrap();
        let channel_3d = templates
            .iter()
            .find(|template| template.id == "channel-3d")
            .unwrap();
        assert_eq!(
            channel.project_template.mesh_recipe.boundary_layers.len(),
            1
        );
        assert_eq!(cylinder.project_template.mesh_recipe.refinements.len(), 1);
        assert_eq!(channel_3d.project_template.mesh_recipe.refinements.len(), 1);
    }

    #[test]
    fn saved_user_template_round_trips_through_combined_discovery() {
        let directory = tempdir().unwrap();
        let built_in_root = directory.path().join("built-in");
        let user_root = directory.path().join("user");
        let built_in = CaseTemplateManifest {
            format_version: WORKBENCH_PROJECT_FORMAT_VERSION,
            id: "built-in".into(),
            name: "Built in".into(),
            category: "test".into(),
            description: "built-in template".into(),
            capabilities: Vec::new(),
            expectations: TemplateExpectations::default(),
            metadata: Default::default(),
            project_template: WorkbenchProject::blank("Built in"),
        };
        save_case_template(&built_in_root, &built_in).unwrap();

        let user = CaseTemplateManifest {
            format_version: WORKBENCH_PROJECT_FORMAT_VERSION,
            id: "user-case".into(),
            name: "User Case".into(),
            category: "test".into(),
            description: "saved user template".into(),
            capabilities: vec!["Editable".into()],
            expectations: TemplateExpectations::default(),
            metadata: Default::default(),
            project_template: WorkbenchProject::blank("User Case"),
        };
        let saved_path = save_case_template(&user_root, &user).unwrap();
        assert_eq!(saved_path, user_root.join("user-case/case.json"));

        let templates = discover_templates_from_roots([&built_in_root, &user_root])
            .into_iter()
            .collect::<Result<Vec<_>, _>>()
            .unwrap();
        assert_eq!(templates, vec![built_in, user]);
    }

    #[test]
    fn saving_a_case_template_omits_project_run_history() {
        let directory = tempdir().unwrap();
        let mut project = WorkbenchProject::blank("Reusable channel");
        project.runs.push(project.next_run(RunStatus::Converged));
        let manifest = CaseTemplateManifest {
            format_version: WORKBENCH_PROJECT_FORMAT_VERSION,
            id: "reusable-channel".into(),
            name: "Reusable channel".into(),
            category: "user".into(),
            description: "intent only".into(),
            capabilities: Vec::new(),
            expectations: TemplateExpectations::default(),
            metadata: Default::default(),
            project_template: project,
        };

        save_case_template(directory.path(), &manifest).unwrap();
        let discovered = discover_templates(directory.path())
            .into_iter()
            .collect::<Result<Vec<_>, _>>()
            .unwrap();

        assert!(discovered[0].project_template.runs.is_empty());
        assert_eq!(manifest.project_template.runs.len(), 1);
        assert!(matches!(
            save_case_template(directory.path(), &manifest),
            Err(ProjectDocumentError::Invalid(_))
        ));
    }

    #[test]
    fn template_root_combination_deduplicates_roots_and_rejects_unsafe_ids() {
        let directory = tempdir().unwrap();
        let manifest = CaseTemplateManifest {
            format_version: WORKBENCH_PROJECT_FORMAT_VERSION,
            id: "safe".into(),
            name: "Safe".into(),
            category: "test".into(),
            description: "safe template".into(),
            capabilities: Vec::new(),
            expectations: TemplateExpectations::default(),
            metadata: Default::default(),
            project_template: WorkbenchProject::blank("Safe"),
        };
        save_case_template(directory.path(), &manifest).unwrap();
        let templates = discover_templates_from_roots([directory.path(), directory.path()])
            .into_iter()
            .collect::<Result<Vec<_>, _>>()
            .unwrap();
        assert_eq!(templates, vec![manifest.clone()]);

        let mut unsafe_manifest = manifest;
        unsafe_manifest.id = "../outside".into();
        assert!(matches!(
            save_case_template(directory.path(), &unsafe_manifest),
            Err(ProjectDocumentError::Invalid(_))
        ));
        assert!(!directory.path().parent().unwrap().join("outside").exists());
    }
}
