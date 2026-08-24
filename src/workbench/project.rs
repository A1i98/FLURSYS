//! Versioned, portable project-workspace documents for the unstructured workbench.
//!
//! `WorkbenchProject` intentionally contains user intent and persistent run
//! metadata only. Meshes, VTK fields, renderer caches, and worker handles are
//! project-local derived artifacts rather than JSON payloads.

use super::{GeometrySelectionTarget, NamedSelection};
use crate::{GeometryTopology, MeshDimension, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fs::{self, File};
use std::io::Write;
use std::path::{Component, Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

pub const WORKBENCH_PROJECT_FORMAT_VERSION: u32 = 1;
pub const PROJECT_DOCUMENT_FILE: &str = "project.json";

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum PhysicalBoundaryCondition {
    NoSlipWall,
    MovingWall { velocity: Vec3 },
    VelocityInlet { velocity: Vec3 },
    PressureOutlet { pressure: f64 },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BoundaryAssignment {
    pub selection: String,
    pub condition: PhysicalBoundaryCondition,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct WorkbenchMeshSettings {
    pub dimension: MeshDimension,
    pub global_size: f64,
    pub min_size: f64,
    pub max_size: f64,
    pub element_order: u8,
}

impl Default for WorkbenchMeshSettings {
    fn default() -> Self {
        Self {
            dimension: MeshDimension::TwoD,
            global_size: 0.1,
            min_size: 0.05,
            max_size: 0.2,
            element_order: 1,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct WorkbenchMaterial {
    pub density: f64,
    pub kinematic_viscosity: f64,
}

impl Default for WorkbenchMaterial {
    fn default() -> Self {
        Self {
            density: 1.0,
            kinematic_viscosity: 0.01,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct WorkbenchSolverSettings {
    pub max_outer_iterations: usize,
    pub continuity_absolute_tolerance: f64,
    pub velocity_relaxation: f64,
    pub pressure_relaxation: f64,
}

impl Default for WorkbenchSolverSettings {
    fn default() -> Self {
        Self {
            max_outer_iterations: 200,
            continuity_absolute_tolerance: 1.0e-10,
            velocity_relaxation: 0.7,
            pressure_relaxation: 0.3,
        }
    }
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
    pub report_path: PathBuf,
    pub solution_path: Option<PathBuf>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct WorkbenchProject {
    pub format_version: u32,
    pub project_id: String,
    pub name: String,
    pub geometry: GeometryTopology,
    pub named_selections: Vec<NamedSelection>,
    pub mesh: WorkbenchMeshSettings,
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
            format_version: WORKBENCH_PROJECT_FORMAT_VERSION,
            project_id: unique_id("project"),
            name: name.into(),
            geometry: GeometryTopology::new(),
            named_selections: Vec::new(),
            mesh: WorkbenchMeshSettings::default(),
            boundaries: Vec::new(),
            material: WorkbenchMaterial::default(),
            solver: WorkbenchSolverSettings::default(),
            runs: Vec::new(),
        }
    }

    pub fn validate(&self) -> Result<(), ProjectDocumentError> {
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
        self.geometry
            .validate()
            .map_err(|error| ProjectDocumentError::Invalid(error.to_string()))?;
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
    pub project_template: WorkbenchProject,
}

impl CaseTemplateManifest {
    pub fn validate(&self) -> Result<(), ProjectDocumentError> {
        if self.format_version != WORKBENCH_PROJECT_FORMAT_VERSION
            || self.id.trim().is_empty()
            || self.name.trim().is_empty()
        {
            return Err(ProjectDocumentError::Invalid(
                "invalid case template manifest".into(),
            ));
        }
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
    let destination = workspace.join(PROJECT_DOCUMENT_FILE);
    atomic_json_write(&destination, project)?;
    let recovery = workspace.join("autosave").join(PROJECT_DOCUMENT_FILE);
    if recovery.exists() {
        fs::remove_file(recovery).map_err(io_error)?;
    }
    Ok(())
}

pub fn load_workspace(workspace: &Path) -> Result<WorkbenchProject, ProjectDocumentError> {
    let path = workspace.join(PROJECT_DOCUMENT_FILE);
    let text = fs::read_to_string(&path).map_err(io_error)?;
    let project: WorkbenchProject = serde_json::from_str(&text)
        .map_err(|error| ProjectDocumentError::Json(error.to_string()))?;
    project.validate()?;
    Ok(project)
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
    atomic_json_write(&destination, manifest)?;
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

fn validate_boundary(condition: &PhysicalBoundaryCondition) -> Result<(), ProjectDocumentError> {
    let values: &[f64] = match condition {
        PhysicalBoundaryCondition::NoSlipWall => &[],
        PhysicalBoundaryCondition::MovingWall { velocity }
        | PhysicalBoundaryCondition::VelocityInlet { velocity } => {
            &[velocity.x, velocity.y, velocity.z]
        }
        PhysicalBoundaryCondition::PressureOutlet { pressure } => std::slice::from_ref(pressure),
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
    format!("{prefix}-{}", now_ms())
}
fn io_error(error: std::io::Error) -> ProjectDocumentError {
    ProjectDocumentError::Io(error.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{IncompressibleBoundaryCondition, WorkbenchSession};
    use tempfile::tempdir;

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
    fn built_in_blank_templates_are_discovered_from_files() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("cases/templates");
        let templates = discover_templates(&root)
            .into_iter()
            .collect::<Result<Vec<_>, _>>()
            .unwrap();
        assert!(templates.iter().any(|template| template.id == "blank-2d"));
        assert!(templates.iter().any(|template| template.id == "blank-3d"));
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
    fn template_root_combination_deduplicates_roots_and_rejects_unsafe_ids() {
        let directory = tempdir().unwrap();
        let manifest = CaseTemplateManifest {
            format_version: WORKBENCH_PROJECT_FORMAT_VERSION,
            id: "safe".into(),
            name: "Safe".into(),
            category: "test".into(),
            description: "safe template".into(),
            capabilities: Vec::new(),
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
