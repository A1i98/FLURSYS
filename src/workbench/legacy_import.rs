//! Read-only import of historical `.flursys.json` projects.
//!
//! This module is the only bridge from the former structured `Project` schema
//! into canonical workbench workspaces. It never writes the source file and it
//! never participates in normal workspace save, load, or solver execution.

use super::{save_workspace, GeometrySelectionTarget, WorkbenchProject, WorkbenchSession};
use crate::{
    BoundaryConditionKind, BoundaryFace, IncompressibleBoundaryCondition, MeshDimension, Project,
    ProjectCase, Vec3,
};
use std::fs;
use std::path::{Path, PathBuf};

/// Historical project schemas recognised by the read-only importer.
pub const LEGACY_PROJECT_FORMATS: &[(u32, &str)] = &[
    (1, "FLURSYS structured project v1"),
    (2, "FLURSYS structured project v2"),
];

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LegacyProjectInspection {
    pub source_format: String,
    pub detected_project_name: String,
    pub recoverable_items: Vec<String>,
    pub skipped_items: Vec<String>,
    pub warnings: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MigrationReport {
    pub source_format: String,
    pub detected_project_name: String,
    pub migrated_items: Vec<String>,
    pub skipped_items: Vec<String>,
    pub warnings: Vec<String>,
    pub destination: PathBuf,
}

#[derive(Clone, Debug)]
pub struct MigrationResult {
    pub project: WorkbenchProject,
    pub report: MigrationReport,
}

pub fn is_legacy_project_path(path: &Path) -> bool {
    path.file_name()
        .and_then(|name| name.to_str())
        .is_some_and(|name| name.ends_with(".flursys.json"))
}

/// Inspects a legacy source without creating or modifying any workspace.
pub fn inspect_legacy_project(path: &Path) -> Result<LegacyProjectInspection, String> {
    let project = read_legacy_project(path)?;
    let mut recoverable_items = vec![
        "Geometry".into(),
        "Material".into(),
        "Solver settings".into(),
        "Mesh configuration".into(),
    ];
    if !project.preprocessing.boundaries.is_empty() {
        recoverable_items.push("Boundary setup".into());
    }
    Ok(LegacyProjectInspection {
        source_format: legacy_format_name(project.format_version)?.into(),
        detected_project_name: project.name.clone(),
        recoverable_items,
        skipped_items: vec!["Generated mesh".into(), "Historical solver results".into()],
        warnings: legacy_warnings(&project),
    })
}

/// Converts a legacy source to canonical workbench intent without writing it.
pub fn migrate_legacy_project(path: &Path) -> Result<MigrationResult, String> {
    let project = read_legacy_project(path)?;
    let inspection = inspect_project(&project)?;
    let mut session = WorkbenchSession::new();
    let dimension = if matches!(
        project.workbench.dimension,
        crate::AnalysisDimension::ThreeD
    ) {
        MeshDimension::ThreeD
    } else {
        MeshDimension::TwoD
    };

    let migrated_geometry = install_case_geometry(&mut session, &project.case, dimension)?;
    session
        .set_mesh_configuration(
            dimension,
            mesh_size_from_legacy(&project),
            mesh_size_from_legacy(&project) * 0.5,
            mesh_size_from_legacy(&project) * 2.0,
            1,
        )
        .map_err(|error| error.to_string())?;
    session
        .set_material(
            legacy_density(&project.case),
            legacy_viscosity(&project.case),
        )
        .map_err(|error| error.to_string())?;
    session
        .set_solver_controls(
            project.solver.max_iterations,
            project.solver.velocity_relaxation,
            project.solver.pressure_relaxation,
            project.solver.steady_tolerance,
        )
        .map_err(|error| error.to_string())?;

    install_boundaries(&mut session, &project, migrated_geometry)?;
    let document = session.to_project(project.name.clone());
    document.validate().map_err(|error| error.to_string())?;
    WorkbenchSession::from_project(&document).map_err(|error| error.to_string())?;

    Ok(MigrationResult {
        project: document,
        report: MigrationReport {
            source_format: inspection.source_format,
            detected_project_name: inspection.detected_project_name,
            migrated_items: inspection.recoverable_items,
            skipped_items: inspection.skipped_items,
            warnings: inspection.warnings,
            destination: PathBuf::new(),
        },
    })
}

/// Creates a new canonical workspace atomically. The legacy source is read-only.
pub fn import_legacy_project(path: &Path, destination: &Path) -> Result<MigrationResult, String> {
    if destination.exists() {
        return Err(format!(
            "import destination already exists: {}; choose a new workspace folder",
            destination.display()
        ));
    }
    let mut result = migrate_legacy_project(path)?;
    let parent = destination.parent().ok_or_else(|| {
        format!(
            "import destination has no parent directory: {}",
            destination.display()
        )
    })?;
    fs::create_dir_all(parent).map_err(|error| format!("cannot create import parent: {error}"))?;
    let temporary = parent.join(format!(
        ".{}.importing-{}",
        destination
            .file_name()
            .and_then(|name| name.to_str())
            .unwrap_or("workspace"),
        std::process::id()
    ));
    if temporary.exists() {
        return Err(format!(
            "temporary import workspace already exists: {}",
            temporary.display()
        ));
    }
    if let Err(error) = save_workspace(&temporary, &result.project) {
        let _ = fs::remove_dir_all(&temporary);
        return Err(error.to_string());
    }
    if let Err(error) = fs::rename(&temporary, destination) {
        let _ = fs::remove_dir_all(&temporary);
        return Err(format!("cannot finalize imported workspace: {error}"));
    }
    result.report.destination = destination.to_path_buf();
    Ok(result)
}

#[derive(Clone)]
struct MigratedGeometry {
    left: Vec<GeometrySelectionTarget>,
    right: Vec<GeometrySelectionTarget>,
    bottom: Vec<GeometrySelectionTarget>,
    top: Vec<GeometrySelectionTarget>,
    cylinder: Option<Vec<GeometrySelectionTarget>>,
}

fn install_case_geometry(
    session: &mut WorkbenchSession,
    case: &ProjectCase,
    dimension: MeshDimension,
) -> Result<MigratedGeometry, String> {
    if dimension == MeshDimension::ThreeD {
        let (length, height) = case_size(case);
        let box_entities = session
            .add_box(length, height, legacy_depth(case))
            .map_err(|error| error.to_string())?;
        return Ok(MigratedGeometry {
            left: vec![GeometrySelectionTarget::Face(box_entities.x_min)],
            right: vec![GeometrySelectionTarget::Face(box_entities.x_max)],
            bottom: vec![GeometrySelectionTarget::Face(box_entities.y_min)],
            top: vec![GeometrySelectionTarget::Face(box_entities.y_max)],
            cylinder: None,
        });
    }
    match case {
        ProjectCase::Cylinder {
            length,
            height,
            diameter,
            center_x,
            center_y,
            ..
        } => {
            let (rectangle, hole) = session
                .add_rectangle_with_circle(
                    *length,
                    *height,
                    Vec3::new(*center_x, *center_y, 0.0),
                    diameter * 0.5,
                )
                .map_err(|error| error.to_string())?;
            Ok(MigratedGeometry {
                left: vec![GeometrySelectionTarget::Edge(rectangle.left)],
                right: vec![GeometrySelectionTarget::Edge(rectangle.right)],
                bottom: vec![GeometrySelectionTarget::Edge(rectangle.bottom)],
                top: vec![GeometrySelectionTarget::Edge(rectangle.top)],
                cylinder: Some(
                    hole.boundary
                        .into_iter()
                        .map(GeometrySelectionTarget::Edge)
                        .collect(),
                ),
            })
        }
        ProjectCase::BackwardFacingStep {
            length,
            height,
            step_height,
            step_x,
            ..
        } => {
            if !(*step_height > 0.0 && *step_height < *height && *step_x > 0.0 && *step_x < *length)
            {
                return Err("legacy backward-facing step has invalid dimensions and cannot be migrated safely".into());
            }
            let geometry = session.geometry_mut();
            let points = [
                (0.0, *step_height),
                (*step_x, *step_height),
                (*step_x, 0.0),
                (*length, 0.0),
                (*length, *height),
                (0.0, *height),
            ];
            let vertices = points
                .into_iter()
                .map(|(x, y)| geometry.add_vertex(Vec3::new(x, y, 0.0)))
                .collect::<Result<Vec<_>, _>>()
                .map_err(|error| error.to_string())?;
            let edges = (0..vertices.len())
                .map(|index| {
                    geometry.add_line(vertices[index], vertices[(index + 1) % vertices.len()])
                })
                .collect::<Result<Vec<_>, _>>()
                .map_err(|error| error.to_string())?;
            let face = geometry
                .add_planar_face(
                    edges
                        .iter()
                        .copied()
                        .map(|edge| crate::OrientedEdge {
                            edge,
                            reversed: false,
                        })
                        .collect(),
                    Vec::new(),
                )
                .map_err(|error| error.to_string())?;
            session.geometry_changed();
            let _ = face;
            Ok(MigratedGeometry {
                left: vec![GeometrySelectionTarget::Edge(edges[5])],
                right: vec![GeometrySelectionTarget::Edge(edges[3])],
                bottom: vec![
                    GeometrySelectionTarget::Edge(edges[0]),
                    GeometrySelectionTarget::Edge(edges[1]),
                    GeometrySelectionTarget::Edge(edges[2]),
                ],
                top: vec![GeometrySelectionTarget::Edge(edges[4])],
                cylinder: None,
            })
        }
        _ => {
            let (length, height) = case_size(case);
            let rectangle = session
                .add_rectangle(length, height)
                .map_err(|error| error.to_string())?;
            Ok(MigratedGeometry {
                left: vec![GeometrySelectionTarget::Edge(rectangle.left)],
                right: vec![GeometrySelectionTarget::Edge(rectangle.right)],
                bottom: vec![GeometrySelectionTarget::Edge(rectangle.bottom)],
                top: vec![GeometrySelectionTarget::Edge(rectangle.top)],
                cylinder: None,
            })
        }
    }
}

fn install_boundaries(
    session: &mut WorkbenchSession,
    project: &Project,
    geometry: MigratedGeometry,
) -> Result<(), String> {
    let entries = [
        ("inlet", BoundaryFace::Left, geometry.left),
        ("outlet", BoundaryFace::Right, geometry.right),
        ("bottom_wall", BoundaryFace::Bottom, geometry.bottom),
        ("top_wall", BoundaryFace::Top, geometry.top),
    ];
    for (name, face, target) in entries {
        session
            .create_named_selection(name, target)
            .map_err(|error| error.to_string())?;
        let legacy = project
            .preprocessing
            .boundaries
            .iter()
            .find(|boundary| boundary.face == face);
        let condition = legacy
            .and_then(|boundary| map_boundary(&boundary.kind))
            .unwrap_or_else(|| default_case_boundary(&project.case, face));
        session
            .configure_named_boundary(name, condition)
            .map_err(|error| error.to_string())?;
    }
    if let Some(targets) = geometry.cylinder {
        session
            .create_named_selection("cylinder", targets)
            .map_err(|error| error.to_string())?;
        session
            .configure_named_boundary("cylinder", IncompressibleBoundaryCondition::NoSlipWall)
            .map_err(|error| error.to_string())?;
    }
    Ok(())
}

fn map_boundary(kind: &BoundaryConditionKind) -> Option<IncompressibleBoundaryCondition> {
    match kind {
        BoundaryConditionKind::Velocity { u, v, w } => {
            Some(IncompressibleBoundaryCondition::VelocityInlet {
                velocity: Vec3::new(*u, *v, *w),
            })
        }
        BoundaryConditionKind::PressureOutlet { pressure } => {
            Some(IncompressibleBoundaryCondition::PressureOutlet {
                pressure: *pressure,
            })
        }
        BoundaryConditionKind::Wall { u, v, w } if *u == 0.0 && *v == 0.0 && *w == 0.0 => {
            Some(IncompressibleBoundaryCondition::NoSlipWall)
        }
        BoundaryConditionKind::Wall { u, v, w } => {
            Some(IncompressibleBoundaryCondition::MovingWall {
                velocity: Vec3::new(*u, *v, *w),
            })
        }
        BoundaryConditionKind::CaseDefault | BoundaryConditionKind::Symmetry => None,
    }
}

fn default_case_boundary(
    case: &ProjectCase,
    face: BoundaryFace,
) -> IncompressibleBoundaryCondition {
    match (case, face) {
        (ProjectCase::LidDrivenCavity { lid_velocity, .. }, BoundaryFace::Top) => {
            IncompressibleBoundaryCondition::MovingWall {
                velocity: Vec3::new(*lid_velocity, 0.0, 0.0),
            }
        }
        (ProjectCase::LidDrivenCavity { .. }, _) => IncompressibleBoundaryCondition::NoSlipWall,
        (
            ProjectCase::Cylinder {
                freestream_velocity,
                ..
            }
            | ProjectCase::Channel {
                mean_velocity: freestream_velocity,
                ..
            }
            | ProjectCase::BackwardFacingStep {
                mean_velocity: freestream_velocity,
                ..
            },
            BoundaryFace::Left,
        ) => IncompressibleBoundaryCondition::VelocityInlet {
            velocity: Vec3::new(*freestream_velocity, 0.0, 0.0),
        },
        (
            ProjectCase::Cylinder { .. }
            | ProjectCase::Channel { .. }
            | ProjectCase::BackwardFacingStep { .. },
            BoundaryFace::Right,
        ) => IncompressibleBoundaryCondition::PressureOutlet { pressure: 0.0 },
        _ => IncompressibleBoundaryCondition::NoSlipWall,
    }
}

fn read_legacy_project(path: &Path) -> Result<Project, String> {
    if !is_legacy_project_path(path) {
        return Err(format!(
            "expected a legacy .flursys.json file: {}",
            path.display()
        ));
    }
    let project = Project::load(path)?;
    legacy_format_name(project.format_version)?;
    Ok(project)
}

fn inspect_project(project: &Project) -> Result<LegacyProjectInspection, String> {
    Ok(LegacyProjectInspection {
        source_format: legacy_format_name(project.format_version)?.into(),
        detected_project_name: project.name.clone(),
        recoverable_items: vec![
            "Geometry".into(),
            "Boundary setup".into(),
            "Material".into(),
            "Solver settings".into(),
            "Mesh configuration".into(),
        ],
        skipped_items: vec!["Generated mesh".into(), "Historical solver results".into()],
        warnings: legacy_warnings(project),
    })
}

fn legacy_warnings(project: &Project) -> Vec<String> {
    let mut warnings = vec!["Generated legacy mesh and runtime results are intentionally not imported; regenerate with Gmsh.".into()];
    if project.physics != crate::PhysicsSettings::default() {
        warnings.push("Thermal and buoyancy settings are not represented by the current steady incompressible workbench case and were not imported.".into());
    }
    warnings
}

fn legacy_format_name(version: u32) -> Result<&'static str, String> {
    LEGACY_PROJECT_FORMATS
        .iter()
        .find_map(|(known, name)| (*known == version).then_some(*name))
        .ok_or_else(|| format!("unsupported legacy project format version {version}"))
}

fn case_size(case: &ProjectCase) -> (f64, f64) {
    match case {
        ProjectCase::LidDrivenCavity { length, height, .. }
        | ProjectCase::Cylinder { length, height, .. }
        | ProjectCase::BackwardFacingStep { length, height, .. }
        | ProjectCase::Channel { length, height, .. } => (*length, *height),
    }
}

fn legacy_depth(_case: &ProjectCase) -> f64 {
    1.0
}

fn legacy_density(case: &ProjectCase) -> f64 {
    match case {
        ProjectCase::LidDrivenCavity { density, .. }
        | ProjectCase::Cylinder { density, .. }
        | ProjectCase::BackwardFacingStep { density, .. }
        | ProjectCase::Channel { density, .. } => *density,
    }
}

fn legacy_viscosity(case: &ProjectCase) -> f64 {
    match case {
        ProjectCase::LidDrivenCavity {
            length,
            lid_velocity,
            reynolds,
            ..
        } => lid_velocity * length / reynolds,
        ProjectCase::Cylinder {
            diameter,
            freestream_velocity,
            reynolds,
            ..
        } => freestream_velocity * diameter / reynolds,
        ProjectCase::BackwardFacingStep {
            step_height,
            mean_velocity,
            reynolds,
            ..
        } => mean_velocity * step_height / reynolds,
        ProjectCase::Channel {
            height,
            mean_velocity,
            reynolds,
            ..
        } => mean_velocity * height / reynolds,
    }
}

fn mesh_size_from_legacy(project: &Project) -> f64 {
    let (length, height) = case_size(&project.case);
    (length.min(height) / project.solver.nx.max(project.solver.ny) as f64).max(1.0e-6)
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::tempdir;

    #[test]
    fn literal_legacy_cavity_import_creates_a_standalone_workspace() {
        let fixture = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/legacy/cavity-v2.flursys.json");
        let source_before = fs::read_to_string(&fixture).unwrap();
        let directory = tempdir().unwrap();
        let destination = directory.path().join("cavity.flursys");
        let result = import_legacy_project(&fixture, &destination).unwrap();
        assert!(destination.join("project.json").is_file());
        assert!(result.project.geometry.faces().next().is_some());
        assert!(!result.project.runs.len() > 0);
        assert!(super::super::load_workspace(&destination).is_ok());
        assert_eq!(fs::read_to_string(&fixture).unwrap(), source_before);
    }

    #[test]
    fn literal_legacy_channel_recovers_named_boundaries_and_mesh_inputs() {
        let fixture = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/legacy/channel-v2.flursys.json");
        let result = migrate_legacy_project(&fixture).unwrap();
        assert!(result.project.geometry.edges().count() >= 4);
        assert!(result
            .project
            .named_selections
            .iter()
            .any(|selection| selection.name == "inlet"));
        let session = WorkbenchSession::from_project(&result.project).unwrap();
        assert!(session.mesh_generation_inputs().is_ok());
        assert!(!session.has_mesh());
    }

    #[test]
    fn literal_cylinder_and_step_fixtures_recover_canonical_geometry() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/legacy");
        let cylinder = migrate_legacy_project(&root.join("cylinder-v2.flursys.json")).unwrap();
        assert!(cylinder
            .project
            .named_selections
            .iter()
            .any(|selection| selection.name == "cylinder"));
        assert!(WorkbenchSession::from_project(&cylinder.project)
            .unwrap()
            .mesh_generation_inputs()
            .is_ok());

        let step = migrate_legacy_project(&root.join("backward-step-v2.flursys.json")).unwrap();
        assert_eq!(step.project.geometry.edges().count(), 6);
        assert!(WorkbenchSession::from_project(&step.project)
            .unwrap()
            .mesh_generation_inputs()
            .is_ok());
    }

    #[test]
    fn malformed_legacy_source_creates_no_workspace() {
        let directory = tempdir().unwrap();
        let source = directory.path().join("broken.flursys.json");
        let destination = directory.path().join("broken.flursys");
        fs::write(&source, "{not json").unwrap();

        assert!(import_legacy_project(&source, &destination).is_err());
        assert!(!destination.exists());
    }

    #[test]
    fn migrated_cases_generate_real_gmsh_meshes_and_cavity_solves() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/legacy");
        for fixture in [
            "cavity-v2.flursys.json",
            "channel-v2.flursys.json",
            "cylinder-v2.flursys.json",
            "backward-step-v2.flursys.json",
        ] {
            let result = migrate_legacy_project(&root.join(fixture)).unwrap();
            let mut session = WorkbenchSession::from_project(&result.project).unwrap();
            let (export, options) = session.mesh_generation_inputs().unwrap();
            let generated = crate::GmshMesher::auto()
                .generate(&export.document, &options)
                .unwrap_or_else(|error| panic!("{fixture}: {error}"));
            assert!(generated.mesh.cell_count() > 0, "{fixture}");
            session.install_mesh(generated);
            assert!(session.has_mesh(), "{fixture}");

            if fixture == "cavity-v2.flursys.json" {
                let case = session.prepare_case().unwrap();
                let solution = crate::solve_incompressible(&case).unwrap();
                assert!(solution
                    .pressure
                    .values()
                    .iter()
                    .all(|value| value.is_finite()));
            }
        }
    }
}
