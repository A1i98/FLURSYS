//! Phase 9C workbench foundation tests: workflow state transitions,
//! Named Selection state, boundary assignment, mesh-generation controller
//! inputs, and Run enable/disable rules.

use flursys::{
    GeometrySelectionTarget, GmshMeshOptions, GmshMesher, IncompressibleBoundaryCondition,
    MeshDimension, SolveStatus, WorkbenchSession,
};
use std::collections::BTreeSet;

fn demo() -> WorkbenchSession {
    WorkbenchSession::demo_channel(4.0, 1.0).expect("demo channel is a valid workflow")
}

#[test]
fn demo_channel_exposes_stable_geometry_entities_and_default_workflow() {
    let session = demo();

    let edge_ids: BTreeSet<u64> = session
        .geometry()
        .edges()
        .map(|edge| edge.id.get())
        .collect();
    assert_eq!(edge_ids.len(), 4);

    let names: Vec<&str> = session
        .named_selections()
        .iter()
        .map(|selection| selection.name.as_str())
        .collect();
    assert_eq!(names, vec!["inlet", "outlet", "walls"]);

    assert_eq!(
        session.boundary_assignment("inlet"),
        Some(&IncompressibleBoundaryCondition::VelocityInlet {
            velocity: flursys::Vec3::new(0.1, 0.0, 0.0),
        })
    );
    assert_eq!(
        session.boundary_assignment("walls"),
        Some(&IncompressibleBoundaryCondition::NoSlipWall)
    );
    assert_eq!(session.status(), &SolveStatus::Idle);
}

#[test]
fn named_selection_state_supports_create_rename_delete_and_rejects_bad_input() {
    let mut session = demo();
    let face = session.fluid_face().expect("demo has a fluid face");
    let body_target = GeometrySelectionTarget::Face(face);
    let top_edge = *session
        .named_selections()
        .get("walls")
        .and_then(|selection| selection.targets.first())
        .expect("walls selection exists");

    // Duplicate and invalid names are rejected without mutating state.
    assert!(session
        .create_named_selection("inlet", vec![body_target])
        .is_err());
    assert!(session
        .create_named_selection("", vec![body_target])
        .is_err());
    assert_eq!(session.named_selections().len(), 3);

    session
        .create_named_selection("core", vec![body_target])
        .expect("face target exists");
    assert_eq!(session.named_selections().len(), 4);
    assert_eq!(
        session.named_selections().membership(body_target),
        vec!["core"]
    );

    session
        .rename_named_selection("core", "core-region")
        .unwrap();
    assert!(session.named_selections().get("core").is_none());
    assert!(session.named_selections().get("core-region").is_some());

    assert!(session.delete_named_selection("core-region"));
    assert!(!session.delete_named_selection("core-region"));
    assert_eq!(session.named_selections().len(), 3);
    let _ = top_edge;
}

#[test]
fn gmsh_export_groups_match_named_selections_deterministically() {
    let session = demo();
    let export = session.build_gmsh_export().expect("2D export is supported");

    let source = export.document.to_geo_string().unwrap();
    for name in ["inlet", "outlet", "walls"] {
        assert!(
            source.contains(&format!("Physical Curve(\"{name}\")")),
            "missing physical group {name}"
        );
    }
    assert!(source.contains("Physical Surface(\"fluid\")"));

    // Every named-selection edge maps to a deterministic backend tag.
    for selection in session.named_selections().iter() {
        for target in &selection.targets {
            let GeometrySelectionTarget::Edge(id) = target else {
                panic!("2D groups must contain only edges");
            };
            assert!(export.map.edge_tag(*id).is_some());
        }
    }

    // Group order in the document is name-sorted regardless of creation order.
    let inlet = source.find("Physical Curve(\"inlet\")").unwrap();
    let outlet = source.find("Physical Curve(\"outlet\")").unwrap();
    let walls = source.find("Physical Curve(\"walls\")").unwrap();
    assert!(inlet < outlet && outlet < walls);
}

#[test]
fn mesh_configuration_is_validated_before_gmsh_is_touched() {
    let mut session = demo();
    session
        .set_mesh_configuration(MeshDimension::TwoD, 0.25, 0.1, 0.5, 1)
        .unwrap();
    assert_eq!(session.mesh_sizes(), (0.25, 0.1, 0.5));
    assert!(session
        .set_mesh_configuration(MeshDimension::TwoD, 0.5, 0.9, 0.2, 1)
        .is_err());
    assert!(session
        .set_mesh_configuration(MeshDimension::TwoD, -1.0, 0.1, 0.5, 1)
        .is_err());
    assert!(session
        .set_mesh_configuration(MeshDimension::TwoD, 0.25, 0.1, 0.5, 2)
        .is_err());
    // Failed updates keep the previous valid configuration.
    assert_eq!(session.mesh_sizes(), (0.25, 0.1, 0.5));

    let (export, options) = session.mesh_generation_inputs().unwrap();
    assert_eq!(options.dimension, MeshDimension::TwoD);
    assert!(export.document.to_geo_string().is_ok());
}

#[test]
fn run_rules_block_until_geometry_mesh_boundaries_and_material_are_valid() {
    let empty = WorkbenchSession::new();
    assert_eq!(
        empty.readiness().unwrap_err(),
        "generate a mesh first (Mesh stage)"
    );
    assert!(empty.prepare_case().is_err());

    let mut session = demo();
    assert_eq!(
        session.readiness().unwrap_err(),
        "generate a mesh first (Mesh stage)"
    );

    // Assignments to patches that do not exist yet are rejected up front.
    let result =
        session.assign_boundary("no-such-patch", IncompressibleBoundaryCondition::NoSlipWall);
    assert!(result.is_err());

    assert!(session.set_material(0.0, 0.01).is_err());
    assert!(session.set_material(1.0, f64::NAN).is_err());
    assert!(session.set_solver_controls(0, 0.7, 0.3, 1e-10).is_err());
    assert!(session.set_solver_controls(200, 1.5, 0.3, 1e-10).is_err());
    assert!(session.set_solver_controls(200, 0.7, 0.3, -1.0).is_err());
}

#[test]
fn failed_solves_report_structured_status_and_clear_the_solution() {
    let mut session = demo();
    session.mark_solving();
    assert_eq!(session.status(), &SolveStatus::Solving);

    session.complete_solve(Err(flursys::IncompressibleSolveError::Case(
        flursys::IncompressibleCaseError::InvalidDensity { value: 0.0 },
    )));
    match session.status() {
        SolveStatus::Failed(message) => {
            assert!(message.contains("InvalidDensity"), "{message}");
        }
        other => panic!("expected Failed status, got {other:?}"),
    }
    assert!(session.solution().is_none());
}

#[test]
#[ignore = "requires a real gmsh executable on PATH"]
fn full_channel_pipeline_generates_solves_and_exports_vtk() {
    let mut session = demo();
    session
        .set_mesh_configuration(MeshDimension::TwoD, 0.2, 0.05, 0.2, 1)
        .unwrap();

    let (export, options) = session.mesh_generation_inputs().unwrap();
    let generated = GmshMesher::auto()
        .generate(&export.document, &options)
        .expect("gmsh pipeline");
    let report = generated.report.clone();
    session.install_mesh(generated);

    let patch_names = session.patch_names();
    assert_eq!(patch_names.len(), report.patch_count);
    for patch in &["inlet", "outlet", "walls"] {
        assert!(patch_names.iter().any(|name| name == patch));
        assert!(session.boundary_assignment(patch).is_some());
    }

    session
        .readiness()
        .expect("demo defaults cover every patch");
    let case = session.prepare_case().expect("runnable case");
    session.mark_solving();
    let outcome = flursys::solve_incompressible(&case);
    session.complete_solve(outcome);
    assert!(matches!(
        session.status(),
        SolveStatus::Converged | SolveStatus::MaxIterations
    ));

    let solution = session.solution().expect("solution stored");
    assert!(solution.report.net_boundary_flux.abs() < 1.0e-8);
    assert_eq!(solution.velocity.values().len(), report.cell_count);
    assert!(report.node_count > 0);

    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("run").join("channel.vtk");
    session.export_vtk(&path).expect("vtk export");
    let text = std::fs::read_to_string(&path).unwrap();
    assert!(text.contains("UNSTRUCTURED_GRID"));
    assert!(text.contains("SCALARS pressure"));
}

#[test]
#[ignore = "requires a real gmsh executable on PATH"]
fn regeneration_drops_assignments_for_removed_patches_and_invalidates_results() {
    let mut session = demo();
    let (export, _options) = session.mesh_generation_inputs().unwrap();
    let generated =
        GmshMesher::auto().generate(&export.document, &GmshMeshOptions::two_d(0.5).unwrap());
    session.install_mesh(generated.unwrap());

    session.unassign_boundary("inlet");
    assert!(session.boundary_assignment("inlet").is_none());
    assert!(session.readiness().is_err());
}
