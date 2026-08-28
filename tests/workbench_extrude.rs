use flursys::{
    finalize_workspace_run, load_workspace, solve_incompressible, CadSketchPlane,
    GeometrySelectionTarget, GmshMesher, IncompressibleBoundaryCondition, MeshDimension, Vec3,
    WorkbenchSession,
};

#[test]
fn session_commits_canonical_sketch_to_extrude_feature() {
    let mut session = WorkbenchSession::new();
    let plane_sketch = session.create_sketch_on_plane(CadSketchPlane::Xy).unwrap();
    let rectangle = session.add_rectangle(2.0, 1.0).unwrap();
    let face_sketch = session.create_sketch_on_face(rectangle.face).unwrap();

    let (feature, extrusion) = session
        .extrude_sketch_face(face_sketch.id, rectangle.face, 0.5)
        .unwrap();

    assert_ne!(plane_sketch.id, face_sketch.id);
    assert_eq!(feature.body, extrusion.body);
    assert_eq!(session.geometry().extrude_features().count(), 1);
}

#[test]
fn session_materializes_a_canonical_sketch_profile_before_extrusion() {
    let mut session = WorkbenchSession::new();
    let sketch = session.create_sketch_on_plane(CadSketchPlane::Xy).unwrap();

    let face = session
        .materialize_sketch_rectangle(sketch.id, 2.0, 1.0)
        .unwrap();

    assert_eq!(
        session.geometry().sketch(sketch.id).unwrap().profile_face,
        Some(face)
    );
    assert!(session.geometry().face(face).is_some());
}

#[test]
fn body_transform_preserves_named_boundary_intent_while_invalidating_only_artifacts() {
    let mut session = WorkbenchSession::new();
    let rectangle = session.add_rectangle(2.0, 1.0).unwrap();
    let extrusion = session.extrude_face(rectangle.face, 0.5).unwrap();
    session
        .create_named_selection(
            "inlet",
            vec![GeometrySelectionTarget::Face(extrusion.side_faces[0])],
        )
        .unwrap();
    session
        .configure_named_boundary(
            "inlet",
            IncompressibleBoundaryCondition::VelocityInlet {
                velocity: Vec3::new(0.05, 0.0, 0.0),
            },
        )
        .unwrap();

    session
        .translate_body(extrusion.body, Vec3::new(1.0, 0.0, 0.0))
        .unwrap();

    assert!(session.named_selections().get("inlet").is_some());
    assert!(matches!(
        session.boundary_assignment("inlet"),
        Some(IncompressibleBoundaryCondition::VelocityInlet { velocity })
            if *velocity == Vec3::new(0.05, 0.0, 0.0)
    ));
}

#[test]
#[ignore = "requires a real gmsh executable on PATH"]
fn extruded_channel_named_faces_generate_a_real_three_dimensional_mesh() {
    let mut session = WorkbenchSession::new();
    let rectangle = session.add_rectangle(2.0, 1.0).unwrap();
    let extrusion = session.extrude_face(rectangle.face, 0.5).unwrap();
    session
        .set_mesh_configuration(MeshDimension::ThreeD, 0.3, 0.15, 0.3, 1)
        .unwrap();
    session
        .create_named_selection(
            "inlet",
            vec![GeometrySelectionTarget::Face(extrusion.side_faces[3])],
        )
        .unwrap();
    session
        .create_named_selection(
            "outlet",
            vec![GeometrySelectionTarget::Face(extrusion.side_faces[1])],
        )
        .unwrap();
    session
        .create_named_selection(
            "walls",
            vec![
                GeometrySelectionTarget::Face(extrusion.source_face),
                GeometrySelectionTarget::Face(extrusion.top_face),
                GeometrySelectionTarget::Face(extrusion.side_faces[0]),
                GeometrySelectionTarget::Face(extrusion.side_faces[2]),
            ],
        )
        .unwrap();

    let (export, options) = session.mesh_generation_inputs().unwrap();
    let generated = GmshMesher::auto()
        .generate(&export.document, &options)
        .unwrap();
    assert_eq!(generated.mesh.dimension(), MeshDimension::ThreeD);
    assert!(generated.mesh.cell_count() > 0);
    session.install_mesh(generated);
    for patch in ["inlet", "outlet", "walls"] {
        assert!(session.patch_names().iter().any(|name| name == patch));
    }
    session
        .assign_boundary(
            "inlet",
            IncompressibleBoundaryCondition::VelocityInlet {
                velocity: Vec3::new(0.05, 0.0, 0.0),
            },
        )
        .unwrap();
    session
        .assign_boundary(
            "outlet",
            IncompressibleBoundaryCondition::PressureOutlet { pressure: 0.0 },
        )
        .unwrap();
    session
        .assign_boundary("walls", IncompressibleBoundaryCondition::NoSlipWall)
        .unwrap();
    session.readiness().unwrap();

    let solution = solve_incompressible(&session.prepare_case().unwrap()).unwrap();
    assert!(solution
        .pressure
        .values()
        .iter()
        .all(|value| value.is_finite()));
    assert!(solution
        .velocity
        .values()
        .iter()
        .all(|value| value.x.is_finite() && value.y.is_finite() && value.z.is_finite()));
    assert!(solution.report.final_continuity_rms.is_finite());
    assert!(solution.report.net_boundary_flux.is_finite());
    assert!(solution.report.net_boundary_flux.abs() < 1.0e-8);
}

#[test]
#[ignore = "requires a real gmsh executable and solves a persisted 3D workbench project"]
fn professional_3d_workspace_round_trip_preserves_cad_intent_and_run_artifacts() {
    let mut session = WorkbenchSession::new();
    let sketch = session.create_sketch_on_plane(CadSketchPlane::Xy).unwrap();
    let face = session
        .materialize_sketch_rectangle(sketch.id, 2.0, 1.0)
        .unwrap();
    let (feature, extrusion) = session.extrude_sketch_face(sketch.id, face, 0.5).unwrap();
    session
        .set_mesh_configuration(MeshDimension::ThreeD, 0.3, 0.15, 0.3, 1)
        .unwrap();
    session
        .create_named_selection(
            "inlet",
            vec![GeometrySelectionTarget::Face(extrusion.side_faces[3])],
        )
        .unwrap();
    session
        .create_named_selection(
            "outlet",
            vec![GeometrySelectionTarget::Face(extrusion.side_faces[1])],
        )
        .unwrap();
    session
        .create_named_selection(
            "walls",
            vec![
                GeometrySelectionTarget::Face(extrusion.source_face),
                GeometrySelectionTarget::Face(extrusion.top_face),
                GeometrySelectionTarget::Face(extrusion.side_faces[0]),
                GeometrySelectionTarget::Face(extrusion.side_faces[2]),
            ],
        )
        .unwrap();
    session
        .configure_named_boundary(
            "inlet",
            IncompressibleBoundaryCondition::VelocityInlet {
                velocity: Vec3::new(0.05, 0.0, 0.0),
            },
        )
        .unwrap();
    session
        .configure_named_boundary(
            "outlet",
            IncompressibleBoundaryCondition::PressureOutlet { pressure: 0.0 },
        )
        .unwrap();
    session
        .configure_named_boundary("walls", IncompressibleBoundaryCondition::NoSlipWall)
        .unwrap();

    let (export, options) = session.mesh_generation_inputs().unwrap();
    session.install_mesh(
        GmshMesher::auto()
            .generate(&export.document, &options)
            .unwrap(),
    );
    session.readiness().unwrap();
    let case = session.prepare_case().unwrap();
    session.mark_solving();
    session.complete_solve(solve_incompressible(&case));
    let solution = session.solution().unwrap();
    assert!(solution.report.final_continuity_rms.is_finite());
    assert!(solution.report.net_boundary_flux.abs() < 1.0e-8);

    let workspace = tempfile::tempdir().unwrap();
    let mut project = session.to_project("Persistent 3D Channel");
    let run = finalize_workspace_run(workspace.path(), &mut project, &session)
        .unwrap()
        .unwrap();
    assert!(workspace.path().join(&run.report_path).is_file());
    assert!(workspace
        .path()
        .join(run.solution_path.as_ref().unwrap())
        .is_file());

    drop(session);
    let reloaded_project = load_workspace(workspace.path()).unwrap();
    let mut reloaded = WorkbenchSession::from_project(&reloaded_project).unwrap();
    assert!(reloaded.geometry().sketch(sketch.id).is_some());
    assert!(reloaded
        .geometry()
        .extrude_features()
        .any(|persisted| persisted.id == feature.id));
    assert_eq!(
        reloaded.geometry().body(extrusion.body).unwrap().id,
        extrusion.body
    );
    assert_eq!(reloaded_project.runs.len(), 1);
    let persisted_run = &reloaded_project.runs[0];
    assert_eq!(persisted_run.id, run.id);
    assert_eq!(persisted_run.ordinal, run.ordinal);
    assert_eq!(persisted_run.status, run.status);
    assert_eq!(persisted_run.solution_path, run.solution_path);
    assert!(persisted_run
        .continuity_residual
        .is_some_and(f64::is_finite));
    assert!(persisted_run.net_boundary_flux.is_some_and(f64::is_finite));
    for selection in ["inlet", "outlet", "walls"] {
        assert!(reloaded.named_selections().get(selection).is_some());
        assert!(reloaded.boundary_assignment(selection).is_some());
    }
    assert_eq!(reloaded.mesh_dimension(), MeshDimension::ThreeD);
    reloaded
        .translate_body(extrusion.body, Vec3::new(0.1, 0.0, 0.0))
        .unwrap();
    assert!(reloaded.mesh().is_none());
    assert!(reloaded.solution().is_none());
}
