use flursys::{
    CadSketchPlane, GeometrySelectionTarget, GmshMesher, IncompressibleBoundaryCondition,
    MeshDimension, Vec3, WorkbenchSession,
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
            vec![GeometrySelectionTarget::Face(extrusion.side_faces[0])],
        )
        .unwrap();
    session
        .create_named_selection(
            "outlet",
            vec![GeometrySelectionTarget::Face(extrusion.side_faces[2])],
        )
        .unwrap();
    session
        .create_named_selection(
            "walls",
            vec![
                GeometrySelectionTarget::Face(extrusion.source_face),
                GeometrySelectionTarget::Face(extrusion.top_face),
                GeometrySelectionTarget::Face(extrusion.side_faces[1]),
                GeometrySelectionTarget::Face(extrusion.side_faces[3]),
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
}
