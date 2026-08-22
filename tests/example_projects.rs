use flursys::{
    build_example, example_descriptors, expectations, ExampleProjectId, GeometrySelectionTarget,
    IncompressibleBoundaryCondition, MeshDimension,
};

#[test]
fn every_gallery_entry_builds_real_editable_workbench_state() {
    assert_eq!(example_descriptors().len(), 6);
    for descriptor in example_descriptors() {
        let session = build_example(descriptor.id).expect("example factory");
        if descriptor.id == ExampleProjectId::Blank {
            assert_eq!(session.geometry().bodies().count(), 0);
            continue;
        }
        session
            .geometry()
            .validate()
            .expect("real validated geometry");
        assert!(!session.named_selections().is_empty());
        let (_, options) = session
            .mesh_generation_inputs()
            .expect("production Gmsh inputs");
        assert_eq!(options.dimension, descriptor.dimension);
    }
}

#[test]
fn examples_use_expected_stable_selections_and_physical_intent() {
    let cavity = build_example(ExampleProjectId::LidDrivenCavity2D).unwrap();
    assert_eq!(cavity.named_selections().len(), 4);
    assert!(matches!(
        cavity.boundary_assignment("top"),
        Some(IncompressibleBoundaryCondition::MovingWall { .. })
    ));

    let cylinder = build_example(ExampleProjectId::CylinderFlow2D).unwrap();
    let cylinder_selection = cylinder.named_selections().get("cylinder").unwrap();
    assert_eq!(cylinder_selection.targets.len(), 4);
    assert!(cylinder_selection
        .targets
        .iter()
        .all(|target| matches!(target, GeometrySelectionTarget::Edge(_))));
    assert!(expectations(ExampleProjectId::CylinderFlow2D).expects_curved_boundary);

    let channel_3d = build_example(ExampleProjectId::Channel3D).unwrap();
    assert_eq!(
        channel_3d.mesh_generation_inputs().unwrap().1.dimension,
        MeshDimension::ThreeD
    );
    assert!(channel_3d
        .named_selections()
        .get("walls")
        .unwrap()
        .targets
        .iter()
        .all(|target| matches!(target, GeometrySelectionTarget::Face(_))));
}
