use flursys::{
    GeometryError, GeometryFaceRepresentation, GeometryTopology, GmshGeometryExporter,
    GmshMeshOptions, GmshMesher, Vec3,
};

#[test]
fn rectangle_entities_are_stable_across_an_unrelated_edit() {
    let mut geometry = GeometryTopology::new();
    let rectangle = geometry.add_rectangle(2.0, 1.0).unwrap();
    let revision_after_rectangle = geometry.revision();

    let unrelated = geometry.add_vertex(Vec3::new(10.0, 0.0, 0.0)).unwrap();

    assert_eq!(
        geometry.revision().get(),
        revision_after_rectangle.get() + 1
    );
    assert_eq!(geometry.face(rectangle.face).unwrap().id, rectangle.face);
    assert_eq!(geometry.edge(rectangle.left).unwrap().id, rectangle.left);
    assert_eq!(geometry.edge(rectangle.right).unwrap().id, rectangle.right);
    assert_eq!(
        geometry.edge(rectangle.bottom).unwrap().id,
        rectangle.bottom
    );
    assert_eq!(geometry.edge(rectangle.top).unwrap().id, rectangle.top);
    assert_eq!(geometry.vertex(unrelated).unwrap().id, unrelated);
    assert_eq!(geometry.vertices().count(), 5);
    assert_eq!(geometry.edges().count(), 4);
    assert_eq!(geometry.faces().count(), 1);
}

#[test]
fn deletion_is_dependency_safe_and_never_reuses_an_id() {
    let mut geometry = GeometryTopology::new();
    let start = geometry.add_vertex(Vec3::new(0.0, 0.0, 0.0)).unwrap();
    let end = geometry.add_vertex(Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let edge = geometry.add_line(start, end).unwrap();
    let revision = geometry.revision();

    assert!(matches!(
        geometry.remove_vertex(start),
        Err(GeometryError::EntityInUse { .. })
    ));
    assert_eq!(geometry.revision(), revision);
    geometry.remove_edge(edge).unwrap();
    geometry.remove_vertex(start).unwrap();
    let replacement = geometry.add_vertex(Vec3::new(2.0, 0.0, 0.0)).unwrap();

    assert_ne!(replacement, start);
    assert_eq!(geometry.vertex(end).unwrap().id, end);
    assert!(geometry.vertex(start).is_none());
}

#[test]
fn planar_hole_topology_is_closed_and_retains_stable_boundary_edges() {
    let mut geometry = GeometryTopology::new();
    let (rectangle, hole) = geometry
        .add_rectangle_with_circle(4.0, 3.0, 2.0, 1.5, 0.5)
        .unwrap();

    let face = geometry.face(rectangle.face).unwrap();
    let GeometryFaceRepresentation::Planar {
        outer_loop,
        inner_loops,
    } = &face.representation
    else {
        panic!("rectangle must be planar");
    };
    assert_eq!(outer_loop.len(), 4);
    assert_eq!(inner_loops.len(), 1);
    assert_eq!(inner_loops[0].len(), 4);
    for edge in hole.boundary {
        assert_eq!(geometry.edge(edge).unwrap().id, edge);
    }
}

#[test]
fn box_exposes_six_stable_logical_face_ids() {
    let mut geometry = GeometryTopology::new();
    let box_entities = geometry.add_box(2.0, 1.0, 0.5).unwrap();
    let face_ids = [
        box_entities.x_min,
        box_entities.x_max,
        box_entities.y_min,
        box_entities.y_max,
        box_entities.z_min,
        box_entities.z_max,
    ];
    assert_eq!(geometry.bodies().count(), 1);
    assert_eq!(geometry.faces().count(), 6);
    assert_eq!(
        face_ids
            .iter()
            .collect::<std::collections::BTreeSet<_>>()
            .len(),
        6
    );

    geometry.add_vertex(Vec3::new(20.0, 0.0, 0.0)).unwrap();
    for id in face_ids {
        assert_eq!(geometry.face(id).unwrap().id, id);
    }
}

#[test]
fn gmsh_export_has_deterministic_backend_mapping_distinct_from_geometry_ids() {
    let mut geometry = GeometryTopology::new();
    let rectangle = geometry.add_rectangle(2.0, 1.0).unwrap();
    let groups = vec![
        ("inlet", vec![rectangle.left]),
        ("outlet", vec![rectangle.right]),
        ("walls", vec![rectangle.bottom, rectangle.top]),
    ];
    let first =
        GmshGeometryExporter::planar(&geometry, rectangle.face, groups.clone(), "fluid").unwrap();
    let second = GmshGeometryExporter::planar(&geometry, rectangle.face, groups, "fluid").unwrap();

    assert_eq!(first.map, second.map);
    assert_eq!(
        first.document.to_geo_string().unwrap(),
        second.document.to_geo_string().unwrap()
    );
    assert_ne!(
        first.map.edge_tag(rectangle.left).unwrap() as u64,
        rectangle.left.get()
    );
    let source = first.document.to_geo_string().unwrap();
    assert!(source.contains("Point(1000)"));
    assert!(source.contains("Curve Loop(4000)"));
    assert!(source.contains("Plane Surface(3000)"));
}

#[test]
fn validation_and_clone_are_read_only_for_revision_and_identity() {
    let mut geometry = GeometryTopology::new();
    let rectangle = geometry.add_rectangle(2.0, 1.0).unwrap();
    let revision = geometry.revision();
    let clone = geometry.clone();

    geometry.validate().unwrap();
    assert_eq!(geometry.revision(), revision);
    assert_eq!(clone.revision(), revision);
    assert_eq!(clone.face(rectangle.face).unwrap().id, rectangle.face);
}

#[test]
fn body_prevents_face_deletion_until_removed() {
    let mut geometry = GeometryTopology::new();
    let box_entities = geometry.add_box(2.0, 1.0, 0.5).unwrap();

    assert!(matches!(
        geometry.remove_face(box_entities.x_min),
        Err(GeometryError::EntityInUse { .. })
    ));
    geometry.remove_body(box_entities.body).unwrap();
    geometry.remove_face(box_entities.x_min).unwrap();
    assert!(geometry.face(box_entities.x_min).is_none());
}

#[test]
#[ignore = "requires a real gmsh executable on PATH"]
fn geometry_rectangle_round_trips_through_real_gmsh() {
    let mut geometry = GeometryTopology::new();
    let rectangle = geometry.add_rectangle(2.0, 1.0).unwrap();
    let export = GmshGeometryExporter::planar(
        &geometry,
        rectangle.face,
        vec![
            ("inlet", vec![rectangle.left]),
            ("outlet", vec![rectangle.right]),
            ("walls", vec![rectangle.bottom, rectangle.top]),
        ],
        "fluid",
    )
    .unwrap();

    let generated = GmshMesher::auto()
        .generate(&export.document, &GmshMeshOptions::two_d(0.25).unwrap())
        .unwrap();
    assert!(generated.mesh.cell_count() > 0);
    assert!(has_patch(&generated.mesh, "inlet"));
    assert!(has_patch(&generated.mesh, "outlet"));
}

#[test]
#[ignore = "requires a real gmsh executable on PATH"]
fn geometry_hole_round_trips_through_real_gmsh() {
    let mut geometry = GeometryTopology::new();
    let (rectangle, hole) = geometry
        .add_rectangle_with_circle(4.0, 3.0, 2.0, 1.5, 0.5)
        .unwrap();
    let export = GmshGeometryExporter::planar(
        &geometry,
        rectangle.face,
        vec![
            ("inlet", vec![rectangle.left]),
            ("outlet", vec![rectangle.right]),
            ("walls", vec![rectangle.bottom, rectangle.top]),
            ("cylinder", hole.boundary.to_vec()),
        ],
        "fluid",
    )
    .unwrap();

    let generated = GmshMesher::auto()
        .generate(&export.document, &GmshMeshOptions::two_d(0.3).unwrap())
        .unwrap();
    assert!(generated.mesh.cell_count() > 0);
    assert!(has_patch(&generated.mesh, "cylinder"));
}

#[test]
#[ignore = "requires a real gmsh executable on PATH"]
fn geometry_box_round_trips_through_real_gmsh() {
    let mut geometry = GeometryTopology::new();
    let box_entities = geometry.add_box(1.0, 0.5, 0.25).unwrap();
    let export = GmshGeometryExporter::rectangular_box(
        &geometry,
        box_entities.body,
        vec![
            ("inlet", vec![box_entities.x_min]),
            ("outlet", vec![box_entities.x_max]),
            (
                "walls",
                vec![
                    box_entities.y_min,
                    box_entities.y_max,
                    box_entities.z_min,
                    box_entities.z_max,
                ],
            ),
        ],
        "fluid",
    )
    .unwrap();

    let generated = GmshMesher::auto()
        .generate(&export.document, &GmshMeshOptions::three_d(0.25).unwrap())
        .unwrap();
    assert!(generated.mesh.cell_count() > 0);
    assert!(has_patch(&generated.mesh, "inlet"));
    assert!(has_patch(&generated.mesh, "outlet"));
}

#[test]
#[ignore = "requires a real gmsh executable on PATH"]
fn remeshing_changes_mesh_identity_without_changing_geometry_identity() {
    let mut geometry = GeometryTopology::new();
    let rectangle = geometry.add_rectangle(2.0, 1.0).unwrap();
    let revision = geometry.revision();
    let export = GmshGeometryExporter::planar(
        &geometry,
        rectangle.face,
        vec![
            ("inlet", vec![rectangle.left]),
            ("outlet", vec![rectangle.right]),
            ("walls", vec![rectangle.bottom, rectangle.top]),
        ],
        "fluid",
    )
    .unwrap();
    let mesher = GmshMesher::auto();
    let coarse = mesher
        .generate(&export.document, &GmshMeshOptions::two_d(0.4).unwrap())
        .unwrap();
    let fine = mesher
        .generate(&export.document, &GmshMeshOptions::two_d(0.1).unwrap())
        .unwrap();

    assert_ne!(coarse.mesh.id(), fine.mesh.id());
    assert_ne!(coarse.mesh.cell_count(), fine.mesh.cell_count());
    assert_eq!(geometry.revision(), revision);
    assert_eq!(geometry.face(rectangle.face).unwrap().id, rectangle.face);
}

fn has_patch(mesh: &flursys::UnstructuredMesh, name: &str) -> bool {
    mesh.boundary_patches()
        .iter()
        .any(|patch| patch.name == name)
}
