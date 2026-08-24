use flursys::{
    CadSketchPlane, GeometryError, GeometryFaceRepresentation, GeometryTopology,
    GmshGeometryExporter, GmshMeshOptions, GmshMesher, RigidBodyTransform, Vec3,
};

#[test]
fn canonical_sketch_planes_and_face_sketches_keep_stable_ids_across_round_trip() {
    let mut geometry = GeometryTopology::new();
    let xy = geometry.create_sketch_on_plane(CadSketchPlane::Xy).unwrap();
    let yz = geometry.create_sketch_on_plane(CadSketchPlane::Yz).unwrap();
    let zx = geometry.create_sketch_on_plane(CadSketchPlane::Zx).unwrap();
    let rectangle = geometry.add_rectangle(2.0, 1.0).unwrap();
    let face_sketch = geometry.create_sketch_on_face(rectangle.face).unwrap();

    assert_eq!(xy.frame.normal, Vec3::new(0.0, 0.0, 1.0));
    assert_eq!(yz.frame.normal, Vec3::new(1.0, 0.0, 0.0));
    assert_eq!(zx.frame.normal, Vec3::new(0.0, 1.0, 0.0));
    assert_eq!(face_sketch.host_face, Some(rectangle.face));

    let restored: GeometryTopology =
        serde_json::from_str(&serde_json::to_string(&geometry).unwrap()).unwrap();
    assert_eq!(restored.sketch(xy.id).unwrap().id, xy.id);
    assert_eq!(
        restored.sketch(face_sketch.id).unwrap().host_face,
        Some(rectangle.face)
    );
    let next = restored.clone();
    let mut next = next;
    assert!(
        next.create_sketch_on_plane(CadSketchPlane::Xy)
            .unwrap()
            .id
            .get()
            > face_sketch.id.get()
    );
}

#[test]
fn canonical_extrude_feature_retains_its_source_sketch_and_body() {
    let mut geometry = GeometryTopology::new();
    let rectangle = geometry.add_rectangle(2.0, 1.0).unwrap();
    let sketch = geometry.create_sketch_on_face(rectangle.face).unwrap();

    let (feature, extrusion) = geometry
        .extrude_sketch_face(sketch.id, rectangle.face, 0.5)
        .unwrap();

    assert_eq!(feature.sketch, sketch.id);
    assert_eq!(feature.source_face, rectangle.face);
    assert_eq!(feature.body, extrusion.body);
    assert_eq!(
        geometry.extrude_features().collect::<Vec<_>>(),
        vec![&feature]
    );
    assert!(matches!(
        geometry.remove_body(extrusion.body),
        Err(GeometryError::EntityInUse {
            used_by: "feature",
            ..
        })
    ));
}

#[test]
fn canonical_renderable_faces_include_stable_extrusion_surfaces() {
    let mut geometry = GeometryTopology::new();
    let rectangle = geometry.add_rectangle(2.0, 1.0).unwrap();
    let extrusion = geometry.extrude_planar_face(rectangle.face, 0.5).unwrap();

    let rendered = geometry.renderable_faces();

    assert!(rendered
        .iter()
        .any(|surface| surface.face == rectangle.face));
    assert!(rendered.iter().any(|surface| {
        surface.face == extrusion.top_face
            && surface.body == Some(extrusion.body)
            && surface.vertices.iter().all(|vertex| vertex.z == 0.5)
    }));
    assert_eq!(
        rendered
            .iter()
            .filter(|surface| extrusion.side_faces.contains(&surface.face))
            .count(),
        extrusion.side_faces.len()
    );
}

#[test]
fn canonical_xy_sketch_materializes_a_profile_face_with_stable_identity() {
    let mut geometry = GeometryTopology::new();
    let sketch = geometry.create_sketch_on_plane(CadSketchPlane::Xy).unwrap();

    let face = geometry
        .materialize_sketch_rectangle(sketch.id, 2.0, 1.0)
        .unwrap();

    assert!(geometry.face(face).is_some());
    assert_eq!(geometry.sketch(sketch.id).unwrap().profile_face, Some(face));
    assert_eq!(
        geometry.sketch(sketch.id).unwrap().profile,
        Some(flursys::CadSketchProfile::Rectangle {
            width: 2.0,
            height: 1.0,
        })
    );
}

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
fn planar_rectangle_extrusion_preserves_source_and_creates_six_stable_faces() {
    let mut geometry = GeometryTopology::new();
    let rectangle = geometry.add_rectangle(2.0, 1.0).unwrap();
    let extrusion = geometry.extrude_planar_face(rectangle.face, 0.5).unwrap();

    assert_eq!(extrusion.source_face, rectangle.face);
    assert_eq!(extrusion.side_faces.len(), 4);
    assert_eq!(geometry.body(extrusion.body).unwrap().faces.len(), 6);
    geometry.validate().unwrap();
    for face in extrusion
        .side_faces
        .iter()
        .copied()
        .chain([extrusion.top_face])
    {
        assert_eq!(geometry.face(face).unwrap().id, face);
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
fn geometry_extrusion_round_trips_through_real_gmsh_with_named_faces() {
    let mut geometry = GeometryTopology::new();
    let rectangle = geometry.add_rectangle(2.0, 1.0).unwrap();
    let extrusion = geometry.extrude_planar_face(rectangle.face, 0.5).unwrap();
    let export = GmshGeometryExporter::extruded(
        &geometry,
        extrusion.body,
        vec![
            ("inlet", vec![extrusion.side_faces[0]]),
            ("outlet", vec![extrusion.side_faces[2]]),
            (
                "walls",
                vec![
                    extrusion.source_face,
                    extrusion.top_face,
                    extrusion.side_faces[1],
                    extrusion.side_faces[3],
                ],
            ),
        ],
        "fluid",
    )
    .unwrap();

    let generated = GmshMesher::auto()
        .generate(&export.document, &GmshMeshOptions::three_d(0.3).unwrap())
        .unwrap();
    assert!(generated.mesh.cell_count() > 0);
    assert!(has_patch(&generated.mesh, "inlet"));
    assert!(has_patch(&generated.mesh, "outlet"));
    assert!(has_patch(&generated.mesh, "walls"));
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

#[test]
fn body_translation_preserves_stable_ids_and_bumps_revision() {
    let mut geometry = GeometryTopology::new();
    let rectangle = geometry.add_rectangle(2.0, 1.0).unwrap();
    let body = geometry.add_body(vec![rectangle.face]).unwrap();
    let revision = geometry.revision();
    let positions: Vec<_> = rectangle
        .vertices
        .iter()
        .map(|&id| (id, geometry.vertex(id).unwrap().position))
        .collect();

    geometry
        .translate_body(body, Vec3::new(3.0, -2.0, 4.0))
        .unwrap();

    assert_eq!(geometry.revision().get(), revision.get() + 1);
    assert_eq!(geometry.body(body).unwrap().id, body);
    assert_eq!(geometry.face(rectangle.face).unwrap().id, rectangle.face);
    for (id, position) in positions {
        assert_eq!(geometry.vertex(id).unwrap().id, id);
        assert_eq!(
            geometry.vertex(id).unwrap().position,
            position + Vec3::new(3.0, -2.0, 4.0)
        );
    }
}

#[test]
fn body_rotation_preserves_stable_ids_and_rotates_about_the_requested_axis() {
    let mut geometry = GeometryTopology::new();
    let rectangle = geometry.add_rectangle(2.0, 1.0).unwrap();
    let body = geometry.add_body(vec![rectangle.face]).unwrap();

    geometry
        .rotate_body(
            body,
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            std::f64::consts::FRAC_PI_2,
        )
        .unwrap();

    assert_eq!(geometry.body(body).unwrap().id, body);
    assert_eq!(geometry.face(rectangle.face).unwrap().id, rectangle.face);
    let first = geometry.vertex(rectangle.vertices[1]).unwrap().position;
    let second = geometry.vertex(rectangle.vertices[2]).unwrap().position;
    assert!(first.x.abs() < 1.0e-12);
    assert!((first.y - 2.0).abs() < 1.0e-12);
    assert!((second.x + 1.0).abs() < 1.0e-12);
    assert!((second.y - 2.0).abs() < 1.0e-12);
}

#[test]
fn extruded_body_rotation_about_x_preserves_ids_and_exports_current_profile_normal() {
    let mut geometry = GeometryTopology::new();
    let rectangle = geometry.add_rectangle(2.0, 1.0).unwrap();
    let extrusion = geometry.extrude_planar_face(rectangle.face, 0.5).unwrap();

    geometry
        .rotate_body(
            extrusion.body,
            Vec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            std::f64::consts::FRAC_PI_2,
        )
        .unwrap();

    assert_eq!(geometry.body(extrusion.body).unwrap().id, extrusion.body);
    assert_eq!(
        geometry.face(extrusion.top_face).unwrap().id,
        extrusion.top_face
    );
    let source = geometry.vertex(rectangle.vertices[3]).unwrap().position;
    assert!(source.y.abs() < 1.0e-12);
    assert!((source.z - 1.0).abs() < 1.0e-12);
    let export = GmshGeometryExporter::extruded(
        &geometry,
        extrusion.body,
        vec![(
            "walls",
            geometry.body(extrusion.body).unwrap().faces.clone(),
        )],
        "fluid",
    )
    .unwrap();
    assert!(export
        .document
        .to_geo_string()
        .unwrap()
        .contains("Extrude {0.00000000000000000, -0.5"));
}

#[test]
fn body_transform_rejects_shared_vertices_without_mutating_geometry() {
    let mut geometry = GeometryTopology::new();
    let rectangle = geometry.add_rectangle(2.0, 1.0).unwrap();
    let first = geometry.add_body(vec![rectangle.face]).unwrap();
    let second = geometry.add_body(vec![rectangle.face]).unwrap();
    let revision = geometry.revision();
    let before = geometry.vertex(rectangle.vertices[0]).unwrap().position;

    let error = geometry
        .translate_body(first, Vec3::new(1.0, 0.0, 0.0))
        .unwrap_err();

    assert!(matches!(
        error,
        GeometryError::UnsafeBodyTransform {
            body,
            other_body,
            ..
        } if body == first && other_body == second
    ));
    assert_eq!(geometry.revision(), revision);
    assert_eq!(
        geometry.vertex(rectangle.vertices[0]).unwrap().position,
        before
    );
}

#[test]
fn rigid_body_transform_rejects_non_finite_inputs_and_zero_rotation_axes() {
    assert!(matches!(
        RigidBodyTransform::translation(Vec3::new(f64::NAN, 0.0, 0.0)),
        Err(GeometryError::NonFiniteGeometry)
    ));
    assert!(matches!(
        RigidBodyTransform::rotation(Vec3::ZERO, Vec3::ZERO, 0.0),
        Err(GeometryError::InvalidPrimitive { .. })
    ));
}

fn has_patch(mesh: &flursys::UnstructuredMesh, name: &str) -> bool {
    mesh.boundary_patches()
        .iter()
        .any(|patch| patch.name == name)
}
