use flursys::{
    CadBoolean as B, CadPrimitive as P, CadTransform as T, GeometryTopology, Ray3, Vec3,
};
fn v(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(x, y, z)
}
fn volume(g: &GeometryTopology, id: flursys::BodyId) -> f64 {
    g.cad_body(id)
        .unwrap()
        .face_meshes
        .iter()
        .flat_map(|m| &m.triangles)
        .map(|t| t[0].dot(t[1].cross(t[2])) / 6.0)
        .sum::<f64>()
        .abs()
}
#[test]
fn exact_primitives_have_closed_renderable_and_serializable_solids() {
    for (shape, expected) in [
        (
            P::Box {
                length: 2.,
                width: 3.,
                height: 4.,
            },
            24.,
        ),
        (
            P::Cylinder {
                radius: 1.,
                height: 2.,
            },
            2. * std::f64::consts::PI,
        ),
        (P::Sphere { radius: 1. }, 4. * std::f64::consts::PI / 3.),
        (
            P::Cone {
                radius: 1.,
                height: 2.,
            },
            2. * std::f64::consts::PI / 3.,
        ),
        (
            P::Torus {
                major_radius: 3.,
                minor_radius: 1.,
            },
            6. * std::f64::consts::PI.powi(2),
        ),
    ] {
        let mut g = GeometryTopology::new();
        let id = g.cad_add_primitive(shape, v(0., 0., 0.), "test").unwrap();
        g.validate().unwrap();
        assert!(
            (volume(&g, id) / expected - 1.).abs() < 0.04,
            "{shape:?}: {} vs {expected}",
            volume(&g, id)
        );
        let restored: GeometryTopology =
            serde_json::from_str(&serde_json::to_string(&g).unwrap()).unwrap();
        assert!(
            restored == g,
            "B-rep serialization must preserve coordinates and stable IDs"
        );
        assert!(!restored.renderable_faces().is_empty());
    }
}
#[test]
fn transform_keeps_ids_and_copy_is_independent() {
    let mut g = GeometryTopology::new();
    let id = g
        .cad_add_primitive(
            P::Box {
                length: 2.,
                width: 3.,
                height: 4.,
            },
            v(0., 0., 0.),
            "block",
        )
        .unwrap();
    let faces = g.body(id).unwrap().faces.clone();
    let original = g.cad_export_brep(id).unwrap();
    g.cad_transform(id, T::Translate(v(3., 2., 1.)), false)
        .unwrap();
    assert_eq!(g.body(id).unwrap().faces, faces);
    assert!((volume(&g, id) - 24.).abs() < 1e-9);
    let copy = g
        .cad_transform(
            id,
            T::Scale {
                pivot: v(0., 0., 0.),
                factor: 2.,
            },
            true,
        )
        .unwrap();
    assert_ne!(id, copy);
    assert!((volume(&g, copy) - 192.).abs() < 1e-8);
    assert_ne!(g.cad_export_brep(id).unwrap(), original);
}
#[test]
fn failures_do_not_modify_document() {
    let mut g = GeometryTopology::new();
    let id = g
        .cad_add_primitive(
            P::Box {
                length: 1.,
                width: 1.,
                height: 1.,
            },
            v(0., 0., 0.),
            "block",
        )
        .unwrap();
    let before = g.clone();
    assert!(g
        .cad_transform(
            id,
            T::Scale {
                pivot: v(0., 0., 0.),
                factor: f64::NAN
            },
            false
        )
        .is_err());
    assert!(g.cad_pattern(id, 65, v(1., 0., 0.), 0.).is_err());
    assert!(g.cad_boolean(id, id, B::Union, 1e-5).is_err());
    assert!(g
        .cad_add_primitive(P::Sphere { radius: -1. }, v(0., 0., 0.), "bad")
        .is_err());
    assert_eq!(g, before);
}
#[test]
fn kernel_face_picking_visibility_and_deletion_preserve_topology() {
    let mut g = GeometryTopology::new();
    let id = g
        .cad_add_primitive(
            P::Box {
                length: 1.,
                width: 1.,
                height: 1.,
            },
            v(0., 0., 0.),
            "box",
        )
        .unwrap();
    let ray = Ray3::new(v(0.5, 0.5, 3.), v(0., 0., -1.)).unwrap();
    assert_eq!(g.pick_body(ray), Some(id));
    let faces = g.body(id).unwrap().faces.clone();
    g.cad_set_visible(id, false).unwrap();
    assert!(g.pick_body(ray).is_none());
    g.remove_body(id).unwrap();
    assert_eq!(g.cad_bodies().count(), 0);
    assert!(faces.into_iter().all(|f| g.face(f).is_none()));
    g.validate().unwrap();
}
#[test]
fn sketch_extrusion_and_revolution_are_real_kernel_features() {
    let mut g = GeometryTopology::new();
    let r = g.add_rectangle(2., 3.).unwrap();
    let id = g.cad_extrude(r.face, 4.).unwrap();
    assert!((volume(&g, id) - 24.).abs() < 1e-8);
    // rectangle in XY, offset from Y axis, rotates into a closed ring-like solid
    let id = g
        .cad_revolve(r.face, v(-1., 0., 0.), v(0., 1., 0.), 180.)
        .unwrap();
    assert!(volume(&g, id) > 1.);
}
#[test]
fn exchange_round_trip_retains_curves_and_closed_volume() {
    let mut g = GeometryTopology::new();
    let id = g
        .cad_add_primitive(
            P::Cylinder {
                radius: 1.,
                height: 2.,
            },
            v(1., 2., 3.),
            "cylinder",
        )
        .unwrap();
    let native = g.cad_export_brep(id).unwrap();
    let id2 = g.cad_import_brep(&native, "import").unwrap();
    assert!((volume(&g, id2) - volume(&g, id)).abs() < 1e-10);
    let step = g.cad_export_step(id).unwrap();
    assert!(step.starts_with("ISO-10303-21;"));
    assert!(step.contains("MANIFOLD_SOLID_BREP"));
    let stl = g.cad_export_stl(id).unwrap();
    assert!(stl.contains("facet normal"));
}
#[test]
fn overlapping_boxes_boolean_volumes_agree_with_analytic_values() {
    for (op, expected) in [
        (B::Union, 1.784),
        (B::Intersection, 0.216),
        (B::Difference, 0.784),
    ] {
        let mut g = GeometryTopology::new();
        let a = g
            .cad_add_primitive(
                P::Box {
                    length: 1.,
                    width: 1.,
                    height: 1.,
                },
                v(0., 0., 0.),
                "A",
            )
            .unwrap();
        let b = g
            .cad_add_primitive(
                P::Box {
                    length: 1.,
                    width: 1.,
                    height: 1.,
                },
                v(0.4, 0.4, 0.4),
                "B",
            )
            .unwrap();
        let c = g.cad_boolean(a, b, op, 1e-5).unwrap();
        assert!((volume(&g, c) - expected).abs() < 1e-6, "{op:?}");
        assert!(!g.cad_body(a).unwrap().visible);
        assert!(!g.cad_body(b).unwrap().visible);
        g.validate().unwrap();
    }
}
#[test]
fn patterns_and_editor_undo_restore_exact_brep() {
    let mut g = GeometryTopology::new();
    let id = g
        .cad_add_primitive(P::Sphere { radius: 1. }, v(0., 0., 0.), "seed")
        .unwrap();
    let before = g.clone();
    let mut editor = flursys::GeometryEditorState::new();
    editor.record_snapshot(&g);
    let ids = g.cad_pattern(id, 4, v(3., 0., 0.), 0.).unwrap();
    assert_eq!(ids.len(), 4);
    assert_eq!(g.cad_bodies().count(), 4);
    assert!(editor.undo(&mut g));
    assert_eq!(g, before);
    assert!(editor.redo(&mut g));
    assert_eq!(g.cad_bodies().count(), 4);
}
#[test]
fn faceted_meshing_preserves_stable_boundary_face_groups() {
    let mut g = GeometryTopology::new();
    let id = g
        .cad_add_primitive(
            P::Box {
                length: 1.,
                width: 1.,
                height: 1.,
            },
            v(0., 0., 0.),
            "fluid",
        )
        .unwrap();
    let faces = g.body(id).unwrap().faces.clone();
    let export = flursys::GmshGeometryExporter::cad_faceted(
        &g,
        id,
        vec![("walls".into(), faces.clone())],
        "fluid",
    )
    .unwrap();
    for f in faces {
        assert_eq!(export.map.face_tags(f).len(), 2);
    }
    let text = export.document.to_geo_string().unwrap();
    assert!(text.contains("Physical Volume(\"fluid\")"));
    assert!(text.contains("Physical Surface(\"walls\")"));
}
#[test]
fn curved_cad_mesh_export_is_watertight_or_reports_an_explicit_error() {
    let mut g = GeometryTopology::new();
    let id = g
        .cad_add_primitive(
            P::Cylinder {
                radius: 1.,
                height: 2.,
            },
            v(0., 0., 0.),
            "fluid",
        )
        .unwrap();
    let result = flursys::GmshGeometryExporter::cad_faceted(&g, id, vec![], "fluid");
    // Boundary sampling must be welded before exporting a single closed shell.
    assert!(result.is_ok(), "{}", result.unwrap_err());
}
#[test]
fn loft_between_offset_rectangles_has_expected_volume() {
    let mut g = GeometryTopology::new();
    let bottom = g.add_rectangle(2., 3.).unwrap().face;
    let vertices = [v(0., 0., 4.), v(2., 0., 4.), v(2., 3., 4.), v(0., 3., 4.)]
        .map(|p| g.add_vertex(p).unwrap());
    let edges = (0..4)
        .map(|i| flursys::OrientedEdge {
            edge: g.add_line(vertices[i], vertices[(i + 1) % 4]).unwrap(),
            reversed: false,
        })
        .collect();
    let top = g.add_planar_face(edges, vec![]).unwrap();
    let body = g.cad_loft(bottom, top).unwrap();
    assert!((volume(&g, body) - 24.).abs() < 1e-8);
    g.validate().unwrap();
}
#[test]
fn negative_extrusion_and_profiles_with_holes_have_outward_closed_volume() {
    let mut g = GeometryTopology::new();
    let (r, _) = g.add_rectangle_with_circle(4., 4., 2., 2., 0.5).unwrap();
    let body = g.cad_extrude(r.face, -2.).unwrap();
    let expected = (16. - std::f64::consts::PI * 0.25) * 2.;
    assert!((volume(&g, body) / expected - 1.).abs() < 0.01);
    let signed = g
        .cad_body(body)
        .unwrap()
        .face_meshes
        .iter()
        .flat_map(|m| &m.triangles)
        .map(|t| t[0].dot(t[1].cross(t[2])) / 6.)
        .sum::<f64>();
    assert!(signed > 0.);
}
#[test]
fn cad_surface_groups_reach_real_three_dimensional_gmsh_mesh() {
    if flursys::GmshMesher::auto().version().is_err() {
        return;
    }
    for (shape, expected) in [
        (
            P::Box {
                length: 1.,
                width: 1.,
                height: 1.,
            },
            1.,
        ),
        (
            P::Cylinder {
                radius: 1.,
                height: 2.,
            },
            2. * std::f64::consts::PI,
        ),
    ] {
        let mut session = flursys::WorkbenchSession::new();
        let body = session
            .geometry_mut()
            .cad_add_primitive(shape, v(0., 0., 0.), "fluid")
            .unwrap();
        session.geometry_changed();
        let faces = session.geometry().body(body).unwrap().faces.clone();
        session
            .create_named_selection(
                "walls",
                faces
                    .into_iter()
                    .map(flursys::GeometrySelectionTarget::Face)
                    .collect(),
            )
            .unwrap();
        session
            .set_mesh_configuration(flursys::MeshDimension::ThreeD, 0.3, 0.2, 0.4, 1)
            .unwrap();
        let (export, options) = session.mesh_generation_inputs().unwrap();
        let mesh = flursys::GmshMesher::auto()
            .generate(&export.document, &options)
            .unwrap();
        assert!(mesh.mesh.cell_count() > 0);
        assert_eq!(mesh.mesh.dimension(), flursys::MeshDimension::ThreeD);
        let total_volume = mesh
            .mesh
            .cells()
            .iter()
            .map(|cell| cell.volume)
            .sum::<f64>();
        assert!(
            (total_volume / expected - 1.).abs() < 0.015,
            "CAD mesh volume {total_volume} vs {expected}"
        );
        session.install_mesh(mesh);
        assert!(session.patch_names().iter().any(|n| n == "walls"));
    }
}
#[test]
fn sketches_attach_only_to_exact_planes_and_block_unsafe_parent_deletion() {
    let mut g = GeometryTopology::new();
    let body = g
        .cad_add_primitive(
            P::Box {
                length: 2.,
                width: 3.,
                height: 4.,
            },
            v(0., 0., 0.),
            "block",
        )
        .unwrap();
    let face = g.body(body).unwrap().faces[0];
    let sketch = g.create_sketch_on_face(face).unwrap();
    assert!((sketch.frame.normal.norm() - 1.).abs() < 1e-12);
    let before = g.clone();
    assert!(g.remove_body(body).is_err());
    assert_eq!(g, before);
    assert!(g
        .cad_transform(body, T::Translate(v(1., 0., 0.)), false)
        .is_err());
    assert_eq!(g, before);
    let sphere = g
        .cad_add_primitive(P::Sphere { radius: 1. }, v(0., 0., 0.), "round")
        .unwrap();
    let curved = g.body(sphere).unwrap().faces[0];
    assert!(g.create_sketch_on_face(curved).is_err());
    g.validate().unwrap();
}
#[test]
fn workspace_roundtrip_retains_kernel_geometry_and_its_face_identity() {
    let directory = tempfile::tempdir().unwrap();
    let mut session = flursys::WorkbenchSession::new();
    let body = session
        .geometry_mut()
        .cad_add_primitive(
            P::Torus {
                major_radius: 3.,
                minor_radius: 1.,
            },
            v(1., 2., 3.),
            "ring",
        )
        .unwrap();
    session.geometry_changed();
    let faces = session.geometry().body(body).unwrap().faces.clone();
    let project = session.to_project("CAD workspace");
    flursys::save_workspace(directory.path(), &project).unwrap();
    let loaded = flursys::load_workspace(directory.path()).unwrap();
    assert_eq!(loaded.geometry.body(body).unwrap().faces, faces);
    assert!(loaded.geometry == *session.geometry());
}
#[test]
fn full_revolution_of_a_profile_touching_the_axis_produces_a_closed_cylinder() {
    let mut g = GeometryTopology::new();
    let face = g.add_rectangle(2., 3.).unwrap().face;
    let body = g
        .cad_revolve(face, v(0., 0., 0.), v(0., 1., 0.), 360.)
        .unwrap();
    let expected = std::f64::consts::PI * 4. * 3.;
    assert!((volume(&g, body) / expected - 1.).abs() < 0.02);
    g.validate().unwrap();
}
#[test]
#[cfg_attr(
    debug_assertions,
    ignore = "curved CSG stress test requires an optimized build"
)]
fn cylindrical_through_cut_retains_volume_and_exports_parseable_step() {
    let mut g = GeometryTopology::new();
    let block = g
        .cad_add_primitive(
            P::Box {
                length: 10.,
                width: 8.,
                height: 6.,
            },
            v(0., 0., 0.),
            "block",
        )
        .unwrap();
    let tool = g
        .cad_add_primitive(
            P::Cylinder {
                radius: 1.5,
                height: 8.,
            },
            v(5., 4., -1.),
            "drill",
        )
        .unwrap();
    let start = std::time::Instant::now();
    eprintln!("Starting curved CSG stress test");
    let result = g.cad_boolean(block, tool, B::Difference, 1e-3).unwrap();
    eprintln!("Curved CSG completed in {:?}", start.elapsed());
    let expected = 480. - std::f64::consts::PI * 1.5 * 1.5 * 6.;
    assert!((volume(&g, result) / expected - 1.).abs() < 0.01);
    let step = g.cad_export_step(result).unwrap();
    let parsed = truck_stepio::r#in::Table::from_step(&step).unwrap();
    assert!(!parsed.shell.is_empty());
}
#[test]
fn single_shell_step_import_roundtrips_supported_nurbs_geometry_atomically() {
    for shape in [
        P::Box {
            length: 2.,
            width: 3.,
            height: 4.,
        },
        P::Cylinder {
            radius: 1.,
            height: 2.,
        },
    ] {
        let mut g = GeometryTopology::new();
        let original = g.cad_add_primitive(shape, v(1., 2., 3.), "source").unwrap();
        let step = g.cad_export_step(original).unwrap();
        let imported = g.cad_import_step(&step, "STEP").unwrap();
        assert!((volume(&g, imported) / volume(&g, original) - 1.).abs() < 1e-6);
        g.validate().unwrap();
        let before = g.clone();
        assert!(g.cad_import_step("not a STEP document", "bad").is_err());
        assert_eq!(g, before);
    }
}
