use flursys::sketching::{self, core, geom::Vec2, ToolParameters};
use flursys::{CadSketchPlane, GeometryTopology, Vec3};
fn p(x: f64, y: f64) -> Vec2 {
    Vec2::new(x, y)
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
fn imported_curve_families_are_drawable_solvable_and_serializable() {
    for (tool, points) in [
        (3, vec![p(0., 0.), p(20., 10.)]),
        (4, vec![p(10., 10.), p(20., 15.)]),
        (5, vec![p(0., 0.), p(20., 5.), p(10., 15.)]),
        (6, vec![p(10., 10.), p(15., 10.)]),
        (7, vec![p(0., 0.), p(10., 0.), p(5., 5.)]),
        (8, vec![p(0., 0.), p(10., 0.), p(0., 10.)]),
        (9, vec![p(0., 0.), p(5., 5.), p(10., 0.)]),
        (10, vec![p(0., 0.), p(10., 0.), p(0., 5.)]),
        (11, vec![p(0., 0.), p(5., 4.), p(10., 0.)]),
        (12, vec![p(0., 0.), p(3., 5.), p(7., 5.), p(10., 0.)]),
        (13, vec![p(0., 0.), p(10., 0.)]),
        (14, vec![p(0., 0.), p(10., 0.)]),
        (15, vec![p(0., 0.), p(10., 0.), p(5., 8.)]),
        (17, vec![p(0., 0.)]),
    ] {
        let mut sk = core::Sketch::new();
        let params = ToolParameters {
            value: if tool == 15 { 0.5 } else { 2.0 },
            secondary: 0.0,
            ..Default::default()
        };
        let report =
            sketching::transaction(&mut sk, |sk| sketching::draw(sk, tool, &points, &params))
                .unwrap_or_else(|e| panic!("tool {tool}: {e}"));
        assert!(report.ok());
        assert!(!sk.curves.is_empty());
        let restored: core::Sketch =
            serde_json::from_str(&serde_json::to_string(&sk).unwrap()).unwrap();
        assert_eq!(sk, restored);
        sketching::validate(&restored).unwrap();
    }
}
#[test]
fn invalid_constraints_and_hostile_indices_leave_the_whole_sketch_unchanged() {
    let mut sk = core::Sketch::new();
    sketching::draw(
        &mut sk,
        1,
        &[p(0., 0.), p(10., 0.)],
        &ToolParameters::default(),
    )
    .unwrap();
    sketching::transaction(&mut sk, |sk| sketching::constraint(sk, 13, &[0], &[], 10.)).unwrap();
    let before = sk.clone();
    assert!(
        sketching::transaction(&mut sk, |sk| sketching::constraint(sk, 13, &[0], &[], 20.))
            .is_err()
    );
    assert_eq!(before, sk);
    let mut bad = before;
    bad.curves[0].kind = core::CurveKind::Line {
        a: usize::MAX,
        b: 0,
    };
    assert!(sketching::validate(&bad).is_err());
}
#[test]
fn analytic_circle_publishes_in_metres_and_rebuilds_its_pad_with_stable_ids() {
    let mut sk = core::Sketch::new();
    sketching::draw(
        &mut sk,
        6,
        &[p(0., 0.), p(5., 0.)],
        &ToolParameters::default(),
    )
    .unwrap();
    let mut g = GeometryTopology::new();
    let id = g.create_sketch_on_plane(CadSketchPlane::Xy).unwrap().id;
    let faces = g.cad_set_drawing(id, sk.clone()).unwrap();
    assert_eq!(faces.len(), 1);
    let body = g.cad_pad(faces[0], 0.01, false, 1e-7).unwrap();
    let v = volume(&g, body);
    let expected = std::f64::consts::PI * 0.005_f64.powi(2) * 0.01;
    assert!((v / expected - 1.0).abs() < 0.04, "{v} vs {expected}");
    if let core::CurveKind::Circle { r, .. } = &mut sk.curves[0].kind {
        *r = 7.0;
    }
    let updated = g.cad_set_drawing(id, sk).unwrap();
    assert_eq!(faces, updated);
    assert!(g.cad_body(body).is_some());
    assert!((volume(&g, body) / v - 49.0 / 25.0).abs() < 0.06);
    g.validate().unwrap();
    let saved = serde_json::to_string(&g).unwrap();
    let restored: GeometryTopology = serde_json::from_str(&saved).unwrap();
    assert_eq!(g, restored);
}
#[test]
fn ellipse_and_spline_profiles_build_exact_solids_on_an_attached_face() {
    let mut g = GeometryTopology::new();
    let host = g
        .cad_add_primitive(
            flursys::CadPrimitive::Box {
                length: 0.05,
                width: 0.05,
                height: 0.01,
            },
            Vec3::new(0., 0., 0.),
            "support",
        )
        .unwrap();
    let face = g
        .body(host)
        .unwrap()
        .faces
        .iter()
        .copied()
        .find(|f| g.sketch_frame_for_face(*f).is_ok())
        .unwrap();
    let id = g.create_sketch_on_face(face).unwrap().id;
    let mut sk = core::Sketch::new();
    sketching::draw(
        &mut sk,
        10,
        &[p(20., 20.), p(30., 20.), p(20., 25.)],
        &ToolParameters::default(),
    )
    .unwrap();
    let faces = g.cad_set_drawing(id, sk).unwrap();
    let b = g.cad_pad(faces[0], 0.003, false, 1e-7).unwrap();
    assert!(volume(&g, b) > 1e-8);
    g.validate().unwrap();
}
#[test]
fn profiles_with_holes_are_retained_and_region_removal_protects_dependents() {
    let mut sk = core::Sketch::new();
    sketching::draw(
        &mut sk,
        3,
        &[p(0., 0.), p(20., 20.)],
        &ToolParameters::default(),
    )
    .unwrap();
    sketching::draw(
        &mut sk,
        6,
        &[p(10., 10.), p(13., 10.)],
        &ToolParameters::default(),
    )
    .unwrap();
    let mut g = GeometryTopology::new();
    let id = g.create_sketch_on_plane(CadSketchPlane::Xy).unwrap().id;
    let faces = g.cad_set_drawing(id, sk.clone()).unwrap();
    assert_eq!(faces.len(), 2);
    let annulus = faces
        .into_iter()
        .find(|f| {
            g.sketches().any(|s| {
                s.profile_face == Some(*f)
                    && s.drawing_region
                        .as_ref()
                        .is_some_and(|r| !r.holes.is_empty())
            })
        })
        .unwrap();
    g.cad_pad(annulus, 0.005, false, 1e-7).unwrap();
    let before = g.clone();
    sk.remove_curves(&[4]);
    assert!(g.cad_set_drawing(id, sk).is_err());
    assert_eq!(g, before);
}

#[test]
fn copied_corner_tools_retain_dimensions_and_freeform_split_is_exact() {
    let mut sk = core::Sketch::new();
    sketching::draw(
        &mut sk,
        3,
        &[p(0., 0.), p(20., 10.)],
        &ToolParameters::default(),
    )
    .unwrap();
    sketching::transaction(&mut sk, |s| sketching::constraint(s, 13, &[0], &[], 20.)).unwrap();
    sketching::transaction(&mut sk, |s| {
        sketching::edit(
            s,
            45,
            &[0, 1],
            &[],
            p(20., 0.),
            &ToolParameters {
                value: 2.,
                ..Default::default()
            },
        )
    })
    .unwrap();
    assert!(sk
        .constraints
        .iter()
        .any(|c| matches!(c.kind, core::ConstraintKind::Tangent { .. })));
    assert!(sk
        .constraints
        .iter()
        .any(|c| matches!(c.kind,core::ConstraintKind::Distance{value,..} if value==20.)));
    assert!(!core::find_profiles(&sk).is_empty());
    let mut sk = core::Sketch::new();
    sketching::draw(
        &mut sk,
        12,
        &[p(0., 0.), p(2., 5.), p(8., 5.), p(10., 0.)],
        &ToolParameters::default(),
    )
    .unwrap();
    let curve = sk.exact_segs(0).unwrap()[0];
    let at = curve.point_at(0.5);
    sketching::transaction(&mut sk, |s| {
        sketching::edit(s, 43, &[0], &[], at, &ToolParameters::default())
    })
    .unwrap();
    assert_eq!(sk.curves.len(), 2);
    for ci in 0..2 {
        for seg in sk.exact_segs(ci).unwrap() {
            for i in 0..21 {
                let q = seg.point_at(i as f64 / 20.);
                assert!(q.dist(curve.point_at(curve.closest_param(q))) < 1e-7);
            }
        }
    }
}

#[test]
fn stable_drawing_edges_follow_dimension_edits_and_open_wires_are_visible() {
    let mut g = GeometryTopology::new();
    let id = g.create_sketch_on_plane(CadSketchPlane::Xy).unwrap().id;
    let mut sk = core::Sketch::new();
    sketching::draw(
        &mut sk,
        3,
        &[p(0., 0.), p(20., 10.)],
        &ToolParameters::default(),
    )
    .unwrap();
    let f = g.cad_set_drawing(id, sk.clone()).unwrap()[0];
    let before = match &g.face(f).unwrap().representation {
        flursys::GeometryFaceRepresentation::Planar { outer_loop, .. } => outer_loop.clone(),
        _ => panic!(),
    };
    for point in &mut sk.points {
        point.pos.x *= 2.;
    }
    g.cad_set_drawing(id, sk).unwrap();
    let after = match &g.face(f).unwrap().representation {
        flursys::GeometryFaceRepresentation::Planar { outer_loop, .. } => outer_loop.clone(),
        _ => panic!(),
    };
    assert_eq!(before, after);
    let id = g.create_sketch_on_plane(CadSketchPlane::Yz).unwrap().id;
    let mut wire = core::Sketch::new();
    sketching::draw(
        &mut wire,
        1,
        &[p(0., 0.), p(20., 0.)],
        &ToolParameters::default(),
    )
    .unwrap();
    assert!(g.cad_set_drawing(id, wire).unwrap().is_empty());
    let paths = g.cad_open_drawing_paths();
    assert_eq!(paths.len(), 1);
    assert!((paths[0][0] - paths[0][1]).norm() > 0.019);
}

#[test]
fn spline_closed_by_a_line_extrudes_using_exact_bezier_geometry() {
    let mut sk = core::Sketch::new();
    let ps = [p(0., 0.), p(3., 8.), p(7., 8.), p(10., 0.)];
    sketching::draw(&mut sk, 12, &ps, &ToolParameters::default()).unwrap();
    sketching::draw(&mut sk, 1, &[ps[3], ps[0]], &ToolParameters::default()).unwrap();
    let mut g = GeometryTopology::new();
    let id = g.create_sketch_on_plane(CadSketchPlane::Xy).unwrap().id;
    let faces = g.cad_set_drawing(id, sk).unwrap();
    assert_eq!(faces.len(), 1);
    let b = g.cad_pad(faces[0], 0.01, false, 1e-7).unwrap();
    assert!(volume(&g, b) > 1e-8);
    g.validate().unwrap();
}

#[test]
fn copied_trim_and_extend_respect_exact_cutters_and_keep_curve_ids() {
    let mut sk = core::Sketch::new();
    let params = ToolParameters::default();
    sketching::draw(&mut sk, 1, &[p(0., 0.), p(10., 0.)], &params).unwrap();
    sketching::draw(&mut sk, 1, &[p(5., -5.), p(5., 5.)], &params).unwrap();
    let id = sk.curves[0].id.clone();
    sketching::transaction(&mut sk, |s| {
        sketching::edit(s, 42, &[0], &[], p(8., 0.), &params)
    })
    .unwrap();
    assert_eq!(sk.curves[0].id, id);
    let ends = sk.curves[0].kind.ends().unwrap();
    assert!((sk.points[ends.0].pos.dist(sk.points[ends.1].pos) - 5.).abs() < 1e-7);
    let mut sk = core::Sketch::new();
    sketching::draw(&mut sk, 1, &[p(0., 0.), p(5., 0.)], &params).unwrap();
    sketching::draw(
        &mut sk,
        12,
        &[p(10., -5.), p(12., -2.), p(12., 2.), p(10., 5.)],
        &params,
    )
    .unwrap();
    sketching::transaction(&mut sk, |s| {
        sketching::edit(s, 44, &[0], &[], p(5., 0.), &params)
    })
    .unwrap();
    let ends = sk.curves[0].kind.ends().unwrap();
    let end = sk.points[ends.1].pos;
    assert!((end.x - 11.5).abs() < 1e-6 && end.y.abs() < 1e-7, "{end:?}");
}

#[test]
fn mirrored_and_patterned_geometry_follows_its_seed() {
    for action in [34, 35, 37, 38] {
        let mut sk = core::Sketch::new();
        let params = ToolParameters {
            value: 30.,
            secondary: 5.,
            count: 3,
            ..Default::default()
        };
        sketching::draw(&mut sk, 6, &[p(10., 10.), p(12., 10.)], &params).unwrap();
        sketching::transaction(&mut sk, |s| {
            sketching::edit(s, action, &[0], &[], p(0., 0.), &params)
        })
        .unwrap();
        assert!(sk.constraints.iter().any(|c| matches!(
            c.kind,
            core::ConstraintKind::Symmetric { .. } | core::ConstraintKind::PatternPoint { .. }
        )));
        sketching::transaction(&mut sk, |s| sketching::constraint(s, 14, &[0], &[], 2.)).unwrap();
        let index = sk
            .constraints
            .iter()
            .position(|c| matches!(c.kind, core::ConstraintKind::Radius { c: 0, .. }))
            .unwrap();
        sketching::transaction(&mut sk, |s| {
            s.constraints[index].kind.set_value(3.);
            Ok(())
        })
        .unwrap();
        for c in &sk.curves {
            if let core::CurveKind::Circle { r, .. } = c.kind {
                assert!((r - 3.).abs() < 1e-6);
            }
        }
    }
}
