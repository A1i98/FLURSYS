//! Screen-aware picking and segment operations for the XY sketch editor.

use super::*;

fn error(message: &str) -> GeometryError {
    GeometryError::InvalidPrimitive {
        message: message.into(),
    }
}

fn endpoints(topology: &GeometryTopology, edge: &crate::GeometryEdge) -> Option<(Vec3, Vec3)> {
    let EdgeGeometry::Line { start, end } = edge.geometry else {
        return None;
    };
    let (a, b) = (
        topology.vertex(start)?.position,
        topology.vertex(end)?.position,
    );
    (a.z.abs() <= MIN_GEOMETRY && b.z.abs() <= MIN_GEOMETRY).then_some((a, b))
}

fn projection(a: Vec3, b: Vec3, p: Vec3) -> f64 {
    let d = b - a;
    ((p.x - a.x) * d.x + (p.y - a.y) * d.y) / (d.x * d.x + d.y * d.y)
}

// The target is an infinite line; the boundary is a finite segment.
fn intersection(a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> Option<f64> {
    let u = b - a;
    let v = d - c;
    let cross = |p: Vec3, q: Vec3| p.x * q.y - p.y * q.x;
    let denominator = cross(u, v);
    if denominator.abs() <= 1.0e-12 * u.norm() * v.norm() {
        return None;
    }
    let t = cross(c - a, v) / denominator;
    let s = cross(c - a, u) / denominator;
    (t.is_finite() && (-1.0e-9..=1.0 + 1.0e-9).contains(&s)).then_some(t)
}

pub(super) fn pick_line(
    topology: &GeometryTopology,
    transform: &ViewTransform,
    screen: (f64, f64),
) -> Option<(EdgeId, Vec3, Vec3)> {
    let (x, y) = transform.screen_to_world(screen);
    if !x.is_finite() || !y.is_finite() {
        return None;
    }
    let p = Vec3::new(x, y, 0.0);
    let tolerance = 8.0 / transform.pixels_per_unit;
    topology
        .edges()
        .filter(|edge| topology.is_edge_visible(edge.id))
        .filter_map(|edge| {
            let (a, b) = endpoints(topology, edge)?;
            let t = projection(a, b, p).clamp(0.0, 1.0);
            let distance = (p - (a + (b - a) * t)).norm();
            (distance <= tolerance).then_some((distance, edge.id, a, b))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, id, a, b)| (id, a, b))
}

pub(super) fn apply(
    topology: &mut GeometryTopology,
    transform: &ViewTransform,
    screen: (f64, f64),
    tool: GeometryTool,
    snap: bool,
) -> Result<Vec<EdgeId>, GeometryError> {
    let (x, y) = transform.screen_to_world(screen);
    if !x.is_finite() || !y.is_finite() {
        return Err(GeometryError::NonFiniteGeometry);
    }
    let p = Vec3::new(x, y, 0.0);
    let tolerance = 8.0 / transform.pixels_per_unit;
    let (id, a, b) = pick_line(topology, transform, screen)
        .ok_or_else(|| error("Click close to a straight XY sketch line"))?;
    let length = (b - a).norm();
    let epsilon = (MIN_GEOMETRY / length).max(1.0e-10);
    let cursor = projection(a, b, p).clamp(0.0, 1.0);
    let mut cuts = topology
        .edges()
        .filter(|edge| edge.id != id && topology.is_edge_visible(edge.id))
        .filter_map(|edge| {
            let (c, d) = endpoints(topology, edge)?;
            intersection(a, b, c, d)
        })
        .collect::<Vec<_>>();
    cuts.sort_by(f64::total_cmp);
    cuts.dedup_by(|a, b| (*a - *b).abs() <= epsilon);
    let at = |t: f64| a + (b - a) * t;
    let intervals = match tool {
        GeometryTool::Split => {
            let t = cuts
                .iter()
                .copied()
                .filter(|_| snap)
                .filter(|t| *t > epsilon && *t < 1.0 - epsilon)
                .min_by(|a, b| (a - cursor).abs().total_cmp(&(b - cursor).abs()))
                .filter(|t| (t - cursor).abs() * length <= tolerance)
                .unwrap_or(cursor);
            if t <= epsilon || t >= 1.0 - epsilon {
                return Err(error(
                    "Split requires a point inside the line, away from its endpoints",
                ));
            }
            vec![(0.0, t), (t, 1.0)]
        }
        GeometryTool::Extend => {
            let start = cursor < 0.5;
            let t = if start {
                cuts.into_iter().rfind(|t| *t < -epsilon)
            } else {
                cuts.into_iter().find(|t| *t > 1.0 + epsilon)
            }
            .ok_or_else(|| error("No boundary intersects the extension beyond this endpoint"))?;
            if start {
                vec![(t, 1.0)]
            } else {
                vec![(0.0, t)]
            }
        }
        GeometryTool::Trim => {
            cuts.retain(|t| *t > epsilon && *t < 1.0 - epsilon);
            if cuts.is_empty() {
                return Err(error(
                    "Trim requires another line crossing the selected line",
                ));
            }
            if cuts.iter().any(|t| (t - cursor).abs() <= epsilon) {
                return Err(error("Click the part to remove, beside the intersection"));
            }
            let lower = cuts.iter().copied().rfind(|t| *t < cursor).unwrap_or(0.0);
            let upper = cuts.iter().copied().find(|t| *t > cursor).unwrap_or(1.0);
            let mut retained = Vec::new();
            if lower > epsilon {
                retained.push((0.0, lower));
            }
            if upper < 1.0 - epsilon {
                retained.push((upper, 1.0));
            }
            retained
        }
        _ => unreachable!("line editing tools only"),
    };
    let segments = intervals
        .into_iter()
        .map(|(a, b)| (at(a), at(b)))
        .collect::<Vec<_>>();
    topology.replace_open_line(id, &segments)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(t: &mut GeometryTopology, a: (f64, f64), b: (f64, f64)) -> EdgeId {
        let a = add_vertex_if_needed(t, a).unwrap();
        let b = add_vertex_if_needed(t, b).unwrap();
        t.add_line(a, b).unwrap()
    }

    fn editor(tool: GeometryTool) -> GeometryEditorState {
        let mut e = GeometryEditorState::new();
        e.transform.set_viewport(1000.0, 800.0);
        e.set_tool(tool);
        e
    }

    fn click(
        e: &mut GeometryEditorState,
        t: &mut GeometryTopology,
        p: (f64, f64),
    ) -> Result<bool, GeometryError> {
        e.click(t, e.transform.world_to_screen(p), false)
    }

    fn ends(t: &GeometryTopology, id: EdgeId) -> (Vec3, Vec3) {
        endpoints(t, t.edge(id).unwrap()).unwrap()
    }

    fn document(t: &GeometryTopology) -> String {
        serde_json::to_string(t).unwrap()
    }

    #[test]
    fn split_preserves_identity_junction_and_round_trip_history() {
        let mut t = GeometryTopology::new();
        let id = line(&mut t, (0.0, 0.0), (4.0, 0.0));
        let before = document(&t);
        let mut e = editor(GeometryTool::Split);
        assert!(click(&mut e, &mut t, (1.37, 0.02)).unwrap());
        assert_eq!(t.edges().count(), 2);
        assert_eq!(t.vertices().count(), 3);
        assert_eq!(ends(&t, id).1, Vec3::new(1.37, 0.0, 0.0));
        let EdgeGeometry::Line { end, .. } = t.edge(id).unwrap().geometry else {
            panic!()
        };
        let other = t.edges().find(|edge| edge.id != id).unwrap();
        assert!(matches!(other.geometry, EdgeGeometry::Line { start, .. } if start == end));
        let after = document(&t);
        let loaded: GeometryTopology = serde_json::from_str(&after).unwrap();
        loaded.validate().unwrap();
        assert_eq!(document(&loaded), after);
        assert!(e.undo(&mut t));
        assert_eq!(document(&t), before);
        assert!(e.redo(&mut t));
        assert_eq!(document(&t), after);
    }

    #[test]
    fn split_snaps_to_nearby_intersection() {
        let mut t = GeometryTopology::new();
        let id = line(&mut t, (0.0, 0.0), (4.0, 0.0));
        line(&mut t, (2.0, -1.0), (2.0, 1.0));
        let mut e = editor(GeometryTool::Split);
        click(&mut e, &mut t, (2.05, 0.0)).unwrap();
        assert_eq!(ends(&t, id).1.x, 2.0);
        assert!(e.undo(&mut t));
        e.snap_enabled = false;
        click(&mut e, &mut t, (2.05, 0.0)).unwrap();
        assert!((ends(&t, id).1.x - 2.05).abs() < 1.0e-12);
    }

    #[test]
    fn extend_uses_nearest_boundary_on_the_chosen_side() {
        let mut t = GeometryTopology::new();
        let id = line(&mut t, (0.0, 0.0), (1.0, 0.0));
        for x in [-3.0, -1.0, 2.0, 4.0] {
            line(&mut t, (x, -1.0), (x, 1.0));
        }
        // Its infinite extension crosses the target, but this boundary does not.
        line(&mut t, (1.5, 2.0), (1.5, 3.0));
        let mut e = editor(GeometryTool::Extend);
        click(&mut e, &mut t, (0.95, 0.0)).unwrap();
        assert_eq!(
            ends(&t, id),
            (Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0))
        );
        click(&mut e, &mut t, (0.01, 0.0)).unwrap();
        assert_eq!(ends(&t, id).0, Vec3::new(-1.0, 0.0, 0.0));
        t.validate().unwrap();
    }

    #[test]
    fn trim_middle_retains_both_outer_sections() {
        let mut t = GeometryTopology::new();
        let id = line(&mut t, (0.0, 0.0), (4.0, 0.0));
        line(&mut t, (1.0, -1.0), (1.0, 1.0));
        line(&mut t, (3.0, -1.0), (3.0, 1.0));
        let mut e = editor(GeometryTool::Trim);
        click(&mut e, &mut t, (2.0, 0.0)).unwrap();
        assert_eq!(t.edges().count(), 4);
        assert_eq!(
            ends(&t, id),
            (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0))
        );
        assert!(t.edges().any(|edge| endpoints(&t, edge)
            == Some((Vec3::new(3.0, 0.0, 0.0), Vec3::new(4.0, 0.0, 0.0)))));
        t.validate().unwrap();
    }

    #[test]
    fn trim_shared_endpoint_does_not_move_the_adjoining_line() {
        let mut t = GeometryTopology::new();
        let id = line(&mut t, (0.0, 0.0), (4.0, 0.0));
        let adjacent = line(&mut t, (0.0, 0.0), (-1.0, -1.0));
        line(&mut t, (2.0, -1.0), (2.0, 1.0));
        let before = ends(&t, adjacent);
        let mut e = editor(GeometryTool::Trim);
        click(&mut e, &mut t, (1.0, 0.0)).unwrap();
        assert_eq!(ends(&t, id).0, Vec3::new(2.0, 0.0, 0.0));
        assert_eq!(ends(&t, adjacent), before);
        t.validate().unwrap();
    }

    #[test]
    fn invalid_edits_are_atomic_and_do_not_add_undo_entries() {
        let mut t = GeometryTopology::new();
        line(&mut t, (0.0, 0.0), (4.0, 0.0));
        let before = document(&t);
        for tool in [
            GeometryTool::Trim,
            GeometryTool::Extend,
            GeometryTool::Split,
        ] {
            let mut e = editor(tool);
            assert!(click(&mut e, &mut t, (0.0, 0.0)).is_err());
            assert_eq!(document(&t), before);
            assert!(!e.undo(&mut t));
        }
    }

    #[test]
    fn protected_profile_is_not_detached_by_line_tools() {
        let mut t = GeometryTopology::new();
        add_rectangle_at(&mut t, (0.0, 0.0), (4.0, 2.0)).unwrap();
        line(&mut t, (2.0, -1.0), (2.0, 1.0));
        let before = document(&t);
        for tool in [
            GeometryTool::Split,
            GeometryTool::Trim,
            GeometryTool::Extend,
        ] {
            let mut e = editor(tool);
            assert!(click(&mut e, &mut t, (1.0, 0.0)).is_err());
            assert_eq!(document(&t), before);
            assert!(!e.undo(&mut t));
        }
    }

    #[test]
    fn picking_tolerance_is_in_pixels_at_any_zoom() {
        let mut t = GeometryTopology::new();
        let id = line(&mut t, (-1.0, 0.0), (1.0, 0.0));
        for zoom in [10.0, 100.0, 10000.0] {
            let mut e = editor(GeometryTool::Split);
            e.transform.pixels_per_unit = zoom;
            let (x, y) = e.transform.world_to_screen((0.0, 0.0));
            assert_eq!(pick_line(&t, &e.transform, (x, y + 7.0)).unwrap().0, id);
            assert!(pick_line(&t, &e.transform, (x, y + 9.0)).is_none());
            e.cursor_moved(&t, (x, y + 7.0));
            assert_eq!(e.hover_target, Some(GeometrySelectionTarget::Edge(id)));
        }
    }

    #[test]
    fn context_pick_selects_open_line_and_rejects_profile_boundary() {
        let mut t = GeometryTopology::new();
        let id = line(&mut t, (-3.0, 0.0), (-1.0, 0.0));
        add_rectangle_at(&mut t, (0.0, 0.0), (4.0, 2.0)).unwrap();
        let mut e = editor(GeometryTool::Line);
        let before = document(&t);
        assert!(e.select_open_line_at(&t, e.transform.world_to_screen((-2.0, 0.0))));
        assert_eq!(e.active_tool, GeometryTool::Select);
        assert_eq!(e.selection, vec![GeometrySelectionTarget::Edge(id)]);
        assert!(!e.select_open_line_at(&t, e.transform.world_to_screen((1.0, 0.0))));
        assert_eq!(document(&t), before);
        assert_eq!(e.selection, vec![GeometrySelectionTarget::Edge(id)]);
    }

    #[test]
    fn edit_after_undo_discards_redo_branch() {
        let mut t = GeometryTopology::new();
        let id = line(&mut t, (0.0, 0.0), (4.0, 0.0));
        let mut e = editor(GeometryTool::Split);
        click(&mut e, &mut t, (1.0, 0.0)).unwrap();
        assert!(e.undo(&mut t));
        click(&mut e, &mut t, (3.0, 0.0)).unwrap();
        assert!(!e.redo(&mut t));
        assert_eq!(ends(&t, id).1.x, 3.0);
    }

    #[test]
    fn extend_handles_oblique_lines_with_reversed_direction() {
        let mut t = GeometryTopology::new();
        let id = line(&mut t, (2.0, 2.0), (0.0, 0.0));
        line(&mut t, (3.0, 2.0), (3.0, 4.0));
        line(&mut t, (-1.0, -2.0), (-1.0, 0.0));
        let mut e = editor(GeometryTool::Extend);
        click(&mut e, &mut t, (1.95, 1.95)).unwrap();
        assert_eq!(ends(&t, id).0, Vec3::new(3.0, 3.0, 0.0));
        click(&mut e, &mut t, (0.05, 0.05)).unwrap();
        assert_eq!(ends(&t, id).1, Vec3::new(-1.0, -1.0, 0.0));
        t.validate().unwrap();
    }

    #[test]
    fn extending_a_gap_creates_a_persistent_extrudable_profile() {
        let mut t = GeometryTopology::new();
        line(&mut t, (0.0, 0.0), (1.0, 0.0));
        line(&mut t, (2.0, 0.0), (2.0, 2.0));
        line(&mut t, (2.0, 2.0), (0.0, 2.0));
        line(&mut t, (0.0, 2.0), (0.0, 0.0));
        let before = document(&t);
        let mut e = editor(GeometryTool::Extend);
        click(&mut e, &mut t, (0.95, 0.0)).unwrap();
        assert_eq!(t.faces().count(), 1);
        assert_eq!(t.sketches().count(), 1);
        let face = t.faces().next().unwrap().id;
        assert_eq!(e.selection, vec![GeometrySelectionTarget::Face(face)]);
        assert_eq!(t.sketches().next().unwrap().profile_face, Some(face));
        assert!(e.undo(&mut t));
        assert_eq!(document(&t), before);
        assert!(e.redo(&mut t));
        t.cad_extrude(face, 0.5).unwrap();
        t.validate().unwrap();
    }
}
