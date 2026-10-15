//! Transient measurement picks never modify document geometry or undo history.
use super::*;

pub(super) fn command(ui: &MainWindow, state: &mut AppState, action: i32) -> String {
    let active = match action {
        32 => false,
        33 => true,
        _ => !ui.get_cad_measure_active(),
    };
    ui.set_cad_measure_active(active);
    state.cad_measure_points.clear();
    state.wb_selected_targets.clear();
    state.geometry_editor.selection.clear();
    state.geometry_editor.cancel();
    if state.face_sketch_host.is_some() {
        state.show_geometry_3d = true;
        ui.set_geometry_cad_view_3d(true);
    }
    state.face_sketch_host = None;
    state.face_sketch_start = None;
    state.face_sketch_cursor = None;
    let message = if !active {
        "Measurement closed."
    } else if ui.get_cad_measure_kind() == 1 {
        "Pick two surface points in 3D, or two points in the sketch. Distances use model units; 3D picks follow display tessellation."
    } else {
        "Click an edge/profile for dimensions, or two planar faces for distance / angle. No Ctrl needed. Use Point distance for arbitrary surface points."
    };
    ui.set_cad_measure_result(message.into());
    message.into()
}

pub(super) fn pick(
    ui: &MainWindow,
    state: &mut AppState,
    point: (f64, f64),
    three_d: bool,
) -> bool {
    if !ui.get_cad_measure_active() {
        return false;
    }
    let g = state.workbench.geometry();
    if ui.get_cad_measure_kind() == 1 {
        let position = if three_d {
            super::render::measurement_surface_point(
                g,
                state.geometry_yaw,
                state.geometry_pitch,
                state.geometry_zoom,
                point,
            )
        } else {
            let mut editor = state.geometry_editor.clone();
            editor.prefer_faces = false;
            let vertex = editor.pick(g, point, 9.0).and_then(|t| {
                if let GeometrySelectionTarget::Vertex(id) = t {
                    g.vertex(id).map(|v| v.position)
                } else {
                    None
                }
            });
            vertex.or_else(|| {
                let p = state.geometry_editor.transform.screen_to_world(point);
                Some(Vec3::new(p.0, p.1, 0.0))
            })
        };
        let Some(position) = position else {
            ui.set_cad_measure_result("No surface here. Pick on the visible model.".into());
            return true;
        };
        if state.cad_measure_points.len() >= 2 {
            state.cad_measure_points.clear();
        }
        state.cad_measure_points.push(position);
        let text = if let [a, b] = state.cad_measure_points.as_slice() {
            let d = *b - *a;
            format!("Distance {}{:.6} · ΔX {:.6} · ΔY {:.6} · ΔZ {:.6} (model units). Click to start another measurement.",if three_d {"≈ "} else {""},d.norm(),d.x,d.y,d.z)
        } else {
            format!(
                "Point 1: ({:.4}, {:.4}, {:.4}). Pick the second point.",
                position.x, position.y, position.z
            )
        };
        ui.set_cad_measure_result(text.into());
        return true;
    }
    let target = if three_d {
        let modes = if ui.get_cad_measure_kind() == 2 {
            vec![0]
        } else {
            vec![2, 1, 0]
        };
        modes.into_iter().find_map(|mode| {
            pick_canonical_geometry_target(
                g,
                state.geometry_yaw,
                state.geometry_pitch,
                state.geometry_zoom,
                point,
                mode,
            )
        })
    } else {
        let mut editor = state.geometry_editor.clone();
        editor.prefer_faces = ui.get_cad_measure_kind() == 2;
        editor.pick(g, point, 9.0)
    };
    let Some(target) = target else {
        ui.set_cad_measure_result(
            "No measurable entity here. Pick a profile, edge, vertex or visible face.".into(),
        );
        return true;
    };
    if state.wb_selected_targets.len() >= 2
        || state
            .wb_selected_targets
            .first()
            .is_some_and(|a| std::mem::discriminant(a) != std::mem::discriminant(&target))
    {
        state.wb_selected_targets.clear();
    }
    if !state.wb_selected_targets.contains(&target) {
        state.wb_selected_targets.push(target);
    }
    state.geometry_editor.selection = state.wb_selected_targets.clone();
    let text = super::cad::measure_selection(state).unwrap_or_else(|e| e);
    ui.set_cad_measure_result(text.into());
    true
}
