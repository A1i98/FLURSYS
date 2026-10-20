//! Tool activation prepares a task; only the task's Preview/Apply publishes geometry.
use super::*;

pub(super) fn tool(action: i32) -> Option<(i32, &'static str)> {
    Some(match action {
        56 => (0, "Hole"),
        57 => (0, "Revolved cut"),
        53 => (4, "Prism edge fillet"),
        54 => (4, "Prism edge chamfer"),
        55 => (4, "Prism shell"),
        1 => (0, "Pad"),
        24 => (0, "Additive Pad"),
        34 => (0, "Pocket"),
        2 => (0, "Revolve"),
        20 => (0, "Ruled loft"),
        43 => (0, "Vector sweep"),
        46 => (0, "Tapered extrusion"),
        200 => (1, "Box"),
        201 => (1, "Cylinder"),
        202 => (1, "Sphere"),
        203 => (1, "Cone"),
        204 => (1, "Torus"),
        3 => (2, "Move"),
        4 => (2, "Rotate"),
        5 => (2, "Uniform scale"),
        6 => (2, "Translated copy"),
        7 => (2, "Linear pattern"),
        44 => (2, "Circular pattern"),
        41 => (2, "Mirror copy"),
        42 => (2, "Independent scale copy"),
        13 => (2, "Rename body"),
        9 => (3, "Union"),
        10 => (3, "Difference"),
        11 => (3, "Intersection"),
        49 => (4, "Tessellation quality"),
        210 => (0, "New sketch"),
        211 => (0, "Offset sketch"),
        213 => (0, "Sketch on solid face"),
        _ => return None,
    })
}
pub(super) fn bind(ui: &MainWindow, shared: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let shared = shared.clone();
    ui.on_cad_activate_tool(move |action| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        activate(&ui, &mut shared.borrow_mut(), action);
    });
}

fn activate(ui: &MainWindow, state: &mut AppState, action: i32) {
    let Some((workbench, title)) = tool(action) else {
        return;
    };
    if state.cad_job.is_some()
        || super::cad::preview(&state).is_some()
        || super::sketch_edit::valid(&state).is_some()
        || state.face_sketch_host.is_some()
    {
        ui.set_cad_feedback(
            "Finish or cancel the active sketch/preview before changing tools.".into(),
        );
        return;
    }
    let host = if matches!(action, 213 | 56) {
        let g = state.workbench.geometry();
        let selected = state.wb_selected_targets.iter().find_map(|t| match t {
            GeometrySelectionTarget::Face(f) => Some(*f),
            _ => None,
        });
        let Some(face) = selected.filter(|f| g.cad_owner(*f).is_some()) else {
            ui.set_cad_feedback("Select a planar face of a solid first.".into());
            return;
        };
        if let Err(e) = g.sketch_frame_for_face(face) {
            ui.set_cad_feedback(format!("Sketch support: {e}").into());
            return;
        }
        Some(face)
    } else {
        None
    };
    ui.set_cad_region_owner("".into());
    ui.set_cad_region_grouped(false);
    super::cad_constraints::reset(ui);
    ui.set_cad_edit_body_id("".into());
    ui.set_cad_sketch_host(
        host.map(|id| id.get().to_string())
            .unwrap_or_default()
            .into(),
    );
    if matches!(action, 1 | 2 | 20 | 24 | 34 | 43 | 46 | 57) {
        let face = state.wb_selected_targets.iter().find_map(|t| match t {
            GeometrySelectionTarget::Face(id)
                if state.workbench.geometry().cad_validate_profile(*id).is_ok() =>
            {
                Some(*id)
            }
            _ => None,
        });
        ui.set_cad_profile_face(face.map(|f| f.get().to_string()).unwrap_or_default().into());
    }
    if matches!(action, 2 | 57) {
        ui.set_cad_axis_mode(0);
        ui.set_cad_axis_edge("".into());
    }
    if matches!(action, 53..=55) {
        let g = state.workbench.geometry();
        let owner = state
            .wb_selected_targets
            .iter()
            .find_map(|t| match t {
                GeometrySelectionTarget::Body(id) => Some(*id),
                GeometrySelectionTarget::Face(id) => g.cad_owner(*id),
                GeometrySelectionTarget::Edge(id) => g.cad_edge_owner(*id),
                GeometrySelectionTarget::Vertex(id) => g.cad_vertex_owner(*id),
            })
            .or_else(|| {
                g.cad_bodies()
                    .nth(ui.get_cad_source().max(0) as usize)
                    .map(|(id, _)| id)
            });
        let Some(owner) = owner else {
            ui.set_cad_feedback("Select a Box or rectangular Pad first.".into());
            return;
        };
        let edges = if action != 55 {
            state
                .wb_selected_targets
                .iter()
                .filter_map(|t| match t {
                    GeometrySelectionTarget::Edge(id) => Some(*id),
                    _ => None,
                })
                .collect::<Vec<_>>()
        } else {
            vec![]
        };
        let face = if action == 55 {
            state.wb_selected_targets.iter().find_map(|t| match t {
                GeometrySelectionTarget::Face(id) => Some(*id),
                _ => None,
            })
        } else {
            None
        };
        let (mask, opening) = match g.cad_finishing_selection(owner, &edges, face) {
            Ok(v) => v,
            Err(e) => {
                ui.set_cad_feedback(e.to_string().into());
                return;
            }
        };
        if let Some(index) = g.cad_bodies().position(|(id, _)| id == owner) {
            ui.set_cad_source(index as i32);
        }
        ui.set_cad_finish_corner0(mask & 1 != 0);
        ui.set_cad_finish_corner1(mask & 2 != 0);
        ui.set_cad_finish_corner2(mask & 4 != 0);
        ui.set_cad_finish_corner3(mask & 8 != 0);
        ui.set_cad_finish_opening(i32::from(opening));
        if let Ok(limit) = g.cad_finishing_maximum(owner, (action - 53) as u8, opening) {
            ui.set_cad_size_x(super::cad_units::display_length(ui, limit * 0.2));
        }
    }
    if action == 213 {
        let result = super::drawing::begin(ui, state, true, false);
        ui.set_cad_feedback(result.unwrap_or_else(|e| e).into());
        refresh_ui(ui, state);
        return;
    }
    let changed = ui.get_cad_operation() != action;
    state.geometry_editor.cancel();
    state.geometry_editor.set_tool(GeometryTool::Select);
    if matches!(action, 1 | 2 | 20 | 24 | 34 | 43 | 46 | 57) {
        state.geometry_pick_mode = 0;
        state.geometry_editor.prefer_faces = true;
    }
    ui.set_cad_workbench(workbench);
    ui.set_geometry_sidebar_tab(0);
    ui.set_cad_operation(action);
    ui.set_cad_task_open(true);
    ui.set_cad_pick_role(0);
    super::cad_properties::invalidate(ui, state);
    ui.set_cad_reverse(false);
    ui.set_cad_transform_copy(false);
    ui.set_cad_task_title(title.into());
    ui.set_cad_editing_pad(false);
    ui.set_cad_editing_pocket(false);
    ui.set_cad_sketch_editing(false);
    if changed && matches!(action, 3 | 4 | 5 | 6 | 7 | 41 | 42 | 44) {
        ui.set_cad_pos_x(super::cad_units::display_length(
            ui,
            if action == 7 { 1.0 } else { 0.0 },
        ));
        ui.set_cad_pos_y("0".into());
        ui.set_cad_pos_z("0".into());
        if action == 4 {
            ui.set_cad_angle(super::cad_units::display_angle(ui, 45.0));
        }
        if matches!(action, 7 | 44) {
            ui.set_cad_count(4);
        }
    }
    if matches!(action, 210 | 211 | 213) {
        ui.set_cad_edit_sketch_id(0);
        ui.set_cad_sketch_plane(if action == 213 { 3 } else { 0 });
        ui.set_cad_pos_x("0".into());
        ui.set_cad_pos_y("0".into());
        ui.set_cad_sketch_offset(super::cad_units::display_length(
            ui,
            if action == 211 { 1.0 } else { 0.0 },
        ));
        ui.set_cad_sketch_editing(true);
        ui.set_cad_feedback(
            "Choose the sketch plane, profile and dimensions; Finish sketch creates the profile."
                .into(),
        );
    } else if (200..=204).contains(&action) {
        ui.set_cad_primitive_kind(action - 200);
        if changed {
            ui.set_cad_name(title.into());
            ui.set_cad_size_x(super::cad_units::display_length(ui, 10.0));
            ui.set_cad_size_y(super::cad_units::display_length(ui, 8.0));
            ui.set_cad_size_z(super::cad_units::display_length(ui, 6.0));
        }
    } else {
        if action == 13 && changed {
            if let Some((_, body)) = state
                .workbench
                .geometry()
                .cad_bodies()
                .nth(ui.get_cad_source().max(0) as usize)
            {
                ui.set_cad_name(body.name.clone().into());
            }
        }
        match action {
            1 | 24 => ui.set_feature_kind_index(0),
            2 => {
                ui.set_feature_kind_index(1);
                if changed {
                    ui.set_cad_angle(super::cad_units::display_angle(ui, 360.0));
                }
            }
            34 => ui.set_feature_kind_index(2),
            42 if changed => {
                ui.set_cad_size_x("1".into());
                ui.set_cad_size_y("1".into());
                ui.set_cad_size_z("1".into());
            }
            4 | 41 if changed => ui.set_cad_axis(2),
            44 if changed => {
                ui.set_cad_angle(super::cad_units::display_angle(ui, 90.0));
                ui.set_cad_axis(2);
            }
            _ => {}
        }
        ui.set_cad_feedback(
            "Set the task parameters, then Preview. OK or Cancel in the task panel.".into(),
        );
    }
    if action == 57 {
        ui.set_cad_angle(super::cad_units::display_angle(ui, 360.));
    }
    if action == 56 {
        let face = host.unwrap();
        let g = state.workbench.geometry();
        if let Ok(frame) = g.sketch_frame_for_face(face) {
            let points = g
                .renderable_faces()
                .into_iter()
                .filter(|s| s.face == face)
                .flat_map(|s| s.vertices)
                .map(|v| {
                    let d = v - frame.origin;
                    (d.dot(frame.x_axis), d.dot(frame.y_axis))
                })
                .collect::<Vec<_>>();
            let bounds = points.iter().fold(
                (
                    f64::INFINITY,
                    f64::NEG_INFINITY,
                    f64::INFINITY,
                    f64::NEG_INFINITY,
                ),
                |(a, b, c, d), &(x, y)| (a.min(x), b.max(x), c.min(y), d.max(y)),
            );
            if !points.is_empty() {
                ui.set_cad_pos_x(super::cad_units::display_length(
                    ui,
                    (bounds.0 + bounds.1) * 0.5,
                ));
                ui.set_cad_pos_y(super::cad_units::display_length(
                    ui,
                    (bounds.2 + bounds.3) * 0.5,
                ));
                ui.set_cad_size_x(super::cad_units::display_length(
                    ui,
                    ((bounds.1 - bounds.0).min(bounds.3 - bounds.2) * 0.2).max(1e-5),
                ));
            }
        }
        ui.set_cad_size_z(super::cad_units::display_length(ui, 1.));
        ui.set_cad_through_all(false);
    }
    if action == 34 {
        ui.set_cad_through_all(false);
        ui.set_cad_pocket_end_mode(0);
        ui.set_cad_pocket_stop_face("".into());
    }
    refresh_ui(ui, state);
}
