//! One task action resolver shared by the property pane's OK and Preview.
use super::*;
pub(super) fn action(ui: &MainWindow) -> Option<i32> {
    if !ui.get_cad_edit_body_id().is_empty() {
        return Some(70);
    }
    if ui.get_cad_editing_pad() {
        return Some(31);
    }
    if ui.get_cad_sketch_editing() {
        return Some(if ui.get_cad_edit_sketch_id() > 0 {
            29
        } else {
            21
        });
    }
    match ui.get_cad_operation() {
        200..=204 => Some(0),
        20 => Some(47),
        212 => Some(29),
        action @ (1 | 2 | 3 | 4 | 5 | 6 | 7 | 9 | 10 | 11 | 13 | 24 | 34 | 41 | 42 | 43 | 44
        | 46 | 49 | 53 | 54 | 55 | 56 | 57) => Some(action),
        _ => None,
    }
}
/// Synchronize keyboard and legacy commands without resetting entered values.
pub(super) fn open_for_action(ui: &MainWindow, action: i32) {
    let operation = match action {
        0 => 200 + ui.get_cad_primitive_kind(),
        47 => 20,
        29 => 212,
        other => other,
    };
    let meta = match operation {
        212 => Some((0, "Edit sketch dimensions")),
        31 => Some((
            0,
            if ui.get_cad_editing_pocket() {
                "Edit Pocket"
            } else {
                "Edit Pad"
            },
        )),
        other => super::cad_tools::tool(other),
    };
    if let Some((workbench, title)) = meta {
        ui.set_cad_operation(operation);
        ui.set_cad_workbench(workbench);
        ui.set_cad_task_title(title.into());
        ui.set_cad_task_open(true);
        ui.set_geometry_sidebar_tab(0);
    }
}
pub(super) fn close(ui: &MainWindow, state: &mut AppState) {
    super::sketch_edit::cancel(ui, state);
    state.face_sketch_host = None;
    state.face_sketch_start = None;
    state.face_sketch_cursor = None;
    state.geometry_editor.cancel();
    ui.set_cad_task_open(false);
    ui.set_cad_region_owner("".into());
    ui.set_cad_region_grouped(false);
    ui.set_cad_pick_role(0);
    ui.set_cad_edit_body_id("".into());
    ui.set_cad_sketch_host("".into());
    ui.set_cad_profile_face("".into());
    ui.set_cad_axis_edge("".into());
    ui.set_cad_axis_mode(0);
    ui.set_cad_editing_pad(false);
    ui.set_cad_editing_pocket(false);
    ui.set_cad_task_title("Properties".into());
}
pub(super) fn invalidate(ui: &MainWindow, state: &mut AppState) {
    ui.set_cad_preview_generation(ui.get_cad_preview_generation().wrapping_add(1));
    state.cad_preview = None;
    super::cad::invalidate_preview_job(state);
    ui.set_cad_auto_preview_status("Waiting for complete parameters…".into());
}
pub(super) fn bind(ui: &MainWindow, shared: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let changed = shared.clone();
    ui.on_cad_parameters_changed(move || {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut state = changed.borrow_mut();
        let had_preview = state.cad_preview.is_some();
        invalidate(&ui, &mut state);
        super::cad_constraints::resolve_fields(&ui, &state);
        if super::sketch_edit::valid(&state).is_some() {
            state.show_geometry_3d = false;
            ui.set_geometry_cad_view_3d(false);
        }
        if had_preview || super::sketch_edit::valid(&state).is_some() {
            refresh_ui(&ui, &state);
        }
    });
    let weak = ui.as_weak();
    let state_weak = Rc::downgrade(shared);
    let mut previous = String::new();
    let mut submitted = String::new();
    let mut requested = false;
    let mut changed_at = std::time::Instant::now();
    shared.borrow().cad_auto_preview_timer.start(
        TimerMode::Repeated,
        Duration::from_millis(100),
        move || {
            let (Some(ui), Some(shared)) = (weak.upgrade(), state_weak.upgrade()) else {
                return;
            };
            if !ui.get_cad_task_open() || !ui.get_cad_auto_preview() || !ui.get_project_loaded() {
                previous.clear();
                submitted.clear();
                return;
            }
            let Some(action) = action(&ui).filter(|a| !matches!(a, 13 | 21)) else {
                return;
            };
            let mut state = shared.borrow_mut();
            if state.face_sketch_host.is_some() || super::sketch_edit::valid(&state).is_some() {
                return;
            }
            let strings = [
                ui.get_cad_name(),
                ui.get_cad_size_x(),
                format!(
                    "{}:{}:{}:{}:{}",
                    ui.get_cad_finish_corner0(),
                    ui.get_cad_finish_corner1(),
                    ui.get_cad_finish_corner2(),
                    ui.get_cad_finish_corner3(),
                    ui.get_cad_finish_opening()
                )
                .into(),
                ui.get_cad_size_y(),
                ui.get_cad_size_z(),
                ui.get_cad_pos_x(),
                ui.get_cad_pos_y(),
                ui.get_cad_pos_z(),
                ui.get_cad_angle(),
                ui.get_cad_factor(),
                ui.get_cad_tolerance(),
                ui.get_cad_revolve_x(),
                ui.get_cad_revolve_y(),
                ui.get_cad_revolve_z(),
                ui.get_cad_axis_x(),
                ui.get_cad_axis_y(),
                ui.get_cad_axis_z(),
                ui.get_cad_profile_face(),
                ui.get_cad_axis_edge(),
                ui.get_cad_edit_body_id(),
                ui.get_cad_sketch_offset(),
            ];
            let ints = [
                action,
                ui.get_cad_operation(),
                ui.get_cad_source(),
                ui.get_cad_target(),
                ui.get_cad_axis(),
                ui.get_cad_axis_mode(),
                ui.get_cad_count(),
                ui.get_cad_primitive_kind(),
                ui.get_cad_edit_sketch_id(),
                ui.get_cad_feature_index(),
                ui.get_cad_sketch_kind(),
                ui.get_cad_sketch_plane(),
                ui.get_cad_preview_generation(),
            ];
            let flags = [
                ui.get_cad_reverse(),
                ui.get_cad_through_all(),
                ui.get_cad_transform_copy(),
                ui.get_cad_drive_x(),
                ui.get_cad_drive_y(),
                ui.get_cad_drive_width(),
                ui.get_cad_drive_height(),
                ui.get_cad_equal_sides(),
                ui.get_cad_centered(),
            ];
            let key = format!(
                "{:?}|{:?}|{:?}|{:?}|{}|{:?}|{:?}",
                strings,
                ints,
                flags,
                state.workbench.geometry().revision(),
                state.workbench_project.project_id,
                state.workspace_path,
                state.wb_selected_targets
            );
            if key != previous {
                requested = false;
                previous = key.clone();
                changed_at = std::time::Instant::now();
                let had_preview = state.cad_preview.take().is_some();
                super::cad::invalidate_preview_job(&mut state);
                ui.set_cad_auto_preview_status("Waiting for complete parameters…".into());
                if had_preview {
                    refresh_ui(&ui, &state);
                }
            }
            if ui.get_cad_pick_role() != 0 {
                ui.set_cad_auto_preview_status("Pick the requested profile or axis…".into());
                return;
            }
            if state.cad_job.is_some() || changed_at.elapsed() < Duration::from_millis(350) {
                return;
            }
            if super::cad::preview(&state).is_some() {
                submitted = key;
                requested = false;
                ui.set_cad_auto_preview_status("Preview is up to date".into());
                return;
            }
            if submitted == key {
                if requested {
                    ui.set_cad_auto_preview_status(
                        "Preview failed. Adjust parameters; see the operation message above."
                            .into(),
                    );
                    requested = false;
                }
                return;
            }
            if let Err(error) = super::cad::validate_preview(&ui, &state, action) {
                submitted = key;
                requested = false;
                ui.set_cad_auto_preview_status(error.into());
                return;
            }
            submitted = key;
            requested = true;
            ui.set_cad_auto_preview_status("Updating preview…".into());
            drop(state);
            ui.invoke_cad_command(62);
        },
    );
}
