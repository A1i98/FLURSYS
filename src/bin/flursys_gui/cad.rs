//! CAD commands run on a document snapshot; the UI adopts a result only if the
//! source document is still current. No kernel work runs on the event loop.
use super::*;
use flursys::{
    BodyId, CadBoolean, CadFeatureId, CadPrimitive, CadSketchId, CadTransform, FaceId,
    GeometryTopology,
};
use slint::Model;
use std::sync::mpsc::{self, TryRecvError};
pub(super) struct CadPreview {
    source: GeometryTopology,
    workspace: Option<PathBuf>,
    project_id: String,
    pub(super) geometry: GeometryTopology,
    pub(super) selection: Vec<BodyId>,
    message: String,
}
pub(super) fn preview(state: &AppState) -> Option<&CadPreview> {
    state.cad_preview.as_ref().filter(|p| {
        state.workbench.geometry() == &p.source
            && state.workspace_path == p.workspace
            && state.workbench_project.project_id == p.project_id
    })
}
pub(super) struct CadJob {
    generation: i32,
    preview: bool,
    cancelled: bool,
    source: GeometryTopology,
    workspace: Option<PathBuf>,
    project_id: String,
    result: Receiver<Result<(GeometryTopology, Vec<BodyId>, bool, String), String>>,
}
struct Command {
    action: i32,
    edit_body: Option<BodyId>,
    axis_reference: Option<flursys::CadAxisReference>,
    axis_reversed: bool,
    through_all: bool,
    copy: bool,
    sketch_offset: f64,
    constraints: Option<flursys::CadSketchConstraints>,
    source: Option<BodyId>,
    target: Option<BodyId>,
    face: Option<FaceId>,
    axis: Vec3,
    second_face: Option<FaceId>,
    stop_face: Option<FaceId>,
    sections: Vec<FaceId>,
    edit_sketch: Option<CadSketchId>,
    edit_feature: Option<CadFeatureId>,
    kind: i32,
    name: String,
    size: [f64; 3],
    position: Vec3,
    angle: f64,
    factor: f64,
    tolerance: f64,
    count: usize,
    path: Option<PathBuf>,
}
fn number(s: SharedString, label: &str) -> Result<f64, String> {
    let v = s
        .trim()
        .parse::<f64>()
        .map_err(|_| format!("{label} must be a number"))?;
    if !v.is_finite() {
        Err(format!("{label} must be finite"))
    } else {
        Ok(v)
    }
}
pub(super) fn invalidate_preview_job(state: &mut AppState) {
    if let Some(job) = state.cad_job.as_mut() {
        if job.preview {
            job.cancelled = true;
        }
    }
}
pub(super) fn validate_preview(
    ui: &MainWindow,
    state: &AppState,
    action: i32,
) -> Result<(), String> {
    let c = read_command(ui, state, action)?.ok_or("Complete the operation parameters")?;
    if c.edit_body.is_none() && matches!(c.action, 1 | 2 | 24 | 34 | 43 | 46 | 57) {
        state
            .workbench
            .geometry()
            .cad_validate_profile(c.face.ok_or("Choose a closed sketch region")?)
            .map_err(|e| e.to_string())?;
    }
    if matches!(c.action,3..=11|24|34|49|53..=55) && c.source.is_none() {
        return Err("Choose a source body".into());
    }
    if matches!(c.action, 9..=11)
        && c.edit_body.is_none()
        && (c.target.is_none() || c.target == c.source)
    {
        return Err("Choose a different tool body".into());
    }
    if matches!(c.action, 2 | 57) {
        if !(c.angle > 0.0 && c.angle <= 360.0) {
            return Err("Revolution angle must be in (0,360] degrees".into());
        }
        if let Some(reference) = c.axis_reference {
            state
                .workbench
                .geometry()
                .cad_reference_axis(reference)
                .map_err(|e| e.to_string())?;
        } else if c.axis.norm() < 1e-12 {
            return Err("Choose a nonzero revolution axis".into());
        }
    }
    if matches!(c.action, 1 | 24 | 31 | 34 | 46) && !c.through_all && c.size[2].abs() < 1e-12 {
        return Err("Enter a nonzero depth".into());
    }
    if matches!(c.action, 53..=55) {
        let source = if let Some(body) = c.edit_body {
            match state
                .workbench
                .geometry()
                .cad_body(body)
                .and_then(|b| b.recipe.as_ref())
            {
                Some(flursys::CadRecipe::Finish { source, .. }) => *source,
                _ => return Err("Finishing source recipe is missing".into()),
            }
        } else {
            c.source.ok_or("Choose a source body")?
        };
        let limit = state
            .workbench
            .geometry()
            .cad_finishing_maximum(source, (c.action - 53) as u8, c.kind as u8)
            .map_err(|e| e.to_string())?;
        if c.size[0] <= 0. || c.size[0] >= limit {
            return Err(format!(
                "Enter a positive radius / thickness below {}",
                super::cad_units::display_length(ui, limit)
            ));
        }
        if c.action != 55 && c.count == 0 {
            return Err("Select at least one longitudinal edge".into());
        }
    }
    if c.action == 56 {
        let face = c.face.ok_or("Hole support face no longer exists")?;
        state
            .workbench
            .geometry()
            .sketch_frame_for_face(face)
            .map_err(|e| e.to_string())?;
        if c.size[0] <= 0. || c.size[2] <= 0. {
            return Err("Hole diameter and depth must be positive".into());
        }
    }
    if c.action == 29 {
        let profile = if c.kind == 0 {
            flursys::CadSketchProfile::Circle { radius: c.size[0] }
        } else {
            flursys::CadSketchProfile::Rectangle {
                width: c.size[0],
                height: c.size[1],
            }
        };
        state
            .workbench
            .geometry()
            .cad_resolve_sketch_parameters_using(
                c.edit_sketch.ok_or("Select a sketch region")?,
                c.position.x,
                c.position.y,
                profile,
                c.constraints.as_ref(),
            )
            .map_err(|e| e.to_string())?;
    }
    if c.action == 47 && c.sections.len() < 2 {
        return Err("Choose at least two loft sections".into());
    }
    Ok(())
}
pub(super) fn bind(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
    super::sketch_edit::bind(ui, state);
    super::cad_tools::bind(ui, state);
    super::cad_units::bind(ui, state);
    super::cad_properties::bind(ui, state);
    super::cad_tree::bind(ui, state);
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_cad_pick_orientation(move |x, y| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if let Some(axis) =
            super::render::pick_orientation_overlay(state.geometry_yaw, state.geometry_pitch, x, y)
        {
            let (yaw, pitch) = geometry_view_angles(axis);
            state.geometry_yaw = yaw;
            state.geometry_pitch = pitch;
            state.geometry_view_axis = axis;
            state.geometry_drag_anchor = None;
            refresh_ui(&ui, &state);
        }
    });

    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_cad_select_body(move |index| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        let id = state
            .workbench
            .geometry()
            .cad_bodies()
            .nth(index.max(0) as usize)
            .map(|(id, _)| id);
        if let Some(id) = id {
            select_tree_geometry_target(&mut state, GeometrySelectionTarget::Body(id), false);
            state.show_geometry_3d = true;
            state.show_mesh = false;
            ui.set_geometry_cad_view_3d(true);
            rebuild_tree_rows(&mut state);
            refresh_ui(&ui, &state);
        }
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_cad_unselect(move |index| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut state = shared.borrow_mut();
        if state.cad_job.is_some() || preview(&state).is_some() {
            return;
        }
        if index >= 0 && (index as usize) < state.wb_selected_targets.len() {
            state.wb_selected_targets.remove(index as usize);
            state.geometry_editor.selection = state.wb_selected_targets.clone();
            state.selected_tree = state
                .wb_selected_targets
                .last()
                .copied()
                .and_then(tree_selection_for_target);
            rebuild_tree_rows(&mut state);
            refresh_ui(&ui, &state);
        }
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_cad_context_pick(move |x, y, w, h| {
        let Some(ui) = weak.upgrade() else {
            return false;
        };
        let mut state = shared.borrow_mut();
        if state.cad_job.is_some()
            || (ui.get_cad_task_open() && !matches!(ui.get_cad_operation(), 2 | 57 | 34))
            || preview(&state).is_some()
            || state.face_sketch_host.is_some()
        {
            return false;
        }
        let Some(point) = preview_image_point(x, y, w, h) else {
            return false;
        };
        let target = pick_canonical_geometry_target(
            state.workbench.geometry(),
            state.geometry_yaw,
            state.geometry_pitch,
            state.geometry_zoom,
            point,
            state.geometry_pick_mode,
        )
        .or_else(|| {
            pick_canonical_geometry_target(
                state.workbench.geometry(),
                state.geometry_yaw,
                state.geometry_pitch,
                state.geometry_zoom,
                point,
                0,
            )
        });
        if let Some(target) = target {
            // A context menu acts on the exact entity under the pointer. Keep
            // an older multi-selection from steering the command to a
            // different face (notably New Sketch on Face).
            select_tree_geometry_target(&mut state, target, false);
        } else {
            state.wb_selected_targets.clear();
            state.geometry_editor.selection.clear();
            state.selected_tree = None;
        }
        rebuild_tree_rows(&mut state);
        refresh_ui(&ui, &state);
        true
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_cad_command(move |mut action| {
        let Some(ui) = weak.upgrade() else { return; };
        if action==72 {ui.invoke_reset_geometry_view();return;}
        if matches!(action,75|76) {
            if !ui.get_cad_busy() && !ui.get_cad_task_open() && !ui.get_cad_preview_active() {
                ui.invoke_geometry_shortcut(if action==75 {6}else{7});
            }
            return;
        }
        let mut state = shared.borrow_mut();
        if matches!(action,87..=91|93..=98) {
            if state.cad_job.is_some(){return;}
            let result=super::cad_profiles::command(&ui,&mut state,action);
            ui.set_cad_feedback(result.unwrap_or_else(|e|e).into());rebuild_tree_rows(&mut state);refresh_ui(&ui,&state);return;
        }
        if matches!(action,78..=86) {
            if state.cad_job.is_some(){return;}
            let result=super::cad_axis::command(&ui,&mut state,action);
            super::cad_properties::invalidate(&ui,&mut state);
            ui.set_cad_feedback(result.unwrap_or_else(|e|e).into());rebuild_tree_rows(&mut state);refresh_ui(&ui,&state);return;
        }
        if matches!(action,66..=68) {
            if state.cad_job.is_some() || ui.get_cad_task_open() || preview(&state).is_some(){return;}
            let g=state.workbench.geometry();
            let selected=state.wb_selected_targets.iter().filter_map(|t|match t {
                GeometrySelectionTarget::Body(id)=>Some(*id),GeometrySelectionTarget::Face(id)=>g.cad_owner(*id),
                GeometrySelectionTarget::Edge(id)=>g.cad_edge_owner(*id),GeometrySelectionTarget::Vertex(id)=>g.cad_vertex_owner(*id),
            }).collect::<BTreeSet<_>>();
            if selected.is_empty() && action!=68 {return;}
            let changes=g.cad_bodies().filter_map(|(id,b)| {
                let visible=match action {66=>selected.contains(&id),67=>if selected.contains(&id){false}else{b.visible},_=>true};
                (b.visible!=visible).then_some((id,visible))
            }).collect::<Vec<_>>();
            if !changes.is_empty(){
                let mut next=g.clone();
                for (id,visible) in changes {let _=next.cad_set_visible(id,visible);}
                let before=state.workbench.geometry().clone();
                state.geometry_editor.record_snapshot(&before);
                *state.workbench.geometry_mut()=next;
                state.workbench.geometry_changed();state.mark_workbench_dirty();rebuild_tree_rows(&mut state);
            }
            ui.set_cad_feedback("Visibility updated. Undo restores the previous visibility.".into());
            refresh_ui(&ui,&state);return;
        }
        if action==71 {ui.set_geometry_sidebar_tab(2);return;}
        if action==61 {
            if let Some(job)=state.cad_job.as_mut() {job.cancelled=true;}
            state.cad_preview=None;
            super::cad_properties::close(&ui,&mut state);
            refresh_ui(&ui,&state);
            ui.set_cad_feedback("Task cancelled. Document unchanged.".into());
            return;
        }
        if matches!(action,60|62) {
            if ui.get_cad_pick_role()!=0 {ui.set_cad_feedback("Finish or cancel reference picking before confirming the operation.".into());return;}
            if state.cad_job.is_some() {return;}
            if action==60 && preview(&state).is_some() {action=35;}
            else {
                let Some(resolved)= (if state.face_sketch_host.is_some() {Some(26)} else {super::cad_properties::action(&ui)}) else {return;};
                action=if action==62 && !matches!(resolved,13|21) {resolved+100}else{resolved};
            }
        }
        if action == -1 {
            if let Some(job) = state.cad_job.as_mut() {
                job.cancelled = true;
                ui.set_cad_feedback("Result will be discarded when the kernel operation finishes.".into());
            }
            return;
        }
        if action==36 {
            state.cad_preview=None;
            if super::sketch_edit::valid(&state).is_some() {state.show_geometry_3d=false;ui.set_geometry_cad_view_3d(false);}
            refresh_ui(&ui,&state);
            ui.set_cad_feedback("Preview discarded. Source model unchanged.".into());
            return;
        }
        if state.cad_job.is_some() { return; }
        if action==35 {
            if let Some(p)=state.cad_preview.take() {
                let (tx,rx)=mpsc::channel();
                let _=tx.send(Ok((p.geometry,p.selection,true,p.message)));
                state.cad_job=Some(CadJob {generation:ui.get_cad_preview_generation(),preview:false,cancelled:false,source:p.source,workspace:p.workspace,project_id:p.project_id,result:rx});
                poll(&ui,&mut state);
                refresh_ui(&ui,&state);
            }
            return;
        }
        if !require_project(&mut state) { refresh_ui(&ui, &state); return; }
        if matches!(action, 63 | 64 | 65 | 77 | 92 | 21 | 22 | 23 | 26 | 27 | 28 | 30 | 32 | 33 | 39 | 40) {
            let outcome = sketch_command(&ui, &mut state, action);
            ui.set_cad_feedback(outcome.unwrap_or_else(|e| e).into());
            rebuild_tree_rows(&mut state);
            refresh_ui(&ui, &state);
            return;
        }
        let is_preview=action>=100;
        let action=if is_preview {action-100} else {action};
        if is_preview && !matches!(action,0|1|2|3|4|5|6|7|8|9|10|11|20|24|29|31|34|41|42|43|44|46|47|49|53|54|55|56|57|70) {return;}
        state.cad_preview=None;
        if !ui.get_cad_task_open() && matches!(action,1|2|24|34|43|46|57) && ui.get_cad_profile_face().is_empty() {
            if let Ok(face)=selected_extrusion_face(&state) {ui.set_cad_profile_face(face.get().to_string().into());}
        }
        super::cad_properties::open_for_action(&ui,action);
        if matches!(action,1|2|24|34|43|46|57) {
            if let Err(error)=validate_preview(&ui,&state,action){ui.set_cad_feedback(error.into());return;}
        }
        let command = match read_command(&ui, &state, action) {
            Ok(Some(command)) => command,
            Ok(None) => return,
            Err(error) => { ui.set_cad_feedback(error.into()); return; }
        };
        let source = state.workbench.geometry().clone();
        let geometry = source.clone();
        let workspace = state.workspace_path.clone();
        let project_id = state.workbench_project.project_id.clone();
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let outcome = std::panic::catch_unwind(|| execute(geometry, command))
                .unwrap_or_else(|_| Err("The geometry kernel rejected this operation. The source document was preserved.".into()));
            let _ = tx.send(outcome);
        });
        state.cad_job = Some(CadJob { generation:ui.get_cad_preview_generation(),preview:is_preview, cancelled: false, source, workspace, project_id, result: rx });
        ui.set_cad_busy(true);
        ui.set_cad_feedback("Building geometry…".into());
    });
}
fn editable_features(g: &GeometryTopology) -> Vec<(CadFeatureId, SharedString)> {
    let mut features = g
        .extrude_features()
        .map(|f| {
            (
                f.id,
                format!(
                    "Pad {} · Sketch {} · Body {}",
                    f.id.get(),
                    f.sketch.get(),
                    f.body.get()
                )
                .into(),
            )
        })
        .chain(g.pocket_features().map(|f| {
            (
                f.id,
                format!(
                    "Pocket {} · Sketch {} · Body {}",
                    f.id.get(),
                    f.sketch.get(),
                    f.body.get()
                )
                .into(),
            )
        }))
        .collect::<Vec<_>>();
    features.sort_by_key(|(id, _)| *id);
    features
}
fn read_command(ui: &MainWindow, state: &AppState, action: i32) -> Result<Option<Command>, String> {
    if action == 70 {
        let body = state
            .workbench
            .geometry()
            .cad_bodies()
            .find(|(id, _)| id.get().to_string() == ui.get_cad_edit_body_id().as_str())
            .map(|(id, _)| id)
            .ok_or("The edited body no longer exists")?;
        let operation = ui.get_cad_operation();
        let mut command = read_command(
            ui,
            state,
            if (200..=204).contains(&operation) {
                0
            } else {
                operation
            },
        )?
        .ok_or("Operation cannot be edited")?;
        command.edit_body = Some(body);
        return Ok(Some(command));
    }
    let length = |value, label| super::cad_units::length(ui, value, label);
    let bodies = state
        .workbench
        .geometry()
        .cad_bodies()
        .map(|(id, _)| id)
        .collect::<Vec<_>>();
    let source = bodies.get(ui.get_cad_source().max(0) as usize).copied();
    let target = bodies.get(ui.get_cad_target().max(0) as usize).copied();
    let face = if matches!(action, 1 | 2 | 24 | 34 | 43 | 46 | 57) {
        if ui.get_cad_task_open() {
            super::cad_axis::profile(ui, state.workbench.geometry())
        } else {
            selected_extrusion_face(state).ok()
        }
    } else {
        selected_extrusion_face(state).ok()
    };
    let second_face = state
        .wb_selected_targets
        .iter()
        .filter_map(|t| {
            if let GeometrySelectionTarget::Face(id) = t {
                Some(*id)
            } else {
                None
            }
        })
        .nth(1);
    let mut c = Command {
        action,
        edit_body: None,
        axis_reference: None,
        axis_reversed: ui.get_cad_reverse(),
        through_all: (matches!(action, 31 | 34) && ui.get_cad_pocket_end_mode() == 1)
            || ui.get_cad_through_all(),
        copy: ui.get_cad_transform_copy(),
        sketch_offset: 0.0,
        constraints: None,
        source,
        target,
        face,
        second_face,
        stop_face: if matches!(action, 31 | 34)
            && ui.get_cad_editing_pocket()
            && ui.get_cad_pocket_end_mode() == 2
            || action == 34 && ui.get_cad_pocket_end_mode() == 2
        {
            let id = ui
                .get_cad_pocket_stop_face()
                .parse::<u64>()
                .map_err(|_| "Pick a Pocket stop face")?;
            Some(
                state
                    .workbench
                    .geometry()
                    .faces()
                    .find(|face| face.id.get() == id)
                    .map(|face| face.id)
                    .ok_or("Pocket stop face no longer exists")?,
            )
        } else {
            None
        },
        sections: state
            .wb_selected_targets
            .iter()
            .filter_map(|t| match t {
                GeometrySelectionTarget::Face(id) => Some(*id),
                _ => None,
            })
            .collect(),
        edit_sketch: state
            .workbench
            .geometry()
            .sketches()
            .find(|s| s.id.get() == ui.get_cad_edit_sketch_id().max(0) as u64)
            .map(|s| s.id),
        edit_feature: editable_features(state.workbench.geometry())
            .get(ui.get_cad_feature_index().max(0) as usize)
            .map(|(id, _)| *id),
        axis: if !matches!(action, 2 | 4 | 8 | 41 | 44 | 57)
            || (matches!(action, 2 | 57) && ui.get_cad_axis_mode() != 0)
        {
            Vec3::new(0., 1., 0.)
        } else {
            match ui.get_cad_axis() {
                0 => Vec3::new(1., 0., 0.),
                2 => Vec3::new(0., 0., 1.),
                3 => Vec3::new(
                    number(ui.get_cad_axis_x(), "Axis X")?,
                    number(ui.get_cad_axis_y(), "Axis Y")?,
                    number(ui.get_cad_axis_z(), "Axis Z")?,
                ),
                _ => Vec3::new(0., 1., 0.),
            }
        },
        kind: ui.get_cad_primitive_kind(),
        name: ui.get_cad_name().to_string(),
        size: [1.0; 3],
        position: Vec3::new(0.0, 0.0, 0.0),
        angle: 0.0,
        factor: 1.0,
        tolerance: 1e-3,
        count: ui.get_cad_count().max(0) as usize,
        path: None,
    };
    if action == 20 {
        c.face = state.wb_selected_targets.iter().find_map(|t| {
            if let GeometrySelectionTarget::Face(id) = t {
                Some(*id)
            } else {
                None
            }
        });
    }
    if matches!(action, 0 | 3 | 4 | 5 | 6 | 7 | 8 | 41 | 42 | 44) {
        c.position = Vec3::new(
            length(ui.get_cad_pos_x(), "X")?,
            length(ui.get_cad_pos_y(), "Y")?,
            length(ui.get_cad_pos_z(), "Z")?,
        );
    }
    if matches!(action, 37 | 38 | 50..=52) {
        c.edit_sketch = state
            .workbench
            .geometry()
            .sketches()
            .nth(ui.get_cad_sketch_index().max(0) as usize)
            .map(|s| s.id);
    }
    if matches!(action, 2 | 57) && ui.get_cad_axis_mode() == 0 {
        c.position = Vec3::new(
            length(ui.get_cad_revolve_x(), "Pivot X")?,
            length(ui.get_cad_revolve_y(), "Pivot Y")?,
            length(ui.get_cad_revolve_z(), "Pivot Z")?,
        );
    }
    if action == 29 {
        c.sketch_offset = length(ui.get_cad_sketch_offset(), "Sketch plane offset")?;
        c.kind = ui.get_cad_sketch_kind();
        c.position = Vec3::new(
            length(ui.get_cad_pos_x(), "Local X")?,
            length(ui.get_cad_pos_y(), "Local Y")?,
            0.0,
        );
        c.size[0] = length(ui.get_cad_size_x(), "Radius / width")?;
        if c.kind == 1 {
            c.size[1] = length(ui.get_cad_size_y(), "Height")?;
        }
    }
    if action == 29 {
        c.constraints = Some(super::cad_constraints::read(
            ui,
            c.position.x,
            c.position.y,
            c.size[0],
            c.size[1],
        ));
    }
    if action == 56 {
        c.face = state
            .workbench
            .geometry()
            .faces()
            .find(|f| f.id.get().to_string() == ui.get_cad_sketch_host().as_str())
            .map(|f| f.id);
        c.size[0] = length(ui.get_cad_size_x(), "Hole diameter")?;
        c.size[2] = if c.through_all {
            1.
        } else {
            length(ui.get_cad_size_z(), "Hole depth")?
        };
        c.position = Vec3::new(
            length(ui.get_cad_pos_x(), "Hole center X")?,
            length(ui.get_cad_pos_y(), "Hole center Y")?,
            0.,
        );
    }
    if matches!(action, 53..=55) {
        c.size[0] = length(ui.get_cad_size_x(), "Radius / thickness")?;
        c.count = usize::from(ui.get_cad_finish_corner0())
            + 2 * usize::from(ui.get_cad_finish_corner1())
            + 4 * usize::from(ui.get_cad_finish_corner2())
            + 8 * usize::from(ui.get_cad_finish_corner3());
        c.kind = ui.get_cad_finish_opening();
    }
    if action == 42 {
        c.size = [
            number(ui.get_cad_size_x(), "X scale")?,
            number(ui.get_cad_size_y(), "Y scale")?,
            number(ui.get_cad_size_z(), "Z scale")?,
        ];
    }
    if matches!(action, 0 | 43) {
        c.size[0] = length(ui.get_cad_size_x(), "Length / radius")?;
        if c.kind != 2 || matches!(action, 42 | 43) {
            c.size[1] = length(ui.get_cad_size_y(), "Width / height")?;
        }
        if c.kind == 0 || matches!(action, 42 | 43) {
            c.size[2] = length(ui.get_cad_size_z(), "Height")?;
        }
    }
    if action == 34 && c.through_all {
        c.size[2] = 1.0;
    }
    let pocket_edit = action == 31
        && c.edit_feature.is_some_and(|id| {
            state
                .workbench
                .geometry()
                .pocket_features()
                .any(|f| f.id == id)
        });
    if pocket_edit && c.through_all {
        c.size[2] = 1.0;
    }
    if matches!(action, 1 | 24 | 46)
        || (action == 31 && !(pocket_edit && c.through_all))
        || (action == 34 && !c.through_all)
    {
        c.size[2] = length(ui.get_cad_size_z(), "Extrusion distance")?;
    }
    if matches!(action, 2 | 4 | 8 | 44 | 57) {
        c.angle = super::cad_units::angle(ui, ui.get_cad_angle())?;
    }
    if matches!(action, 8 | 44) && c.angle.abs() < 1e-8 {
        return Err("Circular pattern angular step must be nonzero".into());
    }
    if matches!(action, 5 | 46) {
        c.factor = number(ui.get_cad_factor(), "Scale")?;
    }
    if matches!(action, 9..=11 | 24 | 34 | 49 | 56 | 57) || pocket_edit {
        c.tolerance = length(ui.get_cad_tolerance(), "Boolean tolerance")?;
    }
    if matches!(action, 15..=19 | 45) {
        c.path = if action == 18 || action == 19 {
            if action == 19 {
                rfd::FileDialog::new()
                    .add_filter("STEP solid", &["step", "stp"])
                    .pick_file()
            } else {
                rfd::FileDialog::new()
                    .add_filter("Truck B-rep", &["json"])
                    .pick_file()
            }
        } else {
            let (ext, desc) = match action {
                15 => ("step", "STEP CAD"),
                16 => ("stl", "STL mesh"),
                45 => ("obj", "Wavefront OBJ mesh"),
                _ => ("json", "Truck B-rep"),
            };
            rfd::FileDialog::new()
                .add_filter(desc, &[ext])
                .set_file_name(format!("body.{ext}"))
                .save_file()
        };
        if c.path.is_none() {
            return Ok(None);
        }
    }
    if matches!(action, 2 | 57) {
        c.axis_reference = super::cad_axis::reference(ui, state.workbench.geometry(), c.face)?;
    }
    if ui.get_cad_reverse() {
        if matches!(action, 1 | 24 | 46) {
            c.size[2] = -c.size[2];
        }
        if matches!(action, 2 | 57) {
            c.axis = c.axis * -1.0;
        }
    }
    Ok(Some(c))
}
fn execute(
    mut g: GeometryTopology,
    c: Command,
) -> Result<(GeometryTopology, Vec<BodyId>, bool, String), String> {
    let source = || {
        c.source
            .ok_or_else(|| "Select a CAD body from the model tree".to_string())
    };
    if let Some(body) = c.edit_body {
        let mut recipe = g
            .cad_body(body)
            .and_then(|b| b.recipe.clone())
            .ok_or("Saved construction recipe not found")?;
        match &mut recipe {
            flursys::CadRecipe::Finish {
                amount,
                corners,
                opening,
                ..
            } => {
                *amount = c.size[0];
                *corners = c.count as u8;
                *opening = c.kind as u8;
            }
            flursys::CadRecipe::Primitive { shape, position } => {
                *position = c.position;
                *shape = match c.kind {
                    0 => CadPrimitive::Box {
                        length: c.size[0],
                        width: c.size[1],
                        height: c.size[2],
                    },
                    1 => CadPrimitive::Cylinder {
                        radius: c.size[0],
                        height: c.size[1],
                    },
                    2 => CadPrimitive::Sphere { radius: c.size[0] },
                    3 => CadPrimitive::Cone {
                        radius: c.size[0],
                        height: c.size[1],
                    },
                    _ => CadPrimitive::Torus {
                        major_radius: c.size[0],
                        minor_radius: c.size[1],
                    },
                };
            }
            flursys::CadRecipe::Revolve {
                pivot,
                axis,
                degrees,
                axis_reference,
                axis_reversed,
                ..
            } => {
                *pivot = c.position;
                *axis = c.axis;
                *degrees = c.angle;
                *axis_reference = c.axis_reference;
                *axis_reversed = c.axis_reversed;
            }
            flursys::CadRecipe::Sweep { displacement, .. } => {
                *displacement = Vec3::new(c.size[0], c.size[1], c.size[2]);
            }
            flursys::CadRecipe::Taper {
                distance, factor, ..
            } => {
                *distance = c.size[2];
                *factor = c.factor;
            }
            flursys::CadRecipe::Loft { sections } => {
                *sections = c.sections.clone();
            }
            flursys::CadRecipe::Boolean { tolerance, .. } => {
                *tolerance = c.tolerance;
            }
            flursys::CadRecipe::Transform { transform, .. }
            | flursys::CadRecipe::Modified { transform, .. } => {
                *transform = match transform {
                    CadTransform::Translate(_) => CadTransform::Translate(c.position),
                    CadTransform::Rotate { .. } => CadTransform::Rotate {
                        pivot: c.position,
                        axis: c.axis,
                        degrees: c.angle,
                    },
                    CadTransform::Mirror { .. } => CadTransform::Mirror {
                        pivot: c.position,
                        normal: c.axis,
                    },
                    CadTransform::Scale { .. } => CadTransform::Scale {
                        pivot: c.position,
                        factor: c.factor,
                    },
                    CadTransform::ScaleAxes { .. } => CadTransform::ScaleAxes {
                        pivot: c.position,
                        factors: Vec3::new(c.size[0], c.size[1], c.size[2]),
                    },
                };
            }
            _ => return Err("This operation requires its typed feature editor".into()),
        }
        let rebuilt = g
            .cad_edit_body_recipe(body, recipe)
            .map_err(|e| e.to_string())?;
        return Ok((
            g,
            vec![body],
            true,
            format!(
                "Updated saved operation and rebuilt {} body/bodies.",
                rebuilt.len()
            ),
        ));
    }
    if c.action == 48 {
        let id = source()?;
        g.cad_check_solid(id).map_err(|e| e.to_string())?;
        return Ok((
            g,
            vec![id],
            false,
            "Exact solid topology and geometric consistency checks passed.".into(),
        ));
    }
    let result = (|| -> Result<Vec<BodyId>, String> {
        let one = match c.action {
            29 => {
                return g
                    .cad_edit_sketch_constrained(
                        c.edit_sketch.ok_or("Load a sketch for editing first")?,
                        c.position.x,
                        c.position.y,
                        if c.kind == 0 {
                            flursys::CadSketchProfile::Circle { radius: c.size[0] }
                        } else {
                            flursys::CadSketchProfile::Rectangle {
                                width: c.size[0],
                                height: c.size[1],
                            }
                        },
                        c.sketch_offset,
                        c.constraints
                            .clone()
                            .ok_or("Sketch constraints were not loaded")?,
                    )
                    .map_err(|e| e.to_string())
            }
            31 => {
                let id = c.edit_feature.ok_or("Select a Pad or Pocket feature")?;
                return if g.pocket_features().any(|f| f.id == id) {
                    g.cad_edit_pocket_to_face(
                        id,
                        c.size[2],
                        c.through_all,
                        c.stop_face,
                        c.tolerance,
                    )
                } else {
                    g.cad_edit_pad(id, c.size[2])
                }
                .map_err(|e| e.to_string());
            }
            0 => g.cad_add_primitive(
                match c.kind {
                    0 => CadPrimitive::Box {
                        length: c.size[0],
                        width: c.size[1],
                        height: c.size[2],
                    },
                    1 => CadPrimitive::Cylinder {
                        radius: c.size[0],
                        height: c.size[1],
                    },
                    2 => CadPrimitive::Sphere { radius: c.size[0] },
                    3 => CadPrimitive::Cone {
                        radius: c.size[0],
                        height: c.size[1],
                    },
                    _ => CadPrimitive::Torus {
                        major_radius: c.size[0],
                        minor_radius: c.size[1],
                    },
                },
                c.position,
                &c.name,
            ),
            38 => {
                g.cad_delete_sketch(c.edit_sketch.ok_or("Select a sketch")?)
                    .map_err(|e| e.to_string())?;
                return Ok(Vec::new());
            }
            37 => {
                let id = c.edit_sketch.ok_or("Select a sketch")?;
                let face = g
                    .sketch(id)
                    .and_then(|s| s.profile_face)
                    .ok_or("Sketch has no profile")?;
                g.cad_set_sketch_visible(id, !g.is_face_visible(face))
                    .map_err(|e| e.to_string())?;
                return Ok(Vec::new());
            }
            50..=52 => {
                let id = c.edit_sketch.ok_or("Select a dimensioned sketch")?;
                let s = g.sketch(id).ok_or("Sketch not found")?;
                let position = if c.action == 52 {
                    false
                } else if c.action == 50 {
                    s.locked_position.is_none()
                } else {
                    s.locked_position.is_some()
                };
                let size = if c.action == 52 {
                    false
                } else if c.action == 51 {
                    s.locked_profile.is_none()
                } else {
                    s.locked_profile.is_some()
                };
                if c.action == 52 {
                    g.cad_clear_sketch_constraints(id)
                } else {
                    g.cad_set_sketch_locks(id, position, size)
                }
                .map_err(|e| e.to_string())?;
                return Ok(Vec::new());
            }
            56 => g.cad_hole(
                c.face.ok_or("Select a planar solid face")?,
                c.position.x,
                c.position.y,
                c.size[0],
                c.size[2],
                c.through_all,
                c.tolerance,
            ),
            57 => g.cad_revolved_cut(
                c.face.ok_or("Select an attached sketch")?,
                c.position,
                c.axis,
                c.angle,
                c.axis_reference,
                c.axis_reversed,
                c.tolerance,
            ),
            53..=55 => g.cad_finish_prism(
                source()?,
                (c.action - 53) as u8,
                c.size[0],
                if c.action == 55 { 15 } else { c.count as u8 },
                c.kind as u8,
            ),
            49 => {
                let id = source()?;
                g.cad_retessellate(id, c.tolerance).map(|_| id)
            }
            47 => g.cad_loft_sections(&c.sections),
            46 => g.cad_tapered_extrude(
                c.face.ok_or("Select a planar sketch profile")?,
                c.size[2],
                c.factor,
            ),
            41 => g.cad_transform(
                source()?,
                CadTransform::Mirror {
                    pivot: c.position,
                    normal: c.axis,
                },
                true,
            ),
            42 => g.cad_transform(
                source()?,
                CadTransform::ScaleAxes {
                    pivot: c.position,
                    factors: Vec3::new(c.size[0], c.size[1], c.size[2]),
                },
                true,
            ),
            43 => g.cad_sweep_vector(
                c.face.ok_or("Select a planar sketch profile")?,
                Vec3::new(c.size[0], c.size[1], c.size[2]),
            ),
            44 => {
                return g
                    .cad_radial_pattern(source()?, c.count, c.position, c.axis, c.angle)
                    .map_err(|e| e.to_string())
            }
            34 => g.cad_pocket_to_face(
                c.face.ok_or("Select a sketch on the solid's planar face")?,
                c.size[2],
                c.through_all,
                c.stop_face,
                c.tolerance,
            ),
            1 | 24 => g.cad_pad(
                c.face.ok_or("Select a closed planar sketch profile")?,
                c.size[2],
                c.action == 24,
                c.tolerance,
            ),
            2 => {
                if let Some(reference) = c.axis_reference {
                    g.cad_revolve_with_reference(
                        c.face.ok_or("Select a closed planar sketch profile")?,
                        reference,
                        c.angle,
                        c.axis_reversed,
                    )
                } else {
                    g.cad_revolve(
                        c.face.ok_or("Select a closed planar sketch profile")?,
                        c.position,
                        c.axis,
                        c.angle,
                    )
                }
            }
            3 | 6 => g.cad_transform(
                source()?,
                CadTransform::Translate(c.position),
                c.action == 6 || c.copy,
            ),
            4 => g.cad_transform(
                source()?,
                CadTransform::Rotate {
                    pivot: c.position,
                    axis: c.axis,
                    degrees: c.angle,
                },
                c.copy,
            ),
            5 => g.cad_transform(
                source()?,
                CadTransform::Scale {
                    pivot: c.position,
                    factor: c.factor,
                },
                c.copy,
            ),
            7 | 8 => {
                return g
                    .cad_pattern(
                        source()?,
                        c.count,
                        c.position,
                        if c.action == 8 { c.angle } else { 0.0 },
                    )
                    .map_err(|e| e.to_string())
            }
            9..=11 => g.cad_boolean(
                source()?,
                c.target.ok_or("Select a tool body")?,
                match c.action {
                    9 => CadBoolean::Union,
                    10 => CadBoolean::Difference,
                    _ => CadBoolean::Intersection,
                },
                c.tolerance,
            ),
            12 => {
                let id = source()?;
                let visible = !g.cad_body(id).ok_or("Select a CAD body")?.visible;
                g.cad_set_visible(id, visible).map(|_| id)
            }
            13 => {
                let id = source()?;
                g.cad_rename(id, &c.name).map(|_| id)
            }
            14 => {
                g.cad_delete_feature_body(source()?)
                    .map_err(|e| e.to_string())?;
                return Ok(Vec::new());
            }
            15..=17 | 45 => {
                let id = source()?;
                let data = match c.action {
                    15 => g.cad_export_step(id),
                    16 => g.cad_export_stl(id),
                    45 => g.cad_export_obj(id),
                    _ => g.cad_export_brep(id),
                }
                .map_err(|e| e.to_string())?;
                let path = c.path.as_ref().ok_or("Choose an export file")?;
                let mut f =
                    tempfile::NamedTempFile::new_in(path.parent().unwrap_or(Path::new(".")))
                        .map_err(|e| e.to_string())?;
                use std::io::Write;
                f.write_all(data.as_bytes()).map_err(|e| e.to_string())?;
                f.persist(path).map_err(|e| e.to_string())?;
                return Ok(vec![id]);
            }
            20 => g.cad_loft(
                c.face.ok_or("Select two planar profiles")?,
                c.second_face.ok_or("Select two planar profiles")?,
            ),
            18 | 19 => {
                if std::fs::metadata(c.path.as_ref().ok_or("Choose a B-rep file")?)
                    .map_err(|e| e.to_string())?
                    .len()
                    > 64 * 1024 * 1024
                {
                    return Err("B-rep file exceeds 64 MiB".into());
                }
                let data = std::fs::read_to_string(c.path.as_ref().ok_or("Choose a B-rep file")?)
                    .map_err(|e| e.to_string())?;
                if c.action == 19 {
                    g.cad_import_step(&data, &c.name)
                } else {
                    g.cad_import_brep(&data, &c.name)
                }
            }
            _ => return Err("Unknown CAD operation".into()),
        }
        .map_err(|e| e.to_string())?;
        Ok(vec![one])
    })()?;
    let changed = !matches!(c.action, 15..=17 | 45);
    g.validate().map_err(|e| e.to_string())?;
    let message = if matches!(c.action, 29 | 31) {
        format!(
            "Dimensions applied · {} dependent bodies rebuilt · attached sketches updated",
            result.len()
        )
    } else if changed {
        format!(
            "Operation completed · {} CAD bodies",
            g.cad_bodies().count()
        )
    } else {
        format!("Exported {}", c.path.unwrap().display())
    };
    Ok((g, result, changed, message))
}
pub(super) fn poll(ui: &MainWindow, state: &mut AppState) -> bool {
    if state.sketch_edit.is_some() && super::sketch_edit::valid(state).is_none() {
        state.sketch_edit = None;
        ui.set_cad_sketch_editing(false);
        ui.set_cad_edit_sketch_id(0);
        ui.set_cad_feedback(
            "The source document changed. The uncommitted sketch draft was closed.".into(),
        );
        return true;
    }

    let Some(job) = state.cad_job.as_ref() else {
        return false;
    };
    let result = match job.result.try_recv() {
        Ok(v) => v,
        Err(TryRecvError::Empty) => return false,
        Err(_) => Err("CAD worker stopped unexpectedly".into()),
    };
    let job = state.cad_job.take().unwrap();
    ui.set_cad_busy(false);
    if job.cancelled || (job.preview && job.generation != ui.get_cad_preview_generation()) {
        ui.set_cad_feedback("CAD result discarded; source document preserved.".into());
        return true;
    }
    match result {
        Ok((geometry, selection, changed, message)) => {
            if state.workbench.geometry() != &job.source
                || state.workspace_path != job.workspace
                || state.workbench_project.project_id != job.project_id
            {
                ui.set_cad_feedback(
                    "Document changed while CAD was running; result was discarded.".into(),
                );
                return true;
            }
            if job.preview {
                ui.set_cad_task_open(true);
                ui.set_geometry_sidebar_tab(0);
                ui.set_geometry_sidebar_tab(0);
                state.cad_preview = Some(CadPreview {
                    source: job.source,
                    workspace: job.workspace,
                    project_id: job.project_id,
                    geometry,
                    selection,
                    message,
                });
                state.show_geometry_3d = true;
                state.show_mesh = false;
                ui.set_geometry_cad_view_3d(true);
                ui.set_cad_feedback(
                    "Preview ready. Edit the properties or choose OK / Cancel in the task panel."
                        .into(),
                );
                return true;
            }
            if changed {
                state.geometry_editor.record_snapshot(&job.source);
                *state.workbench.geometry_mut() = geometry;
                state.workbench.geometry_changed();
                state.cad_measure_points.clear();
                ui.set_cad_measure_active(false);
                if state.workbench.geometry().cad_bodies().next().is_some()
                    && state.workbench.mesh_dimension() == MeshDimension::TwoD
                {
                    let (global, min, max) = state.workbench.mesh_sizes();
                    let order = state.workbench.element_order();
                    if let Err(error) = state.workbench.set_mesh_configuration(
                        MeshDimension::ThreeD,
                        global,
                        min,
                        max,
                        order,
                    ) {
                        state.log(error.to_string());
                    }
                    ui.set_mesh_dimension_index(1);
                }
                state.mark_workbench_dirty();
                state.sketch_edit = None;
                let edited_profile = state
                    .workbench
                    .geometry()
                    .sketches()
                    .find(|s| s.id.get() == ui.get_cad_edit_sketch_id().max(0) as u64)
                    .and_then(|s| s.profile_face);
                ui.set_cad_sketch_editing(false);
                ui.set_cad_editing_pad(false);
                ui.set_cad_edit_sketch_id(0);
                ui.set_cad_task_open(false);
                ui.set_cad_edit_body_id("".into());
                ui.set_cad_sketch_host("".into());
                ui.set_cad_profile_face("".into());
                state.wb_selected_targets = selection
                    .into_iter()
                    .map(GeometrySelectionTarget::Body)
                    .collect();
                if state.wb_selected_targets.is_empty() {
                    if let Some(face) = edited_profile {
                        state
                            .wb_selected_targets
                            .push(GeometrySelectionTarget::Face(face));
                    }
                }
                state.geometry_editor.selection = state.wb_selected_targets.clone();
                state.selected_tree = state
                    .wb_selected_targets
                    .last()
                    .copied()
                    .and_then(tree_selection_for_target);
                state.show_geometry_3d = true;
                state.show_mesh = false;
                ui.set_geometry_cad_view_3d(true);
                rebuild_tree_rows(state);
            }
            state.log(&message);
            ui.set_cad_feedback(message.into());
        }
        Err(e) => {
            state.log(format!("CAD: {e}"));
            ui.set_cad_feedback(e.into());
        }
    }
    true
}
pub(super) fn refresh(ui: &MainWindow, state: &AppState) {
    super::cad_profiles::refresh(ui, state);
    super::cad_axis::refresh(ui, state);
    super::sketch_edit::refresh(ui, state);
    ui.set_cad_preview_active(preview(state).is_some());
    ui.set_cad_preview_label(
        preview(state)
            .and_then(|p| p.selection.first().and_then(|id| p.geometry.cad_body(*id)))
            .map(|b| b.operation.clone())
            .unwrap_or_default()
            .into(),
    );
    ui.set_cad_orientation_image(super::render::render_orientation_overlay(
        state.geometry_yaw,
        state.geometry_pitch,
    ));
    ui.set_cad_can_loft(
        state
            .wb_selected_targets
            .iter()
            .filter(|t| matches!(t, GeometrySelectionTarget::Face(_)))
            .count()
            >= 2,
    );
    let g = state.workbench.geometry();
    ui.set_cad_selection_items(ModelRc::new(VecModel::from(
        state
            .wb_selected_targets
            .iter()
            .map(|t| {
                let (label, id) = match t {
                    GeometrySelectionTarget::Body(id) => ("Body", id.get()),
                    GeometrySelectionTarget::Face(id) => ("Face", id.get()),
                    GeometrySelectionTarget::Edge(id) => ("Edge", id.get()),
                    GeometrySelectionTarget::Vertex(id) => ("Vertex", id.get()),
                };
                SharedString::from(format!("{label} #{id}"))
            })
            .collect::<Vec<_>>(),
    )));
    ui.set_cad_selection_body(state.wb_selected_targets.iter().any(|t| match t {
        GeometrySelectionTarget::Body(id) => g.cad_body(*id).is_some(),
        GeometrySelectionTarget::Face(id) => g.cad_owner(*id).is_some(),
        GeometrySelectionTarget::Edge(id) => g.cad_edge_owner(*id).is_some(),
        GeometrySelectionTarget::Vertex(id) => g.cad_vertex_owner(*id).is_some(),
    }));
    ui.set_cad_selection_edge(state.wb_selected_targets.iter().any(|t| {
        match t {
            GeometrySelectionTarget::Edge(id) => g
                .cad_reference_axis(flursys::CadAxisReference::Edge(*id))
                .is_ok(),
            _ => false,
        }
    }));
    ui.set_cad_selection_label(
        state
            .wb_selected_targets
            .first()
            .map(|t| format!("{} · {:?}", t.kind_label(), t))
            .unwrap_or_else(|| "No selection".into())
            .into(),
    );
    ui.set_cad_selection_planar(state.wb_selected_targets.iter().any(|t| match t {
        GeometrySelectionTarget::Face(id) => {
            g.cad_owner(*id).is_some() && g.sketch_frame_for_face(*id).is_ok()
        }
        _ => false,
    }));
    ui.set_cad_selection_sketch(state.wb_selected_targets.iter().any(|t| match t {
        GeometrySelectionTarget::Face(id) => g.sketches().any(|s| s.profile_face == Some(*id)),
        _ => false,
    }));
    ui.set_cad_can_pocket(selected_extrusion_face(state).ok().is_some_and(|face| {
        g.sketches().any(|s| {
            s.profile_face == Some(face) && s.host_face.and_then(|host| g.cad_owner(host)).is_some()
        })
    }));
    ui.set_cad_features(ModelRc::new(VecModel::from(
        editable_features(g)
            .into_iter()
            .map(|(_, label)| label)
            .collect::<Vec<_>>(),
    )));
    let selected_sketch = state
        .selected_tree
        .filter(|t| matches!(t.kind, TREE_KIND_SKETCH | TREE_KIND_REGION))
        .and_then(|t| g.sketches().position(|s| s.id.get() == t.payload as u64))
        .or_else(|| {
            state.wb_selected_targets.iter().find_map(|t| {
                if let GeometrySelectionTarget::Face(face) = t {
                    g.sketches().position(|s| s.profile_face == Some(*face))
                } else {
                    None
                }
            })
        });
    if let Some(index) = selected_sketch {
        ui.set_cad_sketch_index(index as i32);
    }
    if let Some(selection) = state
        .selected_tree
        .filter(|t| t.kind == TREE_KIND_EXTRUDE_FEATURE)
    {
        if let Some(index) = editable_features(g)
            .iter()
            .position(|(id, _)| id.get() == selection.payload as u64)
        {
            ui.set_cad_feature_index(index as i32);
        }
    }
    ui.set_cad_sketches(ModelRc::new(VecModel::from(
        g.sketches()
            .map(|s| {
                let profile = match &s.profile {
                    Some(flursys::CadSketchProfile::Circle { radius }) => {
                        format!("Circle · R {radius:.4}")
                    }
                    Some(flursys::CadSketchProfile::Rectangle { width, height }) => {
                        format!("Rectangle · {width:.4} × {height:.4}")
                    }
                    None => {
                        if s.profile_face.is_some() {
                            "Closed profile".into()
                        } else {
                            "Empty sketch".into()
                        }
                    }
                };
                let plane = match s.plane {
                    flursys::CadSketchPlane::Xy => "XY",
                    flursys::CadSketchPlane::Yz => "YZ",
                    flursys::CadSketchPlane::Zx => "ZX",
                    flursys::CadSketchPlane::Face => "Face",
                };
                format!(
                    "{} Sketch {} · Region {} · {} · {} · {}",
                    if s.profile_face.is_some_and(|f| g.is_face_visible(f)) {
                        "●"
                    } else {
                        "○"
                    },
                    s.region_owner.unwrap_or(s.id).get(),
                    s.id.get(),
                    profile,
                    plane,
                    g.cad_sketch_degrees_of_freedom(s.id)
                        .map_or("free profile".into(), |d| format!("{d} DOF"))
                )
                .into()
            })
            .collect::<Vec<SharedString>>(),
    )));
    ui.set_cad_history(
        g.cad_bodies()
            .map(|(id, body)| {
                let parents = body
                    .dependencies
                    .as_ref()
                    .map(|d| {
                        d.bodies
                            .iter()
                            .map(|id| format!("#{}", id.get()))
                            .collect::<Vec<_>>()
                            .join(", ")
                    })
                    .unwrap_or_default();
                let profiles = body.dependencies.as_ref().map_or(0, |d| d.profiles.len());
                format!(
                    "#{} · {}\n{}{} · {} profile(s) · {}",
                    id.get(),
                    body.name,
                    body.operation,
                    if parents.is_empty() {
                        String::new()
                    } else {
                        format!(" ← {parents}")
                    },
                    profiles,
                    if body.recipe.is_some() {
                        "rebuild recipe saved"
                    } else {
                        "no replay recipe"
                    }
                )
            })
            .collect::<Vec<_>>()
            .join("\n\n")
            .into(),
    );
    let bodies = g.cad_bodies().collect::<Vec<_>>();
    // Keep the model identity while only the selection changes. Replacing it
    // would destroy an open context menu in a history delegate.
    let body_labels = bodies
        .iter()
        .map(|(id, b)| {
            SharedString::from(format!(
                "{} {} · #{}",
                if b.visible { "●" } else { "○" },
                b.name,
                id.get()
            ))
        })
        .collect::<Vec<_>>();
    if ui.get_cad_bodies().iter().collect::<Vec<_>>() != body_labels {
        ui.set_cad_bodies(ModelRc::new(VecModel::from(body_labels)));
    }

    if let Some(selected) = state
        .wb_selected_targets
        .iter()
        .find_map(|target| match target {
            GeometrySelectionTarget::Body(id) if g.cad_body(*id).is_some() => Some(*id),
            GeometrySelectionTarget::Face(id) => g.cad_owner(*id),
            GeometrySelectionTarget::Edge(id) => g.cad_edge_owner(*id),
            GeometrySelectionTarget::Vertex(id) => g.cad_vertex_owner(*id),
            _ => None,
        })
    {
        if let Some(index) = bodies.iter().position(|(id, _)| *id == selected) {
            if !ui.get_cad_task_open() {
                ui.set_cad_source(index as i32);
            }
        }
    }
    let body = bodies.get(ui.get_cad_source().max(0) as usize);
    ui.set_cad_selection_editable(body.is_some_and(|(id, _)| super::cad_history::editable(g, *id)));
    ui.set_cad_selection_source_sketch(
        body.is_some_and(|(id, _)| super::cad_history::has_source_sketch(g, *id)),
    );
    let profiles = body
        .map(|(id, _)| {
            super::cad_tree::profiles(g, *id)
                .into_iter()
                .filter_map(|face| {
                    g.sketches()
                        .find(|s| s.profile_face == Some(face) && s.profile.is_some())
                        .map(|s| {
                            SharedString::from(format!(
                                "Sketch {} · Region {} · Face {}",
                                s.region_owner.unwrap_or(s.id).get(),
                                s.id.get(),
                                face.get()
                            ))
                        })
                })
                .collect::<Vec<_>>()
        })
        .unwrap_or_default();
    if ui.get_cad_history_profile_index() < 0
        || ui.get_cad_history_profile_index() as usize >= profiles.len()
    {
        ui.set_cad_history_profile_index(0);
    }
    ui.set_cad_history_profiles(ModelRc::new(VecModel::from(profiles)));
    let parents = body
        .and_then(|(_, b)| b.dependencies.as_ref())
        .map(|d| {
            d.bodies
                .iter()
                .filter_map(|id| {
                    g.cad_body(*id)
                        .map(|b| SharedString::from(format!("#{} · {}", id.get(), b.name)))
                })
                .collect::<Vec<_>>()
        })
        .unwrap_or_default();
    if ui.get_cad_history_parent_index() < 0
        || ui.get_cad_history_parent_index() as usize >= parents.len()
    {
        ui.set_cad_history_parent_index(0);
    }
    ui.set_cad_history_parents(ModelRc::new(VecModel::from(parents)));
    ui.set_cad_inspector(
        body.map(|(id, b)| {
            let triangles = b
                .face_meshes
                .iter()
                .map(|m| m.triangles.len())
                .sum::<usize>();
            let mut area = 0.0;
            let mut volume = 0.0;
            for m in b.face_meshes.iter() {
                for t in &m.triangles {
                    area += (t[1] - t[0]).cross(t[2] - t[0]).norm() * 0.5;
                    volume += t[0].dot(t[1].cross(t[2])) / 6.0;
                }
            }
            SharedString::from(format!(
                "{} · Body {}\n{} faces · {} edges · {} vertices · {} display triangles\nArea ≈ {:.5} · Volume ≈ {:.5}\n{}",
                b.name,
                id.get(),
                b.face_meshes.len(),
                b.edges.len(),
                b.vertices.len(),
                triangles,
                area,
                volume.abs(),
                b.operation
            ))
        })
        .unwrap_or_else(|| "Create a solid or select a closed sketch profile.".into()),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use slint::platform::{
        software_renderer::{MinimalSoftwareWindow, RepaintBufferType},
        Platform, WindowAdapter,
    };
    struct SoftwarePlatform(Rc<MinimalSoftwareWindow>);
    impl Platform for SoftwarePlatform {
        fn create_window_adapter(&self) -> Result<Rc<dyn WindowAdapter>, slint::PlatformError> {
            Ok(self.0.clone())
        }
        fn duration_since_start(&self) -> Duration {
            Duration::from_secs(1)
        }
    }
    #[test]
    fn cad_ui_builds_real_brep_and_restores_it_through_undo_redo() {
        let window = MinimalSoftwareWindow::new(RepaintBufferType::NewBuffer);
        slint::platform::set_platform(Box::new(SoftwarePlatform(window.clone()))).unwrap();
        let ui = MainWindow::new().unwrap();
        window.set_size(slint::PhysicalSize::new(1440, 1000));
        let state = Rc::new(RefCell::new(AppState::new()));
        super::super::bind_callbacks(&ui, &state);
        bind(&ui, &state);
        ui.set_cad_name("CAD verification".into());
        ui.set_cad_size_x("10".into());
        ui.set_cad_size_y("8".into());
        ui.set_cad_size_z("6".into());
        ui.invoke_cad_command(0);
        assert!(state.borrow().cad_job.is_some());
        let deadline = std::time::Instant::now() + Duration::from_secs(15);
        while state.borrow().cad_job.is_some() {
            poll(&ui, &mut state.borrow_mut());
            assert!(std::time::Instant::now() < deadline, "CAD worker timeout");
            std::thread::sleep(Duration::from_millis(1));
        }
        assert_eq!(state.borrow().workbench.geometry().cad_bodies().count(), 1);
        assert_eq!(
            state.borrow().workbench.mesh_dimension(),
            MeshDimension::ThreeD
        );
        super::super::refresh_ui(&ui, &state.borrow());
        ui.show().unwrap();
        // Real software-rendered snapshots also exercise the Slint layout bindings.
        for (tab, label) in [
            (1, "solid"),
            (2, "transform"),
            (3, "boolean"),
            (4, "exchange"),
        ] {
            ui.set_cad_workbench(tab);
            window.request_redraw();
            let mut pixels = vec![slint::Rgb8Pixel::default(); 1440 * 1000];
            window.draw_if_needed(|renderer| {
                renderer.render(&mut pixels, 1440);
            });
            let mut ppm = format!("P6\n1440 1000\n255\n").into_bytes();
            for p in pixels {
                ppm.extend([p.r, p.g, p.b]);
            }
            std::fs::write(format!("/tmp/flursys-cad-{label}.ppm"), ppm).unwrap();
        }
        // A concurrent document edit must never be overwritten by a stale worker.
        let id = state
            .borrow()
            .workbench
            .geometry()
            .cad_bodies()
            .next()
            .unwrap()
            .0;
        ui.invoke_cad_command(0);
        state
            .borrow_mut()
            .workbench
            .geometry_mut()
            .cad_rename(id, "Concurrent edit")
            .unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(15);
        while state.borrow().cad_job.is_some() {
            poll(&ui, &mut state.borrow_mut());
            assert!(std::time::Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        assert_eq!(state.borrow().workbench.geometry().cad_bodies().count(), 1);
        assert_eq!(
            state
                .borrow()
                .workbench
                .geometry()
                .cad_body(id)
                .unwrap()
                .name,
            "Concurrent edit"
        );
        ui.invoke_cad_command(0);
        ui.invoke_cad_command(-1);
        assert!(state.borrow().cad_job.as_ref().unwrap().cancelled);
        let deadline = std::time::Instant::now() + Duration::from_secs(15);
        while state.borrow().cad_job.is_some() {
            poll(&ui, &mut state.borrow_mut());
            assert!(std::time::Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        assert_eq!(state.borrow().workbench.geometry().cad_bodies().count(), 1);
        ui.invoke_geometry_undo();
        assert_eq!(state.borrow().workbench.geometry().cad_bodies().count(), 0);
        ui.invoke_geometry_redo();
        assert_eq!(state.borrow().workbench.geometry().cad_bodies().count(), 1);
        ui.hide().unwrap();
    }
}

fn sketch_command(ui: &MainWindow, state: &mut AppState, action: i32) -> Result<String, String> {
    let length = |value, label| super::cad_units::length(ui, value, label);
    if action == 92 {
        return super::cad_regions::begin(ui, state);
    }
    if action == 21 && ui.get_cad_operation() == 214 {
        return super::cad_regions::create(ui, state);
    }
    if action == 77 {
        let g = state.workbench.geometry();
        let body = g
            .cad_bodies()
            .nth(ui.get_cad_source().max(0) as usize)
            .map(|(_, b)| b)
            .ok_or("Select a history body")?;
        let parent = body
            .dependencies
            .as_ref()
            .and_then(|d| {
                d.bodies
                    .get(ui.get_cad_history_parent_index().max(0) as usize)
            })
            .copied()
            .ok_or("Select an input body")?;
        select_tree_geometry_target(state, GeometrySelectionTarget::Body(parent), false);
        return Ok("Input body selected. Its visibility is unchanged.".into());
    }
    if matches!(action, 63 | 64 | 65) {
        let g = state.workbench.geometry();
        let body = g
            .cad_bodies()
            .nth(ui.get_cad_source().max(0) as usize)
            .map(|(id, _)| id)
            .ok_or("Select a history body")?;
        if action == 65 {
            let profile=super::cad_tree::profiles(g,body).into_iter().filter(|face|g.sketches().any(|s|s.profile_face==Some(*face) && s.profile.is_some())).nth(ui.get_cad_history_profile_index().max(0) as usize).ok_or("This operation has no editable source sketch. Select its input body in history.")?;
            let index = g
                .sketches()
                .position(|s| s.profile_face == Some(profile))
                .ok_or("Source profile has no editable sketch record")?;
            ui.set_cad_sketch_index(index as i32);
            return sketch_command(ui, state, 28);
        }
        let feature = g
            .extrude_features()
            .find(|f| f.body == body)
            .map(|f| f.id)
            .or_else(|| g.pocket_features().find(|f| f.body == body).map(|f| f.id));
        let index =
            feature.and_then(|id| editable_features(g).iter().position(|(key, _)| *key == id));
        select_tree_geometry_target(state, GeometrySelectionTarget::Body(body), false);
        if action == 64 {
            if state
                .workbench
                .geometry()
                .cad_body(body)
                .is_some_and(|b| matches!(b.recipe, Some(flursys::CadRecipe::Modified { .. })))
            {
                return super::cad_history::load(ui, state, body);
            }
            let Some(index) = index else {
                return super::cad_history::load(ui, state, body);
            };
            ui.set_cad_feature_index(index as i32);
            return sketch_command(ui, state, 30);
        }
        return Ok("History body selected. Hidden predecessors remain hidden; use the visibility control to inspect them.".into());
    }
    if action == 39 {
        return super::sketch_edit::begin(ui, state);
    }
    if action == 40 {
        super::cad_properties::close(ui, state);
        return Ok("Sketch edit cancelled. Source model unchanged.".into());
    }

    if action == 28 {
        ui.set_cad_edit_body_id("".into());
        ui.set_cad_sketch_host("".into());
        state.sketch_edit = None;
        let id = state
            .workbench
            .geometry()
            .sketches()
            .nth(ui.get_cad_sketch_index().max(0) as usize)
            .map(|s| s.id)
            .ok_or("Select a sketch")?;
        super::cad_constraints::load(ui, state.workbench.geometry(), id);
        ui.set_cad_region_owner("".into());
        let plane = state.workbench.geometry().sketch(id).unwrap().plane;
        ui.set_cad_sketch_offset(super::cad_units::display_length(
            ui,
            state
                .workbench
                .geometry()
                .sketch(id)
                .unwrap()
                .attachment_offset,
        ));
        ui.set_cad_sketch_plane(match plane {
            flursys::CadSketchPlane::Xy => 0,
            flursys::CadSketchPlane::Yz => 1,
            flursys::CadSketchPlane::Zx => 2,
            flursys::CadSketchPlane::Face => 3,
        });
        ui.set_cad_editing_pad(false);
        ui.set_cad_task_open(true);
        ui.set_geometry_sidebar_tab(0);
        ui.set_cad_workbench(0);
        ui.set_cad_operation(212);
        ui.set_cad_task_title("Edit sketch dimensions".into());
        let (x, y, profile) = state
            .workbench
            .geometry()
            .cad_sketch_parameters(id)
            .map_err(|e| e.to_string())?;
        ui.set_cad_pos_x(super::cad_units::display_length(ui, x));
        ui.set_cad_pos_y(super::cad_units::display_length(ui, y));
        match profile {
            flursys::CadSketchProfile::Circle { radius } => {
                ui.set_cad_sketch_kind(0);
                ui.set_cad_size_x(super::cad_units::display_length(ui, radius));
            }
            flursys::CadSketchProfile::Rectangle { width, height } => {
                ui.set_cad_sketch_kind(1);
                ui.set_cad_size_x(super::cad_units::display_length(ui, width));
                ui.set_cad_size_y(super::cad_units::display_length(ui, height));
            }
        }
        ui.set_cad_edit_sketch_id(id.get() as i32);
        ui.set_cad_sketch_editing(true);
        return Ok(
            "Edit dimensions, then Apply & rebuild. Cancel leaves the document unchanged.".into(),
        );
    }
    if action == 30 {
        let g = state.workbench.geometry();
        let id = editable_features(g)
            .get(ui.get_cad_feature_index().max(0) as usize)
            .map(|(id, _)| *id)
            .ok_or("Select a Pad or Pocket feature")?;
        let body = if let Some(f) = g.pocket_features().find(|f| f.id == id) {
            ui.set_cad_size_z(super::cad_units::display_length(ui, f.depth));
            ui.set_cad_through_all(f.through_all);
            ui.set_cad_pocket_end_mode(if f.stop_face.is_some() {
                2
            } else if f.through_all {
                1
            } else {
                0
            });
            ui.set_cad_pocket_stop_face(
                f.stop_face
                    .map(|face| face.get().to_string())
                    .unwrap_or_default()
                    .into(),
            );
            ui.set_cad_tolerance(super::cad_units::display_length(ui, f.tolerance));
            ui.set_cad_editing_pocket(true);
            f.body
        } else {
            let f = g
                .extrude_features()
                .find(|f| f.id == id)
                .ok_or("Feature not found")?;
            ui.set_cad_size_z(super::cad_units::display_length(
                ui,
                f.distance.ok_or("Feature has no saved depth")?,
            ));
            ui.set_cad_editing_pocket(false);
            f.body
        };
        ui.set_cad_edit_body_id("".into());
        ui.set_cad_editing_pad(true);
        ui.set_cad_task_open(true);
        ui.set_geometry_sidebar_tab(0);
        ui.set_cad_workbench(0);
        ui.set_cad_operation(31);
        ui.set_cad_task_title(
            if ui.get_cad_editing_pocket() {
                "Edit Pocket"
            } else {
                "Edit Pad"
            }
            .into(),
        );
        select_tree_geometry_target(state, GeometrySelectionTarget::Body(body), false);
        return Ok("Feature loaded. Preview the changed parameters, then Apply or Cancel.".into());
    }

    if action == 23 && super::sketch_edit::valid(state).is_some() {
        return Err("Finish or cancel the sketch edit before measuring".into());
    }
    if matches!(action, 23 | 32 | 33) {
        return Ok(super::measure::command(ui, state, action));
    }
    if action == 27 {
        state.face_sketch_host = None;
        state.face_sketch_start = None;
        state.face_sketch_cursor = None;
        state.geometry_editor.cancel();
        state.show_geometry_3d = true;
        ui.set_geometry_cad_view_3d(true);
        return Ok("Sketch cancelled. Document preserved.".into());
    }

    if action == 26 {
        begin_face_sketch(state)?;
        state.geometry_editor.set_tool(GeometryTool::Circle);
        state.show_geometry_3d = false;
        ui.set_geometry_cad_view_3d(false);
        return Ok(
            "Circle sketch: click the center, then the radius point (or drag). Esc cancels.".into(),
        );
    }

    if action == 22 {
        let face = state
            .workbench
            .geometry()
            .sketches()
            .nth(ui.get_cad_sketch_index().max(0) as usize)
            .and_then(|s| s.profile_face)
            .ok_or("Select a completed sketch")?;
        ui.set_cad_profile_face(face.get().to_string().into());
        select_tree_geometry_target(state, GeometrySelectionTarget::Face(face), false);
        return Ok("Sketch selected. Set Pad depth, then apply Pad.".into());
    }
    let mut next = state.workbench.geometry().clone();
    let sketch = if ui.get_cad_sketch_plane() == 3 {
        let face = if !ui.get_cad_sketch_host().is_empty() {
            state
                .workbench
                .geometry()
                .faces()
                .find(|f| f.id.get().to_string() == ui.get_cad_sketch_host().as_str())
                .map(|f| f.id)
        } else {
            state.wb_selected_targets.iter().find_map(|s| match s {
                GeometrySelectionTarget::Face(f) => Some(*f),
                _ => None,
            })
        }
        .ok_or("Sketch support face no longer exists; select a planar solid face again")?;
        next.cad_create_offset_sketch(
            flursys::CadSketchPlane::Face,
            Some(face),
            length(ui.get_cad_sketch_offset(), "Sketch offset")?,
        )
    } else {
        next.cad_create_offset_sketch(
            match ui.get_cad_sketch_plane() {
                1 => flursys::CadSketchPlane::Yz,
                2 => flursys::CadSketchPlane::Zx,
                _ => flursys::CadSketchPlane::Xy,
            },
            None,
            length(ui.get_cad_sketch_offset(), "Sketch offset")?,
        )
    }
    .map_err(|e| e.to_string())?;
    let x = length(ui.get_cad_pos_x(), "Local X")?;
    let y = length(ui.get_cad_pos_y(), "Local Y")?;
    let width = length(ui.get_cad_size_x(), "Radius / width")?;
    let face = if ui.get_cad_sketch_kind() == 0 {
        next.materialize_sketch_circle_at(sketch.id, x, y, width)
    } else {
        next.materialize_sketch_rectangle_at(
            sketch.id,
            x,
            y,
            width,
            length(ui.get_cad_size_y(), "Height")?,
        )
    }
    .map_err(|e| e.to_string())?;
    state
        .geometry_editor
        .record_snapshot(state.workbench.geometry());
    *state.workbench.geometry_mut() = next;
    state.workbench.geometry_changed();
    state.mark_workbench_dirty();
    select_tree_geometry_target(state, GeometrySelectionTarget::Face(face), false);
    state.show_geometry_3d = true;
    state.show_mesh = false;
    ui.set_geometry_cad_view_3d(true);
    ui.set_cad_sketch_editing(false);
    ui.set_cad_task_open(false);
    ui.set_cad_edit_body_id("".into());
    ui.set_cad_sketch_host("".into());
    ui.set_cad_profile_face("".into());
    Ok(format!(
        "Sketch {} finished. Set the depth and apply Pad.",
        sketch.id.get()
    ))
}

pub(super) fn measure_selection(state: &AppState) -> Result<String, String> {
    let g = state.workbench.geometry();
    let selected = &state.wb_selected_targets;
    if let [GeometrySelectionTarget::Face(face)] = selected.as_slice() {
        if let Some(profile) = g
            .sketches()
            .find(|s| s.profile_face == Some(*face))
            .and_then(|s| s.profile.as_ref())
        {
            return Ok(match profile {
                flursys::CadSketchProfile::Circle { radius } => format!("Radius {:.6} · Diameter {:.6} · Area {:.6} · Circumference {:.6} (model units)", radius, 2.0*radius, std::f64::consts::PI*radius*radius, std::f64::consts::TAU*radius),
                flursys::CadSketchProfile::Rectangle { width, height } => format!("Width {:.6} · Height {:.6} · Area {:.6} · Perimeter {:.6} (model units)", width, height, width*height, 2.0*(width+height)),
            });
        }
    }

    if let [GeometrySelectionTarget::Edge(edge)] = selected.as_slice() {
        let edge = g.edge(*edge).ok_or("Missing edge")?;
        return match edge.geometry {
            flursys::EdgeGeometry::Kernel { id, .. } => {
                let data = g.cad_edge(id).ok_or("Missing kernel curve")?;
                Ok(format!("B-rep edge {} · Length ≈ {:.6} (model units; numerical integration of the exact curve)",id.get(),data.length))
            }
            flursys::EdgeGeometry::Line { start, end } => Ok(format!(
                "Length {:.6} (model units)",
                (g.vertex(end).ok_or("Missing vertex")?.position
                    - g.vertex(start).ok_or("Missing vertex")?.position)
                    .norm()
            )),
            flursys::EdgeGeometry::CircularArc { start, center, end } => {
                let c = g.vertex(center).ok_or("Missing center")?.position;
                let a = g.vertex(start).ok_or("Missing vertex")?.position - c;
                let b = g.vertex(end).ok_or("Missing vertex")?.position - c;
                let radius = a.norm();
                let angle = (a.dot(b) / (radius * b.norm())).clamp(-1.0, 1.0).acos();
                Ok(format!(
                    "Radius {:.6} · Diameter {:.6} · Minor arc length {:.6} (model units)",
                    radius,
                    2.0 * radius,
                    radius * angle
                ))
            }
        };
    }
    if let [GeometrySelectionTarget::Face(face)] = selected.as_slice() {
        let area = g
            .renderable_faces()
            .iter()
            .filter(|f| f.face == *face)
            .map(|f| {
                let polygon_area = |v: &[Vec3]| {
                    if v.len() < 3 {
                        0.0
                    } else {
                        (1..v.len() - 1)
                            .map(|i| (v[i] - v[0]).cross(v[i + 1] - v[0]))
                            .fold(Vec3::new(0.0, 0.0, 0.0), |sum, v| sum + v)
                            .norm()
                            * 0.5
                    }
                };
                polygon_area(&f.vertices) - f.holes.iter().map(|v| polygon_area(v)).sum::<f64>()
            })
            .sum::<f64>();
        return Ok(format!("Face area ≈ {:.6} (model units²). Pick another planar face for distance / angle; use Point distance for surface points.",area));
    }
    if let [GeometrySelectionTarget::Vertex(a), GeometrySelectionTarget::Vertex(b)] =
        selected.as_slice()
    {
        let a = g.vertex(*a).ok_or("Missing vertex")?.position;
        let b = g.vertex(*b).ok_or("Missing vertex")?.position;
        let d = b - a;
        return Ok(format!(
            "Distance {:.6} · ΔX {:.6} · ΔY {:.6} · ΔZ {:.6} (model units)",
            d.norm(),
            d.x,
            d.y,
            d.z
        ));
    }
    if let [GeometrySelectionTarget::Face(a), GeometrySelectionTarget::Face(b)] =
        selected.as_slice()
    {
        let a = g.sketch_frame_for_face(*a).map_err(|e| e.to_string())?;
        let b = g.sketch_frame_for_face(*b).map_err(|e| e.to_string())?;
        let dot = a.normal.dot(b.normal).abs().clamp(0.0, 1.0);
        let angle = dot.acos().to_degrees();
        return Ok(if dot > 1.0 - 1e-10 {
            format!(
                "Parallel-plane distance {:.6} (model units)",
                (b.origin - a.origin).dot(a.normal).abs()
            )
        } else {
            format!("Plane angle {:.6}°", angle)
        });
    }
    Err("Pick a second vertex for distance, or two planar faces for distance / angle.".into())
}
