//! Explicit feature profile selection independent of the axis/body selection.
use super::*;
use flursys::{FaceId, GeometryTopology};
use slint::Model;
pub(super) fn choices(g: &GeometryTopology) -> Vec<FaceId> {
    g.faces()
        .filter(|f| g.cad_validate_profile(f.id).is_ok())
        .map(|f| f.id)
        .collect()
}
fn assign(ui: &MainWindow, state: &mut AppState, face: FaceId) -> Result<String, String> {
    let g = state.workbench.geometry();
    g.cad_validate_profile(face).map_err(|e| e.to_string())?;
    ui.set_cad_profile_face(face.get().to_string().into());
    if let Some(index) = g.sketches().position(|s| s.profile_face == Some(face)) {
        ui.set_cad_sketch_index(index as i32);
    }
    if let Some(index) = choices(g).iter().position(|id| *id == face) {
        ui.set_cad_profile_index(index as i32);
    }
    ui.set_cad_pick_role(0);
    select_tree_geometry_target(state, GeometrySelectionTarget::Face(face), false);
    super::cad_properties::invalidate(ui, state);
    Ok(format!(
        "Closed profile Face #{} assigned. Choose the revolution axis separately.",
        face.get()
    ))
}
fn assign_body(ui: &MainWindow, state: &mut AppState, tool: bool) -> Result<String, String> {
    let g = state.workbench.geometry();
    let body = state
        .wb_selected_targets
        .iter()
        .find_map(|t| match t {
            GeometrySelectionTarget::Body(id) => Some(*id),
            GeometrySelectionTarget::Face(id) => g.cad_owner(*id),
            GeometrySelectionTarget::Edge(id) => g.cad_edge_owner(*id),
            GeometrySelectionTarget::Vertex(id) => g.cad_vertex_owner(*id),
        })
        .ok_or("Select a solid body or one of its faces")?;
    let index = g
        .cad_bodies()
        .position(|(id, _)| id == body)
        .ok_or("Selected body no longer exists")?;
    if tool && index as i32 == ui.get_cad_source() {
        return Err("Boolean tool must be different from the source body".into());
    }
    if tool {
        ui.set_cad_target(index as i32);
    } else {
        ui.set_cad_source(index as i32);
    }
    ui.set_cad_pick_role(0);
    super::cad_properties::invalidate(ui, state);
    Ok(format!(
        "{} assigned to body #{}",
        if tool { "Tool" } else { "Source" },
        body.get()
    ))
}
fn assign_pocket_stop_face(ui: &MainWindow, state: &mut AppState) -> Result<String, String> {
    let g = state.workbench.geometry();
    let face = state
        .wb_selected_targets
        .iter()
        .find_map(|target| match target {
            GeometrySelectionTarget::Face(face) => Some(*face),
            _ => None,
        })
        .ok_or("Select a planar stop face on the same solid")?;
    let profile =
        super::cad_axis::profile(ui, g).ok_or("Choose the pocket sketch profile first")?;
    let base = g
        .sketches()
        .find(|sketch| sketch.profile_face == Some(profile))
        .and_then(|sketch| sketch.host_face)
        .and_then(|host| g.cad_owner(host))
        .ok_or("Pocket sketch is not attached to a solid face")?;
    if g.cad_owner(face) != Some(base) {
        return Err("Pocket stop face must belong to the same solid as the sketch".into());
    }
    g.sketch_frame_for_face(face)
        .map_err(|error| error.to_string())?;
    ui.set_cad_pocket_stop_face(face.get().to_string().into());
    ui.set_cad_pick_role(0);
    super::cad_properties::invalidate(ui, state);
    Ok(format!("Pocket end face set to Face #{}", face.get()))
}
pub(super) fn command(
    ui: &MainWindow,
    state: &mut AppState,
    action: i32,
) -> Result<String, String> {
    if action == 98 {
        super::cad_properties::invalidate(ui, state);
        ui.set_cad_pick_role(6);
        state.geometry_pick_mode = 0;
        ui.set_geometry_pick_mode(0);
        state.geometry_editor.set_tool(GeometryTool::Select);
        state.geometry_editor.prefer_faces = true;
        return Ok("Pick a planar face on the same solid as the pocket sketch".into());
    }
    if matches!(action, 93..=95) {
        super::cad_properties::invalidate(ui, state);
        ui.set_cad_pick_role(action - 90);
        let mode = if action == 95 && ui.get_cad_operation() != 41 {
            1
        } else {
            0
        };
        state.geometry_pick_mode = mode;
        ui.set_geometry_pick_mode(mode);
        state.geometry_editor.set_tool(GeometryTool::Select);
        state.geometry_editor.prefer_faces = mode == 0;
        return Ok(match action {
            93 => "Pick the solid body to modify",
            94 => "Pick the second Boolean tool body",
            _ => "Pick a straight axis edge or a planar mirror face",
        }
        .into());
    }
    if matches!(action, 96 | 97) {
        return assign_body(ui, state, action == 97);
    }
    if action == 90 {
        ui.set_cad_pick_role(0);
        state.geometry_pick_mode = 0;
        ui.set_geometry_pick_mode(0);
        state.geometry_editor.prefer_faces = true;
        super::cad_properties::invalidate(ui, state);
        return Ok("Reference picking closed. Existing assignments are retained.".into());
    }
    if action == 91 {
        ui.set_cad_profile_face("".into());
        ui.set_cad_pick_role(0);
        super::cad_properties::invalidate(ui, state);
        return Ok("Profile cleared. Choose a closed sketch region.".into());
    }
    if action == 87 {
        super::cad_properties::invalidate(ui, state);
        ui.set_cad_pick_role(1);
        state.geometry_pick_mode = 0;
        ui.set_geometry_pick_mode(0);
        state.geometry_editor.set_tool(GeometryTool::Select);
        state.geometry_editor.prefer_faces = true;
        return Ok("Pick the shaded interior of a closed sketch region in the viewport, or select its face in Project and capture it.".into());
    }
    let g = state.workbench.geometry();
    let face = if action == 88 {
        choices(g)
            .get(ui.get_cad_profile_index().max(0) as usize)
            .copied()
    } else {
        state.wb_selected_targets.iter().find_map(|t| match t {
            GeometrySelectionTarget::Face(id) => Some(*id),
            _ => None,
        })
    }
    .ok_or("Select a closed sketch face, not a single boundary line")?;
    assign(ui, state, face)
}
pub(super) fn consume(ui: &MainWindow, state: &mut AppState) {
    let result = match ui.get_cad_pick_role() {
        1 => {
            let face = state.wb_selected_targets.iter().find_map(|t| match t {
                GeometrySelectionTarget::Face(id) => Some(*id),
                _ => None,
            });
            face.ok_or_else(|| {
                "Click inside a closed region to select the feature profile".to_string()
            })
            .and_then(|f| assign(ui, state, f))
        }
        2 => super::cad_axis::command(ui, state, 79),
        3 => assign_body(ui, state, false),
        4 => assign_body(ui, state, true),
        5 => super::cad_axis::command(ui, state, 81).map(|message| {
            ui.set_cad_pick_role(0);
            super::cad_properties::invalidate(ui, state);
            message
        }),
        6 => assign_pocket_stop_face(ui, state),
        _ => return,
    };
    ui.set_cad_feedback(result.unwrap_or_else(|e| e).into());
}
pub(super) fn refresh(ui: &MainWindow, state: &AppState) {
    let g = state.workbench.geometry();
    let ids = choices(g);
    let labels = ids
        .iter()
        .map(|id| {
            SharedString::from(
                if let Some(s) = g.sketches().find(|s| s.profile_face == Some(*id)) {
                    format!(
                        "Sketch {} · Region {} · Face #{}",
                        s.region_owner.unwrap_or(s.id).get(),
                        s.id.get(),
                        id.get()
                    )
                } else {
                    format!("Closed region · Face #{}", id.get())
                },
            )
        })
        .collect::<Vec<_>>();
    if ui.get_cad_profiles().iter().collect::<Vec<_>>() != labels {
        ui.set_cad_profiles(ModelRc::new(VecModel::from(labels)));
    }
    let assigned = super::cad_axis::profile(ui, g);
    if let Some(i) = assigned.and_then(|face| ids.iter().position(|id| *id == face)) {
        ui.set_cad_profile_index(i as i32);
    }
    ui.set_cad_profile_valid(assigned.is_some_and(|id| g.cad_validate_profile(id).is_ok()));
}
