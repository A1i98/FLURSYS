//! Reference selection is separate from the profile selection.
use super::*;
use flursys::{CadAxisReference, EdgeId, FaceId, GeometryTopology};
pub(super) fn profile(ui: &MainWindow, g: &GeometryTopology) -> Option<FaceId> {
    g.faces()
        .find(|f| f.id.get().to_string() == ui.get_cad_profile_face().as_str())
        .map(|f| f.id)
}
pub(super) fn edges(g: &GeometryTopology, face: FaceId) -> Vec<EdgeId> {
    let mut out = Vec::new();
    if let Some(f) = g.face(face) {
        if let flursys::GeometryFaceRepresentation::Planar {
            outer_loop,
            inner_loops,
        } = &f.representation
        {
            for entry in outer_loop.iter().chain(inner_loops.iter().flatten()) {
                if g.cad_reference_axis(CadAxisReference::Edge(entry.edge))
                    .is_ok()
                    && !out.contains(&entry.edge)
                {
                    out.push(entry.edge);
                }
            }
        }
    }
    out
}
pub(super) fn reference(
    ui: &MainWindow,
    g: &GeometryTopology,
    face: Option<FaceId>,
) -> Result<Option<CadAxisReference>, String> {
    Ok(match ui.get_cad_axis_mode() {
        1 => Some(CadAxisReference::ProfileX(
            face.ok_or("Select a profile first")?,
        )),
        2 => Some(CadAxisReference::ProfileY(
            face.ok_or("Select a profile first")?,
        )),
        3 => Some(CadAxisReference::Edge(
            g.edges()
                .find(|e| e.id.get().to_string() == ui.get_cad_axis_edge().as_str())
                .map(|e| e.id)
                .ok_or("Choose a straight profile edge or capture a selected line")?,
        )),
        _ => None,
    })
}
pub(super) fn command(
    ui: &MainWindow,
    state: &mut AppState,
    action: i32,
) -> Result<String, String> {
    if matches!(action, 84..=86) {
        let mut sections = state
            .wb_selected_targets
            .iter()
            .copied()
            .filter(|t| matches!(t, GeometrySelectionTarget::Face(_)))
            .collect::<Vec<_>>();
        let index = ui.get_cad_loft_index().max(0) as usize;
        if index >= sections.len() {
            return Err("Select a loft section".into());
        }
        let selected = match action {
            84 if index > 0 => {
                sections.swap(index, index - 1);
                index - 1
            }
            85 if index + 1 < sections.len() => {
                sections.swap(index, index + 1);
                index + 1
            }
            86 => {
                sections.remove(index);
                index.min(sections.len().saturating_sub(1))
            }
            _ => index,
        };
        state.wb_selected_targets = sections.clone();
        state.geometry_editor.selection = sections;
        state.cad_preview = None;
        ui.set_cad_loft_index(selected as i32);
        return Ok("Loft section order updated. Preview to rebuild.".into());
    }
    if matches!(action, 82 | 83) {
        state.cad_preview = None;
        ui.set_cad_angle(super::cad_units::display_angle(
            ui,
            if action == 82 { 360.0 } else { 180.0 },
        ));
        return Ok("Revolution angle updated. Preview to rebuild.".into());
    }
    if action == 81 {
        let g = state.workbench.geometry();
        let target = state
            .wb_selected_targets
            .first()
            .ok_or("Select a straight edge or planar face first")?;
        let (pivot, direction) = match target {
            GeometrySelectionTarget::Edge(id) => g
                .cad_reference_axis(CadAxisReference::Edge(*id))
                .map_err(|e| e.to_string())?,
            GeometrySelectionTarget::Face(id) => {
                let f = g.sketch_frame_for_face(*id).map_err(|e| e.to_string())?;
                (f.origin, f.normal)
            }
            _ => return Err("Select an edge or face reference".into()),
        };
        ui.set_cad_axis(3);
        ui.set_cad_axis_x(direction.x.to_string().into());
        ui.set_cad_axis_y(direction.y.to_string().into());
        ui.set_cad_axis_z(direction.z.to_string().into());
        ui.set_cad_pos_x(super::cad_units::display_length(ui, pivot.x));
        ui.set_cad_pos_y(super::cad_units::display_length(ui, pivot.y));
        ui.set_cad_pos_z(super::cad_units::display_length(ui, pivot.z));
        state.cad_preview = None;
        return Ok("Reference direction and origin copied to the tool. These transform coordinates are fixed values.".into());
    }
    let edge = if action == 78 {
        let g = state.workbench.geometry();
        let face = profile(ui, g).ok_or("Select a profile first")?;
        edges(g, face)
            .get(ui.get_cad_axis_edge_index().max(0) as usize)
            .copied()
            .ok_or("This profile has no straight edge")?
    } else if action == 79 {
        state
            .wb_selected_targets
            .iter()
            .find_map(|t| match t {
                GeometrySelectionTarget::Edge(id) => Some(*id),
                _ => None,
            })
            .ok_or("Select a straight edge in the viewport or Project tree, then capture it")?
    } else {
        super::cad_properties::invalidate(ui,state);
        ui.set_cad_pick_role(2);
        state.geometry_editor.set_tool(GeometryTool::Select);state.geometry_editor.prefer_faces=false;
        state.geometry_pick_mode = 1;
        ui.set_geometry_pick_mode(1);
        return Ok("Edge selection enabled. Select a line, then use Capture selected edge. The profile remains assigned.".into());
    };
    state
        .workbench
        .geometry()
        .cad_reference_axis(CadAxisReference::Edge(edge))
        .map_err(|e| e.to_string())?;
    ui.set_cad_axis_edge(edge.get().to_string().into());
    ui.set_cad_axis_mode(3);ui.set_cad_pick_role(0);
    super::cad_properties::invalidate(ui,state);
    select_tree_geometry_target(state, GeometrySelectionTarget::Edge(edge), false);
    Ok(format!(
        "Axis linked to edge {}. It follows geometry changes during rebuild.",
        edge.get()
    ))
}
pub(super) fn refresh(ui: &MainWindow, state: &AppState) {
    let g = state.workbench.geometry();
    ui.set_cad_loft_sections(ModelRc::new(VecModel::from(
        state
            .wb_selected_targets
            .iter()
            .filter_map(|t| match t {
                GeometrySelectionTarget::Face(id) => {
                    Some(SharedString::from(format!("Profile · Face #{}", id.get())))
                }
                _ => None,
            })
            .collect::<Vec<_>>(),
    )));
    let options = profile(ui, g)
        .map(|face| edges(g, face))
        .unwrap_or_default();
    let labels = options
        .iter()
        .enumerate()
        .map(|(i, id)| SharedString::from(format!("Line {} · Edge #{}", i + 1, id.get())))
        .collect::<Vec<_>>();
    ui.set_cad_axis_edges(ModelRc::new(VecModel::from(labels)));
    if let Some(i) = options
        .iter()
        .position(|id| id.get().to_string() == ui.get_cad_axis_edge().as_str())
    {
        ui.set_cad_axis_edge_index(i as i32);
    }
}
