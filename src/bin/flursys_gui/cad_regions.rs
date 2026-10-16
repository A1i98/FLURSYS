//! Add a closed region on an existing sketch's shared support.
use super::*;
pub(super) fn begin(ui: &MainWindow, state: &mut AppState) -> Result<String, String> {
    let g = state.workbench.geometry();
    let s = g
        .sketches()
        .nth(ui.get_cad_sketch_index().max(0) as usize)
        .ok_or("Select a sketch or region first")?;
    let owner = g.cad_sketch_owner(s.id).map_err(|e| e.to_string())?;
    let (x, y, profile) = g.cad_sketch_parameters(s.id).map_err(|e| e.to_string())?;
    super::cad_constraints::reset(ui);
    ui.set_cad_region_owner(owner.get().to_string().into());
    ui.set_cad_region_grouped(true);
    ui.set_cad_operation(214);
    ui.set_cad_workbench(0);
    ui.set_cad_task_open(true);
    ui.set_cad_task_title(format!("Add region · Sketch {}", owner.get()).into());
    ui.set_cad_sketch_editing(true);
    ui.set_cad_editing_pad(false);
    ui.set_cad_edit_body_id("".into());
    ui.set_cad_edit_sketch_id(0);
    ui.set_cad_pick_role(0);
    ui.set_cad_sketch_plane(match s.plane {
        flursys::CadSketchPlane::Xy => 0,
        flursys::CadSketchPlane::Yz => 1,
        flursys::CadSketchPlane::Zx => 2,
        flursys::CadSketchPlane::Face => 3,
    });
    ui.set_cad_sketch_offset(super::cad_units::display_length(ui, s.attachment_offset));
    let step = match profile {
        flursys::CadSketchProfile::Rectangle { width, height } => {
            ui.set_cad_sketch_kind(1);
            ui.set_cad_size_x(super::cad_units::display_length(ui, width));
            ui.set_cad_size_y(super::cad_units::display_length(ui, height));
            width * 1.25
        }
        flursys::CadSketchProfile::Circle { radius } => {
            ui.set_cad_sketch_kind(0);
            ui.set_cad_size_x(super::cad_units::display_length(ui, radius));
            radius * 2.5
        }
    };
    ui.set_cad_pos_x(super::cad_units::display_length(ui, x + step));
    ui.set_cad_pos_y(super::cad_units::display_length(ui, y));
    ui.set_geometry_sidebar_tab(0);
    super::cad_properties::invalidate(ui, state);
    Ok("Set the new region's shape and local position. It shares this sketch's plane; each region can drive a separate feature.".into())
}
pub(super) fn create(ui: &MainWindow, state: &mut AppState) -> Result<String, String> {
    let mut next = state.workbench.geometry().clone();
    let owner = next
        .sketches()
        .find(|s| s.id.get().to_string() == ui.get_cad_region_owner().as_str())
        .map(|s| s.id)
        .ok_or("The owning sketch no longer exists")?;
    let length = |v, label| super::cad_units::length(ui, v, label);
    let x = length(ui.get_cad_pos_x(), "X")?;
    let y = length(ui.get_cad_pos_y(), "Y")?;
    let w = length(ui.get_cad_size_x(), "Width / radius")?;
    let profile = if ui.get_cad_sketch_kind() == 0 {
        flursys::CadSketchProfile::Circle { radius: w }
    } else {
        flursys::CadSketchProfile::Rectangle {
            width: w,
            height: length(ui.get_cad_size_y(), "Height")?,
        }
    };
    let (region, face) = next
        .cad_add_sketch_region(owner, x, y, profile)
        .map_err(|e| e.to_string())?;
    state
        .geometry_editor
        .record_snapshot(state.workbench.geometry());
    *state.workbench.geometry_mut() = next;
    state.workbench.geometry_changed();
    state.mark_workbench_dirty();
    super::cad_properties::close(ui, state);
    select_tree_geometry_target(state, GeometrySelectionTarget::Face(face), false);
    state.show_geometry_3d = true;
    ui.set_geometry_cad_view_3d(true);
    Ok(format!(
        "Region {} added to Sketch {}. Select the region for Pad or Revolve.",
        region.id.get(),
        owner.get()
    ))
}
