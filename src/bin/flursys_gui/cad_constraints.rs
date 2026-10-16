//! Property bindings for the reduced analytic rectangle/circle solver.
use super::*;
use flursys::{CadSketchConstraints, CadSketchId};
pub(super) fn reset(ui: &MainWindow) {
    ui.set_cad_drive_x(false);
    ui.set_cad_drive_y(false);
    ui.set_cad_drive_width(false);
    ui.set_cad_drive_height(false);
    ui.set_cad_equal_sides(false);
    ui.set_cad_centered(false);
}
pub(super) fn show(ui: &MainWindow, c: &CadSketchConstraints) {
    ui.set_cad_drive_x(c.x.is_some());
    ui.set_cad_drive_y(c.y.is_some());
    ui.set_cad_drive_width(c.size_x.is_some());
    ui.set_cad_drive_height(c.size_y.is_some());
    ui.set_cad_equal_sides(c.equal_sides);
    ui.set_cad_centered(c.centered);
}
pub(super) fn load(ui: &MainWindow, g: &flursys::GeometryTopology, id: CadSketchId) {
    if let Some(s) = g.sketch(id) {
        show(ui, &s.constraints);
        let dof = g.cad_sketch_degrees_of_freedom(id).unwrap_or(0);
        ui.set_cad_constraint_status(format!("{dof} DOF · saved constraints").into());
        ui.set_cad_region_grouped(g.cad_sketch_region_count(id) > 1);
    }
}
pub(super) fn read(ui: &MainWindow, x: f64, y: f64, w: f64, h: f64) -> CadSketchConstraints {
    CadSketchConstraints {
        x: ui.get_cad_drive_x().then_some(x),
        y: ui.get_cad_drive_y().then_some(y),
        size_x: ui.get_cad_drive_width().then_some(w),
        size_y: (ui.get_cad_sketch_kind() == 1 && ui.get_cad_drive_height()).then_some(h),
        equal_sides: ui.get_cad_sketch_kind() == 1 && ui.get_cad_equal_sides(),
        centered: ui.get_cad_centered(),
    }
}

pub(super) fn resolve_fields(ui: &MainWindow, state: &AppState) {
    if !ui.get_cad_sketch_editing()
        || ui.get_cad_edit_sketch_id() <= 0
        || state.sketch_edit.is_some()
    {
        return;
    }
    let g = state.workbench.geometry();
    let Some(s) = g
        .sketches()
        .find(|s| s.id.get() == ui.get_cad_edit_sketch_id() as u64)
    else {
        return;
    };
    let result = (|| -> Result<_, String> {
        let parse = |v, label| super::cad_units::length(ui, v, label);
        let x = parse(ui.get_cad_pos_x(), "X")?;
        let y = parse(ui.get_cad_pos_y(), "Y")?;
        let w = parse(ui.get_cad_size_x(), "Width/radius")?;
        let h = if ui.get_cad_sketch_kind() == 1 {
            parse(ui.get_cad_size_y(), "Height")?
        } else {
            0.0
        };
        let c = read(ui, x, y, w, h);
        let profile = if ui.get_cad_sketch_kind() == 0 {
            flursys::CadSketchProfile::Circle { radius: w }
        } else {
            flursys::CadSketchProfile::Rectangle {
                width: w,
                height: h,
            }
        };
        let resolved = g
            .cad_resolve_sketch_parameters_using(s.id, x, y, profile, Some(&c))
            .map_err(|e| e.to_string())?;
        Ok((
            resolved,
            g.cad_sketch_degrees_of_freedom_using(s.id, Some(&c))
                .unwrap_or(0),
        ))
    })();
    match result {
        Ok(((x, y, profile), dof)) => {
            ui.set_cad_pos_x(super::cad_units::display_length(ui, x));
            ui.set_cad_pos_y(super::cad_units::display_length(ui, y));
            match profile {
                flursys::CadSketchProfile::Circle { radius } => {
                    ui.set_cad_size_x(super::cad_units::display_length(ui, radius))
                }
                flursys::CadSketchProfile::Rectangle { width, height } => {
                    ui.set_cad_size_x(super::cad_units::display_length(ui, width));
                    ui.set_cad_size_y(super::cad_units::display_length(ui, height));
                }
            }
            ui.set_cad_constraint_status(
                format!(
                    "{} DOF · {}",
                    dof,
                    if dof == 0 {
                        "fully constrained"
                    } else {
                        "under constrained"
                    }
                )
                .into(),
            );
        }
        Err(e) => ui.set_cad_constraint_status(
            format!("Constraint conflict / incomplete input: {e}").into(),
        ),
    }
}
