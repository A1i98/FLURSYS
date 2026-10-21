//! Editable physical-time boundary tables, independent of outer iterations.
use super::*;
use flursys::unstructured_incompressible::{BoundaryProfile, BoundaryQuantity};
pub(super) fn push(ui: &MainWindow, s: &WorkbenchSession, name: &str) {
    if let Some(p) = s.solver_options().boundary_profiles.get(name) {
        ui.set_time_bc_mode(match p.quantity {
            BoundaryQuantity::Velocity => 1,
            BoundaryQuantity::FlowRate => 2,
            BoundaryQuantity::Pressure => 3,
        });
        ui.set_time_bc_table(p.csv().into());
        ui.set_time_bc_status(
            format!(
                "Saved: {} samples; linear interpolation, endpoint hold",
                p.samples.len()
            )
            .into(),
        );
    } else {
        ui.set_time_bc_mode(0);
        ui.set_time_bc_table("".into());
        ui.set_time_bc_status("Constant physical boundary; no time table assigned.".into());
    }
}
pub(super) fn bind(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_apply_time_boundary(move || {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if state.meshing || state.solving {
            return;
        }
        let result = (|| -> Result<(), String> {
            let name = state
                .patch_names
                .get(ui.get_patch_index().max(0) as usize)
                .ok_or("Select a patch")?
                .clone();
            let quantity = match ui.get_time_bc_mode() {
                1 => Some(BoundaryQuantity::Velocity),
                2 => Some(BoundaryQuantity::FlowRate),
                3 => Some(BoundaryQuantity::Pressure),
                _ => None,
            };
            let profile = quantity
                .map(|q| BoundaryProfile::from_csv(q, ui.get_time_bc_table().as_str()))
                .transpose()?;
            state
                .workbench
                .set_boundary_profile(&name, profile)
                .map_err(|e| e.to_string())?;
            state.mark_workbench_dirty();
            push(&ui, &state.workbench, &name);
            state.log(format!("Physical-time boundary table updated: {name}"));
            Ok(())
        })();
        if let Err(e) = result {
            ui.set_time_bc_status(e.clone().into());
            state.log(e);
        }
        refresh_ui(&ui, &state);
    });
}
