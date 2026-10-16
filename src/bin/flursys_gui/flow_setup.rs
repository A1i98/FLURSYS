//! UI bindings for the hand-authored planar fluid-domain workflow.
use super::*;

fn number(value: &str, label: &str) -> Result<f64, String> {
    value
        .trim()
        .parse::<f64>()
        .ok()
        .filter(|v| v.is_finite())
        .ok_or_else(|| format!("{label} must be a finite number"))
}

pub(super) fn boundary_condition(
    ui: &MainWindow,
) -> Result<IncompressibleBoundaryCondition, String> {
    let velocity = || {
        Ok::<_, String>(Vec3::new(
            number(ui.get_bc_value_x().as_str(), "Velocity X")?,
            number(ui.get_bc_value_y().as_str(), "Velocity Y")?,
            number(ui.get_bc_value_z().as_str(), "Velocity Z")?,
        ))
    };
    match ui.get_bc_kind_index() {
        0 => Ok(IncompressibleBoundaryCondition::NoSlipWall),
        1 => Ok(IncompressibleBoundaryCondition::MovingWall {
            velocity: velocity()?,
        }),
        2 => Ok(IncompressibleBoundaryCondition::VelocityInlet {
            velocity: velocity()?,
        }),
        3 => Ok(IncompressibleBoundaryCondition::PressureOutlet {
            pressure: number(ui.get_bc_pressure().as_str(), "Outlet pressure")?,
        }),
        _ => Err("Select a supported boundary condition".into()),
    }
}

fn selected_face(state: &AppState) -> Result<flursys::FaceId, String> {
    match state.wb_selected_targets.as_slice() {
        [GeometrySelectionTarget::Face(face)] => Ok(*face),
        [] => state
            .workbench
            .fluid_face()
            .ok_or_else(|| "Select exactly one fluid face (the rectangle minus its hole)".into()),
        _ => Err("Select exactly one planar fluid face".into()),
    }
}

pub(super) fn bind(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_cut_flow_obstacle(move || {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        let result = (|| {
            if state.meshing || state.solving || state.cad_job.is_some() || state.cad_preview.is_some() {
                return Err("Finish the current CAD, mesh or solve operation first".into());
            }
            let face = selected_face(&state)?;
            let center = Vec3::new(number(ui.get_wb_obstacle_x().as_str(), "Center X")?, number(ui.get_wb_obstacle_y().as_str(), "Center Y")?, 0.0);
            let radius = number(ui.get_wb_obstacle_radius().as_str(), "Obstacle radius")?;
            let before = state.workbench.geometry().clone();
            state.workbench.geometry_mut().add_circle_hole(face, center, radius).map_err(|e| e.to_string())?;
            state.geometry_editor.record_snapshot(&before);
            state.workbench.geometry_changed();
            state.workbench.select_fluid_face(face).map_err(|e| e.to_string())?;
            state.wb_selected_targets = vec![GeometrySelectionTarget::Face(face)];
            state.geometry_editor.selection = state.wb_selected_targets.clone();
            state.mark_workbench_dirty();
            rebuild_tree_rows(&mut state);
            Ok(format!("Circular obstacle subtracted (radius {radius}). Prepare or update its boundary groups, then remesh. Undo restores the original face."))
        })();
        let message = result.unwrap_or_else(|e: String| e);
        ui.set_wb_flow_feedback(message.clone().into());
        state.log(message);
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_prepare_flow_domain(move |channel| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        let result = (|| {
            if state.meshing || state.solving || state.cad_job.is_some() || state.cad_preview.is_some() {
                return Err("Finish the current CAD preview, mesh or solve first".into());
            }
            let face = selected_face(&state)?;
            if channel { state.workbench.prepare_channel_boundaries(face) }
            else { state.workbench.select_fluid_face(face) }.map_err(|e| e.to_string())?;
            state.mark_workbench_dirty();
            rebuild_tree_rows(&mut state);
            ui.set_patch_index(0);
            if let Some(patch) = state.patch_names.first() { push_boundary_fields(&ui, &state.workbench, patch); }
            ui.set_mesh_dimension_index(0);
            ui.set_inspector_mode(2);
            ui.set_current_step(1);
            state.current_step = 1;
            Ok(if channel { "Channel patches prepared. Defaults: inlet U=(1,0,0), outlet p=0, no-slip walls and obstacle. Edit each condition in Setup, then generate the mesh.".to_string() }
            else { format!("Face {} selected as the 2D fluid domain. Group every outer and obstacle boundary before meshing.", face.get()) })
        })();
        let message = result.unwrap_or_else(|e: String| e);
        ui.set_wb_flow_feedback(SharedString::from(&message));
        state.log(message);
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_select_patch_wb(move |index| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if let Some(name) = state.workbench.patch_names().get(index.max(0) as usize) {
            push_boundary_fields(&ui, &state.workbench, name);
            state.selected_tree = Some(TreeSelection {
                kind: TREE_KIND_PATCH,
                payload: index,
            });
        }
        refresh_ui(&ui, &state);
    });
}

pub(super) fn refresh(ui: &MainWindow, state: &AppState) {
    let message = match state.workbench.fluid_face().and_then(|id| state.workbench.geometry().face(id)) {
        Some(face) => match &face.representation {
            flursys::GeometryFaceRepresentation::Planar { inner_loops, .. } => format!("2D fluid face {} · {} obstacle loop(s)", face.id.get(), inner_loops.len()),
            _ => "Select a planar fluid face".into(),
        },
        None => "Select one planar fluid face explicitly; multiple profiles are not chosen automatically".into(),
    };
    ui.set_wb_domain_status(message.into());
}
