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
        10 => {
            let half_width = number(
                ui.get_bc_profile_half_width().as_str(),
                "Profile half-width",
            )?;
            if half_width <= 0.0 {
                return Err("Profile half-width must be positive".into());
            }
            Ok(IncompressibleBoundaryCondition::ParabolicInlet {
                velocity: velocity()?,
                axis: ui.get_bc_profile_axis().clamp(0, 2) as usize,
                center: number(ui.get_bc_profile_center().as_str(), "Profile centre")?,
                half_width,
            })
        }
        7 | 8 => {
            let rate = number(ui.get_bc_value_x().as_str(), "Flow rate")?;
            if rate < 0.0 {
                return Err("Inlet flow rate must be nonnegative".into());
            }
            Ok(IncompressibleBoundaryCondition::FlowRateInlet {
                rate,
                mass: ui.get_bc_kind_index() == 8,
            })
        }
        9 => Ok(IncompressibleBoundaryCondition::PressureOutletBackflow {
            pressure: number(ui.get_bc_pressure().as_str(), "Outlet pressure")?,
            velocity: velocity()?,
        }),
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
        4 => Ok(IncompressibleBoundaryCondition::Slip),
        5 => Ok(IncompressibleBoundaryCondition::Symmetry),
        6 => Ok(IncompressibleBoundaryCondition::FarField {
            velocity: velocity()?,
            pressure: number(ui.get_bc_pressure().as_str(), "Far-field pressure")?,
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
    super::mesh_setup::bind(ui, state);
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_export_result_csv(move || {
        let Some(ui) = weak.upgrade() else { return };
        let dataset = shared.borrow().active_result().cloned();
        let Some(dataset) = dataset else {
            shared
                .borrow_mut()
                .log("Select a saved workbench result before exporting cell data.");
            refresh_ui(&ui, &shared.borrow());
            return;
        };
        let Some(path) = rfd::FileDialog::new()
            .set_title("Export result cell data")
            .add_filter("CSV", &["csv"])
            .set_file_name("cell-data.csv")
            .save_file()
        else {
            return;
        };
        let result = dataset.write_cell_csv(&path);
        shared.borrow_mut().log(match result {
            Ok(()) => format!(
                "Cell data exported: {} (pressure column is p/rho)",
                path.display()
            ),
            Err(e) => format!("CSV export failed: {e}"),
        });
        refresh_ui(&ui, &shared.borrow());
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_select_fluid_domain(move || {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        let result = (|| {
            if state.meshing || state.solving || state.cad_job.is_some() || state.cad_preview.is_some() {
                return Err("Finish the current CAD preview, mesh or solve first".into());
            }
            let face = selected_face(&state)?;
            state.workbench.select_fluid_face(face).map_err(|e|e.to_string())?;
            state.show_geometry_3d=true;state.show_mesh=false;state.show_sketch_editor=false;
            state.mark_workbench_dirty();
            rebuild_tree_rows(&mut state);
            ui.set_patch_index(0);
            if let Some(patch) = state.patch_names.first() { push_boundary_fields(&ui, &state.workbench, patch); }
            ui.set_mesh_dimension_index(0);
            ui.set_inspector_mode(2);
            ui.set_current_step(1);
            state.current_step = 1;
            Ok(format!("Face {} selected as the fluid domain. Select its boundary edges and create groups before meshing.",face.get()))
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
            ui.set_patch_index(index);
            push_boundary_fields(&ui, &state.workbench, name);
            state.show_mesh = state.workbench.has_mesh();
            state.show_geometry_3d = !state.show_mesh;
            if !state.show_mesh {
                state.wb_selected_targets = state
                    .workbench
                    .named_selections()
                    .get(name)
                    .map(|g| g.targets.clone())
                    .unwrap_or_default();
                state.geometry_editor.selection = state.wb_selected_targets.clone();
            }
            state.mesh_color_mode = MeshColorMode::Patches;
            state.selected_tree = Some(TreeSelection {
                kind: TREE_KIND_PATCH,
                payload: index,
            });
        }
        refresh_ui(&ui, &state);
    });
}

pub(super) fn refresh(ui: &MainWindow, state: &AppState) {
    super::mesh_setup::refresh(ui, state);
    ui.set_result_probe_text(state.active_result_probe().map(|p| format!(
        "Cell {} · center ({:.4e}, {:.4e}, {:.4e}) · p/rho {:.5e} · U=({:.5e}, {:.5e}, {:.5e}) · |U|={:.5e}",
        p.cell_index,p.center.x,p.center.y,p.center.z,p.pressure,p.velocity.x,p.velocity.y,p.velocity.z,p.velocity_magnitude
    )).unwrap_or_else(|| "Click a result cell to inspect pressure and velocity.".into()).into());

    if state.workbench.mesh_dimension() == flursys::MeshDimension::ThreeD {
        ui.set_wb_domain_status(
            "3D fluid volume · review all surface patches before solving".into(),
        );
        return;
    }
    let message = match state.workbench.fluid_face().and_then(|id| state.workbench.geometry().face(id)) {
        Some(face) => match &face.representation {
            flursys::GeometryFaceRepresentation::Planar { inner_loops, .. } => format!("2D fluid face {} · {} obstacle loop(s)", face.id.get(), inner_loops.len()),
            _ => "Select a planar fluid face".into(),
        },
        None => "Select one planar fluid face explicitly; multiple profiles are not chosen automatically".into(),
    };
    ui.set_wb_domain_status(message.into());
}
