//! Workbench mesh preparation and boundary readiness presentation.
use super::*;

fn mesh_configuration(ui: &MainWindow, session: &mut WorkbenchSession) -> Result<(), String> {
    let number = |value: SharedString, label: &str| {
        value
            .trim()
            .parse::<f64>()
            .ok()
            .filter(|v| v.is_finite())
            .ok_or_else(|| format!("{label} must be a finite number"))
    };
    session
        .set_mesh_configuration(
            if ui.get_mesh_dimension_index() == 0 {
                MeshDimension::TwoD
            } else {
                MeshDimension::ThreeD
            },
            number(ui.get_wb_global_size(), "Target size")?,
            number(ui.get_wb_min_size(), "Minimum size")?,
            number(ui.get_wb_max_size(), "Maximum size")?,
            1,
        )
        .map_err(|e| e.to_string())
}

pub(super) fn bind(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_view_drag(move |x, y, px, py, w, h| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if state.workbench.mesh_dimension() != MeshDimension::TwoD || !state.workbench.has_mesh() {
            return;
        }
        let fresh = state.mesh_pan_anchor.is_none_or(|a| a.0 != px || a.1 != py);
        if fresh {
            state.mesh_pan_anchor =
                Some((px, py, state.mesh_view.center_x, state.mesh_view.center_y));
        }
        if let Some((_, _, cx, cy)) = state.mesh_pan_anchor {
            let scale = (w.max(1.0) / PREVIEW_WIDTH as f32).min(h.max(1.0) / PREVIEW_HEIGHT as f32);
            state.mesh_view.center_x =
                cx - f64::from((x - px) / scale) / state.mesh_view.pixels_per_unit;
            state.mesh_view.center_y =
                cy + f64::from((y - py) / scale) / state.mesh_view.pixels_per_unit;
        }
        state.show_mesh = true;
        state.show_geometry_3d = false;
        refresh_ui(&ui, &state);
    });
    let shared = state.clone();
    ui.on_mesh_view_end(move || {
        shared.borrow_mut().mesh_pan_anchor = None;
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_view_zoom(move |delta| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if state.workbench.mesh_dimension() != MeshDimension::TwoD || !state.workbench.has_mesh() {
            return;
        }
        let center = (
            f64::from(PREVIEW_WIDTH) * 0.5,
            f64::from(PREVIEW_HEIGHT) * 0.5,
        );
        state.mesh_view.zoom_at(center, f64::from(delta));
        state.mesh_pan_anchor = None;
        state.show_mesh = true;
        state.show_geometry_3d = false;
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_view_fit(move || {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if let Some(cache) = state.workbench.mesh_render_cache() {
            let (lo, hi) = cache.bounds();
            state.mesh_view.fit(Some((lo.x, lo.y, hi.x, hi.y)));
        }
        state.mesh_pan_anchor = None;
        state.show_mesh = true;
        state.show_geometry_3d = false;
        refresh_ui(&ui, &state);
    });

    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_select_mesh_control(move |index| {
        let Some(ui) = weak.upgrade() else { return };
        let state = shared.borrow();
        let r = state.workbench.mesh_recipe();
        if let Some(id) = r
            .local_sizes
            .iter()
            .map(|c| c.id)
            .chain(r.refinements.iter().map(|c| c.id))
            .chain(r.boundary_layers.iter().map(|c| c.id))
            .nth(index.max(0) as usize)
        {
            ui.set_wb_recipe_control_id(id.to_string().into());
        }
    });

    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_check_mesh_inputs(move || {
        let Some(ui)=weak.upgrade() else{return};
        let result=(|| {
            let state=shared.borrow();
            let mut draft=WorkbenchSession::from_project(&state.workbench.to_project("Mesh preflight")).map_err(|e|e.to_string())?;
            mesh_configuration(&ui,&mut draft)?;
            draft.mesh_generation_inputs().map_err(|e|e.to_string())?;
            Ok::<_,String>("Geometry, boundary coverage, element sizes and refinement recipe are valid. Generate Mesh to run Gmsh.".to_string())
        })();
        let message=result.unwrap_or_else(|e|format!("Mesh blocked: {e}"));
        ui.set_wb_flow_feedback(message.clone().into());
        shared.borrow_mut().log(message);
        refresh_ui(&ui,&shared.borrow());
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_size_preset(move |level| {
        let Some(ui)=weak.upgrade() else{return};
        let mut state=shared.borrow_mut();
        if state.meshing || state.solving {return;}
        let mut lo=Vec3::new(f64::INFINITY,f64::INFINITY,f64::INFINITY);
        let mut hi=Vec3::new(f64::NEG_INFINITY,f64::NEG_INFINITY,f64::NEG_INFINITY);
        for v in state.workbench.geometry().vertices() {
            let p=v.position;
            lo.x=lo.x.min(p.x);lo.y=lo.y.min(p.y);lo.z=lo.z.min(p.z);
            hi.x=hi.x.max(p.x);hi.y=hi.y.max(p.y);hi.z=hi.z.max(p.z);
        }
        let extent=hi-lo;
        let length=extent.x.max(extent.y).max(extent.z);
        if !length.is_finite() || length<=0.0 {
            ui.set_wb_flow_feedback("Draw a fluid domain before choosing a mesh preset".into());return;
        }
        let size=length/match level {0=>20.0,1=>40.0,_=>80.0};
        ui.set_wb_global_size(format!("{size:.8e}").into());
        ui.set_wb_min_size(format!("{:.8e}",size*0.25).into());
        ui.set_wb_max_size(format!("{size:.8e}").into());
        let message=format!("Mesh size draft: h={size:.4e}, min=h/4, max=h. Check inputs, then generate to apply. Preset uses the model bounding extent.");
        ui.set_wb_flow_feedback(message.clone().into());state.log(message);
        refresh_ui(&ui,&state);
    });
}

pub(super) fn refresh(ui: &MainWindow, state: &AppState) {
    ui.set_wb_operation_feedback(state.logs.back().cloned().unwrap_or_default().into());
    let r = state.workbench.mesh_recipe();
    let controls = r
        .local_sizes
        .iter()
        .map(|c| SharedString::from(format!("{} · Local {} · h={:.3e}", c.id, c.name, c.size)))
        .chain(
            r.refinements
                .iter()
                .map(|c| SharedString::from(format!("{} · Distance {}", c.id, c.name))),
        )
        .chain(r.boundary_layers.iter().map(|c| {
            SharedString::from(format!(
                "{} · Layers {} · n={}",
                c.id, c.name, c.layer_count
            ))
        }))
        .collect::<Vec<_>>();
    ui.set_mesh_control_model(ModelRc::new(VecModel::from(controls)));
    let names = state.workbench.patch_names();
    let mut assigned = 0;
    let rows = names
        .iter()
        .map(|name| {
            let description = state
                .workbench
                .boundary_assignment(name)
                .map(|bc| {
                    assigned += 1;
                    match bc {
                        IncompressibleBoundaryCondition::NoSlipWall => "No-slip wall".to_string(),
                        IncompressibleBoundaryCondition::MovingWall { .. } => {
                            "Moving wall".to_string()
                        }
                        IncompressibleBoundaryCondition::VelocityInlet { .. } => {
                            "Velocity inlet".to_string()
                        }
                        IncompressibleBoundaryCondition::PressureOutlet { .. } => {
                            "Pressure outlet".to_string()
                        }
                    }
                })
                .unwrap_or_else(|| "UNASSIGNED".into());
            let count = state.workbench.mesh().and_then(|m| {
                m.mesh
                    .boundary_patches()
                    .iter()
                    .find(|p| p.name == *name)
                    .map(|p| p.face_indices.len())
            });
            SharedString::from(format!(
                "{name} · {description}{}",
                count.map(|n| format!(" · {n} faces")).unwrap_or_default()
            ))
        })
        .collect::<Vec<_>>();
    ui.set_patch_status_model(ModelRc::new(VecModel::from(rows)));
    ui.set_wb_boundary_summary(
        format!(
            "{assigned}/{} boundary groups assigned{}",
            names.len(),
            if assigned == names.len() && !names.is_empty() {
                " · complete"
            } else {
                " · assign every patch"
            }
        )
        .into(),
    );
    ui.set_wb_mesh_readiness(if state.meshing {
        "Gmsh is generating the mesh…".into()
    } else if state.workbench.has_mesh() {
        "Mesh generated. Changing sizes or refinement requires regeneration.".into()
    } else if names.is_empty() {
        "Prepare cavity/channel boundaries or create Named Selections first.".into()
    } else {
        "Boundary groups available. Check inputs before generation.".into()
    });
    let readiness = if state.workspace_path.is_none() {
        Err("Save the project workspace before running".to_string())
    } else {
        state.workbench.readiness()
    };
    ui.set_wb_run_readiness(readiness.map(|_|"Ready for the steady incompressible solver. Run also applies the material and solver fields shown above.".into()).unwrap_or_else(|reason|format!("Run blocked: {reason}")).into());
    ui.set_can_run_workbench(
        !state.meshing
            && !state.solving
            && state.workspace_path.is_some()
            && state.workbench.readiness().is_ok(),
    );
}
