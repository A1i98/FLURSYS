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

fn show_geometry(ui: &MainWindow, state: &mut AppState, geometry: bool) {
    if !geometry && !state.workbench.has_mesh() {
        ui.set_wb_flow_feedback("Generate a mesh before switching to Mesh view.".into());
        return;
    }
    state.show_geometry_3d = geometry;
    state.show_sketch_editor = false;
    state.show_mesh = !geometry;
    state.animation_playing = false;
}

fn group_name(state: &AppState, index: i32) -> Option<String> {
    usize::try_from(index)
        .ok()
        .and_then(|i| state.workbench.named_selections().iter().nth(i))
        .map(|g| g.name.clone())
}

fn bind_grouping(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_edit_condition(move |index| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        let Some(name) = group_name(&state, index) else {
            return;
        };
        if state.meshing || state.solving {
            return;
        }
        apply_workflow_step(&ui, &mut state, 2);
        if let Some(patch) = state.patch_names.iter().position(|p| p == &name) {
            ui.set_patch_index(patch as i32);
            push_boundary_fields(&ui, &state.workbench, &name);
            state.selected_tree = Some(TreeSelection {
                kind: TREE_KIND_PATCH,
                payload: patch as i32,
            });
            if !state.workbench.has_mesh() {
                state.wb_selected_targets = state
                    .workbench
                    .named_selections()
                    .get(&name)
                    .unwrap()
                    .targets
                    .clone();
                state.geometry_editor.selection = state.wb_selected_targets.clone();
            }
            rebuild_tree_rows(&mut state);
            refresh_ui(&ui, &state);
        }
    });

    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_task_changed(move |task| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        let geometry = task == 0 || !state.workbench.has_mesh();
        show_geometry(&ui, &mut state, geometry);
        if task == 2 {
            state.mesh_color_mode = MeshColorMode::Quality;
        }
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_workspace_view(move |geometry| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        show_geometry(&ui, &mut state, geometry);
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_entity_filter(move |mode| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if state.meshing || state.solving {
            return;
        }
        let dimension = if ui.get_mesh_dimension_index() == 0 {
            MeshDimension::TwoD
        } else {
            MeshDimension::ThreeD
        };
        if dimension != state.workbench.mesh_dimension() {
            let (a, b, c) = state.workbench.mesh_sizes();
            if let Err(error) = state
                .workbench
                .set_mesh_configuration(dimension, a, b, c, 1)
            {
                state.log(error.to_string());
                return;
            }
            state.mark_workbench_dirty();
        }
        ui.set_mesh_entity_mode(mode);
        state.wb_selected_targets.clear();
        state.geometry_editor.selection.clear();
        if dimension == MeshDimension::TwoD {
            let (yaw, pitch) = geometry_view_angles(2);
            state.geometry_yaw = yaw;
            state.geometry_pitch = pitch;
        }
        show_geometry(&ui, &mut state, true);
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_pick_geometry(move |x, y, w, h, additive| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if state.meshing || state.solving {
            return;
        }
        let Some(point) = preview_image_point(x, y, w, h) else {
            return;
        };
        let target = pick_canonical_geometry_target(
            state.workbench.geometry(),
            state.geometry_yaw,
            state.geometry_pitch,
            state.geometry_zoom,
            point,
            ui.get_mesh_entity_mode(),
        );
        if let Some(target) = target {
            select_tree_geometry_target(&mut state, target, additive);
        } else if !additive {
            state.wb_selected_targets.clear();
            state.geometry_editor.selection.clear();
            state.selected_tree = None;
        }
        rebuild_tree_rows(&mut state);
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_select_group(move |index| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if state.meshing || state.solving {
            return;
        }
        let Some(name) = group_name(&state, index) else {
            return;
        };
        let targets = state
            .workbench
            .named_selections()
            .get(&name)
            .unwrap()
            .targets
            .clone();
        ui.set_mesh_group_index(index);
        ui.set_ns_edit_name(name.clone().into());
        ui.set_wb_local_target(name.into());
        state.wb_selected_targets = targets;
        state.geometry_editor.selection = state.wb_selected_targets.clone();
        state.selected_tree = Some(TreeSelection {
            kind: TREE_KIND_NAMED_SELECTION,
            payload: index,
        });
        show_geometry(&ui, &mut state, true);
        rebuild_tree_rows(&mut state);
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_group_action(move |action| {
        let Some(ui)=weak.upgrade() else{return};let mut state=shared.borrow_mut();
        if state.meshing || state.solving {return;}
        let result=(|| -> Result<String,String> {
            if action==0 {
                state.wb_selected_targets.clear();state.geometry_editor.selection.clear();state.selected_tree=None;
                ui.set_mesh_group_index(-1);ui.set_ns_edit_name("".into());
                return Ok("Selection cleared. Shift-click boundary entities to build a new group.".into());
            }
            if action==5 {
                let targets=state.workbench.boundary_geometry_targets().map_err(|e|e.to_string())?.into_iter()
                    .filter(|target| !state.workbench.named_selections().iter().any(|g|g.targets.contains(target))).collect::<Vec<_>>();
                let count=targets.len();state.wb_selected_targets=targets;state.geometry_editor.selection=state.wb_selected_targets.clone();
                state.selected_tree=None;ui.set_mesh_group_index(-1);ui.set_ns_edit_name("".into());
                show_geometry(&ui,&mut state,true);
                return Ok(format!("Selected {count} ungrouped boundary entities. Enter a name to create a zone."));
            }
            let edited=group_name(&state,ui.get_mesh_group_index());
            let name=ui.get_ns_edit_name().trim().to_string();
            let message=match action {
                1=> {
                    let targets=state.wb_selected_targets.clone();let count=targets.len();
                    state.workbench.set_boundary_group(&name,targets,false).map_err(|e|e.to_string())?;
                    format!("Created {name:?} with {count} entities. Physical condition remains unassigned.")
                }
                2=> {
                    let old=edited.as_ref().ok_or("Select a boundary group first")?;
                    let targets=state.wb_selected_targets.clone();
                    state.workbench.set_boundary_group(old,targets,true).map_err(|e|e.to_string())?;
                    format!("Updated members of {old:?}. Regenerate the mesh.")
                }
                3=> {
                    let old=edited.as_ref().ok_or("Select a boundary group first")?;
                    state.workbench.rename_named_selection(old,&name).map_err(|e|e.to_string())?;
                    if ui.get_wb_local_target().as_str()==old {ui.set_wb_local_target(name.clone().into());}
                    format!("Renamed {old:?} to {name:?}. Regenerate the mesh.")
                }
                4=> {
                    let old=edited.as_ref().ok_or("Select a boundary group first")?;
                    state.workbench.delete_named_selection(old);
                    format!("Deleted boundary group {old:?}. Its entities are now ungrouped.")
                }
                _=>return Err("Unknown boundary group action".into()),
            };
            state.mark_workbench_dirty();state.selected_tree=None;
            ui.set_mesh_group_index(-1);ui.set_ns_edit_name("".into());
            state.wb_selected_targets.clear();state.geometry_editor.selection.clear();
            show_geometry(&ui,&mut state,true);
            Ok(message)
        })();
        let message=result.unwrap_or_else(|e|format!("Boundary group: {e}"));
        ui.set_wb_flow_feedback(message.clone().into());state.log(message);
        rebuild_tree_rows(&mut state);refresh_ui(&ui,&state);
    });
}

pub(super) fn bind(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
    bind_grouping(ui, state);
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_view_drag(move |x, y, px, py, w, h| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if state.show_geometry_3d {
            if state.workbench.mesh_dimension() == MeshDimension::ThreeD {
                if state.geometry_drag_anchor.is_none() {
                    state.geometry_drag_anchor =
                        Some((px, py, state.geometry_yaw, state.geometry_pitch));
                }
                if let Some((sx, sy, yaw, pitch)) = state.geometry_drag_anchor {
                    state.geometry_yaw = yaw + (x - sx) * 0.012;
                    state.geometry_pitch = (pitch + (y - sy) * 0.008).clamp(-1.55, 1.55);
                }
                refresh_ui(&ui, &state);
            }
            return;
        }
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
        let mut state = shared.borrow_mut();
        state.mesh_pan_anchor = None;
        state.geometry_drag_anchor = None;
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_view_zoom(move |delta| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if state.show_geometry_3d {
            state.geometry_zoom = (state.geometry_zoom * (-delta * 0.0015).exp()).clamp(0.2, 3.0);
            refresh_ui(&ui, &state);
            return;
        }
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
        if state.show_geometry_3d {
            state.geometry_zoom = 1.0;
            let axis = if state.workbench.mesh_dimension() == MeshDimension::TwoD {
                2
            } else {
                3
            };
            let (yaw, pitch) = geometry_view_angles(axis);
            state.geometry_yaw = yaw;
            state.geometry_pitch = pitch;
            state.geometry_drag_anchor = None;
            refresh_ui(&ui, &state);
            return;
        }
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
    ui.set_mesh_show_geometry(state.show_geometry_3d);
    let local_index = state
        .workbench
        .patch_names()
        .iter()
        .position(|name| name == ui.get_wb_local_target().as_str())
        .map_or(-1, |i| i as i32);
    ui.set_mesh_local_group_index(local_index);
    let rows = state
        .workbench
        .named_selections()
        .iter()
        .map(|g| {
            SharedString::from(format!(
                "{} · {} {}",
                g.name,
                g.targets.len(),
                if state.workbench.mesh_dimension() == MeshDimension::TwoD {
                    if g.targets.len() == 1 {
                        "edge"
                    } else {
                        "edges"
                    }
                } else {
                    if g.targets.len() == 1 {
                        "face"
                    } else {
                        "faces"
                    }
                }
            ))
        })
        .collect::<Vec<_>>();
    if ui.get_mesh_group_index() >= rows.len() as i32 {
        ui.set_mesh_group_index(-1);
    }
    ui.set_mesh_group_model(ModelRc::new(VecModel::from(rows)));
    ui.set_mesh_selection_summary(if state.wb_selected_targets.is_empty() {
        "No boundary entities selected.".into()
    } else {
        format!(
            "{} selected · {}",
            state.wb_selected_targets.len(),
            describe_targets(&state.wb_selected_targets)
        )
        .into()
    });
    let coverage = state
        .workbench
        .boundary_geometry_targets()
        .map(|targets| {
            let grouped = targets
                .iter()
                .filter(|t| {
                    state
                        .workbench
                        .named_selections()
                        .iter()
                        .any(|g| g.targets.contains(t))
                })
                .count();
            format!(
                "Boundary coverage: {grouped}/{} entities · {} ungrouped",
                targets.len(),
                targets.len() - grouped
            )
        })
        .unwrap_or_else(|e| e.to_string());
    ui.set_mesh_coverage(coverage.into());
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
        "Create named boundary groups, then choose Sizing.".into()
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
    fn manual_mesh_workspace_selects_groups_and_renders_without_case_defaults() {
        let window = MinimalSoftwareWindow::new(RepaintBufferType::NewBuffer);
        slint::platform::set_platform(Box::new(SoftwarePlatform(window.clone()))).unwrap();
        let ui = MainWindow::new().unwrap();
        let state = Rc::new(RefCell::new(AppState::new()));
        let r = state
            .borrow_mut()
            .workbench
            .add_rectangle(1.0, 1.0)
            .unwrap();
        super::super::bind_callbacks(&ui, &state);
        super::super::flow_setup::bind(&ui, &state);
        apply_workflow_step(&ui, &mut state.borrow_mut(), 1);
        assert!(state.borrow().show_geometry_3d);
        assert!(!state.borrow().show_sketch_editor);
        assert!(state.borrow().workbench.patch_names().is_empty());
        let clicks = {
            let s = state.borrow();
            render::project_measurement(
                s.workbench.geometry(),
                s.geometry_yaw,
                s.geometry_pitch,
                s.geometry_zoom,
                &[
                    Vec3::new(0.5, 1.0, 0.0),
                    Vec3::new(0.0, 0.5, 0.0),
                    Vec3::new(0.5, 0.0, 0.0),
                    Vec3::new(1.0, 0.5, 0.0),
                ],
            )
        };
        let pick = |i: usize, add| {
            let (x, y) = clicks[i];
            ui.invoke_mesh_pick_geometry(
                x as f32,
                y as f32,
                PREVIEW_WIDTH as f32,
                PREVIEW_HEIGHT as f32,
                add,
            );
        };
        pick(0, false);
        assert_eq!(
            state.borrow().wb_selected_targets,
            vec![GeometrySelectionTarget::Edge(r.top)]
        );
        ui.set_ns_edit_name("moving_top".into());
        ui.invoke_mesh_group_action(1);
        assert!(state
            .borrow()
            .workbench
            .boundary_assignment("moving_top")
            .is_none());
        pick(1, false);
        pick(2, true);
        pick(3, true);
        assert_eq!(state.borrow().wb_selected_targets.len(), 3);
        pick(3, true);
        assert_eq!(state.borrow().wb_selected_targets.len(), 2);
        pick(3, true);
        ui.set_ns_edit_name("fixed_walls".into());
        ui.invoke_mesh_group_action(1);
        assert!(state
            .borrow()
            .workbench
            .to_project("manual")
            .boundaries
            .is_empty());
        ui.invoke_mesh_select_group(1);
        assert_eq!(state.borrow().wb_selected_targets.len(), 3);
        ui.set_ns_edit_name("stationary".into());
        ui.invoke_mesh_group_action(3);
        ui.invoke_mesh_group_action(5);
        assert!(state.borrow().wb_selected_targets.is_empty());
        ui.set_wb_global_size("0.1".into());
        ui.set_wb_min_size("0.05".into());
        ui.set_wb_max_size("0.1".into());
        ui.invoke_check_mesh_inputs();
        assert!(!ui.get_wb_flow_feedback().starts_with("Mesh blocked"));
        ui.invoke_mesh_select_group(1);
        assert_eq!(ui.get_wb_local_target().as_str(), "stationary");
        ui.show().unwrap();
        for (w, h) in [(1440u32, 900u32), (1100, 650)] {
            window.set_size(slint::PhysicalSize::new(w, h));
            for task in 0..3 {
                ui.set_mesh_task(task);
                window.request_redraw();
                let mut pixels = vec![slint::Rgb8Pixel::default(); (w * h) as usize];
                window.draw_if_needed(|renderer| {
                    renderer.render(&mut pixels, w as usize);
                });
                let mut ppm = format!("P6\n{w} {h}\n255\n").into_bytes();
                for p in pixels {
                    ppm.extend([p.r, p.g, p.b]);
                }
                std::fs::write(format!("/tmp/flursys-mesh-{w}-{task}.ppm"), ppm).unwrap();
            }
        }
        for (w, h) in [(1440u32, 900u32), (1100, 650)] {
            window.set_size(slint::PhysicalSize::new(w, h));
            for step in [2, 3] {
                apply_workflow_step(&ui, &mut state.borrow_mut(), step);
                window.request_redraw();
                let mut pixels = vec![slint::Rgb8Pixel::default(); (w * h) as usize];
                window.draw_if_needed(|renderer| {
                    renderer.render(&mut pixels, w as usize);
                });
                let mut ppm = format!("P6\n{w} {h}\n255\n").into_bytes();
                for p in pixels {
                    ppm.extend([p.r, p.g, p.b]);
                }
                std::fs::write(format!("/tmp/flursys-workflow-{w}-{step}.ppm"), ppm).unwrap();
            }
        }
        ui.invoke_mesh_edit_condition(0);
        assert_eq!(state.borrow().current_step, 2);
        assert_eq!(ui.get_patch_index(), 0);
        assert_eq!(ui.get_bc_kind_index(), -1);
        assert!(super::super::flow_setup::boundary_condition(&ui).is_err());
        ui.set_bc_kind_index(0);
        ui.invoke_apply_boundary_wb();
        assert_eq!(
            state
                .borrow()
                .workbench
                .to_project("manual")
                .boundaries
                .len(),
            1
        );
        assert_eq!(
            state.borrow().workbench.named_selections().iter().count(),
            2
        );
    }
}
