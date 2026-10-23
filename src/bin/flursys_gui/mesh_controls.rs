//! Scoped mesh-control editing; physical boundary names remain independent.
use super::*;
use flursys::workbench::{BoxRefinement, EdgeDivisionControl, MeshRecipe};

fn number(value: SharedString, label: &str) -> Result<f64, String> {
    value
        .trim()
        .parse::<f64>()
        .ok()
        .filter(|v| v.is_finite())
        .ok_or_else(|| format!("{label} must be a finite number"))
}
fn scope(ui: &MainWindow, state: &AppState) -> Result<Vec<MeshControlTarget>, String> {
    let dimension = if ui.get_mesh_dimension_index() == 0 {
        MeshDimension::TwoD
    } else {
        MeshDimension::ThreeD
    };
    let entities = if ui.get_mesh_scope_mode() == 0 {
        state
            .workbench
            .named_selections()
            .get(ui.get_wb_local_target().trim())
            .ok_or("Choose a named boundary or use the current geometry selection")?
            .targets
            .clone()
    } else {
        state.wb_selected_targets.clone()
    };
    if entities.is_empty() {
        return Err("Select geometry first; Shift-click adds/removes entities".into());
    }
    let available = state
        .workbench
        .boundary_geometry_targets()
        .map_err(|e| e.to_string())?;
    for entity in &entities {
        let valid_kind = matches!(
            (dimension, entity),
            (MeshDimension::TwoD, GeometrySelectionTarget::Edge(_))
                | (MeshDimension::ThreeD, GeometrySelectionTarget::Face(_))
        );
        if !valid_kind || !available.contains(entity) {
            return Err("Scope must contain fluid-boundary edges in 2D or surfaces in 3D".into());
        }
    }
    if ui.get_mesh_scope_mode() == 0 {
        Ok(vec![MeshControlTarget::NamedSelection(
            ui.get_wb_local_target().trim().into(),
        )])
    } else {
        Ok(entities
            .into_iter()
            .map(MeshControlTarget::Geometry)
            .collect())
    }
}
fn id(ui: &MainWindow, recipe: &MeshRecipe) -> u32 {
    ui.get_mesh_edit_id().parse().ok().unwrap_or_else(|| {
        recipe
            .local_sizes
            .iter()
            .map(|c| c.id)
            .chain(recipe.refinements.iter().map(|c| c.id))
            .chain(recipe.boundary_layers.iter().map(|c| c.id))
            .chain(recipe.regions.iter().map(|c| c.id))
            .chain(recipe.edge_divisions.iter().map(|c| c.id))
            .max()
            .unwrap_or(0)
            + 1
    })
}
fn erase(recipe: &mut MeshRecipe, id: u32) {
    recipe.edge_divisions.retain(|c| c.id != id);
    recipe.local_sizes.retain(|c| c.id != id);
    recipe.refinements.retain(|c| c.id != id);
    recipe.boundary_layers.retain(|c| c.id != id);
    recipe.regions.retain(|c| c.id != id);
}
fn inflation(ui: &MainWindow) -> Result<BoundaryLayerControl, String> {
    let count = ui.get_wb_layer_count().max(0) as u32;
    let growth = number(ui.get_wb_layer_growth(), "Growth ratio")?;
    let input = number(ui.get_wb_layer_first(), "Inflation height")?;
    let mut control = BoundaryLayerControl {
        id: 0,
        name: "Inflation".into(),
        targets: vec![],
        first_layer_height: input,
        growth_ratio: growth,
        layer_count: count,
        enabled: true,
    };
    if ui.get_mesh_layer_mode() == 1 {
        let unit = BoundaryLayerControl {
            first_layer_height: 1.,
            ..control.clone()
        };
        let (factor, _) = unit.layer_dimensions()?;
        control.first_layer_height = input / factor;
    }
    control.layer_dimensions()?;
    Ok(control)
}
fn preview(ui: &MainWindow) {
    let text=inflation(ui).and_then(|c|c.layer_dimensions().map(|(total,last)|
        format!("First: {:.4e} m · last: {last:.4e} m\nTotal: {total:.4e} m · {} target layers\nLayers can truncate near corners or collisions.",c.first_layer_height,c.layer_count)))
        .unwrap_or_else(|e|e);
    ui.set_mesh_layer_summary(text.into());
}
fn apply(ui: &MainWindow, state: &mut AppState, kind: i32) -> Result<(), String> {
    if state.meshing || state.solving {
        return Err("Wait for the active mesh/solve job".into());
    }
    if kind == 3 && ui.get_mesh_dimension_index() != 1 {
        return Err("Volume refinement boxes require a 3D fluid mesh".into());
    }
    let targets = if kind == 3 { vec![] } else { scope(ui, state)? };
    let mut recipe = state.workbench.mesh_recipe().clone();
    if kind == 1 {
        recipe.tuning.max_size_growth = if ui.get_mesh_size_growth().trim().is_empty() {
            None
        } else {
            Some(number(
                ui.get_mesh_size_growth(),
                "Distance-field growth limit",
            )?)
        };
        recipe.tuning.validate()?;
    }
    let id = id(ui, &recipe);
    let name = ui.get_mesh_control_name().trim().to_string();
    let name = if name.is_empty() {
        format!(
            "{}-{id}",
            match kind {
                0 => "Sizing",
                1 => "Refinement",
                2 => "Inflation",
                4 => "Edge divisions",
                _ => "Region",
            }
        )
    } else {
        name
    };
    // Replacing an explicit editing ID also allows changing the control type.
    erase(&mut recipe, id);
    let minimum = number(ui.get_wb_min_size(), "Global minimum")?;
    let maximum = number(ui.get_wb_max_size(), "Global maximum")?;
    let check_size = |v: f64| -> Result<(), String> {
        if v <= 0. || v < minimum || v > maximum {
            Err(format!(
                "Size {v} lies outside global bounds [{minimum}, {maximum}]"
            ))
        } else {
            Ok(())
        }
    };
    match kind {
        4 => {
            if ui.get_mesh_dimension_index() != 0 {
                return Err("Exact edge divisions currently support analytic 2D boundaries".into());
            }
            let c = EdgeDivisionControl {
                id,
                name,
                targets,
                cells: ui.get_mesh_edge_cells().max(1) as u32,
                progression: number(ui.get_mesh_edge_growth(), "Edge growth")?,
                reverse: ui.get_mesh_edge_reverse(),
                symmetric: ui.get_mesh_edge_symmetric(),
                enabled: true,
            };
            c.validate()?;
            recipe.edge_divisions.retain(|old| old.targets != c.targets);
            recipe.edge_divisions.push(c);
        }
        0 => {
            let size = number(ui.get_wb_local_size(), "Local size")?;
            check_size(size)?;
            recipe.local_sizes.retain(|c| c.targets != targets);
            recipe.local_sizes.push(LocalMeshSize {
                id,
                name,
                targets,
                size,
                enabled: true,
            });
        }
        1 => {
            let near = number(ui.get_wb_refine_min(), "Near size")?;
            let far = number(ui.get_wb_refine_max(), "Far size")?;
            check_size(near)?;
            check_size(far)?;
            let start = number(ui.get_mesh_refine_start(), "Fine-region distance")?;
            let end = number(ui.get_wb_refine_distance(), "Transition end")?;
            if near > far || start < 0. || end <= start {
                return Err(
                    "Require near ≤ far and 0 ≤ fine-region distance < transition end".into(),
                );
            }
            recipe.refinements.retain(|c| c.targets != targets);
            recipe.refinements.push(ThresholdRefinement {
                id,
                name,
                targets,
                sampling: ui.get_mesh_refine_sampling().clamp(10, 1000) as u32,
                size_min: near,
                size_max: far,
                distance_min: start,
                distance_max: end,
                enabled: true,
            });
        }
        3 => {
            let vector = |text: SharedString| -> Result<Vec3, String> {
                let v = text
                    .split(|c: char| c == ',' || c.is_whitespace())
                    .filter(|v| !v.is_empty())
                    .map(str::parse::<f64>)
                    .collect::<Result<Vec<_>, _>>()
                    .map_err(|_| "Enter three coordinates in metres")?;
                if v.len() != 3 {
                    return Err("Enter X, Y, Z in metres".into());
                }
                Ok(Vec3::new(v[0], v[1], v[2]))
            };
            let size = number(ui.get_mesh_box_size(), "Region size")?;
            check_size(size)?;
            let control = BoxRefinement {
                id,
                name,
                minimum: vector(ui.get_mesh_box_min())?,
                maximum: vector(ui.get_mesh_box_max())?,
                size,
                transition: number(ui.get_mesh_box_transition(), "Transition thickness")?,
                enabled: true,
            };
            control.validate()?;
            recipe.regions.push(control);
        }
        _ => {
            if ui.get_mesh_dimension_index() != 0 {
                return Err("3D prismatic inflation is not supported. Use distance refinement on the selected surfaces".into());
            }
            let mut control = inflation(ui)?;
            control.id = id;
            control.name = name;
            control.targets = targets;
            recipe
                .boundary_layers
                .retain(|c| c.targets != control.targets);
            recipe.boundary_layers.push(control);
        }
    }
    let mut document = state.workbench.to_project("Mesh controls");
    document.mesh_recipe = recipe.clone();
    document.validate().map_err(|e| e.to_string())?;
    state.workbench.set_mesh_recipe(recipe);
    state.mark_workbench_dirty();
    ui.set_mesh_edit_id(id.to_string().into());
    state.log(format!(
        "Applied mesh control {id}; regenerate mesh. Physical boundary assignments are unchanged."
    ));
    Ok(())
}
fn controls(recipe: &MeshRecipe) -> Vec<(u32, String, Vec<MeshControlTarget>)> {
    recipe
        .local_sizes
        .iter()
        .map(|c| (c.id, c.name.clone(), c.targets.clone()))
        .chain(
            recipe
                .refinements
                .iter()
                .map(|c| (c.id, c.name.clone(), c.targets.clone())),
        )
        .chain(
            recipe
                .boundary_layers
                .iter()
                .map(|c| (c.id, c.name.clone(), c.targets.clone())),
        )
        .chain(
            recipe
                .regions
                .iter()
                .map(|c| (c.id, c.name.clone(), vec![])),
        )
        .chain(
            recipe
                .edge_divisions
                .iter()
                .map(|c| (c.id, c.name.clone(), c.targets.clone())),
        )
        .collect()
}
pub(super) fn edit(
    ui: &MainWindow,
    state: &mut AppState,
    index: i32,
    action: i32,
) -> Result<(), String> {
    if state.meshing || state.solving {
        return Err("Wait for the active mesh/solve job".into());
    }
    if action == 3 {
        ui.set_mesh_edit_id("".into());
        ui.set_mesh_control_name("".into());
        return Ok(());
    }
    let mut recipe = state.workbench.mesh_recipe().clone();
    let (id, name, targets) = controls(&recipe)
        .get(usize::try_from(index).map_err(|_| "Select a mesh control")?)
        .cloned()
        .ok_or("Select a mesh control")?;
    if action == 1 {
        for c in &mut recipe.edge_divisions {
            if c.id == id {
                c.enabled = !c.enabled;
            }
        }
        for c in &mut recipe.local_sizes {
            if c.id == id {
                c.enabled = !c.enabled;
            }
        }
        for c in &mut recipe.refinements {
            if c.id == id {
                c.enabled = !c.enabled;
            }
        }
        for c in &mut recipe.boundary_layers {
            if c.id == id {
                c.enabled = !c.enabled;
            }
        }
        for c in &mut recipe.regions {
            if c.id == id {
                c.enabled = !c.enabled;
            }
        }
        state.workbench.set_mesh_recipe(recipe);
        state.mark_workbench_dirty();
        state.log(format!("Toggled mesh control {id}; regenerate mesh."));
        return Ok(());
    }
    if action == 2 {
        erase(&mut recipe, id);
        state.workbench.set_mesh_recipe(recipe);
        state.mark_workbench_dirty();
        if ui.get_mesh_edit_id() == id.to_string() {
            ui.set_mesh_edit_id("".into());
        }
        state.log(format!(
            "Removed mesh control {id}; historical Runs retained."
        ));
        return Ok(());
    }
    ui.set_mesh_edit_id(id.to_string().into());
    ui.set_mesh_control_name(name.into());
    if let [MeshControlTarget::NamedSelection(name)] = targets.as_slice() {
        ui.set_mesh_scope_mode(0);
        ui.set_wb_local_target(name.clone().into());
    } else {
        ui.set_mesh_scope_mode(1);
    }
    let mut geometry = vec![];
    for target in targets {
        match target {
            MeshControlTarget::Geometry(t) => geometry.push(t),
            MeshControlTarget::NamedSelection(n) => {
                if let Some(g) = state.workbench.named_selections().get(&n) {
                    geometry.extend(g.targets.clone());
                }
            }
        }
    }
    state.wb_selected_targets = geometry.clone();
    state.geometry_editor.selection = geometry;
    state.show_geometry_3d = true;
    state.show_mesh = false;
    if let Some(c) = recipe.edge_divisions.iter().find(|c| c.id == id) {
        ui.set_mesh_control_kind(4);
        ui.set_mesh_edge_cells(c.cells as i32);
        ui.set_mesh_edge_growth(c.progression.to_string().into());
        ui.set_mesh_edge_reverse(c.reverse);
        ui.set_mesh_edge_symmetric(c.symmetric);
    }
    if let Some(c) = recipe.local_sizes.iter().find(|c| c.id == id) {
        ui.set_mesh_control_kind(0);
        ui.set_wb_local_size(c.size.to_string().into());
    }
    if let Some(c) = recipe.refinements.iter().find(|c| c.id == id) {
        ui.set_mesh_control_kind(1);
        ui.set_wb_refine_min(c.size_min.to_string().into());
        ui.set_wb_refine_max(c.size_max.to_string().into());
        ui.set_mesh_refine_start(c.distance_min.to_string().into());
        ui.set_wb_refine_distance(c.distance_max.to_string().into());
        ui.set_mesh_refine_sampling(c.sampling as i32);
    }
    if let Some(c) = recipe.boundary_layers.iter().find(|c| c.id == id) {
        ui.set_mesh_control_kind(2);
        ui.set_mesh_layer_mode(0);
        ui.set_wb_layer_first(c.first_layer_height.to_string().into());
        ui.set_wb_layer_growth(c.growth_ratio.to_string().into());
        ui.set_wb_layer_count(c.layer_count as i32);
        preview(ui);
    }
    if let Some(c) = recipe.regions.iter().find(|c| c.id == id) {
        ui.set_mesh_control_kind(3);
        ui.set_mesh_box_min(format!("{}, {}, {}", c.minimum.x, c.minimum.y, c.minimum.z).into());
        ui.set_mesh_box_max(format!("{}, {}, {}", c.maximum.x, c.maximum.y, c.maximum.z).into());
        ui.set_mesh_box_size(c.size.to_string().into());
        ui.set_mesh_box_transition(c.transition.to_string().into());
    }
    Ok(())
}
pub(super) fn bind(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_add_edges(move || {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if let Err(e) = apply(&ui, &mut state, 4) {
            state.log(format!("Edge control not applied: {e}"));
        }
        ui.set_wb_flow_feedback(state.logs.back().cloned().unwrap_or_default().into());
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_calculate_transition(move || {
        let Some(ui) = weak.upgrade() else { return };
        let result = (|| -> Result<f64, String> {
            let near = number(ui.get_wb_refine_min(), "Near size")?;
            let far = number(ui.get_wb_refine_max(), "Far size")?;
            let start = number(ui.get_mesh_refine_start(), "Fine-region distance")?;
            let g = number(ui.get_mesh_size_growth(), "Distance-field growth limit")?;
            if near <= 0.0 || far < near || start < 0.0 || !(1.01..=2.0).contains(&g) {
                return Err("Require 0 < near ≤ far, start ≥ 0 and growth 1.01–2".into());
            }
            Ok(start + ((far - near) / g.ln()).max(near))
        })();
        match result {
            Ok(end) => ui.set_wb_refine_distance(format!("{end:.12e}").into()),
            Err(e) => {
                let mut state = shared.borrow_mut();
                state.log(e.clone());
                ui.set_wb_flow_feedback(e.into());
            }
        }
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_estimate_first_layer(move || {
        let Some(ui)=weak.upgrade() else {return};
        let state=shared.borrow();
        if state.meshing || state.solving {return;}
        let result=(|| -> Result<(f64,f64),String> {
            let yplus=number(ui.get_mesh_layer_yplus(),"Target y+")?;
            let utau=number(ui.get_mesh_layer_utau(),"Friction velocity")?;
            let nu=state.workbench.material().kinematic_viscosity;
            let height=2.0*yplus*nu/utau;
            if yplus<=0.0 || utau<=0.0 || !height.is_finite() || height<=0.0 {
                return Err("Require positive finite y+, friction velocity and saved fluid viscosity".into());
            }
            Ok((height,nu))
        })();
        match result {
            Ok((height,nu))=>{
                ui.set_mesh_layer_mode(0);
                ui.set_wb_layer_first(format!("{height:.12e}").into());
                ui.set_mesh_layer_estimate(format!("Estimated h₁ = {height:.6e} m using ν={nu:.6e} m²/s. Apply inflation separately; this does not generate 3D prisms.").into());
                preview(&ui);
            }
            Err(e)=>ui.set_mesh_layer_estimate(e.into()),
        }
    });
    let weak = ui.as_weak();
    ui.on_mesh_layer_preview(move || {
        if let Some(ui) = weak.upgrade() {
            preview(&ui);
        }
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_control_action(move |action, index| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if let Err(e) = edit(&ui, &mut state, index, action) {
            state.log(e);
        }
        ui.set_wb_flow_feedback(state.logs.back().cloned().unwrap_or_default().into());
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_region_bounds(move || {
        let Some(ui) = weak.upgrade() else { return };
        let state = shared.borrow();
        let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for v in state.workbench.geometry().vertices() {
            let p = v.position;
            low.x = low.x.min(p.x);
            low.y = low.y.min(p.y);
            low.z = low.z.min(p.z);
            high.x = high.x.max(p.x);
            high.y = high.y.max(p.y);
            high.z = high.z.max(p.z);
        }
        if low.x.is_finite() {
            ui.set_mesh_box_min(format!("{}, {}, {}", low.x, low.y, low.z).into());
            ui.set_mesh_box_max(format!("{}, {}, {}", high.x, high.y, high.z).into());
            ui.set_mesh_box_size(ui.get_wb_global_size());
        }
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_add_region(move || {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if let Err(e) = apply(&ui, &mut state, 3) {
            state.log(format!("Region not applied: {e}"));
        }
        ui.set_wb_flow_feedback(state.logs.back().cloned().unwrap_or_default().into());
        refresh_ui(&ui, &state);
    });
    for kind in 0..3 {
        let weak = ui.as_weak();
        let shared = state.clone();
        let callback = move || {
            let Some(ui) = weak.upgrade() else { return };
            let mut state = shared.borrow_mut();
            if let Err(e) = apply(&ui, &mut state, kind) {
                state.log(format!("Mesh control not applied: {e}"));
            }
            preview(&ui);
            ui.set_wb_flow_feedback(state.logs.back().cloned().unwrap_or_default().into());
            refresh_ui(&ui, &state);
        };
        match kind {
            0 => ui.on_add_local_size_wb(callback),
            1 => ui.on_add_threshold_refinement_wb(callback),
            _ => ui.on_add_boundary_layer_wb(callback),
        }
    }
}
