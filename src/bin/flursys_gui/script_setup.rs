//! Project script library and explicit boundary bindings.
use super::*;
use flursys::scripting::{Binding, Quantity, Script};
fn quantity(ui: &MainWindow) -> Quantity {
    match ui.get_script_quantity() {
        1 => Quantity::Pressure,
        2 => Quantity::Temperature,
        _ => Quantity::Velocity,
    }
}
fn draft(ui: &MainWindow) -> Result<(String, Script, Binding), String> {
    let id = ui.get_script_name().trim().to_string();
    if id.is_empty() {
        return Err("Enter a script name".into());
    }
    let source = Script {
        source: ui.get_script_source().to_string(),
        time_dependent: ui.get_script_timed(),
    };
    let parameters = serde_json::from_str(ui.get_script_parameters().as_str())
        .map_err(|e| format!("Parameters must be a JSON object of numbers: {e}"))?;
    let binding = Binding {
        script: id.clone(),
        function: ui.get_script_function().trim().into(),
        quantity: quantity(ui),
        parameters,
    };
    Ok((id, source, binding))
}
fn choices(ui: &MainWindow, s: &WorkbenchSession) {
    ui.set_script_names(ModelRc::from(Rc::new(VecModel::from(
        s.solver_options()
            .scripting
            .scripts
            .keys()
            .map(|s| SharedString::from(s.as_str()))
            .collect::<Vec<_>>(),
    ))));
}
pub(super) fn push(ui: &MainWindow, s: &WorkbenchSession, patch: &str) {
    let bindings = s.solver_options().scripting.bindings.get(patch);
    let text = bindings
        .filter(|v| !v.is_empty())
        .map(|v| {
            v.iter()
                .map(|b| format!("{:?}: {} / {}", b.quantity, b.script, b.function))
                .collect::<Vec<_>>()
                .join("\n")
        })
        .unwrap_or_else(|| "No script attached to this boundary".into());
    ui.set_script_binding_summary(text.into());
}
fn load(ui: &MainWindow, s: &WorkbenchSession, id: &str, patch: &str) {
    if let Some(script) = s.solver_options().scripting.scripts.get(id) {
        ui.set_script_name(id.into());
        ui.set_script_source(script.source.as_str().into());
        ui.set_script_timed(script.time_dependent);
        ui.set_script_function("profile".into());
        ui.set_script_parameters("{}".into());
        if let Some(b) = s
            .solver_options()
            .scripting
            .bindings
            .get(patch)
            .into_iter()
            .flatten()
            .find(|b| b.script == id)
        {
            ui.set_script_function(b.function.as_str().into());
            ui.set_script_quantity(match b.quantity {
                Quantity::Velocity => 0,
                Quantity::Pressure => 1,
                Quantity::Temperature => 2,
            });
            ui.set_script_parameters(
                serde_json::to_string(&b.parameters)
                    .unwrap_or_default()
                    .into(),
            );
        }
    }
}
pub(super) fn bind(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_open_script_editor(move || {
        let Some(ui)=weak.upgrade() else{return};let s=shared.borrow();
        if s.solving || s.meshing {return;}
        choices(&ui,&s.workbench);
        let patch=s.patch_names.get(ui.get_patch_index().max(0) as usize).cloned().unwrap_or_default();
        ui.set_script_patch(patch.clone().into());
        if let Some(id)=s.workbench.solver_options().scripting.bindings.get(&patch).and_then(|v|v.first()).map(|b|b.script.clone()).or_else(||s.workbench.solver_options().scripting.scripts.keys().next().cloned()) {
            load(&ui,&s.workbench,&id,&patch);
            let index=s.workbench.solver_options().scripting.scripts.keys().position(|k|*k==id).unwrap_or(0);
            ui.set_script_index(index as i32);
        }
        ui.set_script_feedback("Edit a draft, preview, then Save or Attach explicitly. Active runs keep their saved revision.".into());
        ui.set_script_editor_open(true);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_load_script(move |index| {
        let Some(ui) = weak.upgrade() else { return };
        let s = shared.borrow();
        if let Some(id) = s
            .workbench
            .solver_options()
            .scripting
            .scripts
            .keys()
            .nth(index.max(0) as usize)
        {
            load(&ui, &s.workbench, id, ui.get_script_patch().as_str());
        }
    });
    let weak = ui.as_weak();
    ui.on_script_template(move || {
        let Some(ui)=weak.upgrade() else{return};
        ui.set_script_name("new-profile".into());ui.set_script_function("profile".into());ui.set_script_parameters("{}".into());
        let args=if ui.get_script_timed(){"x, y, z, t"}else{"x, y, z"};
        let body=match quantity(&ui) {Quantity::Velocity=>"vec3(1.0, 0.0, 0.0)",Quantity::Pressure=>"0.0",Quantity::Temperature=>"300.0"};
        ui.set_script_source(format!("fn profile({args}) {{\n    {body}\n}}\n").into());
        ui.set_script_feedback("New unsaved draft. Parameters are available as params.name; PI is available. Global statements do not execute.".into());
    });
    let weak = ui.as_weak();
    ui.on_preview_script(move || {
        let Some(ui) = weak.upgrade() else { return };
        let result = (|| -> Result<String, String> {
            let (_, s, b) = draft(&ui)?;
            let pos: Vec<f64> = ui
                .get_script_preview_position()
                .split(',')
                .map(|v| {
                    v.trim()
                        .parse()
                        .map_err(|_| "Position must be x,y,z in metres")
                })
                .collect::<Result<_, _>>()?;
            if pos.len() != 3 || !pos.iter().all(|v| v.is_finite()) {
                return Err("Position requires three finite coordinates".into());
            }
            let t = if s.time_dependent {
                let t: f64 = ui
                    .get_script_preview_time()
                    .parse()
                    .map_err(|_| "Invalid time")?;
                if !t.is_finite() || t < 0.0 {
                    return Err("Time must be finite and nonnegative".into());
                }
                Some(t)
            } else {
                None
            };
            let v = s
                .compile()?
                .value(&b, Vec3::new(pos[0], pos[1], pos[2]), t)?;
            Ok(match b.quantity {
                Quantity::Velocity => format!("Preview: ({}, {}, {}) m/s", v.x, v.y, v.z),
                Quantity::Pressure => format!("Preview: {} m²/s² (kinematic gauge pressure)", v.x),
                Quantity::Temperature => format!("Preview: {} K", v.x),
            })
        })();
        ui.set_script_feedback(
            result
                .unwrap_or_else(|e| format!("Script error: {e}"))
                .into(),
        );
    });
    for action in 0..4 {
        let weak = ui.as_weak();
        let shared = state.clone();
        let handler = move || {
            let Some(ui) = weak.upgrade() else { return };
            let mut s = shared.borrow_mut();
            if s.solving || s.meshing {
                return;
            }
            let result = (|| -> Result<(), String> {
                let mut settings = s.workbench.solver_options().scripting.clone();
                let patch = ui.get_script_patch().to_string();
                if action == 2 {
                    if let Some(v) = settings.bindings.get_mut(&patch) {
                        v.retain(|b| b.quantity != quantity(&ui));
                        if v.is_empty() {
                            settings.bindings.remove(&patch);
                        }
                    }
                } else if action == 3 {
                    let id = ui.get_script_name().to_string();
                    if settings.bindings.values().flatten().any(|b| b.script == id) {
                        return Err(
                            "Detach this script from all boundaries before deleting it".into()
                        );
                    }
                    if settings.scripts.remove(&id).is_none() {
                        return Err("No saved script with this name".into());
                    }
                } else {
                    let (id, script, binding) = draft(&ui)?;
                    script.compile()?;
                    settings.scripts.insert(id, script);
                    if action == 1 {
                        if patch.is_empty() {
                            return Err("Select a boundary first".into());
                        }
                        let v = settings.bindings.entry(patch.clone()).or_default();
                        v.retain(|b| b.quantity != binding.quantity);
                        v.push(binding);
                    }
                }
                s.workbench.set_scripting(settings)?;
                s.mark_workbench_dirty();
                choices(&ui, &s.workbench);
                push(&ui, &s.workbench, &patch);
                s.log(format!("Boundary script configuration updated: {patch}"));
                Ok(())
            })();
            ui.set_script_feedback(match result {
                Ok(()) => "Saved project configuration. No running solver was modified.".into(),
                Err(e) => format!("Script error: {e}").into(),
            });
            refresh_ui(&ui, &s);
        };
        match action {
            0 => ui.on_save_script(handler),
            1 => ui.on_attach_script(handler),
            2 => ui.on_detach_script(handler),
            _ => ui.on_delete_script(handler),
        };
    }
}
