//! Draft-only cyclic patch pairing; Apply is the only mutation of the project.
use super::*;
use flursys::periodic::TranslationalCyclicPair;
use slint::Model;
fn show_pairs(ui: &MainWindow, pairs: &[TranslationalCyclicPair]) {
    ui.set_cyclic_pair_list(ModelRc::new(VecModel::from(
        pairs
            .iter()
            .map(|p| {
                SharedString::from(format!(
                    "{} ↔ {} · ({}, {}, {}) m",
                    p.first, p.second, p.translation.x, p.translation.y, p.translation.z
                ))
            })
            .collect::<Vec<_>>(),
    )));
    ui.set_cyclic_pair_index(if pairs.is_empty() {
        -1
    } else {
        pairs.len() as i32 - 1
    });
}
pub(super) fn bind(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
    let draft = Rc::new(RefCell::new(Vec::<TranslationalCyclicPair>::new()));
    let weak = ui.as_weak();
    let shared = state.clone();
    let rows = draft.clone();
    ui.on_open_cyclic_pairs(move||{
        let Some(ui)=weak.upgrade()else{return};let state=shared.borrow();
        if state.solving||state.meshing{return}
        *rows.borrow_mut()=state.workbench.solver_options().cyclic_pairs.clone();
        ui.set_cyclic_patch_list(ModelRc::new(VecModel::from(state.workbench.patch_names().into_iter().map(SharedString::from).collect::<Vec<_>>())));
        show_pairs(&ui,&rows.borrow());
        ui.set_cyclic_feedback("Generate a conformal mesh, select two patches and enter the first-to-second translation.".into());
        ui.set_cyclic_dialog_open(true);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    let rows = draft.clone();
    ui.on_add_cyclic_pair(move || {
        let Some(ui) = weak.upgrade() else { return };
        let state = shared.borrow();
        if state.solving || state.meshing {
            return;
        }
        let result = (|| -> Result<(), String> {
            let mesh = state.workbench.mesh().ok_or("Generate the mesh first")?;
            let names = ui.get_cyclic_patch_list();
            let name = |i: i32| {
                names
                    .row_data(usize::try_from(i).map_err(|_| "Select a patch")?)
                    .map(|n| n.to_string())
                    .ok_or_else(|| "Select a patch".to_string())
            };
            let number = |s: SharedString| {
                s.trim()
                    .parse::<f64>()
                    .map_err(|_| "Invalid translation/tolerance".to_string())
            };
            let pair = TranslationalCyclicPair {
                first: name(ui.get_cyclic_first_index())?,
                second: name(ui.get_cyclic_second_index())?,
                translation: Vec3::new(
                    number(ui.get_cyclic_tx())?,
                    number(ui.get_cyclic_ty())?,
                    number(ui.get_cyclic_tz())?,
                ),
                tolerance: number(ui.get_cyclic_tolerance())?,
            };
            let mut candidate = rows.borrow().clone();
            candidate.push(pair);
            let map = flursys::periodic::resolve(&mesh.mesh, &candidate)?;
            *rows.borrow_mut() = candidate;
            show_pairs(&ui, &rows.borrow());
            ui.set_cyclic_feedback(
                format!(
                    "Draft: {} matched face pairs. Apply to replace physical patch conditions.",
                    map.pair_count()
                )
                .into(),
            );
            Ok(())
        })();
        if let Err(e) = result {
            ui.set_cyclic_feedback(e.into());
        }
    });
    let weak = ui.as_weak();
    let rows = draft.clone();
    let shared = state.clone();
    ui.on_remove_cyclic_pair(move || {
        let Some(ui) = weak.upgrade() else { return };
        let state = shared.borrow();
        if state.solving || state.meshing {
            return;
        }
        if let Ok(i) = usize::try_from(ui.get_cyclic_pair_index()) {
            if i < rows.borrow().len() {
                rows.borrow_mut().remove(i);
                show_pairs(&ui, &rows.borrow());
            }
        }
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_apply_cyclic_pairs(move||{
        let Some(ui)=weak.upgrade()else{return};let mut state=shared.borrow_mut();
        if state.solving||state.meshing{return}
        let pairs=draft.borrow().clone();
        match state.workbench.set_cyclic_pairs(pairs){
            Ok(())=>{state.mark_workbench_dirty();state.log("Cyclic pairs applied. Physical conditions on paired patches were removed. Removed pairs need new physical conditions.");ui.set_cyclic_dialog_open(false);mesh_setup::refresh(&ui,&state);if let Ok(i)=usize::try_from(ui.get_patch_index()){if let Some(name)=state.patch_names.get(i){push_boundary_fields(&ui,&state.workbench,name);}}}
            Err(e)=>ui.set_cyclic_feedback(e.to_string().into()),
        }
    });
}
