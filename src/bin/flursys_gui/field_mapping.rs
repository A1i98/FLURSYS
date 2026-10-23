//! Background field initialization; stale mesh/project results never apply.
use super::*;
type Outcome = Result<flursys::workbench::field_mapping::MappedFields, String>;
#[derive(Default)]
pub(super) struct State {
    archive: Option<PathBuf>,
    cancellation: std::sync::Arc<std::sync::atomic::AtomicBool>,
    receiver: Option<std::sync::mpsc::Receiver<(flursys::MeshId, Outcome)>>,
}
impl Drop for State {
    fn drop(&mut self) {
        self.cancellation
            .store(true, std::sync::atomic::Ordering::Release);
    }
}
impl State {
    pub fn cancel(&mut self) {
        self.cancellation
            .store(true, std::sync::atomic::Ordering::Release);
        self.receiver = None;
    }
    pub fn busy(&self) -> bool {
        self.receiver.is_some()
    }
}
pub(super) fn start(ui: &MainWindow, state: &mut AppState, from_file: bool) {
    if state.solving || state.meshing || state.post.mapping.busy() {
        return;
    }
    let Some(target) = state.workbench.mesh().map(|m| m.mesh.clone()) else {
        state.log("Generate the target mesh first.");
        return;
    };
    let source = if from_file {
        let Some(path) = rfd::FileDialog::new()
            .set_title("Source FLURSYS VTK · same geometry / coordinates")
            .add_filter("FLURSYS legacy VTK", &["vtk"])
            .pick_file()
        else {
            return;
        };
        // File IO and parsing are performed in the worker.
        Err(path)
    } else {
        let Some(source) = state
            .active_result()
            .map(flursys::workbench::field_mapping::SourceFields::copy_from_result)
        else {
            state.log("Select a saved source run in Results first, or choose Map VTK file.");
            return;
        };
        Ok(source)
    };
    let (tx, rx) = std::sync::mpsc::channel();
    let id = target.id();
    state.post.mapping.cancellation =
        std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let cancellation = state.post.mapping.cancellation.clone();
    let archive = state.workspace_path.as_ref().map(|p| {
        p.join("initialization").join(format!(
            "fields-{}.json",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ))
    });
    state.post.mapping.archive = archive.clone();
    std::thread::spawn(move || {
        let result = (|| {
            let source = match source {
                Ok(data) => data,
                Err(path) => flursys::workbench::field_mapping::SourceFields::from_result(
                    flursys::workbench::results::load_legacy_vtk_result(&path, "mapping-source")
                        .map_err(|e| e.to_string())?,
                ),
            };
            let mapped = flursys::workbench::field_mapping::map_initial_fields_with_cancel(
                &source,
                &target,
                || cancellation.load(std::sync::atomic::Ordering::Acquire),
            )?;
            if cancellation.load(std::sync::atomic::Ordering::Acquire) {
                return Err("Field mapping cancelled".into());
            }
            archive_fields(&mapped, &target, archive.as_deref())?;
            Ok(mapped)
        })();
        let _ = tx.send((id, result));
    });
    state.post.mapping.receiver = Some(rx);
    state.log("Mapping U and p/ρ in the background. Same geometry/coordinates and convex source cells are required. This initializes a new solve; it is not a conservative remap or physical-time restart.");
    refresh_ui(ui, state);
}
pub(super) fn poll(ui: &MainWindow, state: &mut AppState) -> bool {
    let Some(rx) = state.post.mapping.receiver.take() else {
        return false;
    };
    match rx.try_recv() {
        Err(std::sync::mpsc::TryRecvError::Empty) => {
            state.post.mapping.receiver = Some(rx);
            false
        }
        Err(_) => {
            state.log("Field-mapping worker disconnected; initialization was unchanged.");
            true
        }
        Ok((id, result)) => {
            if state.solving
                || state.meshing
                || state.workbench.mesh().is_none_or(|m| m.mesh.id() != id)
            {
                state.log("Stale mapped fields discarded: mesh or job state changed.");
                return true;
            }
            match result {
                Err(e) => state.log(format!(
                    "Field mapping rejected; initialization unchanged: {e}"
                )),
                Ok(mapped) => {
                    let archive = state.post.mapping.archive.take();
                    state.workbench.initialize_solution();
                    state.workbench_jobs.set_initial_fields(id, mapped.initial);
                    state.active_result = None;
                    state.result_render_cache = None;
                    state.active_result_probe = None;
                    state.post.reset_result();
                    state.wb_residual_history.clear();
                    state.log(format!("{}. U and p/ρ initialized; iteration/time reset. Face flux, thermal and turbulence initialization will be rebuilt.",mapped.description));
                    if let Some(path) = archive {
                        state.log(format!("Initial fields saved: {}. Load restart can restore this initialization on the matching mesh.",path.display()));
                    }
                    state.mark_workbench_dirty();
                    ui.set_solver_paused(false);
                }
            }
            true
        }
    }
}

pub(super) fn start_patch(ui: &MainWindow, state: &mut AppState) {
    if state.solving || state.meshing || state.post.mapping.busy() {
        return;
    }
    let Some(mesh) = state.workbench.mesh().map(|m| m.mesh.clone()) else {
        state.log("Generate a target mesh first.");
        return;
    };
    let strings = [
        ui.get_initial_box_xmin(),
        ui.get_initial_box_ymin(),
        ui.get_initial_box_zmin(),
        ui.get_initial_box_xmax(),
        ui.get_initial_box_ymax(),
        ui.get_initial_box_zmax(),
        ui.get_initial_x(),
        ui.get_initial_y(),
        ui.get_initial_z(),
        ui.get_initial_p(),
    ];
    let values = match strings
        .iter()
        .map(|s| s.trim().parse::<f64>())
        .collect::<Result<Vec<_>, _>>()
    {
        Ok(v) if v.iter().all(|v| v.is_finite()) => v,
        _ => {
            state.log("Box initialization requires finite coordinates, velocity and pressure.");
            return;
        }
    };
    let initial = state
        .workbench_jobs
        .retained_fields(mesh.id())
        .map(|(_, v)| v)
        .unwrap_or(flursys::IncompressibleInitialConditions::uniform(
            Vec3::ZERO,
            0.0,
        ));
    let low = Vec3::new(values[0], values[1], values[2]);
    let high = Vec3::new(values[3], values[4], values[5]);
    let velocity = Vec3::new(values[6], values[7], values[8]);
    let pressure = values[9];
    let id = mesh.id();
    let (tx, rx) = std::sync::mpsc::channel();
    state.post.mapping.cancellation =
        std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let cancelled = state.post.mapping.cancellation.clone();
    let archive = state.workspace_path.as_ref().map(|p| {
        p.join("initialization").join(format!(
            "fields-{}.json",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ))
    });
    state.post.mapping.archive = archive.clone();
    std::thread::spawn(move || {
        let result = flursys::workbench::field_mapping::patch_box(
            &mesh,
            initial,
            low,
            high,
            velocity,
            pressure,
            || cancelled.load(std::sync::atomic::Ordering::Acquire),
        );
        let result = result.and_then(|mapped| {
            if cancelled.load(std::sync::atomic::Ordering::Acquire) {
                return Err("Box initialization cancelled".into());
            }
            archive_fields(&mapped, &mesh, archive.as_deref())?;
            Ok(mapped)
        });
        let _ = tx.send((id, result));
    });
    state.post.mapping.receiver = Some(rx);
    state.log("Box initialization in background. Uses the U/p values above for cell centres inside the box; outside keeps the queued initialization (zero if none). This is not an edit of a historical result.");
    refresh_ui(ui, state);
}

fn archive_fields(
    mapped: &flursys::workbench::field_mapping::MappedFields,
    mesh: &flursys::UnstructuredMesh,
    path: Option<&Path>,
) -> Result<(), String> {
    let Some(path) = path else { return Ok(()) };
    let flursys::IncompressibleInitialConditions::Fields { velocity, pressure } = &mapped.initial
    else {
        return Err("Expected initialization fields".into());
    };
    let signature = flursys::workbench::checkpoint::MeshSignature::new(mesh);
    flursys::workbench::checkpoint::write(
        path,
        &signature,
        0,
        velocity.values(),
        pressure.values(),
        &mapped.description,
    )
    .map_err(|e| format!("Initialization archive failed; queued fields unchanged: {e}"))
}
