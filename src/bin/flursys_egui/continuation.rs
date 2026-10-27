//! Persist the active solve separately from the dataset selected for inspection.
use super::*;
#[derive(Serialize, Deserialize)]
struct SolveSession {
    format: u32,
    #[serde(default)]
    monitors: monitoring::Options,
    run: Option<String>,
    folder: Option<PathBuf>,
    family: serde_json::Value,
    every: usize,
    #[serde(default)]
    recovery: workbench::checkpoint::RecoveryPolicy,
    initial: Option<(Vec3, f64)>,
}
pub(super) fn insert_sample(
    series: &mut Vec<(f64, Option<f64>)>,
    iteration: usize,
    value: Option<f64>,
) {
    let x = iteration as f64;
    match series.binary_search_by(|p| p.0.total_cmp(&x)) {
        Ok(i) => series[i].1 = value,
        Err(i) => series.insert(i, (x, value)),
    }
}
impl App {
    fn equation_family(&self) -> serde_json::Value {
        serde_json::json!({"compressible":self.project.solver.compressible.as_ref().map(|s|s.gas),"transient":self.project.solver.compressible.is_some()||self.project.solver.time.is_transient(),"backward":matches!(self.project.solver.time, flursys::unstructured_incompressible::TimeIntegration::Backward{..}|flursys::unstructured_incompressible::TimeIntegration::AdaptiveBackward{..}),"reduced_pressure":self.project.solver.thermal.reduced_pressure,"turbulence":self.project.solver.turbulence.model,"thermal":self.project.solver.thermal.enabled})
    }
    pub(super) fn reset_solve_session(&mut self) {
        self.jobs = WorkbenchJobExecutor::new();
        self.checkpoint_info = None;
        self.monitoring.reset_history();
        self.continuation_run = None;
        self.recording = None;
        self.solve_recording = None;
        self.solve_histories = Default::default();
        self.histories = Default::default();
        self.iteration = 0;
        self.limit = 0;
        self.segment_start = 0;
        self.initialized = false;
        self.plane_frames.clear();
        self.field_frames.clear();
        self.field_playing = false;
        self.plane_playing = false;
        self.frame_scan = None;
        self.force_history.clear();
        self.force_series.clear();
        self.probe_series.clear();
        self.probe_history_job = None;
        self.show_probe_plot = false;
    }
    pub(super) fn persist_continuation(&self) -> Result<(), String> {
        let Some(workspace) = &self.workspace else {
            return Ok(());
        };
        let folder = self
            .solve_recording
            .as_ref()
            .map(|p| {
                p.strip_prefix(workspace)
                    .map(Path::to_path_buf)
                    .map_err(|_| "Recording folder is outside workspace".to_string())
            })
            .transpose()?;
        let state = SolveSession {
            format: 2,
            monitors: self.monitoring.options.clone(),
            run: self.continuation_run.clone(),
            folder,
            family: self.equation_family(),
            every: self.checkpoint_every,
            recovery: self.recovery_policy.clone(),
            initial: self
                .initialized
                .then_some((self.initial_velocity, self.initial_pressure)),
        };
        let mut file = tempfile::NamedTempFile::new_in(workspace).map_err(|e| e.to_string())?;
        serde_json::to_writer_pretty(file.as_file_mut(), &state).map_err(|e| e.to_string())?;
        use std::io::Write;
        file.flush().map_err(|e| e.to_string())?;
        file.as_file().sync_all().map_err(|e| e.to_string())?;
        file.persist(workspace.join("solve-session.json"))
            .map_err(|e| e.to_string())?;
        Ok(())
    }
    pub(super) fn restore_continuation(&mut self) {
        let Some(workspace) = self.workspace.clone() else {
            return;
        };
        if self.session.mesh().is_none() {
            return;
        }
        let state_path = workspace.join("solve-session.json");
        let mut state = if state_path.exists() {
            match std::fs::read(&state_path)
                .map_err(|e| e.to_string())
                .and_then(|b| serde_json::from_slice::<SolveSession>(&b).map_err(|e| e.to_string()))
            {
                Ok(s) => s,
                Err(e) => {
                    self.message(format!("Saved continuation could not be read: {e}"));
                    return;
                }
            }
        } else {
            let Some(run) = self.project.runs.last() else {
                return;
            };
            let Ok(report) = flursys::headless::report(&workspace, run) else {
                return;
            };
            let Some(folder) = report
                .get("plane_recording_folder")
                .and_then(|v| v.as_str())
            else {
                return;
            };
            let settings = &report["solver_settings"];
            let transient = settings
                .get("time")
                .and_then(|v| {
                    serde_json::from_value::<flursys::unstructured_incompressible::TimeIntegration>(
                        v.clone(),
                    )
                    .ok()
                })
                .is_some_and(|t| t.is_transient());
            let family = serde_json::json!({"transient":transient,"turbulence":settings.pointer("/turbulence/model"),"thermal":settings.pointer("/thermal/enabled").and_then(|v|v.as_bool()).unwrap_or(false)});
            SolveSession {
                format: 1,
                monitors: self.monitoring.options.clone(),
                run: Some(run.id.clone()),
                folder: Some(PathBuf::from(folder)),
                family,
                every: 100,
                recovery: Default::default(),
                initial: None,
            }
        };
        if let Some(family) = state.family.as_object_mut() {
            family
                .entry("reduced_pressure")
                .or_insert(serde_json::json!(false));
            family.entry("backward").or_insert(serde_json::json!(false));
        }
        if ![1, 2].contains(&state.format) || state.family != self.equation_family() {
            self.message(
                "Saved solve uses a different equation family; initialization is required.",
            );
            return;
        }
        self.checkpoint_every = state.every.min(1_000_000);
        if state.recovery.validate().is_ok() {
            self.recovery_policy = state.recovery;
        }
        self.monitoring.options = state.monitors;
        let Some(folder) = state.folder else {
            if self.project.solver.compressible.is_some() {
                return;
            }
            if let Some((velocity, pressure)) = state.initial {
                self.initialized = true;
                self.initial_velocity = velocity;
                self.initial_pressure = pressure;
                self.jobs.set_initial_fields(
                    self.session.mesh().unwrap().mesh.id(),
                    flursys::IncompressibleInitialConditions::Uniform { velocity, pressure },
                );
            }
            return;
        };
        if folder.is_absolute()
            || folder
                .components()
                .any(|c| !matches!(c, std::path::Component::Normal(_)))
        {
            self.message("Saved continuation folder is invalid.");
            return;
        }
        let folder = workspace.join(folder);
        if self.project.solver.compressible.is_some() {
            let path = folder.join("checkpoint.json");
            let recovered = std::iter::once(path.clone())
                .chain(workbench::checkpoint::backup_paths(&path))
                .find(|p| workbench::flow_frame::FlowFrame::read(p).is_ok());
            if let Some(path) = recovered {
                match self.restore_gas_frame(&path, state.run.clone()) {
                    Ok(()) => {
                        self.recording = Some(folder.clone());
                        self.solve_recording = Some(folder);
                        if let Some(run) = state
                            .run
                            .and_then(|id| self.project.runs.iter().find(|r| r.id == id))
                        {
                            if let Ok(report) = flursys::headless::report(&workspace, run) {
                                self.solve_histories = read_histories(&report, self.iteration);
                                self.histories = self.solve_histories.clone();
                            }
                        }
                        self.refresh_field_frames();
                    }
                    Err(e) => self.message(format!("Continuation recovery failed: {e}")),
                }
            }
            return;
        }
        let mesh = &self.session.mesh().unwrap().mesh;
        let restored = workbench::checkpoint::recover(&folder, mesh);
        match restored {
            Ok(recovered) => {
                let iteration = recovered.info.iteration;
                let fields = recovered.fields;
                self.checkpoint_info = Some(recovered.info);
                for warning in recovered.warnings {
                    self.append_log(format!("Recovery: {warning}"));
                }
                self.remember_time(iteration, &fields);
                self.jobs.set_continuation_fields(
                    self.session.mesh().unwrap().mesh.id(),
                    fields,
                    iteration,
                );
                self.iteration = iteration;
                self.segment_start = iteration;
                self.limit = iteration;
                self.solve_recording = Some(folder.clone());
                self.recording = Some(folder);
                self.continuation_run = state
                    .run
                    .filter(|id| self.project.runs.iter().any(|r| &r.id == id));
                if let Some(run) = self
                    .continuation_run
                    .as_ref()
                    .and_then(|id| self.project.runs.iter().find(|r| &r.id == id))
                {
                    if let Ok(report) = flursys::headless::report(&workspace, run) {
                        self.solve_histories = read_histories(
                            &report,
                            if run.time_steps > 0 {
                                run.time_steps
                            } else {
                                run.iterations.unwrap_or(iteration)
                            },
                        );
                        for series in &mut self.solve_histories {
                            series.retain(|(x, _)| *x <= iteration as f64);
                        }
                    }
                }
                self.histories = self.solve_histories.clone();
                self.restore_engineering();
                self.refresh_field_frames();
                self.message(format!("Continuation restored at iteration/step {iteration}. Run continues in the same archive."));
            }
            Err(e) => self.message(format!(
                "Saved continuation is unavailable for this mesh: {e}"
            )),
        }
    }
    pub(super) fn plot_histories(&self) -> &[Vec<(f64, Option<f64>)>; 4] {
        if self.stage == Stage::Solution && self.page == 0
            || (self.result_run == "live" || self.result_run == "current")
                && !self.solve_histories.iter().all(Vec::is_empty)
        {
            &self.solve_histories
        } else {
            &self.histories
        }
    }
}
pub(super) fn read_histories(
    report: &serde_json::Value,
    end: usize,
) -> [Vec<(f64, Option<f64>)>; 4] {
    if let Some(series) = report
        .get("indexed_convergence_history")
        .and_then(|v| serde_json::from_value(v.clone()).ok())
    {
        return series;
    }
    let mut out: [Vec<(f64, Option<f64>)>; 4] = Default::default();
    for (i, values) in [
        report.get("continuity_history"),
        report.pointer("/convergence_histories/momentum"),
        report.pointer("/convergence_histories/velocity"),
        report.pointer("/convergence_histories/pressure_linear"),
    ]
    .into_iter()
    .enumerate()
    {
        if let Some(v) = values.and_then(|v| v.as_array()) {
            out[i] = v
                .iter()
                .enumerate()
                .map(|(i, y)| {
                    (
                        (end.saturating_sub(v.len().saturating_sub(1)) + i) as f64,
                        y.as_f64().filter(|y| y.is_finite()),
                    )
                })
                .collect();
        }
    }
    out
}
impl App {
    pub(super) fn recovery_properties(&mut self, ui: &mut egui::Ui) {
        widgets::heading(ui, "RECOVERY CHECKPOINTS");
        ui.add_enabled_ui(!self.jobs.is_running(),|ui|{
            ui.horizontal(|ui|{ui.label("Every N iterations / accepted steps (0 = off)");ui.add(egui::DragValue::new(&mut self.recovery_policy.every_iterations).range(0..=1_000_000));});
            ui.horizontal(|ui|{ui.label("Every wall-clock seconds (0 = off)");ui.add(egui::DragValue::new(&mut self.recovery_policy.every_seconds).range(0..=86400));});
            ui.checkbox(&mut self.recovery_policy.stop_on_error,"Stop safely if a recovery save fails");
            ui.weak("Recovery keeps the latest state and two verified previous generations. It is independent of playback recording.");
        });
        if ui
            .add_enabled(
                !self.jobs.is_checkpointing(),
                egui::Button::new("Save checkpoint now"),
            )
            .clicked()
        {
            match self.jobs.checkpoint_now() {
                Ok(()) => self.message("Saving accepted fields in the background…"),
                Err(e) => self.message(e),
            }
        }
        if self.jobs.is_checkpointing() {
            ui.label("Writing checkpoint…");
        }
        if let Some(info) = &self.checkpoint_info {
            ui.label(format!(
                "Saved at iteration/step {} · {:.1} MiB",
                info.iteration,
                info.bytes as f64 / (1024. * 1024.)
            ));
            ui.small(info.path.display().to_string());
        }
        ui.separator();
    }
    pub(super) fn recording_properties(&mut self, ui: &mut egui::Ui) {
        self.recovery_properties(ui);
        widgets::heading(ui, "AUTOSAVE FIELD HISTORY");
        ui.add_enabled_ui(!self.jobs.is_running(), |ui| {
            let mut enabled = self.checkpoint_every > 0;
            if ui
                .checkbox(
                    &mut enabled,
                    "Save complete fields every N iterations / accepted time steps",
                )
                .changed()
            {
                self.checkpoint_every = if enabled { 10 } else { 0 };
            }
            if enabled {
                ui.horizontal(|ui| {
                    ui.label("N");
                    ui.add(egui::DragValue::new(&mut self.checkpoint_every).range(1..=1_000_000));
                });
            }
        });
        ui.weak("Retains separate velocity, pressure and available temperature/turbulence snapshots for playback, probes and GIF export. Changing N applies on Run / Continue. Steady iterations are not physical time.");
        ui.horizontal(|ui| {
            ui.label("Maximum saved frames (0 = unlimited)");
            ui.add(
                egui::DragValue::new(&mut self.recovery_policy.max_history_frames)
                    .range(0..=1_000_000),
            );
        });
        let mut mib = self.recovery_policy.max_history_bytes / (1024 * 1024);
        if ui
            .horizontal(|ui| {
                ui.label("History storage budget (MiB, 0 = unlimited)");
                ui.add(egui::DragValue::new(&mut mib).range(0..=1_048_576))
                    .changed()
            })
            .inner
        {
            self.recovery_policy.max_history_bytes = mib * 1024 * 1024;
        }
        if ui
            .button("Save recovery and recording preferences")
            .clicked()
        {
            match self.persist_continuation() {
                Ok(()) => self.message("Recording interval saved for the active solve."),
                Err(e) => self.message(e),
            }
        }
        if ui.button("Refresh saved field frames").clicked() {
            self.refresh_field_frames();
        }
        ui.label(format!(
            "{} saved full-field frames",
            self.field_frames.len()
        ));
    }
    pub(super) fn refresh_field_frames(&mut self) {
        self.field_frames.clear();
        self.field_playing = false;
        if let Some(folder) = &self.recording {
            if let Ok(entries) = std::fs::read_dir(folder.join("fields")) {
                self.field_frames = entries
                    .flatten()
                    .filter(|entry| entry.file_type().is_ok_and(|t| t.is_file()))
                    .map(|e| e.path())
                    .filter(|p| {
                        p.file_name()
                            .is_some_and(|n| n.to_string_lossy().starts_with("iteration-"))
                            && p.extension().is_some_and(|e| e == "json")
                    })
                    .collect();
                self.field_frames.sort();
            }
        }
        self.field_frame = self.field_frames.len().saturating_sub(1);
    }
    pub(super) fn read_field_frame(&mut self) -> bool {
        let Some(path) = self.field_frames.get(self.field_frame) else {
            return false;
        };
        let Some(mesh) = self
            .result
            .as_ref()
            .map(|d| d.mesh())
            .or_else(|| self.session.mesh().map(|m| &m.mesh))
        else {
            return false;
        };
        let loaded = workbench::checkpoint::load(path, mesh).and_then(|(index, initial)| {
            let data = match initial {
                flursys::IncompressibleInitialConditions::Restart {
                    velocity,
                    pressure,
                    temperature,
                    turbulence,
                    ..
                } => ResultDataset::from_values(
                    &self.result_run,
                    mesh,
                    pressure.values().to_vec(),
                    velocity.values().to_vec(),
                )
                .and_then(|d| d.with_temperature(temperature.map(|t| t.values().to_vec())))
                .map(|d| d.with_turbulence(turbulence.as_ref())),
                flursys::IncompressibleInitialConditions::Fields { velocity, pressure } => {
                    ResultDataset::from_values(
                        &self.result_run,
                        mesh,
                        pressure.values().to_vec(),
                        velocity.values().to_vec(),
                    )
                }
                _ => return Err("Saved frame does not contain complete fields".into()),
            }
            .map_err(|e| e.to_string())?;
            Ok((index, data))
        });
        match loaded {
            Ok((index, data)) => {
                self.result_iteration = index;
                self.result_cache = ResultRenderCache::build(&data).ok();
                for probe in &mut self.probes {
                    if let Some(p) = data.probe_cell(probe.cell_index) {
                        *probe = p;
                    }
                }
                self.result = Some(data);
                if self.plane_index > 0 {
                    self.refresh_plane_preview();
                }
                true
            }
            Err(e) => {
                self.field_playing = false;
                self.message(e);
                false
            }
        }
    }
}
impl App {
    pub(super) fn preview_retained_fields(&mut self) -> bool {
        let Some(mesh) = self.session.mesh() else {
            return false;
        };
        let Some((index, initial)) = self.jobs.retained_fields(mesh.mesh.id()) else {
            return false;
        };
        let data = match initial {
            flursys::IncompressibleInitialConditions::Restart {
                velocity,
                pressure,
                temperature,
                turbulence,
                ..
            } => ResultDataset::from_values(
                "current",
                &mesh.mesh,
                pressure.values().to_vec(),
                velocity.values().to_vec(),
            )
            .and_then(|d| d.with_temperature(temperature.map(|t| t.values().to_vec())))
            .map(|d| d.with_turbulence(turbulence.as_ref())),
            flursys::IncompressibleInitialConditions::Fields { velocity, pressure } => {
                ResultDataset::from_values(
                    "current",
                    &mesh.mesh,
                    pressure.values().to_vec(),
                    velocity.values().to_vec(),
                )
            }
            _ => return false,
        };
        match data {
            Ok(data) => {
                self.result_cache = ResultRenderCache::build(&data).ok();
                self.result = Some(data);
                self.recording = self.solve_recording.clone();
                self.restore_engineering();
                self.refresh_field_frames();
                self.result_run = "current".into();
                self.result_iteration = index;
                true
            }
            Err(e) => {
                self.message(e.to_string());
                false
            }
        }
    }
}
impl App {
    pub(super) fn load_continuation_checkpoint(&mut self, path: &Path) -> Result<(), String> {
        if workbench::flow_frame::is_frame(path) {
            self.reset_solve_session();
            self.restore_gas_frame(path, None)?;
            self.solve_recording = None;
            self.recording = None;
            self.persist_continuation()?;
            self.message(
                "Checkpoint loaded; accepted fields and numbering retained for a new run.",
            );
            return Ok(());
        }
        let mesh = &self
            .session
            .mesh()
            .ok_or("Open or generate the matching mesh first")?
            .mesh;
        let mesh_id = mesh.id();
        let recovered = workbench::checkpoint::load_recoverable(path, mesh)?;
        let iteration = recovered.info.iteration;
        let fields = recovered.fields;
        let archived = if let Some(workspace) = &self.workspace {
            let folder = workspace.join("monitors").join(format!(
                "restart-{}",
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos()
            ));
            std::fs::create_dir_all(&folder).map_err(|e| e.to_string())?;
            workbench::checkpoint::write_restart(
                &folder.join("checkpoint.json"),
                &workbench::checkpoint::MeshSignature::new(mesh),
                iteration,
                &fields,
                self.project.solver.time.is_transient(),
                "Explicit restart initialization",
            )?;
            Some(folder)
        } else {
            None
        };
        self.reset_solve_session();
        self.remember_time(iteration, &fields);
        self.solve_recording = archived;
        self.jobs
            .set_continuation_fields(mesh_id, fields, iteration);
        self.iteration = iteration;
        self.limit = iteration;
        self.segment_start = iteration;
        self.preview_retained_fields();
        self.persist_continuation()?;
        self.message(format!(
            "Checkpoint loaded at iteration/step {iteration}. Run / Continue resumes these fields."
        ));
        Ok(())
    }
}
impl App {
    pub(super) fn probe_history_controls(&mut self, ui: &mut egui::Ui) {
        let names = [
            "Speed (m/s)",
            "Pressure p/ρ (m²/s²)",
            "Velocity X (m/s)",
            "Velocity Y (m/s)",
            "Velocity Z (m/s)",
            "Temperature (K)",
        ];
        let previous = self.probe_quantity;
        egui::ComboBox::from_id_salt("probe-history-field")
            .selected_text(names[self.probe_quantity.min(5)])
            .show_ui(ui, |ui| {
                for (i, name) in names.iter().enumerate() {
                    ui.selectable_value(&mut self.probe_quantity, i, *name);
                }
            });
        if previous != self.probe_quantity {
            self.probe_series.clear();
        }
        ui.weak("The plot displays the first eight selected probes; CSV includes all probes.");
        if ui
            .add_enabled(
                self.probe_history_job.is_none() && !self.probes.is_empty(),
                egui::Button::new("Build history from saved frames"),
            )
            .clicked()
        {
            self.build_probe_history();
        }
        if self.probe_history_job.is_some() {
            ui.spinner();
            ui.weak("Reading saved field frames…");
        }
        if !self.probe_series.is_empty() {
            ui.horizontal_wrapped(|ui| {
                if ui.button("Show probe plot").clicked() {
                    self.stage = Stage::Results;
                    self.page = 3;
                    self.show_probe_plot = true;
                }
                if ui.button("Export history CSV…").clicked() {
                    if let Some(path) = rfd::FileDialog::new()
                        .set_file_name("probe-history.csv")
                        .save_file()
                    {
                        if let Err(e) = std::fs::write(
                            path,
                            workbench::series_plot::csv(
                                &self.probe_series,
                                "Iteration / physical step",
                            ),
                        ) {
                            self.message(e.to_string());
                        }
                    }
                }
            });
        }
    }
    fn probe_source_key(&self) -> String {
        format!(
            "{}:{}:{}:{:?}:{}",
            self.probe_quantity,
            self.result_run,
            self.plane_index,
            self.probes.iter().map(|p| p.cell_index).collect::<Vec<_>>(),
            self.result
                .as_ref()
                .map(|d| d.mesh().id().get())
                .unwrap_or(0)
        )
    }
    fn build_probe_history(&mut self) {
        let paths = if self.plane_index > 0 && !self.plane_frames.is_empty() {
            self.plane_frames.clone()
        } else {
            self.field_frames.clone()
        };
        if paths.is_empty() {
            self.message(
                "No saved field frames are available. Enable recording, then Run / Continue.",
            );
            return;
        }
        let Some(mesh) = self.result.as_ref().map(|d| d.mesh().clone()) else {
            return;
        };
        let cells: Vec<_> = self.probes.iter().map(|p| p.cell_index).collect();
        let quantity = self.probe_quantity;
        let plane = self.plane_index > 0 && !self.plane_frames.is_empty();
        let (sender, receiver) = mpsc::channel();
        self.probe_history_job = Some((self.probe_source_key(), receiver));
        std::thread::spawn(move || {
            let result = (|| -> Result<Vec<workbench::series_plot::Series>, String> {
                let mut series: Vec<_> = cells
                    .iter()
                    .enumerate()
                    .map(|(i, cell)| workbench::series_plot::Series {
                        name: format!("P{} · cell {}", i + 1, cell),
                        points: Vec::new(),
                    })
                    .collect();
                for path in paths {
                    if plane {
                        let snapshot: plane_monitor::PlaneSnapshot = serde_json::from_reader(
                            std::fs::File::open(path).map_err(|e| e.to_string())?,
                        )
                        .map_err(|e| e.to_string())?;
                        for (i, cell) in cells.iter().enumerate() {
                            let value = snapshot
                                .samples
                                .iter()
                                .find(|s| s.geometry.cell == *cell)
                                .and_then(|s| match quantity {
                                    0 => Some(s.velocity.norm()),
                                    1 => Some(s.pressure),
                                    2 => Some(s.velocity.x),
                                    3 => Some(s.velocity.y),
                                    4 => Some(s.velocity.z),
                                    _ => None,
                                });
                            series[i].points.push((snapshot.iteration as f64, value));
                        }
                    } else {
                        let (index, initial) = workbench::checkpoint::load(&path, &mesh)?;
                        let (velocity, pressure, temperature) = match initial {
                            flursys::IncompressibleInitialConditions::Restart {
                                velocity,
                                pressure,
                                temperature,
                                ..
                            } => (velocity, pressure, temperature),
                            flursys::IncompressibleInitialConditions::Fields {
                                velocity,
                                pressure,
                            } => (velocity, pressure, None),
                            _ => return Err("Saved frame has no sampled fields".into()),
                        };
                        for (i, &cell) in cells.iter().enumerate() {
                            let value = match quantity {
                                0 => velocity.values().get(cell).map(|v| v.norm()),
                                1 => pressure.values().get(cell).copied(),
                                2 => velocity.values().get(cell).map(|v| v.x),
                                3 => velocity.values().get(cell).map(|v| v.y),
                                4 => velocity.values().get(cell).map(|v| v.z),
                                _ => temperature
                                    .as_ref()
                                    .and_then(|t| t.values().get(cell))
                                    .copied(),
                            };
                            series[i].points.push((index as f64, value));
                        }
                    }
                }
                Ok(series)
            })();
            let _ = sender.send(result);
        });
    }
    pub(super) fn poll_probe_history(&mut self) {
        let received = self
            .probe_history_job
            .as_ref()
            .and_then(|(key, rx)| match rx.try_recv() {
                Ok(value) => Some((key.clone(), value)),
                Err(mpsc::TryRecvError::Disconnected) => {
                    Some((key.clone(), Err("Probe history reader stopped".into())))
                }
                Err(mpsc::TryRecvError::Empty) => None,
            });
        if let Some((key, result)) = received {
            self.probe_history_job = None;
            if key == self.probe_source_key() {
                match result {
                    Ok(series) => {
                        self.probe_series = series;
                        self.message("Probe histories are ready for plotting and CSV export.");
                    }
                    Err(e) => self.message(e),
                }
            }
        }
    }
    pub(super) fn native_probe_plot(&mut self, ui: &mut egui::Ui) {
        let title = [
            "Speed (m/s)",
            "Pressure p/ρ (m²/s²)",
            "Velocity X (m/s)",
            "Velocity Y (m/s)",
            "Velocity Z (m/s)",
            "Temperature (K)",
        ][self.probe_quantity.min(5)];
        let series = std::mem::take(&mut self.probe_series);
        self.native_series_plot(ui, &series, title, "Iteration / physical step");
        self.probe_series = series;
    }
    pub(super) fn native_series_plot(
        &mut self,
        ui: &mut egui::Ui,
        series: &[workbench::series_plot::Series],
        title: &str,
        x_label: &str,
    ) {
        let chart_top = ui.cursor().min;
        ui.label(title);
        let colors = [
            Color32::LIGHT_BLUE,
            Color32::LIGHT_GREEN,
            Color32::YELLOW,
            Color32::LIGHT_RED,
            Color32::from_rgb(196, 155, 255),
            Color32::WHITE,
            Color32::from_rgb(255, 150, 80),
            Color32::from_rgb(85, 210, 200),
        ];
        ui.horizontal_wrapped(|ui| {
            for (i, s) in series.iter().take(8).enumerate() {
                ui.colored_label(colors[i], &s.name);
            }
        });
        let (rect, response) = ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
        self.viewport_rect = Some(egui::Rect::from_min_max(chart_top, rect.max));
        let plot = egui::Rect::from_min_max(
            rect.min + egui::vec2(85., 25.),
            rect.max - egui::vec2(25., 50.),
        );
        if plot.width() < 30. || plot.height() < 30. {
            return;
        }
        let p = ui.painter_at(rect);
        let ink = ui.visuals().text_color();
        let mut low = [f64::INFINITY; 2];
        let mut high = [f64::NEG_INFINITY; 2];
        for s in series.iter().take(8) {
            for &(x, y) in &s.points {
                if let Some(y) = y.filter(|v| v.is_finite()) {
                    low[0] = low[0].min(x);
                    high[0] = high[0].max(x);
                    low[1] = low[1].min(y);
                    high[1] = high[1].max(y);
                }
            }
        }
        if !low[0].is_finite() {
            p.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "No finite data to plot",
                egui::FontId::proportional(14.),
                ink,
            );
            return;
        }
        for i in 0..2 {
            if high[i] <= low[i] {
                let d = low[i].abs().max(1.) * 0.05;
                low[i] -= d;
                high[i] += d;
            }
        }
        let point = |x: f64, y: f64| {
            egui::pos2(
                plot.left() + ((x - low[0]) / (high[0] - low[0])) as f32 * plot.width(),
                plot.bottom() - ((y - low[1]) / (high[1] - low[1])) as f32 * plot.height(),
            )
        };
        for i in 0..=5 {
            let t = i as f64 / 5.;
            let x = low[0] + t * (high[0] - low[0]);
            let y = low[1] + t * (high[1] - low[1]);
            let q = point(x, y);
            let stroke = egui::Stroke::new(1., ui.visuals().widgets.noninteractive.bg_stroke.color);
            p.line_segment(
                [egui::pos2(q.x, plot.top()), egui::pos2(q.x, plot.bottom())],
                stroke,
            );
            p.line_segment(
                [egui::pos2(plot.left(), q.y), egui::pos2(plot.right(), q.y)],
                stroke,
            );
            p.text(
                egui::pos2(q.x, plot.bottom() + 8.),
                egui::Align2::CENTER_TOP,
                format!("{x:.3e}"),
                egui::FontId::proportional(11.),
                ink,
            );
            p.text(
                egui::pos2(plot.left() - 7., q.y),
                egui::Align2::RIGHT_CENTER,
                format!("{y:.3e}"),
                egui::FontId::proportional(11.),
                ink,
            );
        }
        for (i, s) in series.iter().take(8).enumerate() {
            let mut previous = None;
            for (x, y) in performance::envelope(&s.points, plot.width().ceil() as usize) {
                if let Some(y) = y.filter(|v| v.is_finite()) {
                    let q = point(x, y);
                    if let Some(a) = previous {
                        p.line_segment([a, q], egui::Stroke::new(1.6, colors[i]));
                    }
                    previous = Some(q);
                } else {
                    previous = None;
                }
            }
        }
        p.text(
            egui::pos2(plot.center().x, rect.bottom() - 4.),
            egui::Align2::CENTER_BOTTOM,
            x_label,
            egui::FontId::proportional(12.),
            ink,
        );
        p.text(
            rect.right_top(),
            egui::Align2::RIGHT_TOP,
            concat!("FLURSYS v", env!("CARGO_PKG_VERSION")),
            egui::FontId::proportional(11.),
            ink,
        );
        if let Some(pos) = response.hover_pos().filter(|p| plot.contains(*p)) {
            let x = low[0] + ((pos.x - plot.left()) / plot.width()) as f64 * (high[0] - low[0]);
            response.on_hover_ui(|ui| {
                for s in series.iter().take(8) {
                    if let Some((i, v)) = s
                        .points
                        .iter()
                        .min_by(|a, b| (a.0 - x).abs().total_cmp(&(b.0 - x).abs()))
                    {
                        ui.label(format!(
                            "{} · {:.6e} · {:.6e}",
                            s.name,
                            i,
                            v.unwrap_or(f64::NAN)
                        ));
                    }
                }
            });
        }
    }
}
