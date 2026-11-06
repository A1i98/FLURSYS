//! Prescribed-flux multiphase/heat transport, using the same core and archive as CLI.
use super::*;
use flursys::multiphysics as physics;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
#[derive(Default)]
pub struct Transport {
    pub options: physics::Options,
    pub latest: Option<physics::State>,
    pub field: usize,
    pub receiver: Option<Receiver<Event>>,
    pub paused: Arc<AtomicBool>,
    pub stopped: Arc<AtomicBool>,
    pub folder: Option<PathBuf>,
    pub restart: Option<physics::checkpoint::Checkpoint>,
    pub use_flow: bool,
    pub coupled_settings: physics::coupled::Settings,
    pub coupled_latest: Option<(flursys::mesh::MeshId, Vec<f64>, Vec<f64>)>,
    pub coupled_revision: u64,
    pub coupled_checkpoint: Option<physics::coupled::Checkpoint>,
    pub coupled_receiver: Option<Receiver<super::coupled::Event>>,
    pub multiphase_config: physics::multiphase::Configuration,
    pub multiphase_receiver: Option<Receiver<super::multiphase_gui::Event>>,
    pub multiphase_checkpoint: Option<physics::multiphase::Checkpoint>,
    pub multiphase_latest: Option<(flursys::mesh::MeshId, Vec<Vec<f64>>, Vec<f64>)>,
    pub multiphase_selected: usize,
    pub species_options: physics::species::SpeciesOptions,
    pub species_initial: f64,
    pub species_latest: Option<(flursys::mesh::MeshId, Vec<f64>)>,
    pub species_checkpoint: Option<physics::species::SpeciesCheckpoint>,
    pub species_receiver: Option<
        Receiver<
            Result<
                (
                    flursys::mesh::MeshId,
                    physics::species::SpeciesCheckpoint,
                    usize,
                    f64,
                    Option<String>,
                    Option<PathBuf>,
                ),
                String,
            >,
        >,
    >,
}
pub enum Event {
    Sample(physics::State, String),
    Done(Result<(PathBuf, Option<physics::checkpoint::Checkpoint>), String>),
}
impl App {
    pub fn transport_properties(&mut self, ui: &mut egui::Ui) {
        heading(ui, "PHASE AND HEAT TRANSPORT");
        ui.label("Prescribed flux / conduction. This does not feed variable density or capillary force back into momentum.");
        egui::ComboBox::from_id_salt("phase-advection-scheme")
            .selected_text(match self.transport.options.phase.scheme {
                physics::phase::AdvectionScheme::Upwind => "Upwind",
                physics::phase::AdvectionScheme::FluxCorrected => "Bounded flux-corrected",
            })
            .show_ui(ui, |ui| {
                ui.selectable_value(
                    &mut self.transport.options.phase.scheme,
                    physics::phase::AdvectionScheme::Upwind,
                    "Upwind",
                );
                ui.selectable_value(
                    &mut self.transport.options.phase.scheme,
                    physics::phase::AdvectionScheme::FluxCorrected,
                    "Bounded flux-corrected",
                );
            });
        number(ui, "Δt (s)", &mut self.transport.options.dt);
        let mut adaptive = self.transport.options.max_transport_courant.is_some();
        if ui
            .checkbox(&mut adaptive, "Limit time step by transport Courant")
            .changed()
        {
            self.transport.options.max_transport_courant = adaptive.then_some(0.5);
        }
        if let Some(limit) = &mut self.transport.options.max_transport_courant {
            number(ui, "Maximum transport Courant", limit);
            ui.small("For phase/heat transport, Δt is an upper limit; each step may advance less physical time.");
        }

        ui.horizontal(|ui| {
            ui.label("Additional physical steps");
            ui.add(egui::DragValue::new(&mut self.transport.options.steps).range(1..=1_000_000));
        });
        ui.horizontal(|ui| {
            ui.label("Write every N steps");
            ui.add(
                egui::DragValue::new(&mut self.transport.options.write_every).range(1..=1_000_000),
            );
        });
        number(
            ui,
            "Initial temperature (K)",
            &mut self.transport.options.initial_temperature,
        );
        ui.add(
            egui::Slider::new(
                &mut self.transport.options.initial_phase_fraction,
                0.0..=1.0,
            )
            .text("Initial phase-one fraction"),
        );
        ui.checkbox(
            &mut self.transport.use_flow,
            "Use current conservative flow flux (Cartesian)",
        );
        use physics::metrics::Coordinates as C;
        egui::ComboBox::from_id_salt("transport-coordinates")
            .selected_text(format!("{:?}", self.transport.options.coordinates))
            .show_ui(ui, |ui| {
                for (c, n) in [
                    (C::Cartesian, "Cartesian"),
                    (
                        C::AxisymmetricX { radial_origin: 0. },
                        "Axisymmetric about X",
                    ),
                    (
                        C::AxisymmetricY { radial_origin: 0. },
                        "Axisymmetric about Y",
                    ),
                ] {
                    ui.selectable_value(&mut self.transport.options.coordinates, c, n);
                }
            });
        let mut use_cyclic = !self.transport.options.phase.cyclic_pairs.is_empty();
        if ui
            .checkbox(&mut use_cyclic, "Use configured cyclic patch pairs")
            .changed()
        {
            if use_cyclic {
                self.transport.options.phase.cyclic_pairs =
                    self.project.solver.cyclic_pairs.clone();
                if self.transport.options.phase.cyclic_pairs.is_empty() {
                    self.message("Configure cyclic patch pairs in Setup first.");
                }
            } else {
                self.transport.options.phase.cyclic_pairs.clear();
            }
        }
        self.transport.species_options.advection.cyclic_pairs =
            self.transport.options.phase.cyclic_pairs.clone();
        for pair in &self.transport.options.phase.cyclic_pairs {
            self.transport
                .species_options
                .advection
                .incoming
                .remove(&pair.first);
            self.transport
                .species_options
                .advection
                .incoming
                .remove(&pair.second);
            self.transport.options.phase.incoming.remove(&pair.first);
            self.transport.options.phase.incoming.remove(&pair.second);
        }
        for patch in self.session.patch_names() {
            let periodic = self
                .transport
                .options
                .phase
                .cyclic_pairs
                .iter()
                .any(|p| p.first == patch || p.second == patch);
            if !periodic {
                self.transport
                    .options
                    .phase
                    .incoming
                    .entry(patch.clone())
                    .or_insert(self.transport.options.initial_phase_fraction);
            }
            self.transport
                .options
                .energy
                .boundaries
                .entry(patch)
                .or_insert(physics::energy::Boundary::Adiabatic);
        }
        for (patch, boundary) in &mut self.transport.options.energy.boundaries {
            egui::CollapsingHeader::new(format!("Heat boundary · {patch}"))
                .id_salt(("transport-bc", patch))
                .show(ui, |ui| widgets::thermal_boundary(ui, patch, boundary));
        }
        let mut value = serde_json::to_value(&self.transport.options).unwrap_or_default();
        for key in ["first", "second", "regions", "phase", "energy"] {
            if let Some(v) = value.get_mut(key) {
                egui::CollapsingHeader::new(widgets::label(key))
                    .id_salt(("transport", key))
                    .show(ui, |ui| widgets::fields(ui, key, v));
            }
        }
        if let Ok(v) = serde_json::from_value(value) {
            self.transport.options = v;
        }
        if ui.button("Save transport configuration").clicked() {
            self.project.scalar_transport = Some(self.transport.options.clone());
            self.editor.scalar_transport = self.project.scalar_transport.clone();
            self.dirty = true;
        }
        if ui.button("Run / Continue transport").clicked() {
            if let Err(e) = self.start_transport() {
                self.message(e);
            }
        }
        if ui.button("Initialize transport").clicked() {
            self.transport.latest = None;
            self.transport.restart = None;
            self.message("Transport fields reset.");
        }
        if ui.button("Load transport checkpoint…").clicked() {
            if let Some(path) = rfd::FileDialog::new()
                .add_filter("Transport checkpoint", &["json"])
                .pick_file()
            {
                match physics::checkpoint::Checkpoint::load(&path) {
                    Ok(c) => self.transport.restart = Some(c),
                    Err(e) => self.message(e),
                }
            }
        }
        ui.separator();
        self.multiphase_properties(ui);
        ui.separator();
        self.coupled_properties(ui);
        ui.separator();
        egui::CollapsingHeader::new("PASSIVE SPECIES")
            .default_open(false)
            .show(ui, |ui| {
                ui.small("Constant-density species on a prescribed flow. Saved projects receive a checkpoint every N steps using the interval above.");
                number(
                    ui,
                    "Diffusivity (m²/s)",
                    &mut self.transport.species_options.diffusivity,
                );
                ui.add(
                    egui::Slider::new(&mut self.transport.species_initial, 0.0..=1.0)
                        .text("Initial mass fraction"),
                );
                for patch in self.session.patch_names() {
                    if self.transport.species_options.advection.cyclic_pairs.iter().any(|p| p.first == patch || p.second == patch) {
                        if self.transport.species_options.fixed_boundary_fraction.contains_key(&patch) { ui.colored_label(egui::Color32::YELLOW, format!("Remove the fixed species boundary on cyclic patch {patch} before running.")); }
                        if self.transport.species_options.fixed_boundary_fraction.contains_key(&patch) && ui.button(format!("Remove fixed boundary · {patch}")).clicked() { self.transport.species_options.fixed_boundary_fraction.remove(&patch); }
                        continue;
                    }
                    let value = self
                        .transport
                        .species_options
                        .advection
                        .incoming
                        .entry(patch.clone())
                        .or_insert(self.transport.species_initial);
                    ui.add(egui::Slider::new(value, 0.0..=1.0).text(format!("Incoming · {patch}")));
                    let mut fixed = self
                        .transport
                        .species_options
                        .fixed_boundary_fraction
                        .contains_key(&patch);
                    if ui
                        .checkbox(&mut fixed, format!("Fixed diffusion · {patch}"))
                        .changed()
                    {
                        if fixed {
                            self.transport
                                .species_options
                                .fixed_boundary_fraction
                                .insert(patch.clone(), self.transport.species_initial);
                        } else {
                            self.transport
                                .species_options
                                .fixed_boundary_fraction
                                .remove(&patch);
                        }
                    }
                    if let Some(value) = self
                        .transport
                        .species_options
                        .fixed_boundary_fraction
                        .get_mut(&patch)
                    {
                        ui.add(egui::Slider::new(value, 0.0..=1.0).text("Boundary fraction"));
                    }
                }
                if ui.button("Apply species settings").clicked() {
                    self.save_species_settings();
                    self.message("Species settings applied; save the project to retain them.");
                }
                if let Some(checkpoint) = &self.transport.species_checkpoint {
                    ui.small(format!("Species step {} · t = {:.6e} s", checkpoint.step, checkpoint.time));
                }
                if ui.button("Initialize species").clicked() {
                    self.transport.species_checkpoint = None;
                    self.transport.species_latest = None;
                    self.message("Species initialized; the next run starts from the configured fraction.");
                }
                if ui.button("Load species checkpoint…").clicked() {
                    if let Some(path) = rfd::FileDialog::new().add_filter("Species checkpoint", &["json"]).pick_file() {
                        match physics::species::SpeciesCheckpoint::load(&path) {
                            Ok(checkpoint) => {
                                if let Some(mesh) = self.session.mesh() {
                                    match checkpoint.validate_mesh(&mesh.mesh) {
                                        Ok(()) => {
                                            self.transport.species_latest = Some((mesh.mesh.id(), checkpoint.fraction.clone()));
                                            self.transport.field = 3;
                                            self.transport.species_checkpoint = Some(checkpoint);
                                            self.message("Species checkpoint loaded; flow and settings will be checked on continue.");
                                        }
                                        Err(e) => self.message(e),
                                    }
                                } else {
                                    self.message("Generate or load the checkpoint mesh first.");
                                }
                            }
                            Err(e) => self.message(e),
                        }
                    }
                }
                if ui.add_enabled(self.transport.species_checkpoint.is_some(), egui::Button::new("Save species checkpoint…")).clicked() {
                    if let Some(path) = rfd::FileDialog::new().set_file_name("species-checkpoint.json").save_file() {
                        let result = self.transport.species_checkpoint.as_ref().unwrap().save(&path);
                        match result { Ok(()) => self.message("Species checkpoint saved."), Err(e) => self.message(e) }
                    }
                }
                if ui.button("Run / Continue species").clicked() {
                    if let Err(e) = self.start_species() {
                        self.message(e);
                    }
                }
            });
        egui::ComboBox::from_id_salt("transport-field")
            .selected_text(
                [
                    "Temperature (K)",
                    "Phase-one fraction",
                    "Density (kg/m³)",
                    "Species fraction",
                    "Coupled phase fraction",
                    "Coupled speed (m/s)",
                ][self.transport.field.min(5)],
            )
            .show_ui(ui, |ui| {
                for (i, n) in [
                    "Temperature (K)",
                    "Phase-one fraction",
                    "Density (kg/m³)",
                    "Species fraction",
                    "Coupled phase fraction",
                    "Coupled speed (m/s)",
                ]
                .iter()
                .enumerate()
                {
                    ui.selectable_value(&mut self.transport.field, i, *n);
                }
            });
    }
    fn save_species_settings(&mut self) {
        self.project.passive_species = Some(physics::species::SpeciesConfiguration {
            initial_fraction: self.transport.species_initial,
            use_flow: self.transport.use_flow,
            options: self.transport.species_options.clone(),
        });
        self.project.scalar_transport = Some(self.transport.options.clone());
        self.editor.passive_species = self.project.passive_species.clone();
        self.editor.scalar_transport = self.project.scalar_transport.clone();
        self.dirty = true;
    }
    fn start_species(&mut self) -> Result<(), String> {
        if self.busy() {
            return Err("Another job is running".into());
        }
        let mesh = self
            .session
            .mesh()
            .ok_or("Generate a mesh first")?
            .mesh
            .clone();
        let coordinates = self.transport.options.coordinates;
        let flux = if self.transport.use_flow {
            if coordinates != physics::metrics::Coordinates::Cartesian {
                return Err(
                    "Flow flux is Cartesian; axisymmetric transport requires revolved flux".into(),
                );
            }
            self.session
                .solution()
                .ok_or("Complete a flow solve first")?
                .face_flux
                .values()
                .to_vec()
        } else {
            vec![0.0; mesh.face_count()]
        };
        let dt = self.transport.options.dt;
        let steps = self.transport.options.steps;
        if !dt.is_finite() || dt <= 0.0 || steps == 0 || !(dt * steps as f64).is_finite() {
            return Err("Invalid species step size or count".into());
        }
        let options = self.transport.species_options.clone();
        let initial = self.transport.species_initial;
        let write_every = self.transport.options.write_every;
        if write_every == 0 {
            return Err("Species checkpoint interval must be positive".into());
        }
        let checkpoint_path = if let Some(workspace) = &self.workspace {
            let root = workspace.join("species");
            std::fs::create_dir_all(&root).map_err(|e| e.to_string())?;
            let stamp = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_err(|e| e.to_string())?
                .as_nanos();
            let folder = root.join(format!("run-{stamp}"));
            std::fs::create_dir(&folder).map_err(|e| e.to_string())?;
            Some(folder.join("checkpoint.json"))
        } else {
            None
        };
        let restart = self.transport.species_checkpoint.clone();
        if let Some(checkpoint) = &restart {
            checkpoint.validate(&mesh, coordinates, &flux, &options)?;
        }
        self.save_species_settings();
        self.transport.paused.store(false, Ordering::Release);
        self.transport.stopped.store(false, Ordering::Release);
        let paused = self.transport.paused.clone();
        let stopped = self.transport.stopped.clone();
        let (tx, rx) = mpsc::channel();
        self.transport.species_receiver = Some(rx);
        std::thread::spawn(move || {
            let result = (|| {
                let measures = coordinates.measures(&mesh)?;
                let pause_control = paused.clone();
                let stop_control = stopped.clone();
                let control: flursys::compute::LinearControl = Arc::new(move || {
                    while pause_control.load(Ordering::Acquire)
                        && !stop_control.load(Ordering::Acquire)
                    {
                        std::thread::sleep(Duration::from_millis(25));
                    }
                    !stop_control.load(Ordering::Acquire)
                });
                let execution = flursys::compute::Execution::controlled(
                    flursys::compute::ComputeOptions::default(),
                    Some(control),
                )
                .map_err(|e| e.to_string())?;
                let _scope = flursys::compute::Scope::enter(execution);
                let initial_step = restart.as_ref().map_or(0, |c| c.step);
                let initial_time = restart.as_ref().map_or(0.0, |c| c.time);
                initial_step
                    .checked_add(steps)
                    .ok_or("Species step overflow")?;
                if !(initial_time + dt * steps as f64).is_finite() {
                    return Err("Species time overflow".into());
                }
                let mut fraction =
                    restart.map_or_else(|| vec![initial; mesh.cell_count()], |c| c.fraction);
                let mut completed = 0;
                let mut balance = 0.0;
                let mut failure = None;
                for _ in 0..steps {
                    while paused.load(Ordering::Acquire) && !stopped.load(Ordering::Acquire) {
                        std::thread::sleep(Duration::from_millis(25));
                    }
                    if stopped.load(Ordering::Acquire) {
                        break;
                    }
                    let report = physics::species::advance(
                        &mesh,
                        &measures,
                        &flux,
                        &mut fraction,
                        dt,
                        &options,
                    );
                    match report {
                        Ok(report) => balance = report.conservation_error,
                        Err(_) if stopped.load(Ordering::Acquire) => break,
                        Err(e) => {
                            failure = Some(e);
                            break;
                        }
                    }
                    completed += 1;
                    if (initial_step + completed) % write_every == 0 {
                        if let Some(path) = &checkpoint_path {
                            let checkpoint = physics::species::SpeciesCheckpoint::capture(
                                &mesh,
                                coordinates,
                                &flux,
                                &options,
                                initial_step + completed,
                                initial_time + dt * completed as f64,
                                fraction.clone(),
                            )?;
                            if let Err(e) = checkpoint.save(path) {
                                failure = Some(format!("Checkpoint save failed: {e}"));
                                break;
                            }
                        }
                    }
                }
                let checkpoint = physics::species::SpeciesCheckpoint::capture(
                    &mesh,
                    coordinates,
                    &flux,
                    &options,
                    initial_step + completed,
                    initial_time + dt * completed as f64,
                    fraction,
                )?;
                if let Some(path) = &checkpoint_path {
                    if let Err(e) = checkpoint.save(path) {
                        let message = format!("Checkpoint save failed: {e}");
                        failure = Some(match failure {
                            Some(previous) => format!("{previous}; {message}"),
                            None => message,
                        });
                    }
                }
                Ok((
                    mesh.id(),
                    checkpoint,
                    completed,
                    balance,
                    failure,
                    checkpoint_path,
                ))
            })();
            let _ = tx.send(result);
        });
        self.message("Passive species transport started.");
        Ok(())
    }
    fn start_transport(&mut self) -> Result<(), String> {
        if self.busy() {
            return Err("Another job is running".into());
        }
        let mesh = self
            .session
            .mesh()
            .ok_or("Generate a mesh first")?
            .mesh
            .clone();
        let path = self.workspace.clone().ok_or("Save the workspace first")?;
        let mut options = self.transport.options.clone();
        if self.transport.use_flow {
            if options.coordinates != physics::metrics::Coordinates::Cartesian {
                return Err("Axisymmetric transport needs explicitly revolved flux; Cartesian flow flux is not rescaled automatically".into());
            }
            let solution = self
                .session
                .solution()
                .ok_or("Complete a flow solve first")?;
            options.face_flux = Some(solution.face_flux.values().to_vec());
            options.mesh_signature = Some(physics::mesh_signature(&mesh));
        } else {
            options.face_flux = None;
            options.mesh_signature = None;
        }
        self.project.scalar_transport = Some(options.clone());
        self.editor.scalar_transport = self.project.scalar_transport.clone();
        self.dirty = true;
        let folder = path.join("transport").join(format!(
            "run-{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        std::fs::create_dir_all(folder.parent().ok_or("No transport folder")?)
            .map_err(|e| e.to_string())?;
        let restart = self.transport.restart.clone();
        self.transport.paused.store(false, Ordering::Release);
        self.transport.stopped.store(false, Ordering::Release);
        let paused = self.transport.paused.clone();
        let stopped = self.transport.stopped.clone();
        let (tx, rx) = mpsc::channel();
        self.transport.receiver = Some(rx);
        std::thread::spawn(move || {
            let result = (|| -> Result<_, String> {
                let mut archive = physics::archive::Archive::create(
                    &folder,
                    &options,
                    &serde_json::json!({"source":"native egui","pressure_momentum_feedback":false}),
                )?;
                let stop = stopped.clone();
                let control: flursys::compute::LinearControl = Arc::new(move || {
                    while paused.load(Ordering::Acquire) && !stop.load(Ordering::Acquire) {
                        std::thread::sleep(Duration::from_millis(25));
                    }
                    !stop.load(Ordering::Acquire)
                });
                let mut tick = Instant::now();
                let solved = physics::run_controlled(
                    &mesh,
                    &options,
                    LinearSolverOptions::default(),
                    restart.as_ref(),
                    Some(control),
                    |s, r| {
                        archive.record(&mesh, s, r)?;
                        if tick.elapsed() > Duration::from_millis(200) || s.step == 1 {
                            let _=tx.send(Event::Sample(s.clone(),format!("Transport step {} · t={:.6e} s · dt={:.3e} s · Co={:.3e} · max cell mass residual {:.3e} kg/(m³ s) · phase mass errors {:.3e}/{:.3e} {} · energy balance {:.3e}",s.step,s.time,r.time_step,r.transport_courant,r.max_cell_mass_residual,r.phase_budgets[0].mass_balance_error,r.phase_budgets[1].mass_balance_error,r.mass_unit,r.energy.balance_error)));
                            tick = Instant::now();
                        }
                        Ok(())
                    },
                );
                let (status, message) = match &solved {
                    Ok(_) => ("completed", "Requested physical steps completed".into()),
                    Err(_) if stopped.load(Ordering::Acquire) => {
                        ("stopped", "Last accepted fields retained".into())
                    }
                    Err(e) => ("failed", e.clone()),
                };
                archive.finish(&mesh, status, &message)?;
                let retained = archive.last_checkpoint().cloned().or(restart);
                if let Some(c) = &retained {
                    let _ = tx.send(Event::Sample(c.state.clone(), message.clone()));
                }
                if status == "failed" {
                    return Err(message);
                }
                Ok((folder, retained))
            })();
            let _ = tx.send(Event::Done(result));
        });
        self.message("Scalar transport started.");
        Ok(())
    }
    pub fn poll_transport(&mut self) {
        if let Some(result) =
            self.transport
                .species_receiver
                .as_ref()
                .and_then(|rx| match rx.try_recv() {
                    Ok(result) => Some(result),
                    Err(mpsc::TryRecvError::Empty) => None,
                    Err(mpsc::TryRecvError::Disconnected) => Some(Err(
                        "Species worker stopped unexpectedly; check the latest saved checkpoint."
                            .into(),
                    )),
                })
        {
            self.transport.species_receiver = None;
            match result {
                Ok((mesh_id, checkpoint, steps, balance, failure, path)) => {
                    self.transport.species_latest = Some((mesh_id, checkpoint.fraction.clone()));
                    self.transport.species_checkpoint = Some(checkpoint);
                    self.transport.field = 3;
                    let path = path
                        .map(|p| format!(" · checkpoint {}", p.display()))
                        .unwrap_or_else(|| {
                            " · save the project before running to enable periodic checkpoints"
                                .into()
                        });
                    let status = failure
                        .map(|e| format!("Stopped after error: {e}"))
                        .unwrap_or_else(|| "Finished or stopped".into());
                    self.message(format!("{status} · species: {steps} accepted steps · last conservation error {balance:.3e}{path}"));
                }
                Err(e) => self.message(e),
            }
        }
        let mut events = Vec::new();
        let mut disconnected = false;
        if let Some(rx) = &self.transport.receiver {
            loop {
                match rx.try_recv() {
                    Ok(event) => events.push(event),
                    Err(mpsc::TryRecvError::Empty) => break,
                    Err(mpsc::TryRecvError::Disconnected) => {
                        disconnected = true;
                        break;
                    }
                }
            }
        }
        for event in events {
            match event {
                Event::Sample(s, message) => {
                    self.transport.latest = Some(s);
                    self.feedback = message;
                }
                Event::Done(result) => {
                    self.transport.receiver = None;
                    match result {
                        Ok((folder, restart)) => {
                            self.transport.folder = Some(folder);
                            self.transport.restart = restart;
                            self.message("Transport archive and checkpoint saved.");
                        }
                        Err(e) => self.message(e),
                    }
                }
            }
        }
        if disconnected && self.transport.receiver.is_some() {
            self.transport.receiver = None;
            self.message("Transport worker stopped unexpectedly; check the saved run folder.");
        }
    }
}
use widgets::{heading, number};
