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
    pub species_options: physics::species::SpeciesOptions,
    pub species_initial: f64,
    pub species_latest: Option<(flursys::mesh::MeshId, Vec<f64>)>,
    pub species_receiver:
        Option<Receiver<Result<(flursys::mesh::MeshId, Vec<f64>, usize, f64), String>>>,
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
        for patch in self.session.patch_names() {
            self.transport
                .options
                .phase
                .incoming
                .entry(patch.clone())
                .or_insert(self.transport.options.initial_phase_fraction);
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
        egui::CollapsingHeader::new("PASSIVE SPECIES")
            .default_open(false)
            .show(ui, |ui| {
                ui.small("Constant-density species on a prescribed flow.");
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
                if ui.button("Run passive species").clicked() {
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
                ][self.transport.field.min(3)],
            )
            .show_ui(ui, |ui| {
                for (i, n) in [
                    "Temperature (K)",
                    "Phase-one fraction",
                    "Density (kg/m³)",
                    "Species fraction",
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
                let mut fraction = vec![initial; mesh.cell_count()];
                let mut completed = 0;
                let mut balance = 0.0;
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
                        Err(e) => return Err(e),
                    }
                    completed += 1;
                }
                Ok((mesh.id(), fraction, completed, balance))
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
        if let Some(result) = self
            .transport
            .species_receiver
            .as_ref()
            .and_then(|rx| rx.try_recv().ok())
        {
            self.transport.species_receiver = None;
            match result {
                Ok((mesh_id, fraction, steps, balance)) => {
                    self.transport.species_latest = Some((mesh_id, fraction));
                    self.transport.field = 3;
                    self.message(format!(
                        "Species: {steps} steps · last conservation error {balance:.3e}"
                    ));
                }
                Err(e) => self.message(e),
            }
        }
        let mut events = Vec::new();
        if let Some(rx) = &self.transport.receiver {
            events.extend(rx.try_iter());
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
    }
}
use widgets::{heading, number};
