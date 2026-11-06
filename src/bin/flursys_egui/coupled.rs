use super::*;
use flursys::multiphysics as physics;
use std::sync::{atomic::Ordering, Arc};

pub enum Event {
    Sample {
        mesh_id: flursys::MeshId,
        alpha: Vec<f64>,
        speed: Vec<f64>,
        step: usize,
        time: f64,
        report: Option<physics::coupled::Report>,
    },
    Done {
        checkpoint: Option<physics::coupled::Checkpoint>,
        path: PathBuf,
        error: Option<String>,
    },
}

impl App {
    pub fn coupled_properties(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("COUPLED TWO-PHASE FLOW · EXPERIMENTAL").default_open(false).show(ui, |ui| {
            ui.small("Transient, laminar, Cartesian, stationary walls; surface tension is experimental. No thermal coupling. CPU serial. Pressure is in Pa.");
            ui.small("Initial phase fraction and Regions are configured above. Save the project before running.");
            let settings = &mut self.transport.coupled_settings;
            widgets::number(ui, "Phase 1 density (kg/m³)", &mut settings.first_density);
            widgets::number(ui, "Phase 2 density (kg/m³)", &mut settings.second_density);
            widgets::number(ui, "Phase 1 dynamic viscosity (Pa s)", &mut settings.first_viscosity);
            widgets::number(ui, "Phase 2 dynamic viscosity (Pa s)", &mut settings.second_viscosity);
            widgets::vector(ui, "Gravity (m/s²)", &mut settings.gravity);
            widgets::number(ui, "Surface tension (N/m; 0 disables)", &mut settings.surface_tension);
            let mut wetting = settings.contact_angle_degrees.is_some();
            if ui.checkbox(&mut wetting, "Static contact angle on all walls").changed() {
                settings.contact_angle_degrees = wetting.then_some(90.0);
            }
            if let Some(angle) = &mut settings.contact_angle_degrees {
                widgets::number(ui, "Contact angle through phase 1 (degrees)", angle);
            }
            ui.small("Surface tension uses a continuum-force approximation; capillary reference validation is pending.");
            widgets::number(ui, "Maximum coupled Courant", settings.max_courant.get_or_insert(0.5));
            widgets::number(ui, "Minimum Δt (s)", &mut settings.min_time_step);
            ui.horizontal(|ui| { ui.label("Maximum outer corrections"); ui.add(egui::DragValue::new(&mut settings.max_iterations).range(1..=10000)); });
            egui::CollapsingHeader::new("Coupling tolerances").show(ui, |ui| {
                widgets::number(ui, "Velocity change (m/s)", &mut settings.velocity_tolerance);
                widgets::number(ui, "Face flux change (m³/s)", &mut settings.flux_tolerance);
                widgets::number(ui, "Phase fraction change", &mut settings.phase_tolerance);
                widgets::number(ui, "Pressure change (Pa)", &mut settings.pressure_tolerance);
                widgets::number(ui, "Momentum absolute residual", &mut settings.momentum_absolute_tolerance);
                widgets::number(ui, "Momentum residual reduction", &mut settings.momentum_relative_tolerance);
            });
            if let Some(checkpoint)=&self.transport.coupled_checkpoint { ui.small(format!("Step {} · t={:.6e} s",checkpoint.step,checkpoint.time)); }
            if ui.button("Apply coupled settings").clicked() { self.save_coupled_settings(); self.message("Coupled settings applied; save the project to retain them."); }
            if ui.button("Initialize coupled flow").clicked() { self.transport.coupled_checkpoint=None; self.transport.coupled_latest=None; self.message("Coupled fields reset."); }
            if ui.button("Load coupled checkpoint…").clicked() {
                if let Some(path)=rfd::FileDialog::new().add_filter("Coupled checkpoint", &["json"]).pick_file() {
                    match physics::coupled::Checkpoint::load(&path) { Ok(c)=>{self.transport.coupled_checkpoint=Some(c);self.message("Checkpoint loaded; mesh, phase properties and boundaries will be checked before continuation.");}, Err(e)=>self.message(e) }
                }
            }
            if ui.button("Run / Continue coupled flow").clicked() { if let Err(e)=self.start_coupled(){self.message(e);} }
        });
    }
    fn save_coupled_settings(&mut self) {
        let mut settings = self.transport.coupled_settings.clone();
        settings.time_step = self.transport.options.dt;
        settings.phase = self.transport.options.phase.clone();
        self.project.coupled_two_phase = Some(settings.clone());
        self.project.scalar_transport = Some(self.transport.options.clone());
        self.editor.coupled_two_phase = Some(settings);
        self.editor.scalar_transport = self.project.scalar_transport.clone();
        self.dirty = true;
    }
    fn start_coupled(&mut self) -> Result<(), String> {
        if self.busy() {
            return Err("Another job is running".into());
        }
        let workspace = self
            .workspace
            .clone()
            .ok_or("Save the project before running coupled flow")?;
        let case = self.session.prepare_case().map_err(|e| e.to_string())?;
        if case.solver.thermal.enabled
            || case.solver.turbulence.model != flursys::turbulence::TurbulenceModel::Laminar
        {
            return Err("Coupled flow currently supports laminar isothermal flow only".into());
        }
        if case
            .boundaries
            .iter()
            .any(|(_, b)| !matches!(b, IncompressibleBoundaryCondition::NoSlipWall))
        {
            return Err("Coupled flow currently supports stationary no-slip walls only".into());
        }
        if !case.solver.boundary_profiles.is_empty()
            || !case.solver.scripting.bindings.is_empty()
            || case.solver.volume_models != Default::default()
        {
            return Err(
                "Coupled flow does not yet support scripts, boundary tables or volume models"
                    .into(),
            );
        }
        if self
            .transport
            .options
            .regions
            .iter()
            .any(|r| r.solid.is_some())
        {
            return Err("Solid regions are not supported by coupled flow".into());
        }
        let initial_fraction = self.transport.options.initial_phase_fraction;
        if !initial_fraction.is_finite() || !(0.0..=1.0).contains(&initial_fraction) {
            return Err("Initial phase fraction must be in [0,1]".into());
        }
        let regions = self.transport.options.regions.clone();
        for region in &regions {
            let a = region.minimum;
            let b = region.maximum;
            if [a.x, a.y, a.z, b.x, b.y, b.z, region.phase_fraction]
                .iter()
                .any(|value| !value.is_finite())
                || a.x > b.x
                || a.y > b.y
                || a.z > b.z
                || !(0.0..=1.0).contains(&region.phase_fraction)
            {
                return Err("Invalid phase region bounds or fraction".into());
            }
        }
        let steps = self.transport.options.steps;
        let every = self.transport.options.write_every;
        if steps == 0 || every == 0 {
            return Err("Steps and checkpoint interval must be positive".into());
        }
        let mut settings = self.transport.coupled_settings.clone();
        settings.time_step = self.transport.options.dt;
        settings.phase = self.transport.options.phase.clone();
        let restart = self.transport.coupled_checkpoint.clone();
        self.save_coupled_settings();
        let root = workspace.join("coupled-two-phase");
        std::fs::create_dir_all(&root).map_err(|e| e.to_string())?;
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_err(|e| e.to_string())?
            .as_nanos();
        let folder = root.join(format!("run-{stamp}"));
        std::fs::create_dir(&folder).map_err(|e| e.to_string())?;
        let checkpoint_path = folder.join("checkpoint.json");
        self.transport.paused.store(false, Ordering::Release);
        self.transport.stopped.store(false, Ordering::Release);
        let paused = self.transport.paused.clone();
        let stopped = self.transport.stopped.clone();
        let (tx, rx) = mpsc::channel();
        self.transport.coupled_receiver = Some(rx);
        self.transport.multiphase_latest = None;
        self.transport.field = 4;
        std::thread::spawn(move || {
            let result = (|| -> Result<(physics::coupled::State, Option<String>, Option<physics::coupled::Report>), String> {
                let mesh = &case.mesh;
                let resolved = case.resolve_boundaries().map_err(|e| format!("{e:?}"))?;
                let stencil = LeastSquaresGradientStencil::new(mesh, resolved.pressure_boundary())
                    .map_err(|e| format!("{e:?}"))?;
                let mut simple = SimpleOptions::steady(
                    Diffusivity::Constant(settings.first_viscosity),
                    resolved.velocity_boundary(),
                    &stencil,
                );
                simple.pressure_boundary = Some(resolved.pressure_boundary());
                if mesh.faces().iter().enumerate().any(|(i, _)| {
                    resolved
                        .pressure_correction_boundary()
                        .cyclic_link(i)
                        .is_some()
                }) {
                    simple.pressure_correction_boundary =
                        Some(resolved.pressure_correction_boundary());
                }
                simple.pressure_coupling = case.solver.pressure_coupling;
                simple.pressure_solver = case.solver.pressure_solver;
                simple.momentum_solver = case.solver.momentum_solver;
                simple.velocity_relaxation = case.solver.velocity_relaxation;
                simple.pressure_relaxation = case.solver.pressure_relaxation;
                let mut state = if let Some(restart) = restart {
                    restart.restore(mesh, &settings, simple)?
                } else {
                    let mut alpha = vec![initial_fraction; mesh.cell_count()];
                    for (i, cell) in mesh.cells().iter().enumerate() {
                        let mut seen = false;
                        for r in &regions {
                            let p = cell.center;
                            if p.x >= r.minimum.x
                                && p.x <= r.maximum.x
                                && p.y >= r.minimum.y
                                && p.y <= r.maximum.y
                                && p.z >= r.minimum.z
                                && p.z <= r.maximum.z
                            {
                                if seen {
                                    return Err(format!("Overlapping phase regions at cell {i}"));
                                }
                                if !r.phase_fraction.is_finite()
                                    || !(0.0..=1.0).contains(&r.phase_fraction)
                                {
                                    return Err("Invalid region phase fraction".into());
                                }
                                alpha[i] = r.phase_fraction;
                                seen = true;
                            }
                        }
                    }
                    physics::coupled::State {
                        flow: SimpleState::new(
                            mesh,
                            CellField::filled(mesh, Vec3::ZERO),
                            CellField::filled(mesh, 0.0),
                            FaceField::filled(mesh, 0.0),
                        )
                        .map_err(|e| e.to_string())?,
                        phase_fraction: alpha,
                        time: 0.0,
                        step: 0,
                    }
                };
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
                let mut last_sample = Instant::now();
                let mut failure = None;
                let mut last_report = None;
                for _ in 0..steps {
                    if stopped.load(Ordering::Acquire) {
                        break;
                    }
                    let report =
                        match physics::coupled::advance(mesh, &mut state, &settings, simple) {
                            Ok(r) => r,
                            Err(_) if stopped.load(Ordering::Acquire) => break,
                            Err(e) => {
                                failure = Some(e);
                                break;
                            }
                        };
                    if state.step % every == 0 {
                        let checkpoint =
                            physics::coupled::Checkpoint::capture(mesh, &state, &settings, simple)?;
                        if let Err(e) = checkpoint.save(&checkpoint_path) {
                            failure = Some(format!("Checkpoint save failed: {e}"));
                            break;
                        }
                    }
                    if last_sample.elapsed() > Duration::from_millis(250) || state.step == 1 {
                        let _ = tx.send(Event::Sample {
                            mesh_id: mesh.id(),
                            alpha: state.phase_fraction.clone(),
                            speed: state
                                .flow
                                .velocity
                                .values()
                                .iter()
                                .map(|v| v.norm())
                                .collect(),
                            step: state.step,
                            time: state.time,
                            report: Some(report.clone()),
                        });
                        last_sample = Instant::now();
                    }
                    last_report = Some(report);
                }
                Ok((state, failure, last_report))
            })();
            let (checkpoint, error) = match result {
                Ok((state, mut failure, last_report)) => {
                    let _ = tx.send(Event::Sample {
                        mesh_id: case.mesh.id(),
                        alpha: state.phase_fraction.clone(),
                        speed: state
                            .flow
                            .velocity
                            .values()
                            .iter()
                            .map(|v| v.norm())
                            .collect(),
                        step: state.step,
                        time: state.time,
                        report: last_report,
                    });
                    let checkpoint = (|| -> Result<_, String> {
                        let resolved = case.resolve_boundaries().map_err(|e| format!("{e:?}"))?;
                        let stencil = LeastSquaresGradientStencil::new(
                            &case.mesh,
                            resolved.pressure_boundary(),
                        )
                        .map_err(|e| format!("{e:?}"))?;
                        let mut simple = SimpleOptions::steady(
                            Diffusivity::Constant(settings.first_viscosity),
                            resolved.velocity_boundary(),
                            &stencil,
                        );
                        simple.pressure_boundary = Some(resolved.pressure_boundary());
                        if case.mesh.faces().iter().enumerate().any(|(i, _)| {
                            resolved
                                .pressure_correction_boundary()
                                .cyclic_link(i)
                                .is_some()
                        }) {
                            simple.pressure_correction_boundary =
                                Some(resolved.pressure_correction_boundary());
                        }
                        let c = physics::coupled::Checkpoint::capture(
                            &case.mesh, &state, &settings, simple,
                        )?;
                        c.save(&checkpoint_path)?;
                        Ok(c)
                    })();
                    match checkpoint {
                        Ok(c) => (Some(c), failure),
                        Err(e) => {
                            failure = Some(match failure {
                                Some(previous) => format!("{previous}; checkpoint: {e}"),
                                None => format!("Checkpoint: {e}"),
                            });
                            (None, failure)
                        }
                    }
                }
                Err(e) => (None, Some(e)),
            };
            let _ = tx.send(Event::Done {
                checkpoint,
                path: checkpoint_path,
                error,
            });
        });
        self.message("Experimental coupled flow started.");
        Ok(())
    }
    pub fn poll_coupled(&mut self) {
        let mut events = Vec::new();
        let mut disconnected = false;
        if let Some(rx) = &self.transport.coupled_receiver {
            loop {
                match rx.try_recv() {
                    Ok(e) => events.push(e),
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
                Event::Sample {
                    mesh_id,
                    alpha,
                    speed,
                    step,
                    time,
                    report,
                } => {
                    self.transport.coupled_latest = Some((mesh_id, alpha, speed));
                    self.transport.coupled_revision =
                        self.transport.coupled_revision.wrapping_add(1);
                    self.feedback = if let Some(report) = report {
                        format!("Coupled step {step} · t={time:.6e} s · Δt={:.3e} s · Co={:.3e} · continuity={:.3e} · phase balance={:.3e}", report.time_step, report.max_courant, report.continuity_rms, report.phase_balance)
                    } else {
                        format!("Coupled step {step} · t={time:.6e} s")
                    };
                }
                Event::Done {
                    checkpoint,
                    path,
                    error,
                } => {
                    self.transport.coupled_receiver = None;
                    self.transport.coupled_checkpoint = checkpoint;
                    let checkpoint_status = if path.is_file() {
                        format!(" · checkpoint {}", path.display())
                    } else {
                        String::new()
                    };
                    if let Some(e) = error {
                        self.message(format!("Coupled flow stopped: {e}{checkpoint_status}"));
                    } else {
                        self.message(format!(
                            "Coupled flow finished or stopped{checkpoint_status}"
                        ));
                    }
                }
            }
        }
        if disconnected && self.transport.coupled_receiver.is_some() {
            self.transport.coupled_receiver = None;
            self.message("Coupled worker stopped unexpectedly; check the saved run folder.");
        }
    }
}
