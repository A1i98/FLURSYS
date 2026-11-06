use super::*;
use flursys::multiphysics::multiphase as model;
use std::sync::{atomic::Ordering, Arc};

pub enum Event {
    Sample {
        mesh_id: MeshId,
        fractions: Vec<Vec<f64>>,
        speed: Vec<f64>,
        step: usize,
        time: f64,
        report: Option<flursys::multiphysics::coupled::Report>,
    },
    Done {
        checkpoint: Option<model::Checkpoint>,
        path: PathBuf,
        error: Option<String>,
    },
}

impl App {
    pub fn multiphase_properties(&mut self, ui: &mut egui::Ui) {
        let config = &self.transport.multiphase_config;
        let count = config.settings.phases.len();
        if count < 2
            || config.initial_fractions.len() != count
            || config.regions.iter().any(|r| r.fractions.len() != count)
        {
            ui.label("Invalid saved multiphase configuration: phase field counts do not match.");
            if ui.button("Reset multiphase configuration").clicked() {
                self.transport.multiphase_config = Default::default();
            }
            return;
        }
        self.transport.multiphase_selected = self.transport.multiphase_selected.min(count - 1);
        let selected_before = self.transport.multiphase_selected;
        egui::CollapsingHeader::new("MULTIPHASE · EXPERIMENTAL").default_open(false).show(ui, |ui| {
            ui.small("Transient, laminar, isothermal Cartesian flow. Fractions in each region must sum to one. Upwind transport; static wall angles. Capillary validation is pending.");
            let wall_names: Vec<String> = self.session.mesh().map(|m| m.mesh.boundary_patches().iter()
                .filter(|p| p.boundary_type == BoundaryType::Wall).map(|p| p.name.clone()).collect()).unwrap_or_default();
            let config = &mut self.transport.multiphase_config;
            for (i, phase) in config.settings.phases.iter_mut().enumerate() {
                egui::CollapsingHeader::new(format!("Phase {} · {}", i+1, phase.name)).id_salt(("nphase",i)).show(ui, |ui| {
                    ui.text_edit_singleline(&mut phase.name);
                    widgets::number(ui, "Density (kg/m³)", &mut phase.density);
                    widgets::number(ui, "Dynamic viscosity (Pa s)", &mut phase.viscosity);
                    widgets::number(ui, "Initial volume fraction", &mut config.initial_fractions[i]);
                });
            }
            ui.horizontal(|ui| {
                if ui.button("Add phase").clicked() {
                    let i=config.settings.phases.len();
                    config.settings.phases.push(model::MaterialPhase { name:format!("phase {}",i+1), density:1.0, viscosity:0.00001 });
                    config.initial_fractions.push(0.0);
                    for r in &mut config.regions { r.fractions.push(0.0); }
                    for fractions in config.settings.incoming.values_mut() { fractions.push(0.0); }
                }
                if ui.add_enabled(config.settings.phases.len()>2, egui::Button::new("Remove last phase")).clicked() {
                    let last=config.settings.phases.len()-1; config.settings.phases.pop(); config.initial_fractions.pop();
                    config.settings.interfaces.retain(|p| p.first != last && p.second != last);
                    for r in &mut config.regions { r.fractions.pop(); }
                    for fractions in config.settings.incoming.values_mut() { fractions.pop(); }
                }
            });
            egui::CollapsingHeader::new("INTERFACES / WETTING").show(ui, |ui| {
                ui.small("One surface tension per phase pair; contact angle is measured through the first phase on each selected wall.");
                let names: Vec<_>=config.settings.phases.iter().map(|p|p.name.clone()).collect();
                let mut remove=None;
                for (i, pair) in config.settings.interfaces.iter_mut().enumerate() {
                    egui::CollapsingHeader::new(format!("Interface {}",i+1)).id_salt(("pair",i)).show(ui, |ui| {
                        ui.horizontal(|ui| { ui.label("First phase"); egui::ComboBox::from_id_salt(("first",i)).selected_text(names.get(pair.first).map(String::as_str).unwrap_or("invalid")).show_ui(ui,|ui|{for (j,name) in names.iter().enumerate(){ui.selectable_value(&mut pair.first,j,name);}}); });
                        ui.horizontal(|ui| { ui.label("Second phase"); egui::ComboBox::from_id_salt(("second",i)).selected_text(names.get(pair.second).map(String::as_str).unwrap_or("invalid")).show_ui(ui,|ui|{for (j,name) in names.iter().enumerate(){ui.selectable_value(&mut pair.second,j,name);}}); });
                        ui.small("Select phases in their listed order; each pair can appear only once.");
                        widgets::number(ui,"Surface tension (N/m)",&mut pair.surface_tension);
                        for wall in &wall_names {
                            let mut enabled=pair.contact_angles.contains_key(wall);
                            if ui.checkbox(&mut enabled,format!("Contact angle · {wall}")).changed() {
                                if enabled { pair.contact_angles.insert(wall.clone(),90.0); } else { pair.contact_angles.remove(wall); }
                            }
                            if let Some(angle)=pair.contact_angles.get_mut(wall) { widgets::number(ui,"Angle (degrees)",angle); }
                        }
                        if ui.button("Clear wall angles").clicked(){pair.contact_angles.clear();}
                        if ui.button("Remove interface").clicked(){remove=Some(i);}
                    });
                }
                if let Some(i)=remove { config.settings.interfaces.remove(i); }
                if ui.button("Add interface").clicked(){ config.settings.interfaces.push(model::Interface { first:0,second:1,surface_tension:0.0,contact_angles:Default::default() }); }
            });
            egui::CollapsingHeader::new("INITIAL REGIONS").show(ui, |ui| {
                let mut remove=None;
                for (i, region) in config.regions.iter_mut().enumerate() {
                    egui::CollapsingHeader::new(format!("Region {}",i+1)).id_salt(("multi-region",i)).show(ui, |ui| {
                        widgets::vector(ui,"Minimum (m)",&mut region.minimum); widgets::vector(ui,"Maximum (m)",&mut region.maximum);
                        for (j, fraction) in region.fractions.iter_mut().enumerate(){ widgets::number(ui,&format!("{} fraction",config.settings.phases[j].name),fraction); }
                        if ui.button("Remove region").clicked(){remove=Some(i);}
                    });
                }
                if let Some(i)=remove {config.regions.remove(i);}
                if ui.button("Add region").clicked(){config.regions.push(model::PhaseRegion { minimum:Vec3::ZERO, maximum:Vec3::new(1.0,1.0,0.0),fractions:config.initial_fractions.clone() });}
            });
            widgets::vector(ui,"Gravity (m/s²)",&mut config.settings.solver.gravity);
            widgets::number(ui,"Time step (s)",&mut config.settings.solver.time_step);
            widgets::number(ui,"Maximum Courant",config.settings.solver.max_courant.get_or_insert(0.5));
            widgets::number(ui,"Minimum time step (s)",&mut config.settings.solver.min_time_step);
            egui::CollapsingHeader::new("Coupling controls").show(ui, |ui| {
                let solver = &mut config.settings.solver;
                ui.horizontal(|ui| { ui.label("Maximum outer corrections"); ui.add(egui::DragValue::new(&mut solver.max_iterations).range(1..=10000)); });
                widgets::number(ui,"Velocity change (m/s)",&mut solver.velocity_tolerance);
                widgets::number(ui,"Flux change (m³/s)",&mut solver.flux_tolerance);
                widgets::number(ui,"Fraction change",&mut solver.phase_tolerance);
                widgets::number(ui,"Pressure change (Pa)",&mut solver.pressure_tolerance);
                widgets::number(ui,"Momentum absolute residual",&mut solver.momentum_absolute_tolerance);
                widgets::number(ui,"Momentum residual reduction",&mut solver.momentum_relative_tolerance);
            });
            ui.horizontal(|ui|{ui.label("Steps");ui.add(egui::DragValue::new(&mut config.steps).range(1..=100000000));});
            ui.horizontal(|ui|{ui.label("Checkpoint every N steps");ui.add(egui::DragValue::new(&mut config.checkpoint_interval).range(1..=100000000));});
            if let Some(c)=&self.transport.multiphase_checkpoint {ui.small(format!("Step {} · t={:.6e} s",c.step,c.time));}
            ui.horizontal(|ui| { ui.label("Displayed phase"); egui::ComboBox::from_id_salt("multiphase-display").selected_text(config.settings.phases.get(self.transport.multiphase_selected).map(|p|p.name.as_str()).unwrap_or("invalid")).show_ui(ui, |ui| {for (i,p) in config.settings.phases.iter().enumerate() {ui.selectable_value(&mut self.transport.multiphase_selected,i,&p.name);}}); });
            if ui.button("Apply multiphase settings").clicked(){self.project.multiphase=Some(config.clone());self.editor.multiphase=self.project.multiphase.clone();self.dirty=true;self.message("Multiphase settings applied; save the project to retain them.");}
            if ui.button("Initialize multiphase").clicked(){self.transport.multiphase_checkpoint=None;self.transport.multiphase_latest=None;self.transport.coupled_latest=None;}
            if ui.button("Load multiphase checkpoint…").clicked(){if let Some(path)=rfd::FileDialog::new().add_filter("Multiphase checkpoint",&["json"]).pick_file(){match model::Checkpoint::load(&path){Ok(c)=>self.transport.multiphase_checkpoint=Some(c),Err(e)=>self.message(e)}}}
            if ui.button("Run / Continue multiphase").clicked(){if let Err(e)=self.start_multiphase(){self.message(e);}}
        });
        if self.transport.multiphase_selected != selected_before {
            self.refresh_multiphase_display();
        }
    }

    fn refresh_multiphase_display(&mut self) {
        if let Some((id, fields, speed)) = &self.transport.multiphase_latest {
            if let Some(alpha) = fields.get(self.transport.multiphase_selected) {
                self.transport.coupled_latest = Some((*id, alpha.clone(), speed.clone()));
                self.transport.coupled_revision = self.transport.coupled_revision.wrapping_add(1);
            }
        }
    }

    fn start_multiphase(&mut self) -> Result<(), String> {
        if self.busy() {
            return Err("Another job is running".into());
        }
        let workspace = self
            .workspace
            .clone()
            .ok_or("Save the project before running multiphase flow")?;
        let case = self.session.prepare_case().map_err(|e| e.to_string())?;
        if case.solver.thermal.enabled
            || case.solver.turbulence.model != flursys::turbulence::TurbulenceModel::Laminar
        {
            return Err("Multiphase supports laminar isothermal flow only".into());
        }
        if case
            .boundaries
            .iter()
            .any(|(_, b)| !matches!(b, IncompressibleBoundaryCondition::NoSlipWall))
        {
            return Err("Multiphase GUI currently supports stationary no-slip walls only".into());
        }
        if !case.solver.boundary_profiles.is_empty()
            || !case.solver.scripting.bindings.is_empty()
            || case.solver.volume_models != Default::default()
        {
            return Err("Scripts, boundary tables and volume models are not supported".into());
        }
        let mut config = self.transport.multiphase_config.clone();
        config.settings.phase.cyclic_pairs = self.transport.options.phase.cyclic_pairs.clone();
        let initial = config.initial_fields(&case.mesh)?;
        let restart = self.transport.multiphase_checkpoint.clone();
        self.project.multiphase = Some(config.clone());
        self.editor.multiphase = self.project.multiphase.clone();
        self.dirty = true;
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_err(|e| e.to_string())?
            .as_nanos();
        let folder = workspace.join("multiphase").join(format!("run-{stamp}"));
        std::fs::create_dir_all(&folder).map_err(|e| e.to_string())?;
        let path = folder.join("checkpoint.json");
        self.transport.paused.store(false, Ordering::Release);
        self.transport.stopped.store(false, Ordering::Release);
        let paused = self.transport.paused.clone();
        let stopped = self.transport.stopped.clone();
        let (tx, rx) = mpsc::channel();
        self.transport.multiphase_receiver = Some(rx);
        self.transport.coupled_latest = None;
        self.transport.field = 4;
        std::thread::spawn(move || {
            let result = (|| -> Result<(model::Checkpoint, Option<String>), String> {
                let mesh = &case.mesh;
                let resolved = case.resolve_boundaries().map_err(|e| format!("{e:?}"))?;
                let stencil = LeastSquaresGradientStencil::new(mesh, resolved.pressure_boundary())
                    .map_err(|e| format!("{e:?}"))?;
                let mut simple = SimpleOptions::steady(
                    Diffusivity::Constant(config.settings.phases[0].viscosity),
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
                let mut state = if let Some(c) = restart {
                    c.restore(mesh, &config.settings, simple)?
                } else {
                    model::State {
                        flow: SimpleState::new(
                            mesh,
                            CellField::filled(mesh, Vec3::ZERO),
                            CellField::filled(mesh, 0.0),
                            FaceField::filled(mesh, 0.0),
                        )
                        .map_err(|e| e.to_string())?,
                        fractions: initial,
                        time: 0.0,
                        step: 0,
                    }
                };
                let pc = paused.clone();
                let sc = stopped.clone();
                let control: flursys::compute::LinearControl = Arc::new(move || {
                    while pc.load(Ordering::Acquire) && !sc.load(Ordering::Acquire) {
                        std::thread::sleep(Duration::from_millis(25));
                    }
                    !sc.load(Ordering::Acquire)
                });
                let execution = flursys::compute::Execution::controlled(
                    flursys::compute::ComputeOptions::default(),
                    Some(control),
                )
                .map_err(|e| e.to_string())?;
                let _scope = flursys::compute::Scope::enter(execution);
                let mut failure = None;
                let mut last_report = None;
                let mut last_sample = Instant::now();
                for _ in 0..config.steps {
                    if stopped.load(Ordering::Acquire) {
                        break;
                    }
                    let report = match model::advance(mesh, &mut state, &config.settings, simple) {
                        Ok(r) => r,
                        Err(_) if stopped.load(Ordering::Acquire) => break,
                        Err(e) => {
                            failure = Some(e);
                            break;
                        }
                    };
                    if state.step % config.checkpoint_interval == 0 {
                        let c = model::Checkpoint::capture(mesh, &state, &config.settings, simple)?;
                        if let Err(e) = c.save(&path) {
                            failure = Some(e);
                            break;
                        }
                    }
                    if last_sample.elapsed() > Duration::from_millis(250) || state.step == 1 {
                        let _ = tx.send(Event::Sample {
                            mesh_id: mesh.id(),
                            fractions: state.fractions.clone(),
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
                let _ = tx.send(Event::Sample {
                    mesh_id: mesh.id(),
                    fractions: state.fractions.clone(),
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
                let c = model::Checkpoint::capture(mesh, &state, &config.settings, simple)?;
                c.save(&path)?;
                Ok((c, failure))
            })();
            let (checkpoint, error) = match result {
                Ok((c, e)) => (Some(c), e),
                Err(e) => (None, Some(e)),
            };
            let _ = tx.send(Event::Done {
                checkpoint,
                path,
                error,
            });
        });
        self.message("Experimental multiphase flow started.");
        Ok(())
    }
    pub fn poll_multiphase(&mut self) {
        let mut events = Vec::new();
        let mut disconnected = false;
        if let Some(rx) = &self.transport.multiphase_receiver {
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
                    fractions,
                    speed,
                    step,
                    time,
                    report,
                } => {
                    self.transport.multiphase_latest = Some((mesh_id, fractions, speed));
                    self.refresh_multiphase_display();
                    self.feedback = if let Some(r) = report {
                        format!("Multiphase step {step} · t={time:.6e} s · Δt={:.3e} · Co={:.3e} · continuity={:.3e} · max phase balance={:.3e}",r.time_step,r.max_courant,r.continuity_rms,r.phase_balance)
                    } else {
                        format!("Multiphase step {step} · t={time:.6e} s")
                    };
                }
                Event::Done {
                    checkpoint,
                    path,
                    error,
                } => {
                    self.transport.multiphase_receiver = None;
                    if checkpoint.is_some() {
                        self.transport.multiphase_checkpoint = checkpoint;
                    }
                    let suffix = if path.is_file() {
                        format!(" · checkpoint {}", path.display())
                    } else {
                        String::new()
                    };
                    if let Some(e) = error {
                        self.message(format!("Multiphase stopped: {e}{suffix}"));
                    } else {
                        self.message(format!("Multiphase finished or stopped{suffix}"));
                    }
                }
            }
        }
        if disconnected && self.transport.multiphase_receiver.is_some() {
            self.transport.multiphase_receiver = None;
            self.message("Multiphase worker stopped unexpectedly; check the run folder.");
        }
    }
}
