use super::widgets::{heading, number, vector};
use super::*;
use flursys::compressible as gas;
#[derive(Clone, Serialize, Deserialize)]
struct Saved {
    #[serde(default)]
    mesh: Option<flursys::mesh::MeshSnapshot>,
    #[serde(default)]
    run_id: String,
    #[serde(default)]
    cell_indices: Vec<usize>,
    #[serde(default)]
    accepted_history: Vec<(usize, f64)>,
    points: Vec<(f64, gas::Primitive)>,
    report: gas::Report,
    gas: gas::IdealGas,
    #[serde(default)]
    sod_reference: bool,
    #[serde(default)]
    polygons: Vec<Vec<[f64; 2]>>,
    #[serde(default)]
    contours_from_current_mesh: bool,
}
impl Saved {
    fn from_state(
        mesh: &UnstructuredMesh,
        state: &gas::Snapshot,
        settings: Option<&gas::Settings>,
    ) -> Self {
        let mut indices: Vec<_> = (0..mesh.cell_count()).collect();
        indices.sort_by(|&a, &b| {
            mesh.cells()[a]
                .center
                .x
                .total_cmp(&mesh.cells()[b].center.x)
        });
        let points = indices
            .iter()
            .map(|&i| (mesh.cells()[i].center.x, state.fields[i]))
            .collect();
        Self {
            mesh: Some(flursys::mesh::MeshSnapshot::capture(mesh)),
            run_id: "current".into(),
            cell_indices: indices,
            accepted_history: Vec::new(),
            points,
            report: state.report.clone(),
            gas: state.gas,
            sod_reference: settings.is_some_and(|s| gas::benchmarks::is_standard_sod(mesh, s)),
            polygons: Vec::new(),
            contours_from_current_mesh: false,
        }
    }
    fn dataset(&self) -> Result<ResultDataset, String> {
        let (mesh, indices) = if let Some(snapshot) = &self.mesh {
            (snapshot.clone().restore()?, self.cell_indices.clone())
        } else {
            if self.polygons.len() != self.points.len() || self.polygons.iter().any(|p| p.len() < 3)
            {
                return Err("This older archive has no valid mesh snapshot or 2D contour topology; its profiles remain available in Plots".into());
            }
            let mut nodes = std::collections::BTreeMap::new();
            let mut points = Vec::new();
            let mut cells = Vec::new();
            for polygon in &self.polygons {
                let mut vertices = Vec::new();
                for &[x, y] in polygon {
                    let key = (
                        if x == 0. { 0 } else { x.to_bits() },
                        if y == 0. { 0 } else { y.to_bits() },
                    );
                    let index = *nodes.entry(key).or_insert_with(|| {
                        points.push(Point::new(x, y, 0.));
                        points.len() - 1
                    });
                    vertices.push(index);
                }
                if vertices.first() == vertices.last() {
                    vertices.pop();
                }
                cells.push(CellDefinition::polygon(vertices));
            }
            let mesh = UnstructuredMesh::from_cells(MeshDimension::TwoD, points, cells)
                .map_err(|e| e.to_string())?;
            let indices = (0..mesh.cell_count()).collect();
            (mesh, indices)
        };
        if indices.len() != mesh.cell_count() || self.points.len() != mesh.cell_count() {
            return Err("Compressible archive field addressing does not match its mesh".into());
        }
        let mut fields = vec![
            gas::Primitive {
                density: 1.,
                pressure: 1.,
                velocity: Vec3::ZERO
            };
            mesh.cell_count()
        ];
        let mut seen = vec![false; mesh.cell_count()];
        for (&cell, (_, value)) in indices.iter().zip(&self.points) {
            if cell >= fields.len() || seen[cell] {
                return Err("Invalid compressible cell addressing".into());
            }
            seen[cell] = true;
            fields[cell] = *value;
        }
        ResultDataset::from_compressible(
            if self.run_id.is_empty() {
                "compressible"
            } else {
                &self.run_id
            },
            &mesh,
            &fields,
            self.gas,
        )
        .map_err(|e| e.to_string())
    }
}
#[derive(Default)]
pub(super) struct State {
    saved: Option<Saved>,
    step: usize,
    time: f64,
    quantity: usize,
    profile_quantity: Option<usize>,
    profiles: Vec<workbench::series_plot::Series>,
    history: Vec<(usize, f64)>,
    archives: Option<(PathBuf, Vec<PathBuf>)>,
}
impl State {
    pub(super) fn set_defaults(&mut self, field: &str, _mode: &str) {
        self.quantity = match field {
            "absolute-pressure" => 1,
            "velocity-x" => 2,
            "temperature" => 3,
            "mach" => 4,
            _ => 0,
        };
    }
}
impl App {
    pub(super) fn load_compressible_archive(&mut self, path: &Path) -> Result<(), String> {
        let saved: Saved = serde_json::from_slice(&std::fs::read(path).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
        self.gas_flow.step = saved.report.accepted_steps;
        self.gas_flow.time = saved.report.time;
        self.gas_flow.history = saved.accepted_history.clone();
        self.gas_flow.saved = Some(saved);
        self.gas_flow.profile_quantity = None;
        self.activate_compressible_result()?;
        self.result_run = self
            .result
            .as_ref()
            .map(|d| d.run_id().to_string())
            .unwrap_or_default();
        Ok(())
    }
    pub(super) fn compressible_archives(&mut self, ui: &mut egui::Ui) {
        if let Some(root) = self.workspace.as_ref().map(|p| p.join("compressible")) {
            if ui.button("Refresh runs").clicked() {
                self.gas_flow.archives = None;
            }
            if self
                .gas_flow
                .archives
                .as_ref()
                .is_none_or(|(saved, _)| saved != &root)
            {
                let mut files: Vec<_> = std::fs::read_dir(&root)
                    .into_iter()
                    .flatten()
                    .flatten()
                    .map(|e| e.path().join("profiles.json"))
                    .filter(|p| p.is_file())
                    .collect();
                files.sort();
                files.reverse();
                self.gas_flow.archives = Some((root, files));
            }
            let files = self
                .gas_flow
                .archives
                .as_ref()
                .map(|(_, files)| files.clone())
                .unwrap_or_default();
            for path in files {
                let name = path
                    .parent()
                    .and_then(|p| p.file_name())
                    .unwrap_or_default()
                    .to_string_lossy();
                if ui.button(name).clicked() {
                    if let Err(e) = self.load_compressible_archive(&path) {
                        self.message(e);
                    } else {
                        self.stage = Stage::Results;
                        self.page = 0;
                    }
                }
            }
        }
    }
    pub(super) fn activate_compressible_result(&mut self) -> Result<(), String> {
        let saved = self
            .gas_flow
            .saved
            .as_ref()
            .ok_or("No compressible results loaded")?;
        let keep_field = self.result.is_some() && self.result_field != ResultFieldKind::Pressure;
        self.result = Some(saved.dataset()?);
        if !keep_field {
            self.result_field = match self.gas_flow.quantity {
                1 => ResultFieldKind::AbsolutePressure,
                2 => ResultFieldKind::VelocityX,
                3 => ResultFieldKind::Temperature,
                4 => ResultFieldKind::Mach,
                _ => ResultFieldKind::Density,
            };
        }
        self.probes.clear();
        self.selected_probe = None;
        self.result_cache = Some(
            ResultRenderCache::build(self.result.as_ref().ok_or("No result dataset")?)
                .map_err(|e| e.to_string())?,
        );
        self.plane_index = 0;
        self.streamlines.clear();
        Ok(())
    }
    pub(super) fn finish_compressible(&mut self, result: Result<gas::Solution, String>) {
        let failure = result.as_ref().err().cloned();
        let Some(mesh) = self.session.mesh().map(|m| m.mesh.clone()) else {
            return;
        };
        let state = result
            .as_ref()
            .ok()
            .map(|s| {
                std::sync::Arc::new(
                    s.snapshot(self.project.solver.compressible.as_ref().unwrap().gas),
                )
            })
            .or_else(|| self.jobs.retained_compressible(mesh.id()));
        let Some(state) = state else {
            self.message(failure.unwrap_or("No accepted fields".into()));
            return;
        };
        self.iteration = state.report.accepted_steps;
        self.gas_flow.step = self.iteration;
        self.gas_flow.time = state.report.time;
        self.gas_flow.saved = Some(Saved::from_state(
            &mesh,
            &state,
            self.project.solver.compressible.as_ref(),
        ));
        self.gas_flow.profile_quantity = None;
        self.result = state.dataset("current", &mesh).ok();
        self.result_cache = self
            .result
            .as_ref()
            .and_then(|d| ResultRenderCache::build(d).ok());
        if let Some((workspace, recording)) =
            self.workspace.clone().zip(self.solve_recording.clone())
        {
            match workbench::flow_frame::finalize_compressible(
                &workspace,
                &mut self.project,
                &mesh,
                &state,
                &recording,
                self.continuation_run.as_deref(),
                &self.solve_histories,
                failure.clone(),
            ) {
                Ok(run) => {
                    self.continuation_run = Some(run.id.clone());
                    self.result_run = run.id.clone();
                    self.editor.runs = self.project.runs.clone();
                    self.message(format!("Run {} saved: {:?}", run.id, run.status));
                }
                Err(e) => self.message(e),
            }
        }
        self.session.complete_compressible(result);
        self.histories = self.solve_histories.clone();
        self.refresh_field_frames();
        if let Err(e) = self.persist_continuation() {
            self.message(e);
        }
        if let Some(e) = failure {
            self.show_log = true;
            self.message(format!("Solve failed; last accepted fields retained: {e}"));
        }
    }
    pub(super) fn compressible_initialization(&mut self, ctx: &egui::Context) {
        let mut open = true;
        egui::Window::new("Initialize solution").open(&mut open).frame(widgets::dialog_frame(ctx)).show(ctx,|ui|{
            if let Some(settings)=self.editor.solver.compressible.as_mut(){primitive_editor(ui,&mut settings.initial);}
            if ui.add_enabled(!self.busy(),egui::Button::new("Initialize fields")).clicked() {
                let result=(|| -> Result<(),String> {
                    self.apply_editor()?;
                    let settings=self.project.solver.compressible.as_ref().ok_or("No flow settings")?;
                    let mesh=self.session.mesh().ok_or("Generate a mesh first")?.mesh.clone();
                    let state=gas::initialize(&mesh,settings)?;
                    self.reset_solve_session();self.jobs.set_compressible_state(&mesh,state.clone())?;
                    self.result=Some(state.dataset("initialized",&mesh)?);
                    self.result_cache=self.result.as_ref().and_then(|d|ResultRenderCache::build(d).ok());
                    self.gas_flow=State::default();self.initialized=true;self.show_initialization=false;
                    self.persist_continuation()?;
                    if let Some(path)=&self.workspace{workbench::save_workspace(path,&self.project).map_err(|e|e.to_string())?;}
                    Ok(())
                })();
                match result{Ok(())=>self.message("Initialized; physical time and step numbering reset. Archived runs retained."),Err(e)=>self.message(e)}
            }
        });
        if !open {
            self.show_initialization = false;
        }
    }
    pub(super) fn restore_gas_frame(
        &mut self,
        path: &Path,
        continuation: Option<String>,
    ) -> Result<(), String> {
        let mesh = self
            .session
            .mesh()
            .ok_or("Open the matching mesh")?
            .mesh
            .clone();
        let gas = self
            .project
            .solver
            .compressible
            .as_ref()
            .ok_or("Choose compressible flow before loading this state")?
            .gas;
        let frame = workbench::flow_frame::FlowFrame::read(path)?;
        let state = frame.gas_restart(&mesh, gas)?;
        self.jobs.set_compressible_state(&mesh, state.clone())?;
        self.iteration = frame.step;
        self.result_iteration = frame.step;
        self.continuation_run = continuation;
        self.gas_flow.saved = Some(Saved::from_state(
            &mesh,
            &state,
            self.project.solver.compressible.as_ref(),
        ));
        self.result = Some(frame.dataset_on(&mesh)?);
        self.result_cache = self
            .result
            .as_ref()
            .and_then(|d| ResultRenderCache::build(d).ok());
        self.monitoring.time_state = Some((
            frame.step,
            state.report.time,
            Some(state.report.last_dt),
            None,
            state.report.rejected_steps,
        ));
        self.initialized = true;
        Ok(())
    }
    pub(super) fn compressible_properties(&mut self, ui: &mut egui::Ui) {
        heading(ui, self.pages().get(self.page).copied().unwrap_or("Flow"));
        ui.label("Conservative ideal-gas Euler / Navier–Stokes; absolute pressure, density and total energy.");
        if (self.stage == Stage::Solution && self.page == 9)
            || (self.stage == Stage::Results && self.page == 5)
        {
            ui.label("Select a saved run to inspect its original mesh and fields.");
            return;
        }
        let Some(settings) = self.editor.solver.compressible.as_mut() else {
            return;
        };
        if self.stage == Stage::Solution && self.page == 2 {
            number(ui, "End time", &mut settings.end_time);
            number(ui, "Maximum Δt", &mut settings.max_time_step);
            number(ui, "Acoustic Courant number (≤0.5)", &mut settings.courant);
            ui.add(
                egui::DragValue::new(&mut settings.max_steps)
                    .range(1..=10_000_000)
                    .prefix("Maximum steps: "),
            );
        }
        if self.stage == Stage::Setup && self.page == 1 {
            number(ui, "Specific heat ratio γ", &mut settings.gas.gamma);
            number(ui, "Gas constant R", &mut settings.gas.gas_constant);
        }
        if self.stage == Stage::Solution && self.page == 4 {
            egui::ComboBox::from_id_salt("gas-flux")
                .selected_text(format!("{:?}", settings.flux))
                .show_ui(ui, |ui| {
                    for (value, name) in [
                        (gas::FluxScheme::Hllc, "HLLC"),
                        (gas::FluxScheme::Rusanov, "Rusanov"),
                        (gas::FluxScheme::CentralUpwind, "Central upwind"),
                    ] {
                        ui.selectable_value(&mut settings.flux, value, name);
                    }
                });
        }
        if self.stage == Stage::Solution && self.page == 1 {
            ui.collapsing("Initial state and regions", |ui| {
                primitive_editor(ui, &mut settings.initial);
                let mut remove = None;
                for (i, region) in settings.regions.iter_mut().enumerate() {
                    egui::CollapsingHeader::new(format!("Region {}", i + 1))
                        .id_salt(("gas-region", i))
                        .show(ui, |ui| {
                            widgets::vector(ui, "Minimum", &mut region.minimum);
                            widgets::vector(ui, "Maximum", &mut region.maximum);
                            primitive_editor(ui, &mut region.state);
                            if ui.button("Remove region").clicked() {
                                remove = Some(i);
                            }
                        });
                }
                if let Some(i) = remove {
                    settings.regions.remove(i);
                }
                if ui.button("Add initial region").clicked() {
                    settings.regions.push(gas::InitialRegion {
                        minimum: Vec3::ZERO,
                        maximum: Vec3::new(0.5, 1., 1.),
                        state: settings.initial,
                    });
                }
            });
        }
        if self.stage == Stage::Setup && self.page == 0 {
            for patch in self.session.patch_names() {
                if !settings.boundaries.contains_key(&patch)
                    && ui.button(format!("Assign boundary: {patch}")).clicked()
                {
                    settings
                        .boundaries
                        .insert(patch.clone(), gas::Boundary::Transmissive);
                }
            }
            ui.collapsing("Compressible boundary states", |ui| {
                for (name, boundary) in &mut settings.boundaries {
                    egui::CollapsingHeader::new(name)
                        .id_salt(("gas-boundary", name))
                        .show(ui, |ui| {
                            let mut kind = match boundary {
                                gas::Boundary::Transmissive => 0,
                                gas::Boundary::SlipWall => 1,
                                gas::Boundary::FarField { .. } => 2,
                                gas::Boundary::PressureOutlet { .. } => 3,
                                gas::Boundary::NoSlipWall { .. } => 4,
                                gas::Boundary::SupersonicInlet { .. } => 5,
                                gas::Boundary::TotalInlet { .. } => 6,
                                gas::Boundary::MovingWall { .. } => 7,
                                gas::Boundary::CharacteristicFarField { .. } => 8,
                                gas::Boundary::PressureOutletBackflow { .. } => 9,
                            };
                            let old = kind;
                            let labels = [
                                "Transmissive",
                                "Slip wall",
                                "Far-field state",
                                "Pressure outlet",
                                "No-slip wall",
                                "Supersonic inlet",
                                "Total pressure / temperature inlet",
                                "Moving wall",
                                "Characteristic far field",
                                "Pressure outlet / backflow state",
                            ];
                            egui::ComboBox::from_id_salt(("gas-bc", name))
                                .selected_text(labels[kind])
                                .show_ui(ui, |ui| {
                                    for (i, label) in labels.iter().enumerate() {
                                        ui.selectable_value(&mut kind, i, *label);
                                    }
                                });
                            if kind != old {
                                *boundary = match kind {
                                    0 => gas::Boundary::Transmissive,
                                    1 => gas::Boundary::SlipWall,
                                    2 => gas::Boundary::FarField {
                                        state: settings.initial,
                                    },
                                    3 => gas::Boundary::PressureOutlet {
                                        pressure: settings.initial.pressure,
                                    },
                                    4 => gas::Boundary::NoSlipWall { temperature: None },
                                    5 => gas::Boundary::SupersonicInlet {
                                        state: settings.initial,
                                    },
                                    7 => gas::Boundary::MovingWall {
                                        velocity: Vec3::ZERO,
                                        temperature: None,
                                    },
                                    8 => gas::Boundary::CharacteristicFarField {
                                        state: settings.initial,
                                    },
                                    9 => gas::Boundary::PressureOutletBackflow {
                                        pressure: settings.initial.pressure,
                                        state: settings.initial,
                                    },
                                    _ => gas::Boundary::TotalInlet {
                                        pressure: settings.initial.pressure,
                                        temperature: settings.gas.temperature(settings.initial),
                                        direction: Vec3::new(1., 0., 0.),
                                    },
                                };
                            }
                            match boundary {
                                gas::Boundary::TotalInlet {
                                    pressure,
                                    temperature,
                                    direction,
                                } => {
                                    number(ui, "Total pressure (Pa)", pressure);
                                    number(ui, "Total temperature (K)", temperature);
                                    vector(ui, "Incoming direction", direction);
                                }
                                gas::Boundary::FarField { state }
                                | gas::Boundary::SupersonicInlet { state }
                                | gas::Boundary::CharacteristicFarField { state } => {
                                    primitive_editor(ui, state)
                                }
                                gas::Boundary::PressureOutlet { pressure } => {
                                    number(ui, "Absolute pressure", pressure)
                                }
                                gas::Boundary::PressureOutletBackflow { pressure, state } => {
                                    number(ui, "Outlet absolute pressure (Pa)", pressure);
                                    ui.label("Incoming state");
                                    primitive_editor(ui, state);
                                }
                                gas::Boundary::MovingWall {
                                    velocity,
                                    temperature,
                                } => {
                                    vector(ui, "Wall velocity (m/s)", velocity);
                                    let mut fixed = temperature.is_some();
                                    if ui.checkbox(&mut fixed, "Fixed wall temperature").changed() {
                                        *temperature = fixed
                                            .then_some(settings.gas.temperature(settings.initial));
                                    }
                                    if let Some(t) = temperature {
                                        number(ui, "Wall temperature (K)", t);
                                    }
                                }
                                gas::Boundary::NoSlipWall { temperature } => {
                                    let mut fixed = temperature.is_some();
                                    if ui.checkbox(&mut fixed, "Fixed wall temperature").changed() {
                                        *temperature = fixed.then_some(1.);
                                    }
                                    if let Some(t) = temperature {
                                        number(ui, "Wall temperature", t);
                                    }
                                }
                                _ => {}
                            }
                        });
                }
            });
        }
        if self.stage == Stage::Solution && self.page == 5 {
            ui.label("Execution device is configured in Compute.");
        }
        if self.stage == Stage::Setup && self.page == 2 {
            self.editor.solver.thermal.enabled = true;
            number(
                ui,
                "Volumetric heat source (W/m³)",
                &mut self.editor.solver.thermal.source_w_m3,
            );
            ui.checkbox(
                &mut self.editor.solver.thermal.buoyancy,
                "Gravity / density-driven buoyancy",
            );
            if self.editor.solver.thermal.buoyancy {
                vector(
                    ui,
                    "Gravity (m/s²)",
                    &mut self.editor.solver.thermal.gravity,
                );
            }
        }
        if (self.stage == Stage::Setup && matches!(self.page, 2 | 4))
            || (self.stage == Stage::Solution && self.page == 3)
        {
            ui.label("Viscosity and heat conduction");
            let mut viscous = !matches!(settings.viscosity, gas::Viscosity::Inviscid);
            if ui.checkbox(&mut viscous, "Viscous flow").changed() {
                settings.viscosity = if viscous {
                    gas::Viscosity::Constant {
                        mu: 1.8e-5,
                        prandtl: 0.72,
                    }
                } else {
                    gas::Viscosity::Inviscid
                };
            }
            match &mut settings.viscosity {
                gas::Viscosity::Constant { mu, prandtl } => {
                    number(ui, "Dynamic viscosity (Pa s)", mu);
                    number(ui, "Prandtl number", prandtl);
                }
                gas::Viscosity::Sutherland {
                    mu_ref,
                    temperature_ref,
                    sutherland,
                    prandtl,
                } => {
                    number(ui, "Reference viscosity", mu_ref);
                    number(ui, "Reference temperature (K)", temperature_ref);
                    number(ui, "Sutherland temperature (K)", sutherland);
                    number(ui, "Prandtl number", prandtl);
                }
                _ => {}
            }
        }
        if self.stage != Stage::Results {
            return;
        }
        ui.separator();
        if ui.button("Load latest saved field").clicked() {
            if let Some(path) = self
                .workspace
                .as_ref()
                .map(|p| p.join("compressible/latest.json"))
            {
                if let Err(e) = self.load_compressible_archive(&path) {
                    self.message(e);
                }
            }
        }
    }
    pub(super) fn update_gas_result_report(&mut self, state: gas::Snapshot) {
        if let Some(saved) = &mut self.gas_flow.saved {
            saved.report = state.report;
            saved.gas = state.gas;
            self.gas_flow.profile_quantity = None;
        }
    }
    pub(super) fn open_gas_result_frame(
        &mut self,
        frame: workbench::flow_frame::FlowFrame,
    ) -> Result<(), String> {
        let mesh = frame.mesh.clone().restore()?;
        if let workbench::flow_frame::FlowFields::Compressible { state } = frame.fields {
            self.gas_flow.saved = Some(Saved::from_state(
                &mesh,
                &state,
                self.project.solver.compressible.as_ref(),
            ));
        }
        Ok(())
    }
    pub(super) fn sod_profile_reference(
        &self,
        a: Vec3,
        b: Vec3,
        count: usize,
    ) -> Option<workbench::series_plot::Series> {
        let saved = self.gas_flow.saved.as_ref()?;
        if !saved.sod_reference || saved.report.time > 0.2 {
            return None;
        }
        let value = |state: gas::Primitive| match self.result_field {
            ResultFieldKind::Density => Some(state.density),
            ResultFieldKind::AbsolutePressure => Some(state.pressure),
            ResultFieldKind::VelocityX => Some(state.velocity.x),
            ResultFieldKind::VelocityY => Some(state.velocity.y),
            ResultFieldKind::VelocityZ => Some(state.velocity.z),
            ResultFieldKind::VelocityMagnitude => Some(state.velocity.norm()),
            ResultFieldKind::Temperature => Some(saved.gas.temperature(state)),
            ResultFieldKind::Mach => Some(state.velocity.norm() / saved.gas.sound_speed(state)),
            _ => None,
        };
        value(gas::benchmarks::sod(a.x, saved.report.time))?;
        Some(workbench::series_plot::Series {
            name: format!("Exact Sod · t={:.6}", saved.report.time),
            points: (0..count)
                .map(|i| {
                    let fraction = i as f64 / (count - 1) as f64;
                    let point = a + (b - a) * fraction;
                    (
                        fraction * (b - a).norm(),
                        if (0. ..=1.).contains(&point.x) {
                            value(gas::benchmarks::sod(point.x, saved.report.time))
                        } else {
                            None
                        },
                    )
                })
                .collect(),
        })
    }
    pub(super) fn compressible_view(&mut self, ui: &mut egui::Ui) {
        if self.stage == Stage::Results && self.page == 2 {
            ui.horizontal_wrapped(|ui| {
                egui::ComboBox::from_id_salt("gas-display-field")
                    .selected_text(
                        [
                            "Density",
                            "Absolute pressure",
                            "Velocity X",
                            "Temperature",
                            "Mach number",
                        ][self.gas_flow.quantity.min(4)],
                    )
                    .show_ui(ui, |ui| {
                        for (i, name) in [
                            "Density",
                            "Absolute pressure",
                            "Velocity X",
                            "Temperature",
                            "Mach number",
                        ]
                        .iter()
                        .enumerate()
                        {
                            ui.selectable_value(&mut self.gas_flow.quantity, i, *name);
                        }
                    });
            });
        }
        if self.stage == Stage::Solution && self.page == 0 {
            let series = [workbench::series_plot::Series {
                name: "Accepted physical time (s)".into(),
                points: self
                    .gas_flow
                    .history
                    .iter()
                    .map(|(step, time)| (*step as f64, Some(*time)))
                    .collect(),
            }];
            ui.label("Time integration progress; this is not a residual convergence criterion.");
            self.native_series_plot(ui, &series, "Time integration", "Accepted step");
            return;
        }
        if (self.stage == Stage::Results && self.page == 5)
            || (self.stage == Stage::Solution && self.page == 9)
        {
            self.compressible_archives(ui);
            return;
        }
        let Some(saved) = self.gas_flow.saved.as_ref() else {
            ui.label("Generate a mesh and save the workspace, then run the compressible example. Completed profiles can be reloaded after reopening.");
            return;
        };
        ui.label(format!(
            "Accepted {} · rejected {} · t={:.6e} · max Mach {:.4} · reached end: {}",
            saved.report.accepted_steps,
            saved.report.rejected_steps,
            saved.report.time,
            saved.report.max_mach,
            saved.report.reached_end_time
        ));
        if self.stage == Stage::Results && self.page == 1 {
            ui.label(format!(
                "Minimum density {:.6e} kg/m³ · minimum pressure {:.6e} Pa · last Δt {:.6e} s",
                saved.report.min_density, saved.report.min_pressure, saved.report.last_dt
            ));
            for (name, defect) in [
                "Mass",
                "Momentum X",
                "Momentum Y",
                "Momentum Z",
                "Total energy",
            ]
            .iter()
            .zip(saved.report.conservation_defect)
            {
                ui.label(format!("{name} conservation defect: {defect:.6e}"));
            }
            return;
        }
        let q = self.gas_flow.quantity;
        let title = [
            "Density",
            "Absolute pressure",
            "Velocity X",
            "Temperature",
            "Mach number",
        ][q.min(4)];
        let value = |p: gas::Primitive| match q {
            0 => p.density,
            1 => p.pressure,
            2 => p.velocity.x,
            3 => saved.gas.temperature(p),
            _ => p.velocity.norm() / saved.gas.sound_speed(p),
        };
        if self.gas_flow.profile_quantity != Some(q) {
            let mut series = vec![workbench::series_plot::Series {
                name: "Finite volume".into(),
                points: saved
                    .points
                    .iter()
                    .map(|(x, p)| (*x, Some(value(*p))))
                    .collect(),
            }];
            if saved.sod_reference && saved.report.time <= 0.2 {
                series.push(workbench::series_plot::Series {
                    name: "Exact Sod reference".into(),
                    points: saved
                        .points
                        .iter()
                        .map(|(x, _)| {
                            (*x, Some(value(gas::benchmarks::sod(*x, saved.report.time))))
                        })
                        .collect(),
                });
            }
            self.gas_flow.profiles = series;
            self.gas_flow.profile_quantity = Some(q);
        }
        let series = std::mem::take(&mut self.gas_flow.profiles);
        if ui.button("Export profile CSV").clicked() {
            if let Some(path) = rfd::FileDialog::new()
                .set_file_name("compressible-profile.csv")
                .save_file()
            {
                match std::fs::write(path, workbench::series_plot::csv(&series, "x")) {
                    Ok(()) => self.message("Profile CSV saved."),
                    Err(e) => self.message(e.to_string()),
                }
            }
        }
        self.native_series_plot(ui, &series, title, "x");
        self.gas_flow.profiles = series;
    }
}

fn primitive_editor(ui: &mut egui::Ui, state: &mut gas::Primitive) {
    number(ui, "Density", &mut state.density);
    number(ui, "Absolute pressure", &mut state.pressure);
    widgets::vector(ui, "Velocity", &mut state.velocity);
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn archive_restores_cell_order_and_mesh_without_the_current_project() {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0., 0., 0.),
                Point::new(1., 0., 0.),
                Point::new(1., 1., 0.),
                Point::new(0., 1., 0.),
            ],
            vec![
                CellDefinition::polygon(vec![0, 1, 2]),
                CellDefinition::polygon(vec![0, 2, 3]),
            ],
        )
        .unwrap();
        let boundary_faces = mesh
            .faces()
            .iter()
            .enumerate()
            .filter_map(|(i, f)| f.neighbour.is_none().then_some(i))
            .collect();
        let mesh = mesh
            .with_boundary_patches(vec![flursys::BoundaryPatch {
                name: "wall".into(),
                face_indices: boundary_faces,
                boundary_type: flursys::BoundaryType::Wall,
            }])
            .unwrap();
        let state = |density| gas::Primitive {
            density,
            pressure: 100000.,
            velocity: Vec3::new(1., 0., 0.),
        };
        let report = gas::Report {
            source_integrals: [0.; 5],
            performance: Default::default(),
            stopped: false,
            residuals: [0.; 4],
            time: 0.1,
            accepted_steps: 1,
            cyclic_face_pairs: 0,
            rejected_steps: 0,
            hllc_fallback_evaluations: 0,
            reached_end_time: true,
            last_dt: 0.1,
            min_density: 1.,
            min_pressure: 100000.,
            max_mach: 0.1,
            initial_integrals: [0.; 5],
            final_integrals: [0.; 5],
            boundary_outflow_integrals: [0.; 5],
            conservation_defect: [0.; 5],
        };
        let saved = Saved {
            run_id: "test-gas".into(),
            mesh: Some(flursys::mesh::MeshSnapshot::capture(&mesh)),
            cell_indices: vec![1, 0],
            accepted_history: vec![(1, 0.1)],
            points: vec![(0.3, state(2.)), (0.7, state(1.))],
            report,
            gas: gas::IdealGas::default(),
            sod_reference: false,
            polygons: vec![],
            contours_from_current_mesh: false,
        };
        let mut restored: Saved =
            serde_json::from_slice(&serde_json::to_vec(&saved).unwrap()).unwrap();
        let dataset = restored.dataset().unwrap();
        assert_eq!(dataset.scalar_values(ResultFieldKind::Density), &[1., 2.]);
        assert_eq!(dataset.mesh().cell_count(), 2);
        assert_eq!(restored.accepted_history, vec![(1, 0.1)]);
        restored.cell_indices = vec![1, 1];
        assert!(restored.dataset().is_err());
        restored.mesh = None;
        assert!(restored.dataset().is_err());
        restored.polygons = vec![
            vec![[0., 0.], [1., 1.], [0., 1.]],
            vec![[0., 0.], [1., 0.], [1., 1.]],
        ];
        let legacy = restored.dataset().unwrap();
        assert_eq!(legacy.scalar_values(ResultFieldKind::Density), &[2., 1.]);
    }
}
