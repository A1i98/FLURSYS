use super::widgets::{heading, number};
use super::*;
use flursys::compressible as gas;
use std::sync::{
    atomic::{AtomicU8, Ordering},
    Arc,
};
#[derive(Clone, Serialize, Deserialize)]
struct Saved {
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
enum Event {
    Progress(usize, f64),
    Done(Result<Saved, String>),
}
#[derive(Default)]
pub(super) struct State {
    receiver: Option<Receiver<Event>>,
    control: Arc<AtomicU8>,
    saved: Option<Saved>,
    step: usize,
    time: f64,
    quantity: usize,
    profile_quantity: Option<usize>,
    profiles: Vec<workbench::series_plot::Series>,
    display: usize,
}
impl State {
    pub(super) fn set_defaults(&mut self, field: &str, mode: &str) {
        self.quantity = match field {
            "absolute-pressure" => 1,
            "velocity-x" => 2,
            "temperature" => 3,
            "mach" => 4,
            _ => 0,
        };
        self.display = usize::from(mode == "profiles");
    }
    pub fn busy(&self) -> bool {
        self.receiver.is_some()
    }
}
impl App {
    pub(super) fn start_compressible(&mut self) {
        if self.busy() {
            return;
        }
        let result = (|| -> Result<(), String> {
            self.apply_editor()?;
            let settings = self
                .project
                .solver
                .compressible
                .clone()
                .ok_or("No compressible configuration")?;
            let mesh = self
                .session
                .mesh()
                .ok_or("Generate a mesh before running")?
                .mesh
                .clone();
            settings.validate(&mesh)?;
            let workspace = self
                .workspace
                .as_ref()
                .ok_or("Save this workspace before running")?;
            if let Some(generated) = self.session.mesh() {
                self.project.mesh_asset =
                    Some(mesh_storage::save(workspace, &self.project, generated)?);
                self.editor.mesh_asset = self.project.mesh_asset.clone();
            }
            workbench::save_workspace(workspace, &self.project).map_err(|e| e.to_string())?;
            let folder = workspace.join("compressible").join(format!(
                "run-{}",
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map_err(|e| e.to_string())?
                    .as_nanos()
            ));
            std::fs::create_dir_all(&folder).map_err(|e| e.to_string())?;
            std::fs::write(
                folder.join("settings.json"),
                serde_json::to_vec_pretty(&settings).map_err(|e| e.to_string())?,
            )
            .map_err(|e| e.to_string())?;
            let (tx, rx) = mpsc::channel();
            self.gas_flow = State {
                receiver: Some(rx),
                ..Default::default()
            };
            let control = self.gas_flow.control.clone();
            std::thread::spawn(move || {
                let solved = (|| -> Result<Saved, String> {
                    let solution = gas::solve_controlled(&mesh, &settings, |step, time| {
                        while control.load(Ordering::Acquire) == 1 {
                            std::thread::sleep(Duration::from_millis(25));
                        }
                        if control.load(Ordering::Acquire) == 2 {
                            return Err("Compressible solve stopped by user; incomplete output was not published".into());
                        }
                        if step % 20 == 0 {
                            let _ = tx.send(Event::Progress(step, time));
                        }
                        Ok(())
                    })?;
                    gas::write_vtu(&folder.join("fields.vtu"), &mesh, &solution, settings.gas)?;
                    let render = MeshRenderCache::build(&mesh)?;
                    let mut indices: Vec<_> = (0..mesh.cell_count()).collect();
                    indices.sort_by(|&a, &b| {
                        mesh.cells()[a]
                            .center
                            .x
                            .total_cmp(&mesh.cells()[b].center.x)
                    });
                    let points = indices
                        .iter()
                        .map(|&i| (mesh.cells()[i].center.x, solution.fields[i]))
                        .collect();
                    let polygons = indices
                        .iter()
                        .map(|&i| {
                            render
                                .cell_polygon(i)
                                .unwrap_or(&[])
                                .iter()
                                .map(|&(x, y)| [x, y])
                                .collect()
                        })
                        .collect();
                    let saved = Saved {
                        points,
                        polygons,
                        contours_from_current_mesh: false,
                        report: solution.report,
                        gas: settings.gas,
                        sod_reference: gas::benchmarks::is_standard_sod(&mesh, &settings),
                    };
                    let bytes = serde_json::to_vec(&saved).map_err(|e| e.to_string())?;
                    std::fs::write(folder.join("profiles.json"), &bytes)
                        .map_err(|e| e.to_string())?;
                    let mut file = tempfile::NamedTempFile::new_in(
                        folder.parent().ok_or("Missing archive parent")?,
                    )
                    .map_err(|e| e.to_string())?;
                    use std::io::Write;
                    file.write_all(&bytes).map_err(|e| e.to_string())?;
                    file.persist(
                        folder
                            .parent()
                            .ok_or("Missing archive parent")?
                            .join("latest.json"),
                    )
                    .map_err(|e| e.to_string())?;
                    Ok(saved)
                })();
                let _ = tx.send(Event::Done(solved));
            });
            Ok(())
        })();
        match result { Ok(()) => self.message("Compressible finite-volume solve started; absolute pressure and total energy are used."), Err(e) => self.message(e) }
    }
    pub(super) fn poll_compressible(&mut self) {
        let events: Vec<_> = self
            .gas_flow
            .receiver
            .as_ref()
            .map(|rx| rx.try_iter().collect())
            .unwrap_or_default();
        for event in events {
            match event {
                Event::Progress(step, time) => {
                    self.gas_flow.step = step;
                    self.gas_flow.time = time;
                }
                Event::Done(result) => {
                    self.gas_flow.receiver = None;
                    match result {
                        Ok(saved) => {
                            self.gas_flow.time = saved.report.time;
                            self.gas_flow.step = saved.report.accepted_steps;
                            self.gas_flow.saved = Some(saved);
                            self.gas_flow.profile_quantity = None;
                            self.message("Compressible run saved under compressible/; VTU fields and JSON profiles are available.");
                        }
                        Err(e) => self.message(e.to_string()),
                    }
                }
            }
        }
    }
    pub(super) fn compressible_properties(&mut self, ui: &mut egui::Ui) {
        heading(ui, "COMPRESSIBLE FLOW");
        ui.label("Conservative ideal-gas Euler / Navier–Stokes; absolute pressure, density and total energy.");
        let Some(settings) = self.editor.solver.compressible.as_mut() else {
            return;
        };
        number(ui, "End time", &mut settings.end_time);
        number(ui, "Maximum Δt", &mut settings.max_time_step);
        number(ui, "Acoustic Courant number (≤0.5)", &mut settings.courant);
        number(ui, "Specific heat ratio γ", &mut settings.gas.gamma);
        number(ui, "Gas constant R", &mut settings.gas.gas_constant);
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
                        };
                        let old = kind;
                        let labels = [
                            "Transmissive",
                            "Slip wall",
                            "Far-field state",
                            "Pressure outlet",
                            "No-slip wall",
                            "Supersonic inlet",
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
                                _ => gas::Boundary::SupersonicInlet {
                                    state: settings.initial,
                                },
                            };
                        }
                        match boundary {
                            gas::Boundary::FarField { state }
                            | gas::Boundary::SupersonicInlet { state } => {
                                primitive_editor(ui, state)
                            }
                            gas::Boundary::PressureOutlet { pressure } => {
                                number(ui, "Absolute pressure", pressure)
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
        ui.weak("Run starts from the configured states. Pause/resume is supported; stopped compressible runs do not yet retain a restart checkpoint.");
        ui.separator();
        if ui.button("Load latest saved profiles").clicked() {
            let result = self
                .workspace
                .as_ref()
                .ok_or("Save or open a workspace first".into())
                .and_then(|p| {
                    std::fs::read(p.join("compressible/latest.json")).map_err(|e| e.to_string())
                })
                .and_then(|b| serde_json::from_slice::<Saved>(&b).map_err(|e| e.to_string()));
            match result {
                Ok(mut saved) => {
                    if saved.polygons.is_empty() {
                        if let Some(mesh) = self.session.mesh() {
                            let _ = recover_polygons(&mut saved, &mesh.mesh);
                        }
                    }
                    self.gas_flow.saved = Some(saved);
                    self.gas_flow.profile_quantity = None;
                }
                Err(e) => self.message(e.to_string()),
            }
        }
    }
    pub(super) fn compressible_view(&mut self, ui: &mut egui::Ui) {
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
            ui.selectable_value(&mut self.gas_flow.display, 0, "Contours");
            ui.selectable_value(&mut self.gas_flow.display, 1, "Profiles + reference");
        });
        let running = self.gas_flow.busy();
        ui.horizontal_wrapped(|ui| {
            if ui
                .add_enabled(!self.busy(), egui::Button::new("Run compressible"))
                .clicked()
            {
                self.start_compressible();
            }
            if ui
                .add_enabled(
                    running,
                    egui::Button::new(if self.gas_flow.control.load(Ordering::Acquire) == 1 {
                        "Resume"
                    } else {
                        "Pause"
                    }),
                )
                .clicked()
            {
                let current = self.gas_flow.control.load(Ordering::Acquire);
                if current < 2 {
                    let _ = self.gas_flow.control.compare_exchange(
                        current,
                        1 - current,
                        Ordering::AcqRel,
                        Ordering::Acquire,
                    );
                }
            }
            if ui.add_enabled(running, egui::Button::new("Stop")).clicked() {
                self.gas_flow.control.store(2, Ordering::Release);
            }
            ui.label(format!(
                "Step {} · t={:.6e}",
                self.gas_flow.step, self.gas_flow.time
            ));
        });
        if let Some(settings) = &self.project.solver.compressible {
            ui.add(
                egui::ProgressBar::new(
                    (self.gas_flow.time / settings.end_time).clamp(0., 1.) as f32
                )
                .show_percentage(),
            );
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
        if self.gas_flow.display == 0 {
            if saved.polygons.len() == saved.points.len() && !saved.polygons.is_empty() {
                if saved.contours_from_current_mesh {
                    ui.weak("Contour geometry reconstructed from the current workspace mesh; this older archive did not store its topology.");
                }
                paint_contours(ui, saved, q);
                return;
            }
            ui.weak("This archive has no contour topology. Load its original mesh or rerun to save contours; profiles remain available.");
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

fn recover_polygons(saved: &mut Saved, mesh: &UnstructuredMesh) -> Result<(), String> {
    if mesh.dimension() != MeshDimension::TwoD || saved.points.len() != mesh.cell_count() {
        return Err("Archive and mesh do not match".into());
    }
    let mut indices: Vec<_> = (0..mesh.cell_count()).collect();
    indices.sort_by(|&a, &b| {
        mesh.cells()[a]
            .center
            .x
            .total_cmp(&mesh.cells()[b].center.x)
    });
    if indices
        .iter()
        .zip(&saved.points)
        .any(|(&i, (x, _))| (mesh.cells()[i].center.x - x).abs() > 1e-12 * (1. + x.abs()))
    {
        return Err("Archive and mesh coordinates differ".into());
    }
    let render = MeshRenderCache::build(mesh)?;
    saved.polygons = indices
        .iter()
        .map(|&i| {
            render
                .cell_polygon(i)
                .unwrap_or(&[])
                .iter()
                .map(|&(x, y)| [x, y])
                .collect()
        })
        .collect();
    saved.contours_from_current_mesh = true;
    Ok(())
}
fn paint_contours(ui: &mut egui::Ui, saved: &Saved, quantity: usize) {
    let value = |p: gas::Primitive| match quantity {
        0 => p.density,
        1 => p.pressure,
        2 => p.velocity.x,
        3 => saved.gas.temperature(p),
        _ => p.velocity.norm() / saved.gas.sound_speed(p),
    };
    let range = saved
        .points
        .iter()
        .map(|(_, p)| value(*p))
        .filter(|v| v.is_finite())
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
            (lo.min(v), hi.max(v))
        });
    let mut low = [f64::INFINITY; 2];
    let mut high = [f64::NEG_INFINITY; 2];
    for p in saved.polygons.iter().flatten() {
        for k in 0..2 {
            low[k] = low[k].min(p[k]);
            high[k] = high[k].max(p[k]);
        }
    }
    let (rect, _) = ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
    if !low[0].is_finite() || !range.0.is_finite() {
        return;
    }
    let area = rect.shrink2(egui::vec2(32., 35.));
    let scale = (area.width() as f64 / (high[0] - low[0]).max(1e-12))
        .min(area.height() as f64 / (high[1] - low[1]).max(1e-12));
    let screen = |p: [f64; 2]| {
        egui::pos2(
            area.center().x + ((p[0] - 0.5 * (low[0] + high[0])) * scale) as f32,
            area.center().y - ((p[1] - 0.5 * (low[1] + high[1])) * scale) as f32,
        )
    };
    let mut mesh = egui::Mesh::default();
    for (polygon, (_, state)) in saved.polygons.iter().zip(&saved.points) {
        if polygon.len() < 3 {
            continue;
        }
        let color = viewport::scalar_color(value(*state), range.0, range.1, 1);
        let start = mesh.vertices.len() as u32;
        for p in polygon {
            mesh.colored_vertex(screen(*p), color);
        }
        for i in 1..polygon.len() - 1 {
            mesh.add_triangle(start, start + i as u32, start + i as u32 + 1);
        }
    }
    ui.painter().add(egui::Shape::mesh(mesh));
    viewport::legend(ui.painter(), rect, range.0, range.1, 1);
    ui.painter().text(
        rect.left_top() + egui::vec2(8., 8.),
        egui::Align2::LEFT_TOP,
        format!(
            "Cell-centred field · t={:.6e} · FLURSYS v{}",
            saved.report.time,
            env!("CARGO_PKG_VERSION")
        ),
        egui::FontId::proportional(12.),
        ui.visuals().text_color(),
    );
}
