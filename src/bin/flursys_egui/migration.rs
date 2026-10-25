use super::*;
use widgets::{heading, number, vector};
use workbench::{BoxRefinement, EdgeDivisionControl};
#[derive(Default)]
pub(super) struct State {
    time_csv: String,
    time_quantity: usize,
    patch: String,
    cyclic_first: String,
    cyclic_second: String,
    translation: Vec3,
    tolerance: f64,
    low: Vec3,
    high: Vec3,
    velocity: Vec3,
    pressure: f64,
    mapping: Option<(
        flursys::MeshId,
        Receiver<Result<workbench::field_mapping::MappedFields, String>>,
    )>,
    profile_start: Vec3,
    profile_end: Vec3,
    samples: usize,
    curves: Vec<workbench::series_plot::Series>,
    context: Option<(Vec3, Vec3, ResultFieldKind)>,
    pub chart: bool,
    yplus: f64,
    utau: f64,
}
impl App {
    pub(super) fn boundary_table_editor(&mut self, ui: &mut egui::Ui) {
        use flursys::unstructured_incompressible::{BoundaryProfile, BoundaryQuantity as Q};
        egui::CollapsingHeader::new("Physical-time boundary table").show(ui, |ui| {
            if self.migration.patch != self.boundary_patch {
                self.migration.patch = self.boundary_patch.clone();
                self.migration.time_csv.clear();
                self.migration.time_quantity = 0;
                if let Some(p) = self
                    .editor
                    .solver
                    .boundary_profiles
                    .get(&self.boundary_patch)
                {
                    self.migration.time_csv = p.csv();
                    self.migration.time_quantity = match p.quantity {
                        Q::Velocity => 0,
                        Q::FlowRate => 1,
                        Q::Pressure => 2,
                        Q::Temperature => 3,
                        Q::HeatFlux => 4,
                    };
                }
            }
            let names = [
                "Velocity: time,Ux,Uy,Uz",
                "Flow rate: time,value",
                "Pressure: time,value",
                "Temperature: time,value",
                "Heat flux: time,value",
            ];
            egui::ComboBox::from_id_salt("table-quantity")
                .selected_text(names[self.migration.time_quantity])
                .show_ui(ui, |ui| {
                    for (i, n) in names.iter().enumerate() {
                        ui.selectable_value(&mut self.migration.time_quantity, i, *n);
                    }
                });
            ui.label("Physical seconds; linear interpolation and endpoint hold.");
            ui.add(
                egui::TextEdit::multiline(&mut self.migration.time_csv)
                    .desired_rows(5)
                    .desired_width(f32::INFINITY),
            );
            if ui.button("Apply time table").clicked() {
                let q = [
                    Q::Velocity,
                    Q::FlowRate,
                    Q::Pressure,
                    Q::Temperature,
                    Q::HeatFlux,
                ][self.migration.time_quantity];
                match BoundaryProfile::from_csv(q, &self.migration.time_csv) {
                    Ok(p) => {
                        self.editor
                            .solver
                            .boundary_profiles
                            .insert(self.boundary_patch.clone(), p);
                        self.apply_editor_feedback();
                    }
                    Err(e) => self.message(e),
                }
            }
            if ui.button("Remove time table").clicked() {
                self.editor
                    .solver
                    .boundary_profiles
                    .remove(&self.boundary_patch);
                self.migration.time_csv.clear();
                self.apply_editor_feedback();
            }
        });
    }
    fn apply_editor_feedback(&mut self) {
        if let Err(e) = self.apply_editor() {
            self.message(e)
        } else {
            self.message("Settings applied.")
        }
    }
    pub(super) fn advanced_physics_editor(&mut self, ui: &mut egui::Ui) {
        use flursys::volume_models::{CellRegion, PorousResistance, RotatingFrame};
        egui::CollapsingHeader::new("Volume models and periodic forcing").show(ui, |ui| {
            let m = &mut self.editor.solver.volume_models;
            let mut driven = m.mean_velocity.is_some();
            if ui
                .checkbox(&mut driven, "Periodic mean-flow forcing")
                .changed()
            {
                m.mean_velocity = driven.then_some(flursys::volume_models::MeanVelocity {
                    target: Vec3::new(1., 0., 0.),
                    relaxation: 0.5,
                });
            }
            if let Some(mean) = &mut m.mean_velocity {
                vector(ui, "Volume-mean target velocity (m/s)", &mut mean.target);
                number(ui, "Forcing relaxation", &mut mean.relaxation);
            }
            let mut rotation = m.rotating_frame.is_some();
            if ui
                .checkbox(&mut rotation, "Global rotating frame")
                .changed()
            {
                m.rotating_frame = rotation.then_some(RotatingFrame {
                    angular_velocity: Vec3::new(0., 0., 1.),
                    origin: Vec3::ZERO,
                });
            }
            if let Some(r) = &mut m.rotating_frame {
                vector(ui, "Angular velocity (rad/s)", &mut r.angular_velocity);
                vector(ui, "Origin (m)", &mut r.origin);
                ui.small("Boundary and solved velocities are relative to this frame.");
            }
            let mut porous = m.resistance.is_some();
            if ui
                .checkbox(&mut porous, "Porous momentum resistance")
                .changed()
            {
                m.resistance = porous.then_some(PorousResistance {
                    region: CellRegion::All,
                    darcy: 1e6,
                    forchheimer: 0.,
                });
            }
            if let Some(p) = &mut m.resistance {
                number(ui, "Darcy (1/m²)", &mut p.darcy);
                number(ui, "Forchheimer (1/m)", &mut p.forchheimer);
                let mut bounded = matches!(p.region, CellRegion::Box { .. });
                if ui.checkbox(&mut bounded, "Limit to a box").changed() {
                    p.region = if bounded {
                        CellRegion::Box {
                            minimum: Vec3::ZERO,
                            maximum: Vec3::new(1., 1., 1.),
                        }
                    } else {
                        CellRegion::All
                    };
                }
                if let CellRegion::Box { minimum, maximum } = &mut p.region {
                    vector(ui, "Minimum (m)", minimum);
                    vector(ui, "Maximum (m)", maximum);
                }
            }
        });
        egui::CollapsingHeader::new("Translational cyclic pairs").show(ui, |ui| {
            let names: Vec<_> = self
                .session
                .named_selections()
                .iter()
                .map(|n| n.name.clone())
                .collect();
            for (id, label, value) in [
                ("cyclic-a", "First patch", &mut self.migration.cyclic_first),
                (
                    "cyclic-b",
                    "Second patch",
                    &mut self.migration.cyclic_second,
                ),
            ] {
                ui.label(label);
                egui::ComboBox::from_id_salt(id)
                    .selected_text(value.as_str())
                    .show_ui(ui, |ui| {
                        for n in &names {
                            ui.selectable_value(value, n.clone(), n);
                        }
                    });
            }
            vector(
                ui,
                "Translation first → second (m)",
                &mut self.migration.translation,
            );
            number(ui, "Matching tolerance (m)", &mut self.migration.tolerance);
            if ui.button("Add and validate pair").clicked() {
                let mut pairs = self.editor.solver.cyclic_pairs.clone();
                pairs.push(flursys::periodic::TranslationalCyclicPair {
                    first: self.migration.cyclic_first.clone(),
                    second: self.migration.cyclic_second.clone(),
                    translation: self.migration.translation,
                    tolerance: self.migration.tolerance,
                });
                let result = self
                    .session
                    .mesh()
                    .ok_or("Generate mesh first".to_string())
                    .and_then(|m| {
                        flursys::periodic::resolve(&m.mesh, &pairs)
                            .map(|_| ())
                            .map_err(|e| e.to_string())
                    });
                match result {
                    Ok(()) => {
                        self.editor.solver.cyclic_pairs = pairs;
                        self.apply_editor_feedback()
                    }
                    Err(e) => self.message(e),
                }
            }
            let mut remove = None;
            for (i, p) in self.editor.solver.cyclic_pairs.iter().enumerate() {
                ui.horizontal(|ui| {
                    ui.label(format!("{} ↔ {}", p.first, p.second));
                    if ui.small_button("Remove").clicked() {
                        remove = Some(i);
                    }
                });
            }
            if let Some(i) = remove {
                self.editor.solver.cyclic_pairs.remove(i);
                self.apply_editor_feedback();
            }
        });
    }
    pub(super) fn mapping_editor(&mut self, ui: &mut egui::Ui) {
        heading(ui, "FIELD INITIALIZATION");
        ui.small("Mapping transfers U and p/ρ between meshes of the same geometry. It starts a new solve; it is not conservative remapping.");
        let busy = self.busy() || self.migration.mapping.is_some();
        if ui
            .add_enabled(
                !busy,
                egui::Button::new("Map selected result to current mesh"),
            )
            .clicked()
        {
            if let (Some(d), Some(m)) = (&self.result, self.session.mesh()) {
                let source = workbench::field_mapping::SourceFields::copy_from_result(d);
                let mesh = m.mesh.clone();
                let id = mesh.id();
                let (tx, rx) = mpsc::channel();
                std::thread::spawn(move || {
                    let _ = tx.send(workbench::field_mapping::map_initial_fields(&source, &mesh));
                });
                self.migration.mapping = Some((id, rx));
            } else {
                self.message("Select a saved result and generate the target mesh first.");
            }
        }
        if ui
            .add_enabled(!busy, egui::Button::new("Map VTK file…"))
            .clicked()
        {
            if let Some(path) = rfd::FileDialog::new()
                .add_filter("FLURSYS VTK", &["vtk"])
                .pick_file()
            {
                if let Some(m) = self.session.mesh() {
                    let mesh = m.mesh.clone();
                    let id = mesh.id();
                    let (tx, rx) = mpsc::channel();
                    std::thread::spawn(move || {
                        let result =
                            workbench::results::load_legacy_vtk_result(&path, "mapping-source")
                                .map_err(|e| e.to_string())
                                .and_then(|d| {
                                    workbench::field_mapping::map_initial_fields(
                                        &workbench::field_mapping::SourceFields::from_result(d),
                                        &mesh,
                                    )
                                });
                        let _ = tx.send(result);
                    });
                    self.migration.mapping = Some((id, rx));
                }
            }
        }
        egui::CollapsingHeader::new("Initialize cells inside a box").show(ui, |ui| {
            vector(ui, "Minimum (m)", &mut self.migration.low);
            vector(ui, "Maximum (m)", &mut self.migration.high);
            vector(ui, "Velocity (m/s)", &mut self.migration.velocity);
            number(ui, "Pressure p/ρ", &mut self.migration.pressure);
            if ui
                .add_enabled(!busy, egui::Button::new("Apply box initialization"))
                .clicked()
            {
                if let Some(m) = self.session.mesh() {
                    let mesh = m.mesh.clone();
                    let id = mesh.id();
                    let initial = self.jobs.retained_fields(id).map(|(_, v)| v).unwrap_or(
                        IncompressibleInitialConditions::uniform(
                            self.initial_velocity,
                            self.initial_pressure,
                        ),
                    );
                    let (low, high, v, p) = (
                        self.migration.low,
                        self.migration.high,
                        self.migration.velocity,
                        self.migration.pressure,
                    );
                    let (tx, rx) = mpsc::channel();
                    std::thread::spawn(move || {
                        let _ = tx.send(workbench::field_mapping::patch_box(
                            &mesh,
                            initial,
                            low,
                            high,
                            v,
                            p,
                            || false,
                        ));
                    });
                    self.migration.mapping = Some((id, rx));
                }
            }
        });
        if self.migration.mapping.is_some() {
            ui.spinner();
            ui.label("Preparing fields…");
        }
    }
    pub(super) fn poll_mapping(&mut self) {
        let received = self
            .migration
            .mapping
            .as_ref()
            .and_then(|(id, rx)| match rx.try_recv() {
                Ok(r) => Some((*id, r)),
                Err(mpsc::TryRecvError::Disconnected) => {
                    Some((*id, Err("Initialization worker disconnected".into())))
                }
                Err(mpsc::TryRecvError::Empty) => None,
            });
        if let Some((id, result)) = received {
            self.migration.mapping = None;
            if self.busy() || self.session.mesh().is_none_or(|m| m.mesh.id() != id) {
                self.message("Discarded initialization because the active mesh changed.");
                return;
            }
            match result {
                Ok(mapped) => {
                    let archive = (|| -> Result<Option<PathBuf>, String> {
                        let Some(workspace) = &self.workspace else {
                            return Ok(None);
                        };
                        let folder = workspace.join("monitors").join(format!(
                            "initialization-{}",
                            std::time::SystemTime::now()
                                .duration_since(std::time::UNIX_EPOCH)
                                .unwrap_or_default()
                                .as_nanos()
                        ));
                        std::fs::create_dir_all(&folder).map_err(|e| e.to_string())?;
                        let IncompressibleInitialConditions::Fields { velocity, pressure } =
                            &mapped.initial
                        else {
                            return Err("Expected mapped fields".into());
                        };
                        let mesh = &self.session.mesh().unwrap().mesh;
                        workbench::checkpoint::write(
                            &folder.join("checkpoint.json"),
                            &workbench::checkpoint::MeshSignature::new(mesh),
                            0,
                            velocity.values(),
                            pressure.values(),
                            &mapped.description,
                        )?;
                        Ok(Some(folder))
                    })();
                    let folder = match archive {
                        Ok(p) => p,
                        Err(e) => {
                            self.message(e);
                            return;
                        }
                    };
                    self.reset_solve_session();
                    self.jobs.set_initial_fields(id, mapped.initial);
                    self.solve_recording = folder;
                    self.session.initialize_solution();
                    self.result = None;
                    self.result_cache = None;
                    self.result_run.clear();
                    self.archived_report = None;
                    self.probes.clear();
                    self.streamlines.clear();
                    self.dirty = true;
                    self.initialized = false;
                    if let Err(e) = self.persist_continuation() {
                        self.message(e)
                    }
                    self.message(mapped.description);
                }
                Err(e) => self.message(e),
            }
        }
    }
    pub(super) fn layer_tools(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Inflation calculator and preview").show(ui,|ui|{
            number(ui,"Target y+",&mut self.migration.yplus);number(ui,"Estimated friction velocity (m/s)",&mut self.migration.utau);
            let h=2.*self.migration.yplus*self.editor.material.kinematic_viscosity/self.migration.utau;
            if h.is_finite()&&h>0.{ui.label(format!("Estimated first-layer thickness: {h:.6e} m"));}
            for c in &mut self.editor.mesh_recipe.boundary_layers{
                ui.label(&c.name);if ui.add_enabled(h.is_finite()&&h>0.,egui::Button::new(format!("Use estimate · {}",c.id))).clicked(){c.first_layer_height=h;}
                match c.layer_dimensions(){Ok((total,last))=>{ui.label(format!("{} layers · first {:.4e} · last {last:.4e} · total {total:.4e} m",c.layer_count,c.first_layer_height));},Err(e)=>{widgets::error(ui,&e);}}
            }
            ui.small("Layer stacks can truncate near corners and collisions. A y+ estimate is not a wall-function model.");
        });
        if ui.button("Add edge divisions for selected edges").clicked() {
            let targets: Vec<_> = self
                .selected
                .iter()
                .filter(|t| matches!(t, GeometrySelectionTarget::Edge(_)))
                .copied()
                .map(MeshControlTarget::Geometry)
                .collect();
            if targets.is_empty() {
                self.message("Select geometry edges first.");
            } else {
                let id = self
                    .editor
                    .mesh_recipe
                    .edge_divisions
                    .iter()
                    .map(|c| c.id)
                    .max()
                    .unwrap_or(0)
                    + 1;
                self.editor
                    .mesh_recipe
                    .edge_divisions
                    .push(EdgeDivisionControl {
                        id,
                        name: format!("Edge divisions {id}"),
                        targets,
                        cells: 20,
                        progression: 1.,
                        reverse: false,
                        symmetric: false,
                        enabled: true,
                    });
            }
        }
        if ui.button("Add volume refinement box").clicked() {
            let id = self
                .editor
                .mesh_recipe
                .regions
                .iter()
                .map(|c| c.id)
                .max()
                .unwrap_or(0)
                + 1;
            self.editor.mesh_recipe.regions.push(BoxRefinement {
                id,
                name: format!("Volume region {id}"),
                minimum: self.migration.low,
                maximum: self.migration.high,
                size: self.editor.mesh.min_size,
                transition: self.editor.mesh.global_size,
                enabled: true,
            });
        }
        if ui.button("Calculate smooth distance transitions").clicked() {
            let growth = self
                .editor
                .mesh_recipe
                .tuning
                .max_size_growth
                .unwrap_or(1.2);
            if growth > 1. && growth <= 2. {
                for r in &mut self.editor.mesh_recipe.refinements {
                    r.distance_max =
                        r.distance_min + ((r.size_max - r.size_min) / growth.ln()).max(r.size_min);
                }
            } else {
                self.message("Growth must be greater than 1 and at most 2.");
            }
        }
    }
}
impl State {
    pub fn mapping_active(&self) -> bool {
        self.mapping.is_some()
    }
    pub fn new() -> Self {
        Self {
            tolerance: 1e-6,
            high: Vec3::new(1., 1., 1.),
            profile_end: Vec3::new(1., 0., 0.),
            samples: 100,
            yplus: 1.,
            utau: 0.05,
            ..Self::default()
        }
    }
}
impl App {
    pub(super) fn profile_editor(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Line profiles and run comparison")
            .default_open(true)
            .show(ui, |ui| {
                ui.small("2D cell values sampled along a line; missing samples remain gaps. Select each saved run before adding its curve.");
                egui::ComboBox::from_id_salt("profile-field")
                    .selected_text(self.result_field.label())
                    .show_ui(ui, |ui| {
                        use ResultFieldKind::*;
                        for field in [VelocityMagnitude, Pressure, VelocityX, VelocityY, VelocityZ,
                            Temperature, VorticityMagnitude, VelocityDivergence, QCriterion,
                            StrainRate, TurbulentK, SpecificDissipation, TurbulentViscosity] {
                            let available = self.result.as_ref().is_some_and(|d|
                                d.scalar_values(field).len() == d.mesh().cell_count());
                            ui.add_enabled_ui(available, |ui|
                                ui.selectable_value(&mut self.result_field, field, field.label()));
                        }
                    });
                vector(ui, "Start (m)", &mut self.migration.profile_start);
                vector(ui, "End (m)", &mut self.migration.profile_end);
                ui.add(egui::DragValue::new(&mut self.migration.samples).range(2..=2000).prefix("Samples "));
                if ui.button("Add selected run / field").clicked() {
                    if let Err(e) = self.add_profile_curve() { self.message(e); }
                }
                ui.checkbox(&mut self.migration.chart, "Show profile chart");
                if ui.button("Import reference CSV…").clicked() {
                    if let Err(e) = self.import_profile_reference() { self.message(e); }
                }
                if ui.button("Export comparison CSV…").clicked() {
                    if let Some(path) = rfd::FileDialog::new().set_file_name("line-comparison.csv").save_file() {
                        let header = self.migration.context.map(|(a,b,f)| format!(
                            "# {}\n# start_m: {},{},{}\n# end_m: {},{},{}\n",
                            f.label(), a.x,a.y,a.z,b.x,b.y,b.z)).unwrap_or_default();
                        let text = format!("{}{}",header,workbench::series_plot::csv(&self.migration.curves,"distance_m"));
                        if let Err(e) = std::fs::write(path,text) { self.message(e.to_string()); }
                    }
                }
                if ui.button("Clear comparison").clicked() {
                    self.migration.curves.clear();
                    self.migration.context = None;
                    self.migration.chart = false;
                }
            });
    }
    pub(super) fn add_profile_curve(&mut self) -> Result<(), String> {
        let data = self.result.as_ref().ok_or("Select a result")?;
        if data.mesh().dimension() != MeshDimension::TwoD {
            return Err("Line sampling currently requires a 2D mesh".into());
        }
        let (a, b) = (self.migration.profile_start, self.migration.profile_end);
        let length = (b - a).norm();
        if !length.is_finite() || length <= f64::EPSILON {
            return Err("Choose distinct finite endpoints".into());
        }
        let context = (a, b, self.result_field);
        if self.migration.context.is_some_and(|old| old != context) {
            return Err("Clear comparison before changing the line or field".into());
        }
        let locator = StreamlineField::from_dataset(data).map_err(|e| e.to_string())?;
        let values = data.scalar_values(self.result_field);
        if values.len() != data.mesh().cell_count() {
            return Err("Selected field unavailable".into());
        }
        let count = self.migration.samples.clamp(2, 2000);
        let points: Vec<_> = (0..count)
            .map(|i| {
                let t = i as f64 / (count - 1) as f64;
                let value = locator
                    .locate_cell(a + (b - a) * t)
                    .and_then(|c| values.get(c).copied())
                    .filter(|v| v.is_finite());
                (t * length, value)
            })
            .collect();
        if points.iter().all(|p| p.1.is_none()) {
            return Err("Line contains no fluid samples".into());
        }
        let curve = workbench::series_plot::Series {
            name: format!("{} · {}", data.run_id(), self.result_field.label()),
            points,
        };
        if let Some(i) = self
            .migration
            .curves
            .iter()
            .position(|s| s.name == curve.name)
        {
            self.migration.curves[i] = curve;
        } else {
            if self.migration.curves.len() >= 8 {
                return Err("Maximum eight comparison curves.".into());
            }
            self.migration.curves.push(curve);
        }
        self.migration.context = Some(context);
        self.migration.chart = true;
        Ok(())
    }
    fn import_profile_reference(&mut self) -> Result<(), String> {
        if self.migration.context.is_none() || self.migration.curves.len() >= 8 {
            return Err("Add a run curve first; at most eight curves.".into());
        }
        let Some(path) = rfd::FileDialog::new()
            .add_filter("Distance,value CSV", &["csv"])
            .pick_file()
        else {
            return Ok(());
        };
        let text = std::fs::read_to_string(&path).map_err(|e| e.to_string())?;
        let mut points = Vec::new();
        let mut first = true;
        for (line_no, line) in text.lines().enumerate() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let columns: Vec<_> = line.split(',').collect();
            if columns.len() != 2 {
                return Err(format!("Line {}: require distance,value", line_no + 1));
            }
            match (
                columns[0].trim().parse::<f64>(),
                columns[1].trim().parse::<f64>(),
            ) {
                (Ok(x), Ok(y)) if x.is_finite() && y.is_finite() => points.push((x, Some(y))),
                _ if first && columns[0].trim().eq_ignore_ascii_case("distance") => {}
                _ => return Err(format!("Invalid reference row {}", line_no + 1)),
            }
            first = false;
        }
        if points.len() < 2 {
            return Err("At least two finite reference points required".into());
        }
        points.sort_by(|a, b| a.0.total_cmp(&b.0));
        if points.windows(2).any(|p| p[0].0 == p[1].0) {
            return Err("Reference distances must be unique".into());
        }
        self.migration.curves.push(workbench::series_plot::Series {
            name: format!(
                "Reference · {}",
                path.file_name().unwrap_or_default().to_string_lossy()
            ),
            points,
        });
        self.migration.chart = true;
        Ok(())
    }
    pub(super) fn profile_chart(&mut self, ui: &mut egui::Ui) {
        let series = self.migration.curves.clone();
        let title = self
            .migration
            .context
            .map(|(_, _, f)| f.label())
            .unwrap_or("Line profiles");
        self.native_series_plot(ui, &series, title, "Distance (m)");
    }
    pub(super) fn export_diagnostics(&mut self, ui: &mut egui::Ui) {
        if ui.button("Export engineering report…").clicked() {
            if let Some(path) = rfd::FileDialog::new()
                .set_file_name("engineering-report.json")
                .save_file()
            {
                let result = (|| -> Result<(), String> {
                    let mut report = self
                        .archived_report
                        .clone()
                        .unwrap_or_else(|| serde_json::json!({}));
                    if self.archived_report.is_none() {
                        if let (Some(m), Some(s)) = (self.session.mesh(), self.session.solution()) {
                            report["conservation"] = serde_json::to_value(
                                workbench::boundary_reports::conservation_report(
                                    &m.mesh,
                                    s,
                                    self.session.material().density,
                                ),
                            )
                            .map_err(|e| e.to_string())?;
                            report["surface_budgets"] =
                                serde_json::to_value(&s.budgets).map_err(|e| e.to_string())?;
                        }
                    }
                    if let Some(d) = &self.result {
                        report["run"] = serde_json::json!(d.run_id());
                        report["cells"] = serde_json::json!(d.mesh().cell_count());
                        report["field"] = serde_json::json!(self.result_field.label());
                        if let Ok((min, max)) = d.scalar_range(self.result_field) {
                            report["range"] = serde_json::json!({"min":min,"max":max});
                        }
                    }
                    report["version"] = serde_json::json!(env!("CARGO_PKG_VERSION"));
                    let mut file =
                        tempfile::NamedTempFile::new_in(path.parent().unwrap_or(Path::new(".")))
                            .map_err(|e| e.to_string())?;
                    serde_json::to_writer_pretty(file.as_file_mut(), &report)
                        .map_err(|e| e.to_string())?;
                    file.persist(path).map_err(|e| e.to_string())?;
                    Ok(())
                })();
                if let Err(e) = result {
                    self.message(e)
                } else {
                    self.message("Engineering report exported.")
                }
            }
        }
    }
}

impl State {
    pub(super) fn prepare_case_profile(&mut self, start: Vec3, end: Vec3) {
        self.profile_start = start;
        self.profile_end = end;
        self.curves.clear();
        self.context = None;
    }
    pub(super) fn add_case_reference(&mut self, reference: workbench::series_plot::Series) {
        self.curves.push(reference);
    }
}
