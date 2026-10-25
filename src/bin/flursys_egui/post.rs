//! Shared Solution/Results controls and native plots/exports.
use super::*;
use egui::{Align2, FontId, Pos2, Rect, Stroke};
use widgets::heading;

impl App {
    pub(super) fn post_view(&self) -> bool {
        self.stage == Stage::Results || (self.stage == Stage::Solution && self.page >= 7)
    }
    pub(super) fn compute_properties(&mut self, ui: &mut egui::Ui) {
        use flursys::compute::Backend;
        heading(ui, "EXECUTION DEVICE");
        egui::ComboBox::from_id_salt("execution-device")
            .selected_text(match self.editor.solver.compute.backend {
                Backend::Serial => "CPU · single thread",
                Backend::Parallel => "CPU · multiple threads",
                Backend::Gpu => "GPU · mixed precision",
            })
            .show_ui(ui, |ui| {
                ui.selectable_value(
                    &mut self.editor.solver.compute.backend,
                    Backend::Serial,
                    "CPU · single thread",
                );
                ui.selectable_value(
                    &mut self.editor.solver.compute.backend,
                    Backend::Parallel,
                    "CPU · multiple threads",
                );
                ui.selectable_value(
                    &mut self.editor.solver.compute.backend,
                    Backend::Gpu,
                    "GPU · mixed precision",
                );
            });
        heading(ui, "CPU WORKERS");
        let available = std::thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1);
        ui.label(format!("{available} logical CPUs available"));
        ui.horizontal(|ui| {
            ui.label("Threads · 0 = automatic");
            ui.add_enabled(
                self.editor.solver.compute.backend != Backend::Serial,
                egui::DragValue::new(&mut self.editor.solver.threads).range(0..=available.max(1)),
            );
        });
        ui.weak("GPU solving still uses CPU workers for assembly, checks and fallback.");
        if self.editor.solver.compute.backend == Backend::Gpu {
            heading(ui, "GPU ADAPTER");
            if ui.button("Refresh available GPUs").clicked() {
                match flursys::compute::devices() {
                    Ok(devices) => self.gpu_devices = devices,
                    Err(e) => self.message(e),
                }
            }
            egui::ComboBox::from_id_salt("gpu-adapter")
                .selected_text(if self.editor.solver.compute.adapter.is_empty() {
                    "Automatic hardware adapter"
                } else {
                    &self.editor.solver.compute.adapter
                })
                .show_ui(ui, |ui| {
                    ui.selectable_value(
                        &mut self.editor.solver.compute.adapter,
                        String::new(),
                        "Automatic hardware adapter",
                    );
                    for device in &self.gpu_devices {
                        ui.selectable_value(
                            &mut self.editor.solver.compute.adapter,
                            device.name.clone(),
                            format!("{} · {}", device.name, device.backend),
                        );
                    }
                });
            ui.checkbox(
                &mut self.editor.solver.compute.cpu_fallback,
                "Allow CPU fallback when GPU cannot complete",
            );
            ui.weak("Accepted corrections are checked against the f64 residual. Small systems may use CPU according to the row threshold.");
        }
        egui::CollapsingHeader::new("Advanced execution limits").show(ui, |ui| {
            let mut value = serde_json::to_value(&self.editor.solver.compute).unwrap_or_default();
            if let Some(map) = value.as_object_mut() {
                map.remove("backend");
                map.remove("adapter");
                map.remove("cpu_fallback");
                if self.editor.solver.compute.backend != Backend::Gpu {
                    map.retain(|k, _| !k.starts_with("gpu_"));
                }
            }
            widgets::fields(ui, "advanced-compute", &mut value);
            let mut merged = serde_json::to_value(&self.editor.solver.compute).unwrap_or_default();
            if let (Some(dst), Some(src)) = (merged.as_object_mut(), value.as_object()) {
                for (k, v) in src {
                    dst.insert(k.clone(), v.clone());
                }
            }
            if let Ok(compute) = serde_json::from_value(merged) {
                self.editor.solver.compute = compute;
            }
        });
    }
    pub(super) fn native_convergence(&mut self, ui: &mut egui::Ui) {
        let chart_top = ui.cursor().min;
        ui.horizontal_wrapped(|ui| {
            ui.checkbox(&mut self.curve_log, "Log Y");
            ui.checkbox(&mut self.curve_normalized, "Normalize");
        });
        let colors = [
            Color32::from_rgb(88, 196, 246),
            Color32::from_rgb(245, 184, 91),
            Color32::from_rgb(101, 212, 156),
            Color32::from_rgb(196, 155, 255),
        ];
        let names = [
            "Continuity RMS",
            "Momentum relative",
            "Velocity change",
            "Pressure linear",
        ];
        ui.horizontal_wrapped(|ui| {
            for i in 0..4 {
                ui.scope(|ui| {
                    ui.visuals_mut().override_text_color = Some(colors[i]);
                    ui.checkbox(&mut self.curve_visible[i], names[i]);
                });
            }
        });
        let (r, response) = ui.allocate_exact_size(
            ui.available_size().max(egui::vec2(180., 180.)),
            egui::Sense::hover(),
        );
        self.viewport_rect = Some(egui::Rect::from_min_max(chart_top, r.max));
        ui.painter().text(
            r.right_top(),
            Align2::RIGHT_TOP,
            concat!("FLURSYS v", env!("CARGO_PKG_VERSION")),
            FontId::proportional(11.),
            ui.visuals().weak_text_color(),
        );

        let plot = Rect::from_min_max(r.min + egui::vec2(82., 26.), r.max - egui::vec2(24., 48.));
        if plot.width() < 40. || plot.height() < 40. {
            return;
        }
        let p = ui.painter_at(r);
        let ink = ui.visuals().text_color();
        let mut series: [Vec<(f64, f64)>; 4] = Default::default();
        for (i, out) in series.iter_mut().enumerate() {
            if !self.curve_visible[i] {
                continue;
            }
            let base = if self.curve_normalized {
                self.plot_histories()[i]
                    .iter()
                    .find_map(|(_, v)| v.filter(|v| v.is_finite() && v.abs() > 0.))
                    .unwrap_or(1.)
                    .abs()
            } else {
                1.
            };
            for &(x, y) in &self.plot_histories()[i] {
                if let Some(y) = y.filter(|y| y.is_finite()) {
                    let y = y / base;
                    if self.curve_log && y <= 0. {
                        continue;
                    }
                    out.push((x, if self.curve_log { y.log10() } else { y }));
                }
            }
        }
        let mut low = [f64::INFINITY; 2];
        let mut high = [f64::NEG_INFINITY; 2];
        for &(x, y) in series.iter().flatten() {
            low[0] = low[0].min(x);
            low[1] = low[1].min(y);
            high[0] = high[0].max(x);
            high[1] = high[1].max(y);
        }
        if !low[0].is_finite() {
            p.text(
                r.center(),
                Align2::CENTER_CENTER,
                "No finite samples in selected curves",
                FontId::proportional(14.),
                ink,
            );
            return;
        }
        for i in 0..2 {
            if high[i] <= low[i] {
                high[i] = low[i] + 1.;
            }
        }
        let point = |x: f64, y: f64| {
            Pos2::new(
                plot.left() + ((x - low[0]) / (high[0] - low[0])) as f32 * plot.width(),
                plot.bottom() - ((y - low[1]) / (high[1] - low[1])) as f32 * plot.height(),
            )
        };
        for i in 0..=5 {
            let t = i as f64 / 5.;
            let x = low[0] + t * (high[0] - low[0]);
            let y = low[1] + t * (high[1] - low[1]);
            let q = point(x, y);
            let grid = Stroke::new(1., ui.visuals().widgets.noninteractive.bg_stroke.color);
            p.line_segment(
                [Pos2::new(q.x, plot.top()), Pos2::new(q.x, plot.bottom())],
                grid,
            );
            p.line_segment(
                [Pos2::new(plot.left(), q.y), Pos2::new(plot.right(), q.y)],
                grid,
            );
            p.text(
                Pos2::new(q.x, plot.bottom() + 8.),
                Align2::CENTER_TOP,
                format!("{x:.0}"),
                FontId::proportional(11.),
                ink,
            );
            let label = if self.curve_log {
                format!("{:.1e}", 10f64.powf(y))
            } else {
                format!("{y:.2e}")
            };
            p.text(
                Pos2::new(plot.left() - 8., q.y),
                Align2::RIGHT_CENTER,
                label,
                FontId::proportional(11.),
                ink,
            );
        }
        p.text(
            Pos2::new(plot.center().x, r.bottom() - 4.),
            Align2::CENTER_BOTTOM,
            "Iteration / physical step",
            FontId::proportional(12.),
            ink,
        );
        p.text(
            Pos2::new(plot.left(), r.top()),
            Align2::LEFT_TOP,
            if self.curve_normalized {
                "Residual / initial residual"
            } else {
                "Residual (individual definitions)"
            },
            FontId::proportional(12.),
            ink,
        );
        for i in 0..4 {
            for pair in series[i].windows(2) {
                p.line_segment(
                    [point(pair[0].0, pair[0].1), point(pair[1].0, pair[1].1)],
                    Stroke::new(1.7, colors[i]),
                );
            }
        }
        if let Some(pos) = response.hover_pos().filter(|pos| plot.contains(*pos)) {
            let x = low[0] + ((pos.x - plot.left()) / plot.width()) as f64 * (high[0] - low[0]);
            p.line_segment(
                [
                    Pos2::new(pos.x, plot.top()),
                    Pos2::new(pos.x, plot.bottom()),
                ],
                Stroke::new(1., ink),
            );
            response.on_hover_ui(|ui| {
                for i in 0..4 {
                    if self.curve_visible[i] {
                        if let Some((iteration, y)) = self.plot_histories()[i]
                            .iter()
                            .min_by(|a, b| (a.0 - x).abs().total_cmp(&(b.0 - x).abs()))
                        {
                            ui.colored_label(
                                colors[i],
                                format!(
                                    "{} · iteration {:.0} · {:.6e}",
                                    names[i],
                                    iteration,
                                    y.unwrap_or(f64::NAN)
                                ),
                            );
                        }
                    }
                }
            });
        }
    }
    pub(super) fn post_toolbar(&mut self, ui: &mut egui::Ui) {
        let mut chosen = None;
        ui.horizontal_wrapped(|ui| {
            if self.post_view() {
                ui.checkbox(&mut self.live_fields, "Live fields");
                ui.label("Refresh");
                let response = ui.add(
                    egui::DragValue::new(&mut self.preferences.live_refresh_seconds)
                        .range(0.1..=60.0)
                        .speed(0.1)
                        .max_decimals(2)
                        .suffix(" s"),
                ).on_hover_text("Minimum interval between live full-field updates. Changes apply immediately; disk recording and playback FPS are independent.");
                if response.changed() {
                    if let Some(path) = Self::preferences_path() {
                        let result = (|| -> Result<(), String> {
                            let parent = path.parent().ok_or("No preferences directory")?;
                            std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
                            let mut file = tempfile::NamedTempFile::new_in(parent).map_err(|e| e.to_string())?;
                            serde_json::to_writer_pretty(file.as_file_mut(), &self.preferences).map_err(|e| e.to_string())?;
                            file.persist(path).map_err(|e| e.to_string())?;
                            Ok(())
                        })();
                        if let Err(e) = result { self.message(e); }
                    }
                }
            }
            ui.label("Dataset");
            egui::ComboBox::from_id_salt("visible-run-selector")
                .selected_text(if self.result_run.is_empty() {
                    "Latest field"
                } else {
                    &self.result_run
                })
                .show_ui(ui, |ui| {
                    if ui
                        .add_enabled(
                            self.session.solution().is_some()
                                || self.jobs.is_running()
                                || self.jobs.completed_iterations() > 0,
                            egui::Button::new("Latest accepted solution").selected(
                                self.result_run == "current" || self.result_run == "live",
                            ),
                        )
                        .clicked()
                    {
                        chosen = Some("current".to_string());
                    }
                    for run in &self.project.runs {
                        if run.solution_path.is_some()
                            && ui
                                .selectable_label(
                                    self.result_run == run.id,
                                    format!(
                                        "{} · {:?} · {} iterations",
                                        run.id,
                                        run.status,
                                        run.iterations.unwrap_or(0)
                                    ),
                                )
                                .clicked()
                        {
                            chosen = Some(run.id.clone());
                        }
                    }
                });
            if ui.button("PNG image…").clicked() {
                self.begin_export(false);
            }
            if (self.stage == Stage::Results && self.page == 0
                || self.stage == Stage::Solution && self.page == 7)
                && ui.button("Animated GIF…").clicked()
            {
                self.begin_export(true);
            }
        });
        if let Some(id) = chosen {
            if id == "current" {
                if self.session.solution().is_some() {
                    self.activate_solution();
                    self.result_run = "current".into();
                } else if !self.jobs.is_running() && self.jobs.completed_iterations() > 0 {
                    self.preview_retained_fields();
                } else if self.jobs.is_running() {
                    self.result = None;
                    self.result_cache = None;
                    self.result_run = "live".into();
                    self.live_iteration = usize::MAX;
                }
                self.probes.clear();
                self.streamlines.clear();
                self.plane_frames.clear();
                self.plane_snapshot = None;
                self.plane_index = 0;
                self.plane_playing = false;
            } else {
                let stage = self.stage;
                let page = self.page;
                if let Err(e) = self.open_run(&id) {
                    self.message(e);
                }
                self.stage = stage;
                self.page = page;
            }
        }
        ui.horizontal_wrapped(|ui| {
            if !self.field_frames.is_empty()
                && (self.plane_index == 0 || self.plane_frames.is_empty())
            {
                if ui
                    .checkbox(&mut self.field_playing, "Play saved fields")
                    .changed()
                    && self.field_playing
                {
                    self.live_fields = false;
                    self.plane_playing = false;
                }
                let last = self.field_frames.len() - 1;
                if ui
                    .add(egui::Slider::new(&mut self.field_frame, 0..=last).text("Field frame"))
                    .changed()
                {
                    self.live_fields = false;
                    self.read_field_frame();
                }
                ui.label(format!("Iteration / step {}", self.result_iteration));
            }
            if !self.plane_frames.is_empty() {
                if ui
                    .checkbox(&mut self.plane_playing, "Play recorded fields")
                    .changed()
                    && self.plane_playing
                {
                    self.live_fields = false;
                    self.field_playing = false;
                }
                let end = self.plane_frames.len() - 1;
                if ui
                    .add(egui::Slider::new(&mut self.plane_frame, 0..=end).text("Frame"))
                    .changed()
                {
                    self.read_plane_frame();
                }
            }
            if !self.streamlines.is_empty() {
                ui.checkbox(&mut self.flow_playing, "Animate tracers");
            }
            if self.plane_index > 0 {
                if let Some(s) = &self.plane_snapshot {
                    ui.label(format!(
                        "Iteration {}{}",
                        s.iteration,
                        s.physical_time
                            .map(|t| format!(" · t={t:.5e} s"))
                            .unwrap_or_default()
                    ));
                }
            }

            ui.add(
                egui::DragValue::new(&mut self.playback_fps)
                    .range(1. ..=60.)
                    .suffix(" FPS"),
            );
        });
    }
    pub(super) fn history_properties(&mut self, ui: &mut egui::Ui) {
        heading(ui, "SAVED RUNS");
        ui.weak("Each dataset retains its own mesh. Choose a run to display fields, then choose a recorded plane for its history.");
        let mut chosen = None;
        for run in &self.project.runs {
            if ui
                .add_enabled(
                    run.solution_path.is_some(),
                    egui::Button::new(format!(
                        "{} · {:?} · {}",
                        run.id,
                        run.status,
                        run.iterations.unwrap_or(0)
                    )),
                )
                .clicked()
            {
                chosen = Some(run.id.clone());
            }
        }
        if let Some(id) = chosen {
            let stage = self.stage;
            if let Err(e) = self.open_run(&id) {
                self.message(e);
            }
            if stage == Stage::Solution {
                self.stage = stage;
                self.page = 7;
            }
        }
        heading(ui, "AUTOSAVED SOLVER FIELDS");
        if ui.button("View saved checkpoint…").clicked() {
            if let Some(path) = rfd::FileDialog::new()
                .add_filter("Checkpoint", &["json"])
                .pick_file()
            {
                if let Err(e) = self.preview_checkpoint(&path) {
                    self.message(e);
                }
            }
        }
        heading(ui, "AUTOSAVED PLANE FIELDS");
        if ui.button("Open recording folder…").clicked() {
            if let Some(folder) = rfd::FileDialog::new().pick_folder() {
                if self.plane_index == 0 {
                    self.message("Select a recorded plane in Display first.");
                } else {
                    self.load_plane_frames(&folder);
                }
            }
        }
        if let Some(folder) = &self.recording {
            ui.label(folder.display().to_string());
        }
        ui.weak("Restart checkpoints retain solver state; plane recordings retain sampled velocity and pressure frames. They are separate from a completed run.");
    }
    pub(super) fn probe_properties(&mut self, ui: &mut egui::Ui) {
        heading(ui, "POINT PROBES");
        self.probe_history_controls(ui);

        ui.checkbox(&mut self.probe_enabled, "Click the field to add probes");
        ui.weak("Values are sampled at the selected cell centre; repeated clicks on that cell do not duplicate it.");
        widgets::vector(ui, "Coordinate (m)", &mut self.probe_position);
        if ui.button("Add nearest cell to coordinate").clicked() {
            if let Some(data) = &self.result {
                if let Some((i, _)) =
                    data.mesh()
                        .cells()
                        .iter()
                        .enumerate()
                        .min_by(|(_, a), (_, b)| {
                            (a.center - self.probe_position)
                                .norm()
                                .total_cmp(&(b.center - self.probe_position).norm())
                        })
                {
                    if let Some(probe) = data.probe_cell(i) {
                        if !self.probes.iter().any(|p| p.cell_index == i) {
                            self.probes.push(probe);
                        }
                    }
                }
            }
        }
        ui.horizontal(|ui| {
            if ui.button("Clear probes").clicked() {
                self.probes.clear();
            }
            if ui.button("Export CSV…").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .set_file_name("probes.csv")
                    .save_file()
                {
                    let mut csv =
                        String::from("run,cell,x,y,z,p_over_rho,ux,uy,uz,speed,temperature\n");
                    for p in &self.probes {
                        let t = self
                            .result
                            .as_ref()
                            .filter(|_| self.plane_index == 0)
                            .and_then(|d| {
                                d.scalar_values(ResultFieldKind::Temperature)
                                    .get(p.cell_index)
                            })
                            .copied()
                            .unwrap_or(f64::NAN);
                        csv += &format!(
                            "{},{},{},{},{},{},{},{},{},{},{}\n",
                            p.run_id,
                            p.cell_index,
                            p.center.x,
                            p.center.y,
                            p.center.z,
                            p.pressure,
                            p.velocity.x,
                            p.velocity.y,
                            p.velocity.z,
                            p.velocity_magnitude,
                            t
                        );
                    }
                    if let Err(e) = std::fs::write(path, csv) {
                        self.message(e.to_string());
                    }
                }
            }
        });
        let mut remove = None;
        for (i, p) in self.probes.iter().enumerate() {
            egui::CollapsingHeader::new(format!("P{} · cell {}", i + 1, p.cell_index))
                .default_open(true)
                .show(ui, |ui| {
                    ui.label(format!(
                        "Position: {:.4e}, {:.4e}, {:.4e} m",
                        p.center.x, p.center.y, p.center.z
                    ));
                    ui.label(format!("p/ρ: {:.6e} m²/s²", p.pressure));
                    ui.label(format!(
                        "U: {:.5e}, {:.5e}, {:.5e} m/s",
                        p.velocity.x, p.velocity.y, p.velocity.z
                    ));
                    ui.label(format!("Speed: {:.6e} m/s", p.velocity_magnitude));
                    if let Some(t) = self
                        .result
                        .as_ref()
                        .filter(|_| self.plane_index == 0)
                        .and_then(|d| {
                            d.scalar_values(ResultFieldKind::Temperature)
                                .get(p.cell_index)
                        })
                    {
                        ui.label(format!("Temperature: {t:.6} K"));
                    }
                    if ui.small_button("Remove").clicked() {
                        remove = Some(i);
                    }
                });
        }
        if let Some(i) = remove {
            self.probes.remove(i);
        }
    }
    pub(super) fn flow_properties(&mut self, ui: &mut egui::Ui) {
        heading(ui, "FLOW PATHS");
        let supported = self.plane_index > 0 && self.plane_snapshot.is_some()
            || self
                .result
                .as_ref()
                .is_some_and(|d| d.mesh().dimension() == MeshDimension::TwoD);
        if ui
            .add_enabled(
                supported,
                egui::Button::new("Seed streamlines on selected section"),
            )
            .on_hover_text("Use a 2D dataset or select a section through a 3D dataset")
            .clicked()
        {
            self.seed_flow();
        }
        ui.add_enabled_ui(!self.streamlines.is_empty(), |ui| {
            ui.checkbox(&mut self.flow_playing, "Animate flow tracers");
        });
        ui.weak("Tracer motion illustrates the selected stationary velocity field; it is not a transient solution.");
    }
    pub(super) fn seed_flow(&mut self) {
        self.streamlines.clear();
        if self.plane_index > 0 {
            if let Some(s) = &self.plane_snapshot {
                let plane = &s.plane;
                let project = |p: Vec3| {
                    egui::pos2(
                        (p - plane.origin).dot(plane.u) as f32,
                        (p - plane.origin).dot(plane.v) as f32,
                    )
                };
                let polygons: Vec<Vec<_>> = s
                    .samples
                    .iter()
                    .map(|sample| {
                        sample
                            .geometry
                            .polygon
                            .iter()
                            .map(|p| project(*p))
                            .collect()
                    })
                    .collect();
                let step = self.camera.radius * 0.006;
                for seed in s
                    .samples
                    .iter()
                    .step_by(s.samples.len().div_ceil(12).max(1))
                {
                    let mut point = seed.geometry.point;
                    let mut path = StreamlinePath { points: Vec::new() };
                    for _ in 0..200 {
                        let Some(i) = polygons
                            .iter()
                            .position(|poly| inside(project(point), poly))
                        else {
                            break;
                        };
                        let velocity = s.samples[i].velocity;
                        let tangent =
                            plane.u * velocity.dot(plane.u) + plane.v * velocity.dot(plane.v);
                        let speed = tangent.norm();
                        if speed < 1e-12 {
                            break;
                        }
                        path.points.push(point);
                        point += tangent * (step / speed);
                    }
                    if path.points.len() > 1 {
                        self.streamlines.push(path);
                    }
                }
            }
            return;
        }

        if let Some(data) = &self.result {
            match StreamlineField::from_dataset(data) {
                Ok(field) => {
                    let stride = (data.mesh().cell_count() / 24).max(1);
                    for cell in data.mesh().cells().iter().step_by(stride).take(24) {
                        if let Ok(path) = field.integrate(
                            cell.center,
                            StreamlineDirection::Forward,
                            StreamlineOptions {
                                step_size: self.camera.radius * 0.005,
                                max_steps: 1500,
                                max_length: self.camera.radius * 12.,
                                stagnation_speed: 1e-10,
                            },
                        ) {
                            if path.points.len() > 1 {
                                self.streamlines.push(path);
                            }
                        }
                    }
                }
                Err(e) => self.message(e.to_string()),
            }
        }
    }
    pub(super) fn post_logic(&mut self, ctx: &egui::Context) {
        self.poll_probe_history();
        self.export_logic(ctx);

        if self.jobs.is_running()
            && self.post_view()
            && self.live_fields
            && self.export.is_none()
            && !self.field_playing
            && !self.plane_playing
            && self.jobs.completed_iterations() != self.live_iteration
            && (self.result.is_none() || self.result_run == "current" || self.result_run == "live")
            && self.live_refresh.elapsed().as_secs_f64() >= self.preferences.live_refresh_seconds
        {
            self.live_refresh = Instant::now();
            if let Some(mesh) = self.session.mesh() {
                if let Some((
                    _,
                    flursys::IncompressibleInitialConditions::Restart {
                        velocity,
                        pressure,
                        temperature,
                        turbulence,
                        ..
                    },
                )) = self.jobs.retained_fields(mesh.mesh.id())
                {
                    if let Ok(data) = ResultDataset::from_values(
                        "live",
                        &mesh.mesh,
                        pressure.values().to_vec(),
                        velocity.values().to_vec(),
                    )
                    .and_then(|d| d.with_temperature(temperature.map(|t| t.values().to_vec())))
                    {
                        let data = data.with_turbulence(turbulence.as_ref());
                        self.live_iteration = self.jobs.completed_iterations();
                        if self.plane_index == 0 {
                            for p in &mut self.probes {
                                if let Some(sample) = data.probe_cell(p.cell_index) {
                                    *p = sample;
                                }
                            }
                        }
                        self.result_cache = ResultRenderCache::build(&data).ok();
                        self.result = Some(data);
                        self.result_run = "live".into();
                    }
                }
            }
        }
        if self.field_playing
            && !self.field_frames.is_empty()
            && self.plane_tick.elapsed().as_secs_f64() > 1. / self.playback_fps.max(1.)
        {
            self.plane_tick = Instant::now();
            self.field_frame = (self.field_frame + 1) % self.field_frames.len();
            self.read_field_frame();
        }
        let dt = self.post_tick.elapsed().as_secs_f64().min(0.2);
        self.post_tick = Instant::now();
        if self.flow_playing {
            self.flow_phase = (self.flow_phase + dt * 0.18) % 1.;
        }
    }
}
impl App {
    pub(super) fn quality_properties(&mut self, ui: &mut egui::Ui) {
        heading(ui, "MESH QUALITY");
        let metrics = [
            MeshQualityMetric::NonOrthogonality,
            MeshQualityMetric::AspectRatio,
            MeshQualityMetric::Skewness,
            MeshQualityMetric::CellMeasure,
            MeshQualityMetric::AdjacentVolumeRatio,
        ];
        self.mesh_quality = self.mesh_quality.min(4);
        egui::ComboBox::from_id_salt("quality-metric")
            .selected_text(metrics[self.mesh_quality].label())
            .show_ui(ui, |ui| {
                for (i, m) in metrics.iter().enumerate() {
                    ui.selectable_value(&mut self.mesh_quality, i, m.label());
                }
            });
        ui.checkbox(&mut self.quality_edges, "Cell edges over metric colours");
        let mut fixed = self.quality_range.is_some();
        if ui
            .checkbox(&mut fixed, "Fixed metric colour range")
            .changed()
        {
            self.quality_range = if fixed {
                Some([0., if self.mesh_quality == 0 { 90. } else { 1. }])
            } else {
                None
            };
        }
        if let Some([a, b]) = &mut self.quality_range {
            widgets::number(ui, "Minimum", a);
            widgets::number(ui, "Maximum", b);
            if *b <= *a {
                *b = *a + 1.;
            }
        }
        let mut threshold = self.quality_above.is_some();
        if ui
            .checkbox(&mut threshold, "Only cells above threshold")
            .changed()
        {
            self.quality_above = threshold.then_some(70.);
        }
        if let Some(v) = &mut self.quality_above {
            widgets::number(ui, "Metric threshold", v);
        }
        ui.checkbox(&mut self.mesh_cut, "Cutaway · whole cells");
        if self.mesh_cut {
            ui.horizontal(|ui| {
                for (i, n) in ["X", "Y", "Z"].iter().enumerate() {
                    ui.selectable_value(&mut self.mesh_cut_axis, i, *n);
                }
            });
            ui.add(egui::Slider::new(&mut self.mesh_cut_fraction, 0. ..=1.).text("Cut position"));
        }
        if let Some(cache) = &self.mesh_cache {
            let values = cache.quality().values(metrics[self.mesh_quality]);
            let mut sorted: Vec<_> = values.iter().copied().filter(|v| v.is_finite()).collect();
            sorted.sort_by(f64::total_cmp);
            if sorted.is_empty() {
                return;
            }
            let low = sorted[0];
            let high = *sorted.last().unwrap();
            let mean = sorted.iter().sum::<f64>() / sorted.len() as f64;
            ui.label(format!(
                "{} cells · {} finite samples",
                values.len(),
                sorted.len()
            ));
            egui::Grid::new("quality-statistics")
                .num_columns(2)
                .striped(true)
                .show(ui, |ui| {
                    for (name, value) in [
                        ("Minimum", low),
                        ("Mean", mean),
                        ("Median", sorted[sorted.len() / 2]),
                        ("95th percentile", sorted[(sorted.len() - 1) * 95 / 100]),
                        ("Maximum", high),
                    ] {
                        ui.label(name);
                        ui.monospace(format!("{value:.5e}"));
                        ui.end_row();
                    }
                });
            let mut bins = [0usize; 12];
            for v in &sorted {
                let i = (((v - low) / (high - low).max(1e-30) * 12.).floor() as usize).min(11);
                bins[i] += 1;
            }
            heading(ui, "CELL DISTRIBUTION");
            let (rect, response) = ui
                .allocate_exact_size(egui::vec2(ui.available_width(), 140.), egui::Sense::hover());
            let p = ui.painter_at(rect);
            let max = *bins.iter().max().unwrap_or(&1);
            let plot = rect.shrink2(egui::vec2(4., 22.));
            for (i, count) in bins.iter().enumerate() {
                let x = plot.left() + plot.width() * i as f32 / 12.;
                let width = plot.width() / 12. - 2.;
                let height = plot.height() * *count as f32 / max.max(1) as f32;
                let color = Color32::from_rgb((80 + i * 14) as u8, (190 - i * 9) as u8, 110);
                p.rect_filled(
                    Rect::from_min_max(
                        Pos2::new(x, plot.bottom() - height),
                        Pos2::new(x + width, plot.bottom()),
                    ),
                    1.,
                    color,
                );
            }
            p.text(
                rect.left_bottom(),
                Align2::LEFT_BOTTOM,
                format!("{low:.2e}"),
                FontId::proportional(11.),
                ui.visuals().text_color(),
            );
            p.text(
                rect.right_bottom(),
                Align2::RIGHT_BOTTOM,
                format!("{high:.2e}"),
                FontId::proportional(11.),
                ui.visuals().text_color(),
            );
            response.on_hover_ui(|ui| {
                for (i, count) in bins.iter().enumerate() {
                    ui.label(format!(
                        "{:.3e}–{:.3e}: {count} cells",
                        low + (high - low) * i as f64 / 12.,
                        low + (high - low) * (i + 1) as f64 / 12.
                    ));
                }
            });
            if self.mesh_quality == 0 {
                ui.label(format!(
                    "Above 70°: {} cells",
                    values.iter().filter(|v| **v > 70.).count()
                ));
            }
            ui.weak("Colour range uses the actual selected metric. Skewness is this solver's centre-line displacement ratio, not another mesher's normalized definition.");
            egui::CollapsingHeader::new("Worst cells").show(ui, |ui| {
                let mut order: Vec<_> = values
                    .iter()
                    .enumerate()
                    .filter(|(_, v)| v.is_finite())
                    .collect();
                order.sort_by(|a, b| b.1.total_cmp(a.1));
                for (i, v) in order.into_iter().take(15) {
                    ui.monospace(format!("Cell {i}: {v:.6e}"));
                }
            });
            if ui.button("Export quality CSV…").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .set_file_name("mesh-quality.csv")
                    .save_file()
                {
                    let mut csv=String::from("cell,non_orthogonality,aspect_ratio,skewness,cell_measure,adjacent_volume_ratio\n");
                    for i in 0..values.len() {
                        csv += &format!(
                            "{i},{},{},{},{},{}\n",
                            cache.quality().values(metrics[0])[i],
                            cache.quality().values(metrics[1])[i],
                            cache.quality().values(metrics[2])[i],
                            cache.quality().values(metrics[3])[i],
                            cache.quality().values(metrics[4])[i]
                        );
                    }
                    if let Err(e) = std::fs::write(path, csv) {
                        self.message(e.to_string());
                    }
                }
            }
        } else {
            ui.weak("Generate a mesh before inspecting quality.");
        }
    }
}
impl App {
    pub(super) fn ensure_result(&mut self) {
        let stage = self.stage;
        let page = self.page;
        if self.session.solution().is_some() {
            self.activate_solution();
        } else if self.preview_retained_fields() {
        } else if let Some(id) = self
            .project
            .runs
            .iter()
            .rev()
            .find(|r| r.solution_path.is_some())
            .map(|r| r.id.clone())
        {
            if let Err(e) = self.open_run(&id) {
                self.message(e);
            }
        }
        self.stage = stage;
        self.page = page;
    }
}
pub(super) fn plane_csv(s: &plane_monitor::PlaneSnapshot) -> String {
    plane_monitor::csv_text(s)
}
fn inside(point: egui::Pos2, polygon: &[egui::Pos2]) -> bool {
    let mut hit = false;
    for i in 0..polygon.len() {
        let a = polygon[i];
        let b = polygon[(i + 1) % polygon.len()];
        if (a.y > point.y) != (b.y > point.y)
            && point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x
        {
            hit = !hit;
        }
    }
    hit
}
impl App {
    pub(super) fn pick_plane_probe(&mut self, position: egui::Pos2, rect: egui::Rect) {
        if !self.probe_enabled {
            return;
        }
        if let Some(s) = &self.plane_snapshot {
            for sample in &s.samples {
                let polygon: Vec<_> = sample
                    .geometry
                    .polygon
                    .iter()
                    .filter_map(|p| self.camera.screen(*p, rect).map(|p| p.0))
                    .collect();
                if inside(position, &polygon) {
                    let probe = ResultProbe {
                        run_id: format!("{}@{}", self.result_run, s.iteration),
                        cell_index: sample.geometry.cell,
                        center: sample.geometry.point,
                        pressure: sample.pressure,
                        velocity: sample.velocity,
                        velocity_magnitude: sample.velocity.norm(),
                    };
                    if !self.probes.iter().any(|p| p.cell_index == probe.cell_index) {
                        self.probes.push(probe);
                    }
                    break;
                }
            }
        }
    }
    pub(super) fn section_creation(&mut self, ui: &mut egui::Ui) {
        if self
            .result
            .as_ref()
            .is_none_or(|d| d.mesh().dimension() != MeshDimension::ThreeD)
        {
            return;
        }
        heading(ui, "SECTION LOCATION");
        ui.horizontal(|ui| {
            for (name, u, v) in [
                ("XY", Vec3::new(1., 0., 0.), Vec3::new(0., 1., 0.)),
                ("XZ", Vec3::new(1., 0., 0.), Vec3::new(0., 0., 1.)),
                ("YZ", Vec3::new(0., 1., 0.), Vec3::new(0., 0., 1.)),
            ] {
                if ui
                    .button(format!("+ {name}"))
                    .on_hover_text(
                        "Create a section at the dataset centre; edit its origin in Display recording settings",
                    )
                    .clicked()
                {
                    let data = self.result.as_ref().unwrap();
                    let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
                    let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
                    for p in data.mesh().points() {
                        let p = p.position;
                        lo.x = lo.x.min(p.x);
                        lo.y = lo.y.min(p.y);
                        lo.z = lo.z.min(p.z);
                        hi.x = hi.x.max(p.x);
                        hi.y = hi.y.max(p.y);
                        hi.z = hi.z.max(p.z);
                    }
                    let id = format!(
                        "section-{}",
                        std::time::SystemTime::now()
                            .duration_since(std::time::UNIX_EPOCH)
                            .unwrap_or_default()
                            .as_nanos()
                    );
                    self.project
                        .plane_monitors
                        .push(plane_monitor::FlowPlaneMonitor {
                            id,
                            name: format!(
                                "{name} section {}",
                                self.project.plane_monitors.len() + 1
                            ),
                            origin: (lo + hi) * 0.5,
                            u,
                            v,
                            enabled: false,
                            every: 10,
                            display: Default::default(),
                        });
                    self.plane_index = self.project.plane_monitors.len();
                    self.plane_frames.clear();
                    self.streamlines.clear();
                    self.probes.clear();
                    self.dirty = true;
                    self.refresh_plane_preview();
                }
            }
        });
    }
    pub(super) fn refresh_plane_preview(&mut self) {
        if self.plane_index == 0 {
            return;
        }
        self.plane_playing = false;
        self.plane_frames.clear();
        self.frame_scan = None;
        self.streamlines.clear();

        if let Some(p) = self.project.plane_monitors.get(self.plane_index - 1) {
            if let Some(d) = &self.result {
                match plane_monitor::intersect(d.mesh(), p).and_then(|cells| {
                    plane_monitor::sample(p, &cells, self.iteration, d.velocity(), d.pressure())
                }) {
                    Ok(snapshot) => {
                        self.camera.fit_mesh(d.mesh());
                        let n = p.normal();
                        self.camera.camera = self
                            .camera
                            .camera
                            .looking_from(cad_geom::Vec3::new(n.x, n.y, n.z));
                        self.plane_snapshot = Some(snapshot);
                    }
                    Err(e) => self.message(e),
                }
            }
        }
    }
}
impl App {
    fn preview_checkpoint(&mut self, path: &Path) -> Result<(), String> {
        let mesh = &self
            .session
            .mesh()
            .ok_or("Open the matching project mesh before viewing a checkpoint")?
            .mesh;
        let (iteration, initial) = workbench::checkpoint::load(path, mesh)?;
        let data = match initial {
            flursys::IncompressibleInitialConditions::Restart {
                velocity,
                pressure,
                temperature,
                turbulence,
                ..
            } => ResultDataset::from_values(
                format!("checkpoint@{iteration}"),
                mesh,
                pressure.values().to_vec(),
                velocity.values().to_vec(),
            )
            .and_then(|d| d.with_temperature(temperature.map(|t| t.values().to_vec())))
            .map(|d| d.with_turbulence(turbulence.as_ref())),
            flursys::IncompressibleInitialConditions::Fields { velocity, pressure } => {
                ResultDataset::from_values(
                    format!("checkpoint@{iteration}"),
                    mesh,
                    pressure.values().to_vec(),
                    velocity.values().to_vec(),
                )
            }
            _ => return Err("Checkpoint does not contain sampled fields".into()),
        }
        .map_err(|e| e.to_string())?;
        self.result_run = format!("checkpoint@{iteration}");
        self.result_iteration = iteration;
        self.result_cache = Some(ResultRenderCache::build(&data).map_err(|e| e.to_string())?);
        self.camera.fit_mesh(data.mesh());
        self.result = Some(data);
        self.archived_report = None;
        self.probes.clear();
        self.streamlines.clear();
        self.plane_index = 0;
        self.plane_snapshot = None;
        self.plane_frames.clear();
        self.plane_playing = false;
        if self.stage == Stage::Solution {
            self.page = 7;
        } else {
            self.stage = Stage::Results;
            self.page = 0;
        }
        self.message(format!(
            "Viewing checkpoint at iteration {iteration}; solver state was not replaced."
        ));
        Ok(())
    }
}
