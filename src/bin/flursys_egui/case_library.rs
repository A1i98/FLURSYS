use super::*;
use workbench::case_format::{CaseContext, CaseMetadata};

pub(super) fn metadata_panel(
    ui: &mut egui::Ui,
    metadata: &CaseMetadata,
    preset: Option<&mut String>,
) {
    ui.strong(format!(
        "Case revision {} · {}",
        metadata.revision,
        if metadata.verification.status.is_empty() {
            "unvalidated"
        } else {
            &metadata.verification.status
        }
    ));
    ui.label(&metadata.verification.notes);
    if let Some(preset) = preset {
        if !metadata.presets.iter().any(|p| p.id == *preset) {
            preset.clear();
        }
        egui::ComboBox::from_id_salt("case-study-preset")
            .selected_text(
                metadata
                    .presets
                    .iter()
                    .find(|p| p.id == *preset)
                    .map_or("Original settings", |p| p.name.as_str()),
            )
            .show_ui(ui, |ui| {
                ui.selectable_value(preset, String::new(), "Original settings");
                for p in &metadata.presets {
                    ui.selectable_value(preset, p.id.clone(), &p.name)
                        .on_hover_text(&p.description);
                }
            });
    }
    ui.collapsing("Scientific sources", |ui| {
        for s in &metadata.sources {
            ui.strong(&s.title);
            ui.small(format!(
                "{} · {} · {}",
                s.authors.join(", "),
                s.year.map(|y| y.to_string()).unwrap_or_default(),
                s.relationship
            ));
            if let Some(url) = &s.url {
                ui.hyperlink_to("Open source", url);
            } else if let Some(doi) = &s.doi {
                ui.hyperlink_to(doi, format!("https://doi.org/{doi}"));
            }
            ui.label(&s.notes);
            ui.add_space(5.);
        }
    });
    ui.collapsing("Dimensionless definitions", |ui| {
        for n in &metadata.dimensionless {
            ui.strong(format!(
                "{} = {}",
                n.symbol,
                n.value
                    .map(|v| format!("{v:.6e}"))
                    .unwrap_or_else(|| "unspecified".into())
            ));
            ui.monospace(&n.formula);
            for s in &n.scales {
                ui.small(format!(
                    "{} = {:.6e} {} · {}",
                    s.symbol, s.value, s.unit, s.definition
                ));
            }
            ui.label(&n.notes);
        }
    });
    ui.collapsing("References and error criteria", |ui| {
        for r in &metadata.comparisons {
            ui.strong(format!("{} · {}", r.quantity, r.unit));
            ui.small(format!("{} · {}", r.kind, r.error_metric));
            if let Some(value) = r.target {
                ui.monospace(format!("Reference target: {value:.8e}"));
            }
            if let Some(t) = r.time {
                ui.label(format!("Reference time: {t:.6e}"));
            }
            if let Some(expr) = &r.expression {
                ui.monospace(expr);
            }
            ui.label(&r.procedure);
            ui.small(format!(
                "Absolute tolerance: {:?} · relative tolerance: {:?}",
                r.absolute_tolerance, r.relative_tolerance
            ));
        }
        if metadata.comparisons.is_empty() {
            ui.weak("No reference values are assigned to this illustrative recipe.");
        }
    });
    ui.collapsing("Recommended display and sampling", |ui| {
        ui.label(format!(
            "{} · {} · {} · {}",
            metadata.display.field,
            metadata.display.mode,
            metadata.display.palette,
            metadata.display.plane
        ));
        for p in &metadata.display.probes {
            ui.small(format!(
                "{}: ({:.4e}, {:.4e}, {:.4e}) · {}",
                p.name, p.position.x, p.position.y, p.position.z, p.sampling
            ));
        }
        for p in &metadata.display.plots {
            ui.strong(&p.name);
            ui.small(format!("{} · {}", p.kind, p.quantity));
            ui.label(&p.notes);
        }
    });
    ui.collapsing("Portable assets", |ui| {
        for asset in &metadata.assets {
            ui.label(format!("{} · {}", asset.path.display(), asset.role));
            if let Some(hash) = &asset.sha256 {
                ui.small(format!("SHA256 {hash}"));
            }
        }
    });
}
impl App {
    pub(super) fn create_angle_design_points(&mut self) -> Result<usize, String> {
        if self.jobs.is_running() {
            return Err("Stop the active solve before generating design points.".into());
        }
        self.apply_editor()?;
        let root = self
            .workspace
            .as_ref()
            .ok_or("Save the workspace before generating design points.")?
            .join("design-points");
        let start = self.parametric_start_angle;
        let end = self.parametric_end_angle;
        let step = self.parametric_angle_step;
        if ![start, end, step].iter().all(|v| v.is_finite()) || step <= 0.0 || end < start {
            return Err("Use finite angles, end ≥ start, and a positive angle step.".into());
        }
        let count = ((end - start) / step).floor() as usize + 1;
        if count < 2 || count > 101 {
            return Err("A sweep must contain 2–101 design points.".into());
        }
        let mesh = self
            .session
            .mesh()
            .ok_or("Generate a mesh before creating design points.")?;
        let angles = (0..count)
            .map(|i| start + i as f64 * step)
            .collect::<Vec<_>>();
        for angle in &angles {
            let folder = root.join(format!("alpha-{angle:+07.2}"));
            if folder.exists() {
                return Err(format!(
                    "Design-point folder already exists: {}",
                    folder.display()
                ));
            }
        }
        let mut created = 0;
        for angle in angles {
            let mut point = self.project.clone();
            set_angle_of_attack(&mut point, angle)?;
            point.runs.clear();
            point.name = format!("{} · α={angle:.2}°", self.project.name);
            point.project_id = format!("{}-alpha-{angle:+.2}", self.project.project_id)
                .replace('+', "p")
                .replace('-', "m")
                .replace('.', "d");
            let folder = root.join(format!("alpha-{angle:+07.2}"));
            std::fs::create_dir_all(&folder).map_err(|e| e.to_string())?;
            point.mesh_asset = Some(mesh_storage::save(&folder, &point, mesh)?);
            workbench::save_workspace(&folder, &point).map_err(|e| e.to_string())?;
            created += 1;
        }
        Ok(created)
    }

    pub(super) fn run_angle_design_points(&mut self, count: usize) -> Result<(), String> {
        if self.parametric_job.is_some() {
            return Err("A parametric run is already active.".into());
        }
        let workspace = self.workspace.clone().ok_or("Save the workspace first.")?;
        let executable = std::env::current_exe().map_err(|e| e.to_string())?;
        let cli_name = if cfg!(windows) {
            "flursys.exe"
        } else {
            "flursys"
        };
        let cli = executable
            .parent()
            .ok_or("Cannot locate the application folder.")?
            .join(cli_name);
        if !cli.is_file() {
            return Err(format!(
                "CLI solver was not found beside the GUI: {}",
                cli.display()
            ));
        }
        let start = self.parametric_start_angle;
        let step = self.parametric_angle_step;
        let iterations = self.parametric_iterations;
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            for i in 0..count {
                let angle = start + i as f64 * step;
                let folder = workspace
                    .join("design-points")
                    .join(format!("alpha-{angle:+07.2}"));
                let result = std::process::Command::new(&cli)
                    .args(["project", "solve"])
                    .arg(&folder)
                    .arg("--iterations")
                    .arg(iterations.to_string())
                    .arg("--json")
                    .output()
                    .map_err(|e| e.to_string())
                    .and_then(|output| {
                        let json: serde_json::Value = serde_json::from_slice(&output.stdout)
                            .map_err(|e| {
                                format!(
                                    "Cannot read solve result: {e}; {}",
                                    String::from_utf8_lossy(&output.stderr)
                                )
                            })?;
                        if output.status.success()
                            && json.get("failed").and_then(serde_json::Value::as_bool) != Some(true)
                        {
                            let summary = json
                                .get("summary")
                                .and_then(serde_json::Value::as_str)
                                .unwrap_or("Completed")
                                .to_owned();
                            match angle_point_coefficients(&folder) {
                                Ok(Some((cd, cl))) => {
                                    Ok(format!("{summary} · Cd {cd:.6} · Cl {cl:.6}"))
                                }
                                Ok(None) => Ok(summary),
                                Err(error) => Ok(format!(
                                    "{summary} · coefficient history unavailable: {error}"
                                )),
                            }
                        } else {
                            Err(json
                                .get("summary")
                                .and_then(serde_json::Value::as_str)
                                .or_else(|| json.get("error").and_then(serde_json::Value::as_str))
                                .unwrap_or("Solve failed")
                                .to_owned())
                        }
                    });
                if tx.send((i + 1, result)).is_err() {
                    break;
                }
            }
        });
        self.parametric_total = count;
        self.parametric_completed = 0;
        self.parametric_failures = 0;
        self.parametric_job = Some(rx);
        self.message(format!(
            "Running design point 1/{count} · sequential solve, {iterations} iterations each."
        ));
        Ok(())
    }

    pub(super) fn poll_angle_design_points(&mut self) {
        let Some(receiver) = &self.parametric_job else {
            return;
        };
        let mut updates = Vec::new();
        while let Ok((index, result)) = receiver.try_recv() {
            updates.push((index, result));
        }
        for (index, result) in updates {
            self.parametric_completed += 1;
            match result {
                Ok(summary) => self.message(format!(
                    "Design point {index}/{} · {summary}",
                    self.parametric_total
                )),
                Err(error) => {
                    self.parametric_failures += 1;
                    self.message(format!(
                        "Design point {index}/{} failed · {error}",
                        self.parametric_total
                    ));
                }
            }
        }
        if self.parametric_completed == self.parametric_total {
            let completed = self.parametric_completed - self.parametric_failures;
            self.message(format!("Parametric sweep complete · {completed}/{} points solved; results are saved in each design-point workspace.", self.parametric_total));
            self.parametric_job = None;
        }
    }

    pub(super) fn apply_case_display(&mut self) {
        let Some(CaseContext { metadata, .. }) = self.project.case_context.as_ref() else {
            return;
        };
        if let Some(angle) = self.project.boundaries.iter().find_map(|boundary| {
            if let workbench::PhysicalBoundaryCondition::FarField { velocity, .. } =
                &boundary.condition
            {
                Some(velocity.y.atan2(velocity.x).to_degrees())
            } else {
                None
            }
        }) {
            self.parametric_angle = angle;
        }
        let d = &metadata.display;
        self.plane_index = self
            .project
            .plane_monitors
            .iter()
            .position(|p| p.id == d.plane)
            .map_or(0, |i| i + 1);
        self.result_field = match d.field.as_str() {
            "pressure" => ResultFieldKind::Pressure,
            "temperature" => ResultFieldKind::Temperature,
            "velocity-x" => ResultFieldKind::VelocityX,
            "velocity-y" => ResultFieldKind::VelocityY,
            "velocity-z" => ResultFieldKind::VelocityZ,
            "vorticity" => ResultFieldKind::VorticityMagnitude,
            _ => ResultFieldKind::VelocityMagnitude,
        };
        self.result_plot = match d.mode.as_str() {
            "contours" => 1,
            "filled-contours" => 2,
            "vectors" => 3,
            _ => 0,
        };
        self.result_palette = if d.palette == "turbo" { 0 } else { 1 };
        self.result_edges = d.cell_edges;
        self.result_vectors = d.vectors;
        self.result_levels = d.levels;
        self.gas_flow.set_defaults(&d.field, &d.mode);
        self.case_comparison_time = metadata
            .comparisons
            .iter()
            .find_map(|r| r.time)
            .unwrap_or(0.);
    }
    pub(super) fn seed_case_probes(&mut self) {
        if self.case_probes_seeded {
            return;
        }
        let Some(data) = &self.result else {
            return;
        };
        self.case_probes_seeded = true;
        let Some(context) = &self.project.case_context else {
            return;
        };
        for spec in &context.metadata.display.probes {
            if let Some((i, _)) = data
                .mesh()
                .cells()
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| {
                    (a.center - spec.position)
                        .norm()
                        .total_cmp(&(b.center - spec.position).norm())
                })
            {
                if !self.probes.iter().any(|p| p.cell_index == i) {
                    if let Some(probe) = data.probe_cell(i) {
                        self.probes.push(probe);
                    }
                }
            }
        }
    }
    pub(super) fn case_reference_properties(&mut self, ui: &mut egui::Ui) {
        if self.project.case_context.is_none() {
            return;
        }
        egui::CollapsingHeader::new("Example references and comparison").show(ui,|ui|{
            let id=self.project.case_context.as_ref().map(|c|c.id.clone()).unwrap_or_default();
            ui.weak("References describe the original case. Reconcile any geometry, material or boundary changes before comparing.");
            if id=="taylor-green-2d"{widgets::number(ui,"Reference time (match displayed field)",&mut self.case_comparison_time);}
            if id=="couette-planar" || id=="taylor-green-2d" {
                if ui.add_enabled(self.result.is_some(),egui::Button::new("Compare displayed cells to analytical field")).clicked(){
                    self.case_comparison_report=match self.compare_case_field(){Ok(text)=>text,Err(e)=>e};
                }
                ui.monospace(&self.case_comparison_report);
            }
            self.case_plot_buttons(ui);
            if let Some(context)=&self.project.case_context {metadata_panel(ui,&context.metadata,None);}
        });
    }
    fn case_plot_buttons(&mut self, ui: &mut egui::Ui) {
        let plots = self
            .project
            .case_context
            .as_ref()
            .map(|c| c.metadata.display.plots.clone())
            .unwrap_or_default();
        for plot in plots {
            let (Some(start), Some(end)) = (plot.start, plot.end) else {
                continue;
            };
            if plot.kind != "line-profile" {
                continue;
            }
            let enabled = self
                .result
                .as_ref()
                .is_some_and(|d| d.mesh().dimension() == MeshDimension::TwoD);
            if !ui
                .add_enabled(enabled, egui::Button::new(format!("Plot: {}", plot.name)))
                .clicked()
            {
                continue;
            }
            self.migration.prepare_case_profile(start, end);
            self.result_field = match plot.quantity.as_str() {
                "velocity-x" => ResultFieldKind::VelocityX,
                "velocity-y" => ResultFieldKind::VelocityY,
                "temperature" => ResultFieldKind::Temperature,
                "pressure" => ResultFieldKind::Pressure,
                _ => ResultFieldKind::VelocityMagnitude,
            };
            match self.add_profile_curve() {
                Ok(()) => {
                    if let Some(context) = &self.project.case_context {
                        if let Some(reference) =
                            reference_curve(&context.metadata, &plot, self.case_comparison_time)
                        {
                            self.migration.add_case_reference(reference);
                        }
                    }
                    self.stage = Stage::Results;
                    self.page = 3;
                    self.migration.chart = true;
                }
                Err(e) => self.message(e),
            }
        }
    }
    fn compare_case_field(&self) -> Result<String, String> {
        if !self.case_comparison_time.is_finite() || self.case_comparison_time < 0. {
            return Err("Reference time must be finite and non-negative".into());
        }
        let data = self.result.as_ref().ok_or("No displayed field")?;
        let context = self
            .project
            .case_context
            .as_ref()
            .ok_or("No case reference")?;
        let weights: Vec<_> = data.mesh().cells().iter().map(|c| c.volume).collect();
        let mut output = String::from(
            "Cell-centre reference sampling; volume-weighted norms. This is not an independence study.\n",
        );
        for (field, name) in [
            (ResultFieldKind::VelocityX, "velocity X"),
            (ResultFieldKind::VelocityY, "velocity Y"),
        ] {
            let expected: Vec<_> = data
                .mesh()
                .cells()
                .iter()
                .map(|cell| {
                    if context.id == "couette-planar" {
                        if field == ResultFieldKind::VelocityX {
                            cell.center.y
                        } else {
                            0.
                        }
                    } else {
                        self.project
                            .solver
                            .initial_profile
                            .as_ref()
                            .map(|p| {
                                let (v, _) = p.sample(
                                    cell.center,
                                    self.case_comparison_time,
                                    self.project.material.kinematic_viscosity,
                                );
                                if field == ResultFieldKind::VelocityX {
                                    v.x
                                } else {
                                    v.y
                                }
                            })
                            .unwrap_or(f64::NAN)
                    }
                })
                .collect();
            let norms =
                workbench::case_format::compare(data.scalar_values(field), &expected, &weights)?;
            output.push_str(&format!(
                "{name}: L1={:.5e}, L2={:.5e}, Linf={:.5e}, relative L2={:?}\n",
                norms.l1, norms.l2, norms.linf, norms.relative_l2
            ));
        }
        Ok(output)
    }
}

pub(super) fn set_angle_of_attack(
    project: &mut WorkbenchProject,
    angle_deg: f64,
) -> Result<(), String> {
    let angle = angle_deg.to_radians();
    let mut found = false;
    for boundary in &mut project.boundaries {
        let velocity = match &mut boundary.condition {
            workbench::PhysicalBoundaryCondition::FarField { velocity, .. } => velocity,
            _ => continue,
        };
        let speed = velocity.x.hypot(velocity.y).hypot(velocity.z);
        if speed <= 0.0 || !speed.is_finite() {
            return Err(format!(
                "Far-field patch '{}' has no positive speed.",
                boundary.selection
            ));
        }
        *velocity = Vec3::new(speed * angle.cos(), speed * angle.sin(), 0.0);
        found = true;
    }
    if !found {
        return Err("No far-field patch is available to parameterize angle of attack.".into());
    }
    project.solver.force_analysis.drag_direction = Vec3::new(angle.cos(), angle.sin(), 0.0);
    project.solver.force_analysis.lift_direction = Vec3::new(-angle.sin(), angle.cos(), 0.0);
    Ok(())
}

fn angle_point_coefficients(workspace: &std::path::Path) -> Result<Option<(f64, f64)>, String> {
    let project = workbench::load_workspace(workspace).map_err(|e| e.to_string())?;
    let mut folders = std::fs::read_dir(workspace.join("monitors"))
        .map_err(|e| e.to_string())?
        .filter_map(Result::ok)
        .filter(|entry| entry.file_name().to_string_lossy().starts_with("solve-"))
        .map(|entry| entry.path())
        .collect::<Vec<_>>();
    folders.sort();
    let Some(latest) = folders.last() else {
        return Ok(None);
    };
    let path = latest.join("force-history.jsonl");
    if !path.is_file() {
        return Ok(None);
    }
    let samples = force_analysis::load(&path)?;
    let series = force_analysis::series(&samples, &project.solver.force_analysis)?;
    let last = |name: &str| {
        series
            .iter()
            .find(|curve| curve.name == name)?
            .points
            .last()?
            .1
    };
    Ok(last("Cd").zip(last("Cl")))
}

fn reference_curve(
    metadata: &CaseMetadata,
    plot: &workbench::case_format::Plot,
    time: f64,
) -> Option<workbench::series_plot::Series> {
    let start = plot.start?;
    let end = plot.end?;
    let delta = end - start;
    let length = delta.norm();
    let axis = if delta.x.abs() > delta.y.abs() {
        "x"
    } else {
        "y"
    };
    let comparison = metadata.comparisons.iter().find(|r| {
        r.asset.is_some()
            && (r.quantity == plot.quantity
                || r.quantity == "velocity" && plot.quantity.starts_with("velocity-"))
            && r.coordinate_column.as_deref() == Some(axis)
    })?;
    if comparison
        .time
        .is_some_and(|t| (t - time).abs() > 1e-10 * (1. + t.abs()))
    {
        return None;
    }
    let asset = metadata
        .assets
        .iter()
        .find(|a| Some(&a.path) == comparison.asset.as_ref())?;
    let mut lines = asset.content.as_ref()?.lines();
    let headers: Vec<_> = lines.next()?.split(',').collect();
    let column = match plot.quantity.as_str() {
        "velocity-x" => "velocity_x",
        "velocity-y" => "velocity_y",
        "pressure" => "kinematic_pressure",
        _ => comparison.value_column.as_deref()?,
    };
    let coordinate = headers.iter().position(|h| *h == axis)?;
    let value = headers.iter().position(|h| *h == column)?;
    let mut points = Vec::new();
    for line in lines {
        let row = line
            .split(',')
            .map(str::parse::<f64>)
            .collect::<Result<Vec<_>, _>>()
            .ok()?;
        if !row.iter().all(|v| v.is_finite()) {
            return None;
        }
        let transverse = if axis == "x" {
            ("y", start.y)
        } else {
            ("x", start.x)
        };
        if let Some(i) = headers.iter().position(|h| *h == transverse.0) {
            if (row.get(i)? - transverse.1).abs() > 1e-10 {
                return None;
            }
        }
        let origin = if axis == "x" { start.x } else { start.y };
        let sign = if axis == "x" {
            delta.x.signum()
        } else {
            delta.y.signum()
        };
        let distance = (row.get(coordinate)? - origin) * sign;
        if distance >= -1e-10 && distance <= length + 1e-10 {
            points.push((distance, Some(*row.get(value)?)));
        }
    }
    points.sort_by(|a, b| a.0.total_cmp(&b.0));
    (points.len() > 1).then_some(workbench::series_plot::Series {
        name: format!("Reference · {}", comparison.id),
        points,
    })
}
