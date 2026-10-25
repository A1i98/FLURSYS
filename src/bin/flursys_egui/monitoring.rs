use super::*;
use workbench::{engineering::Sample, series_plot::Series};
#[derive(Clone, Serialize, Deserialize)]
#[serde(default)]
pub(super) struct Options {
    every: usize,
    quantity: usize,
    first: String,
    second: String,
    physical_axis: bool,
    stable_enabled: bool,
    stop: bool,
    minimum_index: usize,
    window: usize,
    absolute_tolerance: f64,
    relative_tolerance: f64,
    require_balance: bool,
    balance_tolerance: f64,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            every: 10,
            quantity: 0,
            first: String::new(),
            second: String::new(),
            physical_axis: true,
            stable_enabled: false,
            stop: false,
            minimum_index: 100,
            window: 10,
            absolute_tolerance: 1e-8,
            relative_tolerance: 1e-3,
            require_balance: true,
            balance_tolerance: 1e-4,
        }
    }
}
#[derive(Default)]
pub(super) struct State {
    pub options: Options,
    samples: Vec<Sample>,
    archive: Vec<Sample>,
    pub chart: bool,
    reason: Option<String>,
    pub time_state: Option<(usize, f64, Option<f64>, Option<f64>, usize)>,
}
const QUANTITIES: [&str; 8] = [
    "Volume flow",
    "Mass flow",
    "Pressure drop (Pa)",
    "Temperature (K)",
    "Signed mass imbalance",
    "Relative boundary imbalance",
    "Energy imbalance (W; 2D W/m)",
    "Relative energy imbalance",
];
fn value(sample: &Sample, options: &Options) -> Option<f64> {
    let patch = sample.patches.iter().find(|p| p.name == options.first);
    let result = match options.quantity {
        0 => patch.map(|p| p.volume_flux),
        1 => patch.map(|p| p.mass_flux),
        2 => patch
            .and_then(|p| p.pressure_pa)
            .zip(
                sample
                    .patches
                    .iter()
                    .find(|p| p.name == options.second)
                    .and_then(|p| p.pressure_pa),
            )
            .map(|(a, b)| a - b),
        3 => {
            if options.first.is_empty() {
                sample.temperature_mean
            } else {
                patch.and_then(|p| p.temperature_k)
            }
        }
        4 => Some(sample.mass_imbalance),
        5 => sample.relative_mass_imbalance,
        6 => sample.energy_imbalance_w,
        _ => sample.relative_energy_imbalance,
    };
    result.filter(|v| v.is_finite())
}
impl State {
    pub fn reset_history(&mut self) {
        let options = self.options.clone();
        *self = Self::default();
        self.options = options;
    }
}
impl App {
    pub(super) fn monitoring_controls(&mut self, ui: &mut egui::Ui) {
        use widgets::{heading, number};
        heading(ui, "ENGINEERING MONITORS");
        let names: Vec<_> = self
            .project
            .named_selections
            .iter()
            .map(|p| p.name.clone())
            .collect();
        let locked = self.jobs.is_running();
        ui.add_enabled_ui(!locked, |ui| {
            ui.horizontal(|ui| {
                ui.label("Sample every N accepted iterations / steps");
                ui.add(
                    egui::DragValue::new(&mut self.monitoring.options.every).range(1..=1_000_000),
                );
            });
        });
        let options = &mut self.monitoring.options;
        ui.add_enabled_ui(!locked || !options.stable_enabled, |ui| {
            egui::ComboBox::from_id_salt("engineering-quantity")
                .selected_text(QUANTITIES[options.quantity.min(7)])
                .show_ui(ui, |ui| {
                    for (i, name) in QUANTITIES.iter().enumerate() {
                        ui.selectable_value(&mut options.quantity, i, *name);
                    }
                });
            for (id, label, name) in [
                ("monitor-first", "Patch / upstream", &mut options.first),
                (
                    "monitor-second",
                    "Downstream (pressure drop)",
                    &mut options.second,
                ),
            ] {
                ui.label(label);
                egui::ComboBox::from_id_salt(id)
                    .selected_text(if name.is_empty() {
                        "All patches / domain"
                    } else {
                        name.as_str()
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(name, String::new(), "All patches / domain");
                        for n in &names {
                            ui.selectable_value(name, n.clone(), n);
                        }
                    });
            }
        });
        ui.checkbox(
            &mut options.physical_axis,
            "Use physical time when available",
        );
        ui.small("Pressure and temperature at patches use area-weighted adjacent cells. Flow uses conservative face flux; outward is positive. 2D flow/mass values are per unit depth. Closed domains have no relative throughflow imbalance. Energy budgets appear only when the solver supplies them; missing values stay gaps.");
        ui.add_enabled_ui(!locked,|ui|{
            egui::CollapsingHeader::new("Monitor stability action").show(ui,|ui|{
                ui.checkbox(&mut options.stable_enabled,"Act when the selected monitor stabilizes");
                ui.checkbox(&mut options.stop,"Stop and retain fields (otherwise pause)");
                ui.add(egui::DragValue::new(&mut options.minimum_index).range(1..=1_000_000).prefix("Minimum iteration / step "));
                ui.add(egui::DragValue::new(&mut options.window).range(3..=10000).prefix("Window samples "));
                number(ui,"Absolute window tolerance (selected metric units)",&mut options.absolute_tolerance);
                number(ui,"Relative window tolerance (fraction)",&mut options.relative_tolerance);
                ui.checkbox(&mut options.require_balance,"Also require relative boundary mass balance");
                if options.require_balance{number(ui,"Maximum relative mass imbalance",&mut options.balance_tolerance);}
                ui.small("This is a monitor rule, not proof of convergence. Choose an explicit patch for flow stability or two patches for pressure drop. A closed cavity needs a different rule because relative throughflow balance is undefined.");
            });
        });
        if ui
            .add_enabled(!locked, egui::Button::new("Save monitor settings"))
            .clicked()
        {
            match self.validate_monitor_options().and_then(|_|self.persist_continuation()){Ok(())=>self.message("Monitor settings saved; sampling and stability action apply on Run / Continue."),Err(e)=>self.message(e)}
        }
        if ui.button("Export monitor CSV…").clicked() {
            if let Some(path) = rfd::FileDialog::new()
                .set_file_name("engineering-monitors.csv")
                .save_file()
            {
                let series = self.engineering_series();
                let text = workbench::series_plot::csv(
                    &series,
                    if self.monitoring.options.physical_axis {
                        "time_s_or_iteration"
                    } else {
                        "iteration_or_step"
                    },
                );
                if let Err(e) = std::fs::write(path, text) {
                    self.message(e.to_string());
                }
            }
        }
        if let Some(reason) = &self.monitoring.reason {
            ui.label(reason);
        }
        let samples = if self.stage == Stage::Results {
            &self.monitoring.archive
        } else {
            &self.monitoring.samples
        };
        if let Some(last) = samples.last() {
            ui.label(format!(
                "Latest sample: {} · mass imbalance {:+.6e} · relative {:?}",
                last.index, last.mass_imbalance, last.relative_mass_imbalance
            ));
        }
    }
    fn validate_monitor_options(&self) -> Result<(), String> {
        let o = &self.monitoring.options;
        if !(1..=1_000_000).contains(&o.every)
            || !(3..=10000).contains(&o.window)
            || o.quantity > 7
            || [
                o.absolute_tolerance,
                o.relative_tolerance,
                o.balance_tolerance,
            ]
            .iter()
            .any(|v| !v.is_finite() || *v < 0.)
        {
            return Err("Monitor intervals, windows and tolerances must be finite and within their allowed ranges.".into());
        }
        if o.stable_enabled
            && ((o.quantity <= 2 && o.first.is_empty())
                || (o.quantity == 2 && (o.second.is_empty() || o.second == o.first)))
        {
            return Err("Select the patch(s) for the stability criterion.".into());
        }
        Ok(())
    }
    pub(super) fn configure_engineering(&mut self) -> Result<(), String> {
        self.validate_monitor_options()?;
        self.jobs
            .set_engineering_interval(self.monitoring.options.every);
        self.monitoring.reason = None;
        Ok(())
    }
    pub(super) fn accept_engineering(&mut self, sample: Sample) {
        let index = sample.index;
        match self
            .monitoring
            .samples
            .binary_search_by_key(&index, |s| s.index)
        {
            Ok(i) => self.monitoring.samples[i] = sample,
            Err(i) => self.monitoring.samples.insert(i, sample),
        }
        let o = &self.monitoring.options;
        if !o.stable_enabled
            || self.monitoring.reason.is_some()
            || !self.jobs.is_running()
            || index < o.minimum_index
            || self.monitoring.samples.len() < o.window
        {
            return;
        }
        let window = &self.monitoring.samples[self.monitoring.samples.len() - o.window..];
        let values: Option<Vec<_>> = window.iter().map(|s| value(s, o)).collect();
        let Some(values) = values else { return };
        if o.require_balance
            && window.iter().any(|s| {
                s.relative_mass_imbalance
                    .is_none_or(|b| b > o.balance_tolerance)
            })
        {
            return;
        }
        let min = values.iter().copied().fold(f64::INFINITY, f64::min);
        let max = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let scale = min.abs().max(max.abs());
        if max - min <= o.absolute_tolerance + o.relative_tolerance * scale {
            let reason = format!(
                "{} at {}: {} stable over {} samples; this is not a convergence declaration.",
                if o.stop {
                    "Stopped by monitor rule"
                } else {
                    "Paused by monitor rule"
                },
                index,
                QUANTITIES[o.quantity],
                o.window
            );
            if o.stop {
                self.jobs.cancel();
            } else {
                self.jobs.set_paused(true)
            }
            self.monitoring.reason = Some(reason.clone());
            self.message(reason);
        }
    }
    fn engineering_series(&self) -> Vec<Series> {
        let samples = if self.stage == Stage::Results {
            &self.monitoring.archive
        } else {
            &self.monitoring.samples
        };
        let o = &self.monitoring.options;
        let axis = |s: &Sample| {
            if o.physical_axis {
                s.time.unwrap_or(s.index as f64)
            } else {
                s.index as f64
            }
        };
        if o.first.is_empty() && o.quantity <= 1 {
            let Some(last) = samples.last() else {
                return Vec::new();
            };
            last.patches
                .iter()
                .map(|patch| Series {
                    name: patch.name.clone(),
                    points: samples
                        .iter()
                        .map(|s| {
                            (
                                axis(s),
                                s.patches.iter().find(|p| p.name == patch.name).map(|p| {
                                    if o.quantity == 0 {
                                        p.volume_flux
                                    } else {
                                        p.mass_flux
                                    }
                                }),
                            )
                        })
                        .collect(),
                })
                .collect()
        } else {
            vec![Series {
                name: QUANTITIES[o.quantity.min(7)].into(),
                points: samples.iter().map(|s| (axis(s), value(s, o))).collect(),
            }]
        }
    }
    pub(super) fn engineering_chart(&mut self, ui: &mut egui::Ui) {
        let series = self.engineering_series();
        if series.iter().all(|s| s.points.is_empty()) {
            ui.centered_and_justified(|ui|ui.weak("No engineering history in this run yet. Sampling starts on Run / Continue; older archives may not contain it."));
            return;
        }
        let physical = self.monitoring.options.physical_axis
            && (if self.stage == Stage::Results {
                &self.monitoring.archive
            } else {
                &self.monitoring.samples
            })
            .iter()
            .any(|s| s.time.is_some());
        let title = QUANTITIES[self.monitoring.options.quantity.min(7)];
        self.native_series_plot(
            ui,
            &series,
            title,
            if physical {
                "Physical time (s)"
            } else {
                "Iteration / physical step"
            },
        );
    }
    pub(super) fn restore_engineering(&mut self) {
        if let Some(folder) = &self.solve_recording {
            let path = folder.join("engineering.jsonl");
            if path.is_file() {
                match workbench::engineering::load(&path) {
                    Ok(samples) => self.monitoring.samples = samples,
                    Err(e) => self.message(format!("Engineering history: {e}")),
                }
            }
        }
    }
    pub(super) fn inspect_engineering(&mut self) {
        self.monitoring.archive.clear();
        if let Some(folder) = &self.recording {
            let path = folder.join("engineering.jsonl");
            if path.is_file() {
                match workbench::engineering::load(&path) {
                    Ok(samples) => self.monitoring.archive = samples,
                    Err(e) => self.message(format!("Engineering history: {e}")),
                }
            }
        }
    }
    pub(super) fn remember_time(&mut self, index: usize, fields: &IncompressibleInitialConditions) {
        if let IncompressibleInitialConditions::Restart { time, last_dt, .. } = fields {
            if self.project.solver.time.is_transient() {
                self.monitoring.time_state = Some((index, *time, *last_dt, None, 0));
            }
        }
    }
    pub(super) fn solve_identity(&self, ui: &mut egui::Ui) {
        let state = if self.jobs.is_running() {
            if self.jobs.is_paused() {
                "Paused"
            } else {
                "Solving"
            }
        } else {
            match self.jobs.lifecycle() {
                flursys::app::JobLifecycle::Cancelled => "Stopped · fields retained",
                _ => match self.session.status() {
                    SolveStatus::Converged => "Converged",
                    SolveStatus::MaxIterations => "Iteration limit",
                    SolveStatus::Failed(_) => "Failed · inspect diagnostics",
                    _ => "Ready",
                },
            }
        };
        ui.horizontal_wrapped(|ui| {
            ui.strong(state);
            ui.label(format!(
                "Active archive: {}",
                self.continuation_run
                    .as_deref()
                    .unwrap_or("not yet finalized")
            ));
            if let Some((index, time, dt, courant, rejected)) = self.monitoring.time_state {
                ui.label(format!(
                    "Step {index} · t={time:.6e} s · Δt={} s · Co={} · rejected {rejected}",
                    dt.map(|v| format!("{v:.4e}"))
                        .unwrap_or_else(|| "unavailable".into()),
                    courant
                        .map(|v| format!("{v:.3e}"))
                        .unwrap_or_else(|| "unavailable".into())
                ));
            }
        });
        if self.editor.solver != self.project.solver
            || self.editor.material != self.project.material
            || self.editor.mesh != self.project.mesh
            || self.editor.mesh_recipe != self.project.mesh_recipe
        {
            ui.colored_label(
                egui::Color32::from_rgb(235, 184, 90),
                "Unapplied draft settings · Apply settings or Run / Continue to use them.",
            );
        }
        if let Some(reason) = &self.monitoring.reason {
            ui.small(reason);
        }
    }
}
impl App {
    pub(super) fn save_run_display_metadata(
        &self,
        run: &workbench::RunRecord,
    ) -> Result<(), String> {
        let Some(workspace) = &self.workspace else {
            return Ok(());
        };
        let path = workspace.join(&run.report_path);
        let bytes = std::fs::read(&path).map_err(|e| e.to_string())?;
        let mut report: serde_json::Value =
            serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
        report["indexed_convergence_history"] = serde_json::json!(self.solve_histories);
        report["engineering_monitor_settings"] = serde_json::json!(self.monitoring.options);
        report["monitor_action"] = serde_json::json!(self.monitoring.reason);
        report["accepted_time_state"] = serde_json::json!(self.monitoring.time_state);
        if let Some(folder) = &self.solve_recording {
            let relative = folder
                .strip_prefix(workspace)
                .map_err(|_| "Recording folder outside workspace")?;
            report["plane_recording_folder"] = serde_json::json!(relative);
        }
        let mut file =
            tempfile::NamedTempFile::new_in(path.parent().ok_or("Missing report parent")?)
                .map_err(|e| e.to_string())?;
        serde_json::to_writer_pretty(file.as_file_mut(), &report).map_err(|e| e.to_string())?;
        file.persist(path).map_err(|e| e.to_string())?;
        Ok(())
    }
}
