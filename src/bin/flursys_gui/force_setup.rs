//! One reference-load editor and chart for live and archived wall samples.
use super::*;
use flursys::workbench::force_analysis::{self, Settings};
fn vector(text: SharedString) -> Result<Vec3, String> {
    let values: Vec<f64> = text
        .split(|c: char| c == ',' || c.is_whitespace())
        .filter(|s| !s.is_empty())
        .map(|s| {
            s.parse::<f64>()
                .map_err(|_| "Enter three finite vector components".to_string())
        })
        .collect::<Result<_, _>>()?;
    if values.len() != 3 || values.iter().any(|x| !x.is_finite()) {
        return Err("Enter three finite vector components: x, y, z".into());
    }
    Ok(Vec3::new(values[0], values[1], values[2]))
}
pub(super) fn push(ui: &MainWindow, s: &Settings) {
    let v = |v: Vec3| SharedString::from(format!("{}, {}, {}", v.x, v.y, v.z));
    ui.set_force_patches(s.patches.join(", ").into());
    ui.set_force_frequency_start(s.frequency_start_time.to_string().into());
    ui.set_force_speed(s.speed.to_string().into());
    ui.set_force_area(s.measure.to_string().into());
    ui.set_force_length(s.length.to_string().into());
    ui.set_force_center(v(s.center));
    ui.set_force_drag(v(s.drag_direction));
    ui.set_force_lift(v(s.lift_direction));
    ui.set_force_reattach(s.reattachment);
    ui.set_force_start(v(s.wall_start));
    ui.set_force_axis(v(s.wall_direction));
    ui.set_force_tolerance(s.wall_tolerance.to_string().into());
}
pub(super) fn bind(ui: &MainWindow, state: Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    ui.on_force_apply(move |action| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut state = state.borrow_mut();
        let result = (|| -> Result<(), String> {
            let number = |s: SharedString| {
                s.parse::<f64>()
                    .map_err(|_| "Enter finite reference values".to_string())
            };
            let settings = Settings {
                patches: ui
                    .get_force_patches()
                    .split(',')
                    .map(str::trim)
                    .filter(|s| !s.is_empty())
                    .map(str::to_string)
                    .collect(),
                frequency_start_time:number(ui.get_force_frequency_start())?,
                speed: number(ui.get_force_speed())?,
                measure: number(ui.get_force_area())?,
                length: number(ui.get_force_length())?,
                center: vector(ui.get_force_center())?,
                drag_direction: vector(ui.get_force_drag())?,
                lift_direction: vector(ui.get_force_lift())?,
                reattachment: ui.get_force_reattach(),
                wall_start: vector(ui.get_force_start())?,
                wall_direction: vector(ui.get_force_axis())?,
                wall_tolerance: number(ui.get_force_tolerance())?,
            };
            settings.validate()?;
            if !state.post.wall_reports.is_empty() {
                settings.evaluate(
                    &state.post.wall_reports,
                    state
                        .post
                        .force_history
                        .last()
                        .map(|s| s.density)
                        .unwrap_or(state.workbench.material().density),
                )?;
            }
            state.workbench.set_force_analysis(settings.clone())?;
            state.workbench_project.solver.force_analysis = settings.clone();
            state.workbench_dirty = true;
            if state.post.force_history.is_empty() && !state.post.wall_reports.is_empty() {
                let sample = force_analysis::Sample {
                    index: state.post.wall_report_index.unwrap_or(0),
                    time: state.post.wall_report_time,
                    density: state.workbench.material().density,
                    reports: state.post.wall_reports.clone(),
                };
                state.post.force_history.push(sample);
            }
            let mut summary = settings.summary(
                &state.post.wall_reports,
                state
                    .post
                    .force_history
                    .last()
                    .map(|s| s.density)
                    .unwrap_or(state.workbench.material().density),
            );
            if settings.reattachment {
                let curves: Vec<_> = state
                    .post
                    .wall_reports
                    .iter()
                    .filter(|r| settings.patches.is_empty() || settings.patches.contains(&r.patch))
                    .map(|r| {
                        let axis = settings.wall_direction / settings.wall_direction.norm();
                        let mut points: Vec<_> = r
                            .samples
                            .iter()
                            .filter_map(|s| {
                                let d = s.point - settings.wall_start;
                                let x = d.dot(axis);
                                (x >= 0. && (d - axis * x).norm() <= settings.wall_tolerance)
                                    .then_some((x, Some(s.shear_stress.dot(axis))))
                            })
                            .collect();
                        points.sort_by(|a, b| a.0.total_cmp(&b.0));
                        flursys::workbench::series_plot::Series {
                            name: r.patch.clone(),
                            points,
                        }
                    })
                    .collect();
                let svg = flursys::workbench::series_plot::svg(
                    &curves,
                    "Wall shear and reattachment",
                    "Distance from start (m)",
                    "Tangential shear (Pa)",
                )?;
                ui.set_force_image(
                    slint::Image::load_from_svg_data(svg.as_bytes()).map_err(|e| e.to_string())?,
                );
            } else {
                let curves = force_analysis::series(&state.post.force_history, &settings)?;
                let timed = state
                    .post
                    .force_history
                    .first()
                    .is_some_and(|s| s.time.is_some());
                let svg = flursys::workbench::series_plot::svg(
                    &curves,
                    "Selected-wall load coefficients",
                    if timed {
                        "Physical time (s)"
                    } else {
                        "Iteration"
                    },
                    "Coefficient",
                )?;
                ui.set_force_image(
                    slint::Image::load_from_svg_data(svg.as_bytes()).map_err(|e| e.to_string())?,
                );
                summary.push_str(&format!(
                    "\n{} load samples. Press Apply to refresh the chart.",
                    state.post.force_history.len()
                ));
            }
            if state.post.force_history.first().is_some_and(|s|s.time.is_some()) {
                match force_analysis::shedding_frequency(&state.post.force_history,&settings,settings.frequency_start_time)? {
                    Some((f,st))=>summary.push_str(&format!("\nPositive lift zero-crossing estimate: f={f:.6e} Hz, St={st:.6e}. Inspect periodicity before interpreting.")),
                    None=>summary.push_str("\nFrequency: need at least three positive lift crossings after the chosen start time."),
                }
            }
            ui.set_force_caption(summary.into());
            if action == 1 {
                if let Some(path) = rfd::FileDialog::new()
                    .set_file_name("force-history.csv")
                    .save_file()
                {
                    result_analysis::atomic_text(
                        &path,
                        &force_analysis::csv(&state.post.force_history, &settings)?,
                    )?;
                }
            }
            Ok(())
        })();
        if let Err(e) = result {
            ui.set_force_caption(e.clone().into());
            state.log(e);
        }
    });
}
