//! Frozen, run-labelled profile curves. Different fields/lines never mix silently.
use super::*;
use flursys::workbench::series_plot::{self, Series};
#[derive(Default)]
pub(super) struct State {
    context: Option<(ResultFieldKind, Vec3, Vec3)>,
    curves: Vec<Series>,
    reference: Option<usize>,
    workspace: Option<PathBuf>,
}
fn inputs(ui: &MainWindow) -> Result<(Vec3, Vec3), String> {
    let parse = |v: SharedString| {
        v.trim()
            .parse::<f64>()
            .ok()
            .filter(|v| v.is_finite())
            .ok_or("Enter finite profile endpoints".to_string())
    };
    Ok((
        Vec3::new(
            parse(ui.get_post_line_x0())?,
            parse(ui.get_post_line_y0())?,
            0.,
        ),
        Vec3::new(
            parse(ui.get_post_line_x1())?,
            parse(ui.get_post_line_y1())?,
            0.,
        ),
    ))
}
fn sample_at(points: &[(f64, Option<f64>)], x: f64) -> Option<f64> {
    match points.binary_search_by(|p| p.0.total_cmp(&x)) {
        Ok(i) => points[i].1,
        Err(i) if i > 0 && i < points.len() => {
            let (a, va) = points[i - 1];
            let (b, vb) = points[i];
            let t = (x - a) / (b - a);
            Some(va? * (1. - t) + vb? * t)
        }
        _ => None,
    }
}
fn summary(state: &State) -> String {
    let Some((field, a, b)) = state.context else {
        return "No comparison curves".into();
    };
    let reference = state.reference.unwrap_or(0);
    let mut out = format!(
        "{} · line ({:.4e}, {:.4e}) → ({:.4e}, {:.4e}) m\n{}: {}",
        field.label(),
        a.x,
        a.y,
        b.x,
        b.y,
        if state.reference.is_some() {
            "Imported reference"
        } else {
            "Baseline (first curve, not a validated reference)"
        },
        state.curves[reference].name
    );
    for (index, curve) in state.curves.iter().enumerate() {
        if index != reference {
            let mut n = 0usize;
            let (mut error, mut energy, mut maximum) = (0_f64, 0_f64, 0_f64);
            for &(x, value) in &state.curves[reference].points {
                if let (Some(r), Some(v)) = (value, sample_at(&curve.points, x)) {
                    let d = v - r;
                    if !d.is_finite() {
                        continue;
                    }
                    error = error.hypot(d);
                    energy = energy.hypot(r);
                    maximum = maximum.max(d.abs());
                    n += 1;
                }
            }
            if n == 0 {
                out.push_str(&format!("\n{}: no overlapping finite samples", curve.name));
            } else {
                out.push_str(&format!(
                    "\n{} · N={} · RMSE {:.5e} · max |error| {:.5e} · relative L2 {}",
                    curve.name,
                    n,
                    error / (n as f64).sqrt(),
                    maximum,
                    if energy > 0. {
                        format!("{:.4}%", 100. * error / energy)
                    } else {
                        "N/A (zero reference norm)".into()
                    }
                ));
            }
        }
    }
    out
}
fn title(state: &State) -> String {
    state
        .context
        .map(|(_, a, b)| {
            format!(
                "Profile comparison · ({:.3e}, {:.3e}) → ({:.3e}, {:.3e}) m",
                a.x, a.y, b.x, b.y
            )
        })
        .unwrap_or_else(|| "Profile comparison".into())
}
fn render(ui: &MainWindow, state: &AppState) -> Result<(), String> {
    let c = &state.post.comparison;
    let (field, _, _) = c.context.ok_or("Add a profile first")?;
    let svg = series_plot::svg(
        &c.curves,
        &title(c),
        "Distance along line (m)",
        field.label(),
    )?;
    ui.set_post_profile_image(
        slint::Image::load_from_svg_data(svg.as_bytes()).map_err(|e| e.to_string())?,
    );
    ui.set_post_profile_caption(summary(c).into());
    Ok(())
}
pub(super) fn show_monitors(ui: &MainWindow, state: &AppState) {
    match monitor_history::chart(
        state,
        ui.get_flow_plane_index().max(0) as usize,
        ui.get_post_monitor_kind().clamp(0, 2) as usize,
    ) {
        Ok((svg, caption)) => {
            match slint::Image::load_from_svg_data(svg.as_bytes()) {
                Ok(image) => ui.set_post_profile_image(image),
                Err(e) => ui.set_post_profile_caption(e.to_string().into()),
            };
            ui.set_post_profile_caption(caption.into());
        }
        Err(e) => {
            ui.set_post_profile_image(render::render_empty_image());
            ui.set_post_profile_caption(e.into());
        }
    }
}
pub(super) fn bind(ui: &MainWindow, shared: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let state = shared.clone();
    ui.on_post_compare(move |action| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut state = state.borrow_mut();
        let result = (|| -> Result<(), String> {
            match action {
                0 => {
                    if state.workspace_path.is_none() {
                        return Err("Save the workspace before comparing runs".into());
                    }
                    if !state.post.comparison.curves.is_empty()
                        && state.post.comparison.workspace != state.workspace_path
                    {
                        return Err("Clear comparison before changing workspaces".into());
                    }
                    let data = state.active_result().ok_or("Choose a saved result run")?;
                    let field = post_processing::field(ui.get_result_field_index());
                    let (a, b) = inputs(&ui)?;
                    let points = result_analysis::sample_line(
                        data,
                        a,
                        b,
                        ui.get_post_line_samples() as usize,
                        field,
                    )?;
                    if points.iter().all(|p| p.value.is_none()) {
                        return Err("The line has no finite fluid samples".into());
                    }
                    if let Some((f, start, end)) = state.post.comparison.context {
                        if f != field || start != a || end != b {
                            return Err(
                                "Clear comparison before changing field or line endpoints".into()
                            );
                        }
                    }
                    let curve = Series {
                        name: format!("{} · {}", data.run_id(), field.label()),
                        points: points.into_iter().map(|p| (p.distance, p.value)).collect(),
                    };
                    let workspace = state.workspace_path.clone();
                    let c = &mut state.post.comparison;
                    c.workspace = workspace;
                    if let Some(index) = c.curves.iter().position(|p| p.name == curve.name) {
                        c.curves[index] = curve;
                    } else {
                        if c.curves.len() >= 8 {
                            return Err("At most eight curves, including the reference".into());
                        }
                        c.curves.push(curve);
                    }
                    c.context = Some((field, a, b));
                    render(&ui, &state)?;
                }
                1 => {
                    state.post.comparison = Default::default();
                    ui.set_post_profile_image(render::render_empty_image());
                    ui.set_post_profile_caption("Comparison cleared".into());
                }
                2 | 3 => {
                    let c = &state.post.comparison;
                    let (field, _, _) = c.context.ok_or("Add a comparison curve first")?;
                    let ext = if action == 2 { "csv" } else { "svg" };
                    let text = if action == 2 {
                        {let (_,a,b)=c.context.ok_or("Add a comparison first")?;
                        format!("# field: {}\n# start_xy_m: {:.17e},{:.17e}\n# end_xy_m: {:.17e},{:.17e}\n{}{}",field.label(),a.x,a.y,b.x,b.y,summary(c).lines().map(|l|format!("# {l}\n")).collect::<String>(),series_plot::csv(&c.curves,"distance_m"))}
                    } else {
                        series_plot::svg(
                            &c.curves,
                            &title(c),
                            "Distance along line (m)",
                            field.label(),
                        )?
                    };
                    if let Some(path) = rfd::FileDialog::new()
                        .set_file_name(format!("profile-comparison.{ext}"))
                        .save_file()
                    {
                        result_analysis::atomic_text(&path, &text)?;
                    }
                }
                6 => show_monitors(&ui, &state),
                7 | 8 => {
                    let index = ui.get_flow_plane_index().max(0) as usize;
                    let (text, ext) = if action == 7 {
                        (monitor_history::csv(&state, index)?, "csv")
                    } else {
                        (
                            monitor_history::chart(
                                &state,
                                index,
                                ui.get_post_monitor_kind().clamp(0, 2) as usize,
                            )?
                            .0,
                            "svg",
                        )
                    };
                    if let Some(path) = rfd::FileDialog::new()
                        .set_file_name(format!("section-monitor.{ext}"))
                        .save_file()
                    {
                        result_analysis::atomic_text(&path, &text)?;
                    }
                }
                4 => {
                    if state.post.comparison.context.is_none() {
                        return Err(
                            "Add the active run profile before importing reference data".into()
                        );
                    }
                    let (expected,a,b)=state.post.comparison.context.ok_or("Add a run first")?;
                    if expected!=post_processing::field(ui.get_result_field_index()) || inputs(&ui)?!=(a,b) || state.post.comparison.workspace!=state.workspace_path {
                        return Err("Restore the comparison field/line/workspace or clear comparison before importing".into());
                    }
                    let Some(path) = rfd::FileDialog::new()
                        .add_filter("Two-column reference CSV", &["csv"])
                        .pick_file()
                    else {
                        return Ok(());
                    };
                    if std::fs::metadata(&path).map_err(|e| e.to_string())?.len() > 2_000_000 {
                        return Err("Reference CSV is limited to 2 MB".into());
                    }
                    let text = std::fs::read_to_string(&path).map_err(|e| e.to_string())?;
                    let mut points = Vec::new();
                    for line in text
                        .lines()
                        .filter(|l| !l.trim().is_empty() && !l.trim().starts_with('#'))
                    {
                        let parts = line.split(',').map(str::trim).collect::<Vec<_>>();
                        if parts.len() != 2 {
                            return Err(
                                "Reference CSV needs two unquoted columns: distance_m,value".into(),
                            );
                        }
                        if points.is_empty() && matches!(parts[0], "distance_m" | "distance" | "x")
                        {
                            continue;
                        }
                        let x = parts[0]
                            .parse::<f64>()
                            .ok()
                            .filter(|v| v.is_finite())
                            .ok_or("Invalid reference distance")?;
                        let value = if parts[1].is_empty() {
                            None
                        } else {
                            Some(
                                parts[1]
                                    .parse::<f64>()
                                    .ok()
                                    .filter(|v| v.is_finite())
                                    .ok_or("Invalid reference value")?,
                            )
                        };
                        if points.last().is_some_and(|p: &(f64, Option<f64>)| p.0 >= x) {
                            return Err("Reference distances must increase strictly".into());
                        }
                        points.push((x, value));
                        if points.len() > 20_000 {
                            return Err("Reference is limited to 20000 samples".into());
                        }
                    }
                    if points.len() < 2 || points.iter().all(|p| p.1.is_none()) {
                        return Err(
                            "Reference requires at least two positions and a finite value".into(),
                        );
                    }
                    let curve = Series {
                        name: format!(
                            "Reference · {}",
                            path.file_name().unwrap_or_default().to_string_lossy()
                        ),
                        points,
                    };
                    let c = &mut state.post.comparison;
                    if let Some(i) = c.reference {
                        c.curves[i] = curve;
                    } else {
                        if c.curves.len() >= 8 {
                            return Err("Clear a comparison before adding a ninth curve".into());
                        }
                        c.reference = Some(c.curves.len());
                        c.curves.push(curve);
                    }
                    render(&ui, &state)?;
                }
                _ => render(&ui, &state)?,
            }
            Ok(())
        })();
        if let Err(e) = result {
            ui.set_post_profile_caption(e.into());
        }
    });
}
