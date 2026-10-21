//! One plotting model for the live Solution view and its vector/raster exports.
use super::*;
use std::fmt::Write;

const NAMES: [&str; 4] = [
    "Continuity · RMS",
    "Momentum · relative",
    "Velocity change · relative",
    "Pressure · inner linear",
];
const COLORS: [&str; 4] = ["#58c4f6", "#f5b85b", "#65d49c", "#c49bff"];

#[derive(Clone)]
pub(super) struct Options {
    pub kind: i32,
    pub visible: [bool; 4],
    pub logarithmic: bool,
    pub normalized: bool,
    pub window: usize,
    pub legend: bool,
    pub width: u32,
    pub height: u32,
}
impl Options {
    pub fn from_ui(ui: &MainWindow) -> Self {
        Self {
            kind: ui.get_wb_chart_kind(),
            visible: [
                ui.get_chart_continuity(),
                ui.get_chart_momentum(),
                ui.get_chart_velocity(),
                ui.get_chart_pressure(),
            ],
            logarithmic: ui.get_chart_logarithmic(),
            normalized: ui.get_chart_normalized(),
            window: match ui.get_chart_window() {
                1 => 400,
                2 => 100,
                _ => usize::MAX,
            },
            legend: ui.get_chart_legend(),
            width: (ui.get_chart_width() as u32).clamp(480, 3200),
            height: (ui.get_chart_height() as u32).clamp(240, 1800),
        }
    }
}
struct Curve {
    name: &'static str,
    color: &'static str,
    values: Vec<f64>,
    first: usize,
    scale: f64,
}
pub(super) struct Plot {
    pub svg: String,
    pub caption: String,
}
fn curves(state: &AppState, options: &Options) -> Vec<Curve> {
    let structured =
        state.last_update.is_some() && state.workbench.geometry().vertices().next().is_none();
    let histories = if structured {
        [
            state
                .residual_history
                .iter()
                .map(|s| s.continuity)
                .collect(),
            state.residual_history.iter().map(|s| s.momentum).collect(),
            vec![],
            state.residual_history.iter().map(|s| s.pressure).collect(),
        ]
    } else {
        result_analysis::histories(state)
    };
    let longest = histories.iter().map(Vec::len).max().unwrap_or(0);
    let last = if structured {
        state
            .last_update
            .as_ref()
            .map_or(longest.saturating_sub(1), |u| u.iteration)
    } else if state.solving
        || (state.workbench.solution().is_none() && !state.post.continuity_full.is_empty())
    {
        state.post.iteration
    } else if let Some(solution) = state.workbench.solution() {
        solution.report.outer_iterations
    } else {
        state
            .active_result()
            .and_then(|d| {
                state
                    .workbench_project
                    .runs
                    .iter()
                    .find(|r| r.id == d.run_id())
            })
            .and_then(|r| r.iterations)
            .unwrap_or_else(|| state.post.iteration.max(longest.saturating_sub(1)))
    };
    histories
        .into_iter()
        .enumerate()
        .filter_map(|(index, values)| {
            if !(if options.kind == 4 {
                options.visible[index]
            } else {
                index == options.kind.clamp(0, 3) as usize
            }) {
                return None;
            }
            let scale = if options.normalized {
                values
                    .iter()
                    .copied()
                    .find(|v| v.is_finite() && *v > 0.)
                    .unwrap_or(1.)
            } else {
                1.
            };
            let first = last.saturating_add(1).saturating_sub(values.len());
            let skip = values.len().saturating_sub(options.window);
            Some(Curve {
                name: if structured && index == 1 {
                    "Momentum · reported"
                } else if structured && index == 3 {
                    "Pressure · reported"
                } else {
                    NAMES[index]
                },
                color: COLORS[index],
                first: first + skip,
                scale,
                values: values.into_iter().skip(skip).collect(),
            })
        })
        .collect()
}
fn number(value: f64) -> String {
    if value == 0. {
        "0".into()
    } else {
        format!("{value:.2e}")
    }
}
fn nice_step(span: f64) -> f64 {
    let p = 10_f64.powf((span.max(1.) / 5.).log10().floor());
    let n = span.max(1.) / 5. / p;
    ([1., 2., 5., 10.]
        .into_iter()
        .find(|v| *v >= n)
        .unwrap_or(10.)
        * p)
        .max(1.)
}
fn x_range(curves: &[Curve]) -> (f64, f64) {
    let first = curves
        .iter()
        .filter(|c| !c.values.is_empty())
        .map(|c| c.first)
        .min()
        .unwrap_or(0) as f64;
    let last = curves
        .iter()
        .filter(|c| !c.values.is_empty())
        .map(|c| c.first + c.values.len() - 1)
        .max()
        .unwrap_or(1) as f64;
    (first, last.max(first + 1.))
}
pub(super) fn build(state: &AppState, options: &Options) -> Plot {
    let curves = curves(state, options);
    let (x0, x1) = x_range(&curves);
    let (w, h) = (options.width as f64, options.height as f64);
    let (left, right, top, bottom) = (
        96.,
        w - 24.,
        if options.legend { 100. } else { 44. },
        h - 62.,
    );
    let valid = |v: f64| v.is_finite() && v >= 0. && (!options.logarithmic || v > 0.);
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    let mut omitted = 0usize;
    let mut samples = 0usize;
    for c in &curves {
        for value in &c.values {
            samples += 1;
            let v = *value / c.scale;
            if valid(v) {
                let v = if options.logarithmic { v.log10() } else { v };
                lo = lo.min(v);
                hi = hi.max(v);
            } else {
                omitted += 1;
            }
        }
    }
    let has_values = lo.is_finite();
    if !has_values {
        (lo, hi) = if options.logarithmic {
            (-6., 0.)
        } else {
            (0., 1.)
        };
    } else if options.logarithmic {
        lo = lo.floor();
        hi = hi.ceil();
        if hi - lo < 2. {
            lo -= 1.;
            hi += 1.;
        }
    } else {
        lo = 0.;
        hi = if hi > 0. { hi * 1.08 } else { 1. };
    }
    let px = |x: f64| left + (x - x0) / (x1 - x0) * (right - left);
    let py = |v: f64| bottom - (v - lo) / (hi - lo) * (bottom - top);
    let mut svg = format!(
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}"><rect width="100%" height="100%" fill="#20242c"/><g font-family="DejaVu Sans, sans-serif" font-size="12" fill="#bec7d4"><text x="{left}" y="24" font-size="15" font-weight="600" fill="#edf1f7">Convergence monitors</text><defs><clipPath id="plot"><rect x="{left}" y="{top}" width="{}" height="{}"/></clipPath></defs>"##,
        right - left,
        bottom - top
    );
    if options.legend {
        for (i, c) in curves.iter().enumerate() {
            let x = left + (i % 2) as f64 * (right - left) / 2.;
            let y = 48. + (i / 2) as f64 * 22.;
            let last = c
                .values
                .last()
                .copied()
                .filter(|v| v.is_finite())
                .map(number)
                .unwrap_or_else(|| "—".into());
            let _ = write!(
                svg,
                r##"<path d="M{x} {} h18" stroke="{}" stroke-width="2.5"/><text x="{}" y="{y}">{}: {last}</text>"##,
                y - 4.,
                c.color,
                x + 25.,
                if w < 720. {
                    c.name.split('·').next().unwrap_or(c.name).trim()
                } else {
                    c.name
                }
            );
        }
    }
    let ticks = ((bottom - top) / 42.).clamp(2., 7.);
    let ystep = if options.logarithmic {
        ((hi - lo) / ticks).ceil().max(1.)
    } else {
        (hi - lo) / ticks.floor()
    };
    let mut tick = (lo / ystep).ceil() * ystep;
    while tick <= hi + ystep * 0.001 {
        let y = py(tick);
        let label = if options.logarithmic {
            format!("1e{:+.0}", tick)
        } else {
            number(tick)
        };
        let _ = write!(
            svg,
            r##"<path d="M{left} {y} H{right}" stroke="#3b424e" stroke-width="0.7"/><text x="{}" y="{}" text-anchor="end">{label}</text>"##,
            left - 12.,
            y + 4.
        );
        tick += ystep;
    }
    let step = nice_step(x1 - x0);
    let mut tick = (x0 / step).ceil() * step;
    while tick <= x1 + step * 0.001 {
        let x = px(tick);
        let _ = write!(
            svg,
            r##"<path d="M{x} {top} V{bottom}" stroke="#343b47" stroke-width="0.7"/><text x="{x}" y="{}" text-anchor="middle">{tick:.0}</text>"##,
            bottom + 22.
        );
        tick += step;
    }
    let _ = write!(
        svg,
        r##"<path d="M{left} {top} V{bottom} H{right}" fill="none" stroke="#a4afbf"/><text x="{}" y="{}" text-anchor="middle">Iteration / physical step</text>"##,
        (left + right) / 2.,
        h - 17.
    );
    let y_label = match (options.logarithmic, options.normalized) {
        (true, true) => "Value / first positive sample · log",
        (true, false) => "Monitor value · log scale",
        (false, true) => "Value / first positive sample",
        _ => "Monitor value · linear scale",
    };
    let cy = (top + bottom) / 2.;
    let _ = write!(
        svg,
        r##"<text transform="translate(20 {cy}) rotate(-90)" text-anchor="middle">{y_label}</text><g clip-path="url(#plot)">"##
    );
    for c in &curves {
        let mut path = String::new();
        let mut connected = false;
        let mut last_point = None;
        // Keep extrema in each screen-space bucket instead of hiding spikes by striding.
        let stride = (c.values.len() / ((right - left).max(1.) as usize * 2)).max(1);
        for (chunk_index, chunk) in c.values.chunks(stride).enumerate() {
            let mut selected = vec![0, chunk.len() - 1];
            let mut min = None::<(usize, f64)>;
            let mut max = None::<(usize, f64)>;
            for (i, raw) in chunk.iter().enumerate() {
                let v = *raw / c.scale;
                if valid(v) {
                    if min.is_none_or(|(_, old)| v < old) {
                        min = Some((i, v));
                    }
                    if max.is_none_or(|(_, old)| v > old) {
                        max = Some((i, v));
                    }
                } else {
                    selected.push(i);
                }
            }
            if let Some((i, _)) = min {
                selected.push(i);
            }
            if let Some((i, _)) = max {
                selected.push(i);
            }
            selected.sort_unstable();
            selected.dedup();
            for i in selected {
                let value = chunk[i] / c.scale;
                if !valid(value) {
                    connected = false;
                    continue;
                }
                let x = px((c.first + chunk_index * stride + i) as f64);
                let y = py(if options.logarithmic {
                    value.log10()
                } else {
                    value
                });
                let _ = write!(path, "{} {x:.2} {y:.2} ", if connected { "L" } else { "M" });
                connected = true;
                last_point = Some((x, y));
            }
        }
        let _ = write!(
            svg,
            r##"<path d="{path}" fill="none" stroke="{}" stroke-width="2" stroke-linejoin="round" stroke-linecap="round"/>"##,
            c.color
        );
        if let Some((x, y)) = last_point {
            let _ = write!(
                svg,
                r##"<circle cx="{x:.2}" cy="{y:.2}" r="3" fill="{}"/>"##,
                c.color
            );
        }
    }
    svg.push_str("</g>");
    if !has_values {
        let text = if samples == 0 {
            "No recorded data for the selected monitors"
        } else {
            "No positive values on logarithmic scale · select Linear to show zeros"
        };
        let _ = write!(
            svg,
            r##"<text x="{}" y="{}" text-anchor="middle" fill="#aeb8c8">{text}</text>"##,
            (left + right) / 2.,
            (top + bottom) / 2.
        );
    }
    svg.push_str("</g></svg>");
    let caption=format!("Iterations / steps {x0:.0}–{x1:.0} · {}{} · {}. Pressure is an inner-solver metric; continuity alone does not prove convergence.{}",if options.logarithmic{"logarithmic"}else{"linear"},if options.normalized{" · normalized display"}else{" · raw values"},if options.normalized{"Stopping criteria use original values"}else{"Monitors have different definitions / units"},if omitted>0 {format!(" {omitted} zero or invalid samples omitted on this scale.")}else{String::new()});
    Plot { svg, caption }
}
pub(super) fn readout(state: &AppState, options: &Options, x: f64) -> String {
    let curves = curves(state, options);
    let (x0, x1) = x_range(&curves);
    let right = options.width as f64 - 24.;
    if x < 96. || x > right {
        return "Hover over the plot to inspect recorded values.".into();
    }
    let iteration = (x0 + (x - 96.) / (right - 96.) * (x1 - x0)).round() as usize;
    let values = curves
        .iter()
        .map(|c| {
            format!(
                "{}: {}",
                c.name,
                iteration
                    .checked_sub(c.first)
                    .and_then(|i| c.values.get(i))
                    .copied()
                    .filter(|v| v.is_finite())
                    .map(number)
                    .unwrap_or_else(|| "—".into())
            )
        })
        .collect::<Vec<_>>();
    format!("Iteration / step {iteration}  |  {}", values.join("  ·  "))
}
pub(super) fn refresh(ui: &MainWindow, state: &AppState) {
    let plot = build(state, &Options::from_ui(ui));
    if let Ok(image) = slint::Image::load_from_svg_data(plot.svg.as_bytes()) {
        ui.set_residual_image(image);
    }
    ui.set_continuity_chart_caption(plot.caption.into());
}
pub(super) fn bind(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_chart_resize(move |w, h| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let (w, h) = ((w as i32).clamp(480, 3200), (h as i32).clamp(240, 1800));
        if (w - ui.get_chart_width()).abs() < 2 && (h - ui.get_chart_height()).abs() < 2 {
            return;
        }
        ui.set_chart_width(w);
        ui.set_chart_height(h);
        refresh(&ui, &shared.borrow());
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_chart_hover(move |x| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        ui.set_chart_readout(readout(&shared.borrow(), &Options::from_ui(&ui), x as f64).into());
    });
}
