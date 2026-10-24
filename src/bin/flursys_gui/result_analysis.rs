//! Run-aware measurements and portable, self-contained reports.
use super::*;
use std::io::Write;

#[derive(Clone)]
pub(super) struct ProfilePoint {
    pub distance: f64,
    pub position: Vec3,
    pub cell: Option<usize>,
    pub value: Option<f64>,
}

pub(super) fn sample_line(
    data: &ResultDataset,
    start: Vec3,
    end: Vec3,
    count: usize,
    field: ResultFieldKind,
) -> Result<Vec<ProfilePoint>, String> {
    if data.mesh().dimension() != MeshDimension::TwoD {
        return Err("Line profiles currently require a 2D result".into());
    }
    let length = (end - start).norm();
    if !length.is_finite() || length <= f64::EPSILON {
        return Err("Choose two distinct finite endpoints".into());
    }
    let locator = StreamlineField::from_dataset(data).map_err(|e| e.to_string())?;
    let values = data.scalar_values(field);
    if values.len() != data.mesh().cell_count() {
        return Err("The selected run has no data for this field".into());
    }
    Ok((0..count.clamp(2, 2000))
        .map(|i| {
            let t = i as f64 / (count.clamp(2, 2000) - 1) as f64;
            let position = start + (end - start) * t;
            let cell = locator.locate_cell(position);
            ProfilePoint {
                distance: t * length,
                position,
                cell,
                value: cell
                    .and_then(|c| values.get(c).copied())
                    .filter(|v| v.is_finite()),
            }
        })
        .collect())
}
pub(super) fn atomic_text(path: &Path, text: &str) -> Result<(), String> {
    let mut file = tempfile::NamedTempFile::new_in(path.parent().unwrap_or(Path::new(".")))
        .map_err(|e| e.to_string())?;
    file.write_all(text.as_bytes()).map_err(|e| e.to_string())?;
    file.flush().map_err(|e| e.to_string())?;
    file.persist(path).map_err(|e| e.to_string())?;
    Ok(())
}
fn escape(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

/// Full histories for export, independent of the 400-sample viewport window.
pub(super) fn histories(state: &AppState) -> [Vec<f64>; 4] {
    if !state.solving
        && (state.current_step == 3
            || state
                .active_result()
                .is_some_and(|r| r.run_id() == "current-solve"))
    {
        if let Some(s) = state.workbench.solution() {
            return [
                s.report.continuity_history.clone(),
                s.report.simple.momentum_relative_history.clone(),
                s.report.simple.velocity_change_history.clone(),
                s.report.simple.pressure_linear_history.clone(),
            ];
        }
    }
    if !state.solving {
        if let Some(run) = state.active_result().and_then(|d| {
            state
                .workbench_project
                .runs
                .iter()
                .find(|r| r.id == d.run_id())
        }) {
            if let Some(report) = state
                .workspace_path
                .as_ref()
                .and_then(|root| std::fs::read(root.join(&run.report_path)).ok())
                .and_then(|b| serde_json::from_slice::<serde_json::Value>(&b).ok())
            {
                let read = |v: Option<&serde_json::Value>| {
                    v.and_then(|v| v.as_array())
                        .map(|a| a.iter().filter_map(|x| x.as_f64()).collect())
                        .unwrap_or_default()
                };
                return [
                    read(report.get("continuity_history")),
                    read(report.pointer("/convergence_histories/momentum")),
                    read(report.pointer("/convergence_histories/velocity")),
                    read(report.pointer("/convergence_histories/pressure_linear")),
                ];
            }
        }
    }
    [
        if state.post.continuity_full.is_empty() {
            state
                .wb_residual_history
                .iter()
                .map(|s| s.continuity)
                .collect()
        } else {
            state.post.continuity_full.clone()
        },
        state.post.metric_histories[0].clone(),
        state.post.metric_histories[1].clone(),
        state.post.metric_histories[2].clone(),
    ]
}
pub(super) fn balance(state: &AppState) -> String {
    if state.solving {
        return "Conservative boundary balance is evaluated after solve completion.".into();
    }
    let v = if state.current_step == 3 {
        state.workbench.solution().map(|s| {
            (
                s.report.total_inflow,
                s.report.total_outflow,
                s.report.net_boundary_flux,
            )
        })
    } else {
        None
    }
    .or_else(|| {
        state
            .active_result()
            .and_then(|d| {
                state
                    .workbench_project
                    .runs
                    .iter()
                    .find(|r| r.id == d.run_id())
            })
            .and_then(|r| Some((r.total_inflow?, r.total_outflow?, r.net_boundary_flux?)))
    });
    let Some((qin, qout, net)) = v else {
        return "Conservative boundary flux balance unavailable for this run.".into();
    };
    let scale = qin.abs().max(qout.abs());
    let relative = if scale > 1e-30 {
        format!("{:.5}%", 100.0 * net.abs() / scale)
    } else {
        "N/A · closed/zero-throughflow domain".into()
    };
    format!(
        "CONSERVATIVE BOUNDARY BALANCE\nInflow {qin:.6e}\nOutflow {qout:.6e}\nSigned net flux {net:.6e}\nRelative imbalance {relative}\nVolume flux: m³/s in 3D; m²/s per unit depth in 2D. Density is not applied."
    )
}

pub(super) fn profile_svg(points: &[ProfilePoint], title: &str, log: bool) -> String {
    let series = points
        .iter()
        .map(|p| {
            (
                p.distance,
                p.value
                    .filter(|v| v.is_finite() && (!log || *v > 0.0))
                    .map(|v| if log { v.log10() } else { v }),
            )
        })
        .collect::<Vec<_>>();
    let finite = series
        .iter()
        .filter_map(|(x, y)| y.map(|y| (*x, y)))
        .collect::<Vec<_>>();
    let mut svg = format!(
        "<svg xmlns='http://www.w3.org/2000/svg' width='900' height='350' viewBox='0 0 900 350' role='img'><rect width='900' height='350' fill='#101c26'/><g font-family='sans-serif' font-size='13' fill='#dce9ef'><text x='75' y='25'>{}</text>",
        escape(title)
    );
    if finite.is_empty() {
        svg.push_str("<text x='75' y='100'>No recorded finite samples</text></g></svg>");
        return svg;
    }
    let xmin = series.first().map_or(0.0, |p| p.0);
    let end = series.last().map_or(1.0, |p| p.0);
    let xmax = if end > xmin {
        end
    } else {
        xmin + xmin.abs().max(1.0) * 0.05
    };
    let mut ymin = finite.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
    let mut ymax = finite.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
    if ymin == ymax {
        let pad = ymin.abs().max(1.0) * 0.05;
        ymin -= pad;
        ymax += pad;
    }
    for i in 0..=4 {
        let t = i as f64 / 4.0;
        let y = 285.0 - 230.0 * t;
        let v = ymin + (ymax - ymin) * t;
        let label = if log {
            format!("{:.2e}", 10f64.powf(v))
        } else {
            format!("{v:.3e}")
        };
        svg.push_str(&format!(
            "<path d='M75 {y} H875' stroke='#304552'/><text x='5' y='{y}'>{label}</text>"
        ));
        let x = 75.0 + 800.0 * t;
        svg.push_str(&format!(
            "<text x='{x}' y='315'>{:.3e}</text>",
            xmin + (xmax - xmin) * t
        ));
    }
    let mut path = String::new();
    let mut connected = false;
    for (x, y) in series {
        if let Some(y) = y {
            let x = 75.0 + 800.0 * (x - xmin) / (xmax - xmin);
            let y = 285.0 - 230.0 * (y - ymin) / (ymax - ymin);
            path.push_str(&format!(
                "{} {x:.2} {y:.2} ",
                if connected { "L" } else { "M" }
            ));
            connected = true;
        } else {
            connected = false;
        }
    }
    svg.push_str(&format!(
        "<path d='{path}' stroke='#58d7db' stroke-width='2' fill='none'/></g></svg>"
    ));
    svg
}
pub(super) fn profile_csv(points: &[ProfilePoint], label: &str) -> String {
    let mut out = format!(
        "# {label}; piecewise-constant owning-cell samples; blank = outside domain\ndistance,x,y,z,cell,value\n"
    );
    for p in points {
        out.push_str(&format!(
            "{:.17e},{:.17e},{:.17e},{:.17e},{},{}\n",
            p.distance,
            p.position.x,
            p.position.y,
            p.position.z,
            p.cell.map(|v| v.to_string()).unwrap_or_default(),
            p.value.map(|v| format!("{v:.17e}")).unwrap_or_default()
        ));
    }
    out
}
fn saved_report(state: &AppState) -> Option<serde_json::Value> {
    let run = state.active_result()?.run_id();
    let record = state.workbench_project.runs.iter().find(|r| r.id == run)?;
    let text =
        std::fs::read_to_string(state.workspace_path.as_ref()?.join(&record.report_path)).ok()?;
    serde_json::from_str(&text).ok()
}

pub(super) fn engineering_summary_csv(state: &AppState) -> Result<String, String> {
    let data = state
        .active_result()
        .ok_or("Select a completed result first")?;
    let mut out = String::from("quantity,minimum,maximum,volume_weighted_mean\n");
    for field in [
        ResultFieldKind::Pressure,
        ResultFieldKind::VelocityMagnitude,
        ResultFieldKind::VelocityX,
        ResultFieldKind::VelocityY,
        ResultFieldKind::VelocityZ,
        ResultFieldKind::VorticityMagnitude,
        ResultFieldKind::VelocityDivergence,
        ResultFieldKind::QCriterion,
        ResultFieldKind::FlowType,
        ResultFieldKind::StrainRate,
        ResultFieldKind::Temperature,
        ResultFieldKind::TurbulentK,
        ResultFieldKind::SpecificDissipation,
        ResultFieldKind::TurbulentViscosity,
    ] {
        if let Ok((lo, hi)) = data.scalar_range(field) {
            out.push_str(&format!(
                "\"{}\",{lo:.17e},{hi:.17e},{:.17e}\n",
                field.label().replace('"', "\"\""),
                data.volume_mean(field).unwrap_or(f64::NAN)
            ));
        }
    }
    Ok(out)
}

pub(super) fn report(state: &AppState) -> Result<String, String> {
    let d = state
        .active_result()
        .ok_or("Select a completed result first")?;
    let mut out = format!(
        "<!doctype html><html lang='en'><meta charset='utf-8'><title>FLURSYS — {}</title><style>body{{font:15px system-ui;max-width:1100px;margin:36px auto;padding:20px;color:#172b38}}table{{border-collapse:collapse;width:100%;margin:20px 0}}td,th{{border:1px solid #ccd5da;padding:9px;text-align:left}}svg{{width:100%;break-inside:avoid}}pre{{white-space:pre-wrap}}@media print{{body{{margin:0}}h2{{break-after:avoid}}}}</style><h1>FLURSYS · CFD run report</h1><p>Run: <b>{}</b> · {:?} · {} cells · {} nodes</p><p>Generated Unix seconds: {}</p><pre>{}</pre><h2>Field statistics</h2><table><tr><th>Quantity</th><th>Minimum</th><th>Maximum</th><th>Volume-weighted mean</th></tr>",
        escape(d.run_id()),
        escape(d.run_id()),
        d.mesh().dimension(),
        d.mesh().cell_count(),
        d.mesh().points().len(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs(),
        escape(&balance(state))
    );
    for f in [
        ResultFieldKind::Pressure,
        ResultFieldKind::VelocityMagnitude,
        ResultFieldKind::VelocityX,
        ResultFieldKind::VelocityY,
        ResultFieldKind::VelocityZ,
        ResultFieldKind::VorticityMagnitude,
        ResultFieldKind::VelocityDivergence,
        ResultFieldKind::QCriterion,
        ResultFieldKind::FlowType,
        ResultFieldKind::StrainRate,
        ResultFieldKind::Temperature,
        ResultFieldKind::TurbulentK,
        ResultFieldKind::SpecificDissipation,
        ResultFieldKind::TurbulentViscosity,
    ] {
        if let Ok((lo, hi)) = d.scalar_range(f) {
            out.push_str(&format!(
                "<tr><td>{}</td><td>{lo:.7e}</td><td>{hi:.7e}</td><td>{:.7e}</td></tr>",
                escape(f.label()),
                d.volume_mean(f).unwrap_or(f64::NAN)
            ));
        }
    }
    out.push_str("</table><p>Pressure is kinematic gauge pressure p/ρ. Curl and divergence are reconstructed from cell values; reconstructed divergence is not conservative flux continuity. First-cell wall traction estimates, when stored with this run, are listed separately below.</p>");
    if !state.post.boundary_reports.is_empty() {
        out.push_str("<h2>Conservative patch flow integrals</h2><table><tr><th>Patch</th><th>Measure</th><th>Signed Q</th><th>Signed ρQ</th><th>Adjacent p/ρ mean</th></tr>");
        for r in &state.post.boundary_reports {
            out.push_str(&format!(
                "<tr><td>{}</td><td>{:.7e}</td><td>{:+.7e}</td><td>{:+.7e}</td><td>{}</td></tr>",
                escape(&r.patch),
                r.measure,
                r.volume_flux,
                r.mass_flux,
                r.adjacent_pressure_mean
                    .map(|v| format!("{v:.7e}"))
                    .unwrap_or("N/A".into())
            ));
        }
        out.push_str("</table><p>Negative flux enters the fluid. In 2D, measure is length, Q and ρQ are per unit depth. Pressure is an adjacent-cell estimate; no wall-force inference is made.</p>");
    }
    if !state.post.wall_reports.is_empty() {
        out.push_str("<h2>Wall resolution and force estimates</h2><pre>");
        out.push_str(&escape(&flursys::workbench::wall_reports::summary(
            &state.post.wall_reports,
        )));
        out.push_str("</pre><p>First-cell wall-resolved estimate; pressure force uses adjacent-cell gauge pressure. No high-Re wall function. Integrated 2D forces are per unit depth.</p>");
    }
    out.push_str("<h2>Run provenance and settings</h2>");
    if let Some(saved) = saved_report(state) {
        for (key, title) in [
            ("run", "Run status"),
            ("solver_version", "Solver version"),
            ("material", "Fluid material"),
            ("solver_settings", "Numerical controls"),
            ("boundaries", "Boundary conditions"),
            ("mesh_settings", "Meshing controls"),
            ("mesh_recipe", "Meshing algorithms and refinement"),
            ("spatial_methods", "Spatial discretization"),
            ("mesh_preflight", "Mesh preflight"),
            ("conservation", "Conservative mass balance"),
        ] {
            if let Some(value) = saved.get(key).filter(|v| !v.is_null()) {
                out.push_str(&format!(
                    "<h3>{title}</h3><pre>{}</pre>",
                    escape(&serde_json::to_string_pretty(value).map_err(|e| e.to_string())?)
                ));
            }
        }
    } else {
        out.push_str("<p>Archived run settings unavailable. Current editable settings are not substituted for historical settings.</p>");
    }
    let q = d.mesh().quality();
    out.push_str(&format!("<h2>Result mesh quality</h2><table><tr><th>Metric</th><th>Value</th></tr><tr><td>Minimum cell measure</td><td>{:.7e}</td></tr><tr><td>Maximum cell measure</td><td>{:.7e}</td></tr><tr><td>Maximum aspect ratio</td><td>{:.7e}</td></tr><tr><td>Maximum non-orthogonality (degrees)</td><td>{:.4}</td></tr><tr><td>Centre-line displacement / centre distance</td><td>{:.7e}</td></tr></table>",q.min_cell_volume,q.max_cell_volume,q.max_aspect_ratio,q.max_non_orthogonality_degrees,q.max_skewness));
    out.push_str("<p>Cell measure is area in 2D and volume in 3D. Boundary flux and reconstructed cell divergence have different definitions. Zero boundary imbalance alone is insufficient evidence of convergence in a closed domain. Numerical convergence does not establish mesh or time-step independence.</p>");
    out.push_str("<h2>Recorded convergence</h2>");
    let h = histories(state);
    let last = state
        .active_result()
        .and_then(|d| {
            state
                .workbench_project
                .runs
                .iter()
                .find(|r| r.id == d.run_id())
        })
        .and_then(|r| r.iterations)
        .unwrap_or_else(|| h.iter().map(Vec::len).max().unwrap_or(1).saturating_sub(1));
    for (name, values) in [
        "Continuity RMS",
        "Momentum relative",
        "Velocity change relative",
        "Pressure linear final residual",
    ]
    .iter()
    .zip(h)
    {
        let first = last.saturating_add(1).saturating_sub(values.len());
        let p = values
            .iter()
            .enumerate()
            .map(|(i, v)| ProfilePoint {
                distance: (first + i) as f64,
                position: Vec3::default(),
                cell: None,
                value: Some(*v),
            })
            .collect::<Vec<_>>();
        out.push_str(&profile_svg(
            &p,
            &format!("{name} · logarithmic · iteration"),
            true,
        ));
    }
    if !state.post.profile.is_empty() {
        out.push_str("<h2>Line profile</h2>");
        out.push_str(&profile_svg(
            &state.post.profile,
            &state.post.profile_label,
            false,
        ));
        out.push_str(&format!(
            "<pre>{}</pre>",
            escape(&line_measurements(&state.post.profile))
        ));
        out.push_str("<p>Piecewise-constant owning-cell samples. Gaps indicate locations outside the fluid mesh. No wall values are extrapolated.</p>");
    }
    if let Some(run) = state
        .workbench_project
        .runs
        .iter()
        .find(|r| r.id == d.run_id())
    {
        out.push_str(&format!(
            "<h2>Run metadata</h2><pre>{}</pre>",
            escape(&serde_json::to_string_pretty(run).map_err(|e| e.to_string())?)
        ));
    }
    if let Some(json) = state
        .active_result()
        .and_then(|d| {
            state
                .workbench_project
                .runs
                .iter()
                .find(|r| r.id == d.run_id())
        })
        .and_then(|r| {
            state
                .workspace_path
                .as_ref()
                .and_then(|root| std::fs::read(root.join(&r.report_path)).ok())
        })
        .and_then(|b| serde_json::from_slice::<serde_json::Value>(&b).ok())
    {
        for key in [
            "material",
            "solver_settings",
            "boundaries",
            "mesh_settings",
            "coupling_convergence",
            "boundary_flow_reports",
        ] {
            if let Some(v) = json.get(key) {
                out.push_str(&format!(
                    "<h2>{}</h2><pre>{}</pre>",
                    escape(key),
                    escape(&serde_json::to_string_pretty(v).map_err(|e| e.to_string())?)
                ));
            }
        }
    }
    out.push_str("<p>Steady tracer animations illustrate a stationary field; they are not physical transient solutions. Use the browser Print dialog to save this report as PDF.</p></html>");
    Ok(out)
}

pub(super) fn line_measurements(points: &[ProfilePoint]) -> String {
    let mut integral = 0.0;
    let mut covered = 0.0;
    for pair in points.windows(2) {
        if let (Some(a), Some(b)) = (pair[0].value, pair[1].value) {
            let ds = pair[1].distance - pair[0].distance;
            integral += 0.5 * (a + b) * ds;
            covered += ds;
        }
    }
    let mean = if covered > 0.0 {
        format!("{:.6e}", integral / covered)
    } else {
        "N/A".into()
    };
    let delta = points
        .first()
        .and_then(|a| a.value)
        .zip(points.last().and_then(|b| b.value))
        .map(|(a, b)| format!("{:.6e}", b - a))
        .unwrap_or("N/A · endpoint outside fluid".into());
    format!(
        "Line-weighted mean {mean} · covered length {covered:.6e}\nEnd − start {delta} (selected field units; p/ρ for pressure)"
    )
}
pub(super) fn convergence_summary(state: &AppState) -> String {
    let ends = |v: &[f64]| (v.first().copied(), v.last().copied());
    let series = if state.current_step == 3 && !state.solving {
        state.workbench.solution().map(|s| {
            [
                ends(&s.report.continuity_history),
                ends(&s.report.simple.momentum_relative_history),
                ends(&s.report.simple.velocity_change_history),
                ends(&s.report.simple.pressure_linear_history),
            ]
        })
    } else {
        None
    }
    .unwrap_or_else(|| {
        [
            if state.post.continuity_full.is_empty() {
                (
                    state.wb_residual_history.front().map(|v| v.continuity),
                    state.wb_residual_history.back().map(|v| v.continuity),
                )
            } else {
                ends(&state.post.continuity_full)
            },
            ends(&state.post.metric_histories[0]),
            ends(&state.post.metric_histories[1]),
            ends(&state.post.metric_histories[2]),
        ]
    });
    let mut out = String::from("LATEST RECORDED METRICS\n");
    for (name, (first, last)) in [
        "Continuity RMS",
        "Momentum relative",
        "Velocity change",
        "Pressure linear",
    ]
    .iter()
    .zip(series)
    {
        let change = first
            .filter(|x| *x > 0.0)
            .zip(last)
            .map(|(a, b)| format!(" · final/first {:.3e}", b / a))
            .unwrap_or_default();
        out.push_str(&format!(
            "{name}: {}{change}\n",
            last.map(|x| format!("{x:.5e}"))
                .unwrap_or("unavailable".into())
        ));
    }
    out.push_str("Each curve has its own normalization. Pressure linear residual is an inner-solver metric, not an outer convergence criterion.");
    out
}

pub(super) fn boundary_summary(
    reports: &[flursys::workbench::boundary_reports::BoundaryFlowReport],
) -> String {
    let mut out = String::from("PATCH FLOW INTEGRALS\n");
    for r in reports {
        out.push_str(&format!(
            "{} · {} faces\nQ {:+.6e} · ρQ {:+.6e}\nAdjacent-cell p/ρ mean {}\n",
            r.patch,
            r.faces,
            r.volume_flux,
            r.mass_flux,
            r.adjacent_pressure_mean
                .map(|v| format!("{v:.6e}"))
                .unwrap_or("N/A".into())
        ));
    }
    out.push_str("Flux is owner-oriented: negative enters fluid. In 2D, flow and mass flow are per unit depth. Pressure means are adjacent-cell estimates, not wall pressure or force.");
    out
}

pub(super) fn boundary_csv(
    reports: &[flursys::workbench::boundary_reports::BoundaryFlowReport],
) -> Result<String, String> {
    if reports.is_empty() {
        return Err(
            "Patch flow integrals were not recorded for this run; they are saved on new solves"
                .into(),
        );
    }
    let mut out = String::from(
        "patch,faces,measure,volume_flux,mass_flux,adjacent_p_over_rho_mean,adjacent_speed_mean\n",
    );
    for r in reports {
        out.push_str(&format!(
            "\"{}\",{},{:.17e},{:.17e},{:.17e},{},{}\n",
            r.patch.replace('"', "\"\""),
            r.faces,
            r.measure,
            r.volume_flux,
            r.mass_flux,
            r.adjacent_pressure_mean
                .map(|v| format!("{v:.17e}"))
                .unwrap_or_default(),
            r.adjacent_speed_mean
                .map(|v| format!("{v:.17e}"))
                .unwrap_or_default()
        ));
    }
    Ok(out)
}
