//! Time statistics of archived, fixed-mesh plane samples.
//! Trapezoidal quadrature in physical time; never interpret SIMPLE iterations as time.
use super::plane_monitor::PlaneSnapshot;
use std::{fmt::Write, path::PathBuf};

pub fn temporal_csv(paths: &[PathBuf]) -> Result<String, String> {
    if paths.len() < 2 {
        return Err("Time statistics require at least two physical-time JSON frames".into());
    }
    let read = |path: &PathBuf| -> Result<PlaneSnapshot, String> {
        let bytes = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
        serde_json::from_slice(&bytes).map_err(|e| format!("{}: {e}", path.display()))
    };
    let mut previous = read(&paths[0])?;
    let first_time = previous.physical_time.filter(|t| t.is_finite()).ok_or(
        "Physical timestamps are required; steady iteration recordings are not time statistics",
    )?;
    if previous.samples.is_empty() {
        return Err("No plane samples to average".into());
    }
    let mut previous_time = first_time;
    let mut mean = vec![[0.0; 4]; previous.samples.len()];
    let mut second = vec![[0.0; 7]; previous.samples.len()];
    let values = |s: &super::plane_monitor::PlaneSample| {
        [s.velocity.x, s.velocity.y, s.velocity.z, s.pressure]
    };
    let mut accumulated_weight = 0.0;
    for path in &paths[1..] {
        let current = read(path)?;
        let time = current
            .physical_time
            .filter(|t| t.is_finite() && *t > previous_time)
            .ok_or("Frame timestamps must be finite and strictly increasing")?;
        if current.plane.id != previous.plane.id
            || current.plane.origin != previous.plane.origin
            || current.plane.u != previous.plane.u
            || current.plane.v != previous.plane.v
            || current.samples.len() != previous.samples.len()
        {
            return Err(
                "Time statistics require one fixed plane and unchanged sample geometry".into(),
            );
        }
        let weight = 0.5 * (time - previous_time);
        for (i, (a, b)) in previous.samples.iter().zip(&current.samples).enumerate() {
            if a.geometry.cell != b.geometry.cell
                || a.geometry.polygon != b.geometry.polygon
                || a.geometry.point != b.geometry.point
            {
                return Err("Sample geometry changed between frames; remapping is required".into());
            }
            let va = values(a);
            let vb = values(b);
            if va.iter().chain(&vb).any(|v| !v.is_finite()) {
                return Err("Non-finite archived field sample".into());
            }
            // Weighted online central moments. Trapezoidal quadrature contributes
            // each interval endpoint with half the interval duration.
            let mut total = accumulated_weight;
            for value in [va, vb] {
                let next = total + weight;
                let delta = std::array::from_fn::<_, 4, _>(|j| value[j] - mean[i][j]);
                for j in 0..4 {
                    mean[i][j] += delta[j] * (weight / next);
                }
                let factor = weight * (total / next);
                for j in 0..4 {
                    second[i][j] += factor * delta[j] * delta[j];
                }
                second[i][4] += factor * delta[0] * delta[1];
                second[i][5] += factor * delta[0] * delta[2];
                second[i][6] += factor * delta[1] * delta[2];
                total = next;
            }
        }
        accumulated_weight += 2.0 * weight;
        previous = current;
        previous_time = time;
    }
    if accumulated_weight <= 0.0 || !accumulated_weight.is_finite() {
        return Err("Invalid physical averaging duration".into());
    }
    let mut out = String::from("cell,x,y,z,time_start_s,time_end_s,frames,ux_mean,uy_mean,uz_mean,p_over_rho_mean,ux_rms,uy_rms,uz_rms,p_over_rho_rms,uu,vv,ww,uv,uw,vw\n");
    for (i, sample) in previous.samples.iter().enumerate() {
        let m = mean[i];
        let e = second[i].map(|v| v / accumulated_weight);
        let variance = [0, 1, 2, 3].map(|j| e[j].max(0.0));
        if m.iter().chain(&e).any(|v| !v.is_finite()) {
            return Err("Time statistics overflowed; no result was exported".into());
        }
        let point = sample.geometry.point;
        write!(
            out,
            "{},{:.15e},{:.15e},{:.15e},{:.15e},{:.15e},{}",
            sample.geometry.cell,
            point.x,
            point.y,
            point.z,
            first_time,
            previous_time,
            paths.len()
        )
        .map_err(|e| e.to_string())?;
        for v in m.into_iter().chain(variance.map(f64::sqrt)).chain([
            variance[0],
            variance[1],
            variance[2],
            e[4],
            e[5],
            e[6],
        ]) {
            write!(out, ",{v:.15e}").map_err(|e| e.to_string())?;
        }
        out.push('\n');
    }
    Ok(out)
}

/// Range of one recorded scalar; holds only one decoded frame at a time.
pub fn recorded_range(paths: &[PathBuf], field: usize) -> Result<[f64; 2], String> {
    let mut range = [f64::INFINITY, f64::NEG_INFINITY];
    let mut identity = None;
    for path in paths {
        let bytes = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let snapshot: PlaneSnapshot = serde_json::from_slice(&bytes).map_err(|_| {
            "Colour range requires raw JSON frames; legacy SVG is not supported".to_string()
        })?;
        if identity.as_ref().is_some_and(|id| id != &snapshot.plane.id) {
            return Err("Recording contains different planes".into());
        }
        identity = Some(snapshot.plane.id.clone());
        for sample in &snapshot.samples {
            let value = super::plane_monitor::sample_scalar(&snapshot, sample, field);
            if value.is_finite() {
                range[0] = range[0].min(value);
                range[1] = range[1].max(value);
            }
        }
    }
    if !range[0].is_finite() || !range[1].is_finite() {
        return Err("No finite recorded field samples".into());
    }
    if range[0] == range[1] {
        let padding = range[0].abs().max(1.0) * 1e-6;
        range[0] -= padding;
        range[1] += padding;
    }
    Ok(range)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{workbench::plane_monitor::*, Vec3};
    fn snapshot(t: Option<f64>, speed: f64) -> PlaneSnapshot {
        PlaneSnapshot {
            plane: FlowPlaneMonitor {
                id: "p".into(),
                name: "Section".into(),
                origin: Vec3::new(0., 0., 0.),
                u: Vec3::new(1., 0., 0.),
                v: Vec3::new(0., 1., 0.),
                enabled: true,
                every: 1,
                display: Default::default(),
            },
            iteration: 1,
            physical_time: t,
            path: None,
            samples: vec![PlaneSample {
                geometry: PlaneCell {
                    cell: 0,
                    point: Vec3::new(0.5, 0.5, 0.),
                    polygon: vec![
                        Vec3::new(0., 0., 0.),
                        Vec3::new(1., 0., 0.),
                        Vec3::new(1., 1., 0.),
                        Vec3::new(0., 1., 0.),
                    ],
                },
                velocity: Vec3::new(speed, 2. * speed, 0.),
                pressure: speed,
            }],
        }
    }
    fn archives(samples: &[PlaneSnapshot], folder: &std::path::Path) -> Vec<PathBuf> {
        samples
            .iter()
            .enumerate()
            .map(|(i, s)| {
                let path = folder.join(format!("{i}.json"));
                std::fs::write(&path, serde_json::to_vec(s).unwrap()).unwrap();
                path
            })
            .collect()
    }
    #[test]
    fn section_camera_probes_remain_bound_to_data_after_pan_and_zoom() {
        let sample = snapshot(None, 2.);
        let mut camera = SectionCamera::default();
        for zoom in [0.5, 1., 5.] {
            camera.zoom = zoom;
            camera.pan = [0.03, -0.02];
            let p = Vec3::new(0.52, 0.48, 0.);
            let q = camera.screen_at(&sample, p);
            assert!((camera.world_at(&sample, q[0], q[1]) - p).norm() < 1e-12);
            assert_eq!(
                probe_section(&sample, &camera, q[0], q[1])
                    .unwrap()
                    .velocity
                    .x,
                2.
            );
        }
        assert!(probe_section(&sample, &camera, 990., 300.).is_none());
    }
    #[test]
    fn probe_history_uses_fields_and_marks_missing_coverage() {
        let folder = tempfile::tempdir().unwrap();
        let mut outside = snapshot(Some(1.), 4.);
        outside.plane.origin.z = 1.;
        let paths = archives(&[snapshot(Some(0.), 2.), outside], folder.path());
        let csv = probe_history_csv(&paths, Vec3::new(0.5, 0.5, 0.)).unwrap();
        let rows = csv.lines().collect::<Vec<_>>();
        assert_eq!(rows.len(), 3);
        assert_eq!(rows[1].split(',').count(), 11);
        assert_eq!(rows[2].split(',').count(), 11);
        assert!(rows[1].contains("2.000000000000000e0"));
        assert!(rows[2].ends_with(",,,,,,"));
    }
    #[test]
    fn physical_time_statistics_weight_irregular_intervals() {
        let folder = tempfile::tempdir().unwrap();
        let paths = archives(
            &[
                snapshot(Some(0.), 0.),
                snapshot(Some(1.), 1.),
                snapshot(Some(3.), 3.),
            ],
            folder.path(),
        );
        let csv = temporal_csv(&paths).unwrap();
        let values = csv.lines().nth(1).unwrap().split(',').collect::<Vec<_>>();
        assert!((values[7].parse::<f64>().unwrap() - 1.5).abs() < 1e-12);
        assert!((values[11].parse::<f64>().unwrap() - 1.25_f64.sqrt()).abs() < 1e-12);
        assert!((values[18].parse::<f64>().unwrap() - 2.5).abs() < 1e-12);
    }
    #[test]
    fn statistics_reject_iterations_repeated_time_and_geometry_changes() {
        let folder = tempfile::tempdir().unwrap();
        let mut frames = vec![snapshot(None, 1.), snapshot(None, 2.)];
        assert!(temporal_csv(&archives(&frames, folder.path())).is_err());
        frames[0].physical_time = Some(1.);
        frames[1].physical_time = Some(1.);
        assert!(temporal_csv(&archives(&frames, folder.path())).is_err());
        frames[1].physical_time = Some(2.);
        frames[1].samples[0].geometry.point.x = 0.6;
        assert!(temporal_csv(&archives(&frames, folder.path())).is_err());
    }
    #[test]
    fn colour_range_and_geometric_statistics_use_raw_samples() {
        let folder = tempfile::tempdir().unwrap();
        let frame = snapshot(None, 2.);
        let paths = archives(&[frame.clone(), snapshot(None, 4.)], folder.path());
        assert_eq!(recorded_range(&paths, 1).unwrap(), [2., 4.]);
        let csv = section_statistics_csv(&frame, 1).unwrap();
        assert!(csv.contains("section_area_m2,1.000000000000000e0"));
        assert!(csv.contains("area_mean,2.000000000000000e0"));
        let live =
            section_interactive_svg(&frame, &PlaneDisplay::default(), &SectionCamera::default());
        assert!(live.contains("section-clip"));
    }
}

/// Cell-sampled history at a fixed world point. Empty values mark missing coverage.
pub fn probe_history_csv(paths: &[PathBuf], point: crate::Vec3) -> Result<String, String> {
    let mut out = String::from(
        "iteration,time_s,x_m,y_m,z_m,cell,ux_m_s,uy_m_s,uz_m_s,speed_m_s,p_over_rho_m2_s2\n",
    );
    let mut hits = 0;
    for path in paths {
        let snapshot: PlaneSnapshot =
            serde_json::from_slice(&std::fs::read(path).map_err(|e| e.to_string())?)
                .map_err(|e| e.to_string())?;
        write!(
            out,
            "{},{},{:.15e},{:.15e},{:.15e}",
            snapshot.iteration,
            snapshot
                .physical_time
                .map(|t| t.to_string())
                .unwrap_or_default(),
            point.x,
            point.y,
            point.z
        )
        .map_err(|e| e.to_string())?;
        if let Some(sample) = super::plane_monitor::probe_world(&snapshot, point) {
            writeln!(
                out,
                ",{},{:.15e},{:.15e},{:.15e},{:.15e},{:.15e}",
                sample.geometry.cell,
                sample.velocity.x,
                sample.velocity.y,
                sample.velocity.z,
                sample.velocity.norm(),
                sample.pressure
            )
            .map_err(|e| e.to_string())?;
            hits += 1;
        } else {
            out.push_str(",,,,,,\n");
        }
    }
    if hits == 0 {
        return Err("The selected point is outside all recorded sections".into());
    }
    Ok(out)
}

/// Uniform spatial samples on an arbitrary line in plane u/v coordinates.
/// Owning-cell fields only: missing coverage remains an empty CSV value.
struct ProfilePoint<'a> {
    distance: f64,
    uv: [f64; 2],
    point: crate::Vec3,
    sample: Option<&'a super::plane_monitor::PlaneSample>,
}
fn profile_points(
    snapshot: &PlaneSnapshot,
    start: [f64; 2],
    end: [f64; 2],
    count: usize,
    field: usize,
) -> Result<Vec<ProfilePoint<'_>>, String> {
    snapshot.plane.validate()?;
    if !(2..=2000).contains(&count) || field > 5 || start.iter().chain(&end).any(|x| !x.is_finite())
    {
        return Err(
            "Profile requires 2..2000 samples, finite endpoints and a supported section field"
                .into(),
        );
    }
    let length = (end[0] - start[0]).hypot(end[1] - start[1]);
    if length <= 0.0 || !length.is_finite() {
        return Err("Profile endpoints must be distinct".into());
    }
    Ok((0..count)
        .map(|index| {
            let t = index as f64 / (count - 1) as f64;
            let uv = [
                start[0] * (1.0 - t) + end[0] * t,
                start[1] * (1.0 - t) + end[1] * t,
            ];
            let point = snapshot.plane.origin + snapshot.plane.u * uv[0] + snapshot.plane.v * uv[1];
            ProfilePoint {
                distance: length * t,
                uv,
                point,
                sample: super::plane_monitor::probe_world(snapshot, point),
            }
        })
        .collect())
}

pub fn profile_csv(
    snapshot: &PlaneSnapshot,
    start: [f64; 2],
    end: [f64; 2],
    count: usize,
    field: usize,
) -> Result<String, String> {
    let points = profile_points(snapshot, start, end, count, field)?;
    let mut csv=String::from("iteration,time_s,distance_m,u_m,v_m,x_m,y_m,z_m,cell,ux_m_s,uy_m_s,uz_m_s,speed_m_s,p_over_rho_m2_s2,selected_field_value\n");
    for row in points {
        let [u, v] = row.uv;
        let point = row.point;
        write!(
            csv,
            "{},{},{:.15e},{:.15e},{:.15e},{:.15e},{:.15e},{:.15e}",
            snapshot.iteration,
            snapshot
                .physical_time
                .map(|t| t.to_string())
                .unwrap_or_default(),
            row.distance,
            u,
            v,
            point.x,
            point.y,
            point.z
        )
        .map_err(|e| e.to_string())?;
        if let Some(sample) = row.sample {
            writeln!(
                csv,
                ",{},{:.15e},{:.15e},{:.15e},{:.15e},{:.15e},{:.15e}",
                sample.geometry.cell,
                sample.velocity.x,
                sample.velocity.y,
                sample.velocity.z,
                sample.velocity.norm(),
                sample.pressure,
                super::plane_monitor::sample_scalar(snapshot, sample, field)
            )
            .map_err(|e| e.to_string())?;
        } else {
            csv.push_str(",,,,,,,\n");
        }
    }
    Ok(csv)
}

pub fn centreline_csv(
    snapshot: &PlaneSnapshot,
    vertical: bool,
    count: usize,
    field: usize,
) -> Result<String, String> {
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for sample in &snapshot.samples {
        for p in &sample.geometry.polygon {
            let d = *p - snapshot.plane.origin;
            let q = [d.dot(snapshot.plane.u), d.dot(snapshot.plane.v)];
            for k in 0..2 {
                lo[k] = lo[k].min(q[k]);
                hi[k] = hi[k].max(q[k]);
            }
        }
    }
    let mid = [0.5 * lo[0] + 0.5 * hi[0], 0.5 * lo[1] + 0.5 * hi[1]];
    let (start, end) = if vertical {
        ([mid[0], lo[1]], [mid[0], hi[1]])
    } else {
        ([lo[0], mid[1]], [hi[0], mid[1]])
    };
    profile_csv(snapshot, start, end, count, field)
}

/// Scientific profile chart, sharing the exact samples and gaps used by CSV.
pub fn profile_svg(
    snapshot: &PlaneSnapshot,
    start: [f64; 2],
    end: [f64; 2],
    count: usize,
    field: usize,
) -> Result<String, String> {
    let points = profile_points(snapshot, start, end, count, field)?;
    let scalar = |row: &ProfilePoint<'_>| {
        row.sample
            .map(|s| super::plane_monitor::sample_scalar(snapshot, s, field))
            .filter(|v| v.is_finite())
    };
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for row in &points {
        if let Some(v) = scalar(row) {
            lo = lo.min(v);
            hi = hi.max(v);
        }
    }
    if !lo.is_finite() || !hi.is_finite() {
        return Err("No finite profile samples inside this section".into());
    }
    if lo == hi {
        let padding = lo.abs().max(1.0) * 1e-6;
        lo -= padding;
        hi += padding;
    }
    let length = points.last().map(|p| p.distance).unwrap_or(1.0);
    let escape = |s: &str| {
        s.replace('&', "&amp;")
            .replace('<', "&lt;")
            .replace('>', "&gt;")
    };
    let label = match field {
        1 => "Kinematic pressure p/rho (m2/s2)",
        2 => "Velocity X (m/s)",
        3 => "Velocity Y (m/s)",
        4 => "Velocity Z (m/s)",
        5 => "Normal velocity (m/s)",
        _ => "Speed (m/s)",
    };
    let clock = snapshot
        .physical_time
        .map(|t| format!("t = {t:.6e} s"))
        .unwrap_or_else(|| format!("iteration {}", snapshot.iteration));
    let mut svg = format!(
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="650"><rect width="100%" height="100%" fill="white"/><g font-family="DejaVu Sans,sans-serif" fill="#18212b" font-size="13"><text x="110" y="34" font-size="20">{} · {}</text><text x="110" y="60">u/v: ({:.4e}, {:.4e}) → ({:.4e}, {:.4e}) m</text>"##,
        escape(&snapshot.plane.name),
        clock,
        start[0],
        start[1],
        end[0],
        end[1]
    );
    for i in 0..=5 {
        let t = i as f64 / 5.;
        let x = 110. + 830. * t;
        let y = 550. - 450. * t;
        let _ = write!(
            svg,
            r##"<path d="M{x} 100 V550 M110 {y} H940" stroke="#e3e7ec"/><text x="{x}" y="574" text-anchor="middle">{:.4e}</text><text x="100" y="{}" text-anchor="end">{:.4e}</text>"##,
            length * t,
            y + 4.,
            lo * (1. - t) + hi * t
        );
    }
    let mut path = String::new();
    let mut drawing = false;
    for row in &points {
        if let Some(v) = scalar(row) {
            let x = 110. + 830. * row.distance / length;
            let y = 550. - 450. * ((v - lo) / (hi - lo));
            let _ = write!(path, "{} {x:.4} {y:.4} ", if drawing { "L" } else { "M" });
            drawing = true;
        } else {
            drawing = false;
        }
    }
    let _ = write!(
        svg,
        r##"<path d="{path}" fill="none" stroke="#1672b8" stroke-width="2"/><path d="M110 100 V550 H940" fill="none" stroke="#18212b"/><text x="525" y="606" text-anchor="middle">Distance along line (m)</text><text transform="translate(23 325) rotate(-90)" text-anchor="middle">{label}</text><text x="110" y="632">Cell samples; lines connect samples for display only. Missing fluid coverage remains a gap.</text></g></svg>"##
    );
    Ok(svg)
}
