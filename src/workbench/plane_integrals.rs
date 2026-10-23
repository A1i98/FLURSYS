//! Section-area monitors from sampled cell fields. These fluxes are not mesh face fluxes.
use super::{plane_monitor::PlaneSnapshot, series_plot::Series};
#[derive(Clone, Debug)]
pub struct Sample {
    pub iteration: usize,
    pub physical_time: Option<f64>,
    pub area: f64,
    pub mean_pressure: f64,
    pub mean_speed: f64,
    pub volume_flux: f64,
}
pub fn sample(snapshot: &PlaneSnapshot) -> Result<Sample, String> {
    snapshot.plane.validate()?;
    if snapshot.physical_time.is_some_and(|t| !t.is_finite()) {
        return Err("Invalid physical monitor time".into());
    }
    let (mut area, mut pressure, mut speed, mut flux) = (0., 0., 0., 0.);
    let normal = snapshot.plane.normal();
    for s in &snapshot.samples {
        let p = &s.geometry.polygon;
        if p.len() < 3 {
            continue;
        }
        let mut vector = crate::Vec3::new(0., 0., 0.);
        for i in 1..p.len() - 1 {
            vector += (p[i] - p[0]).cross(p[i + 1] - p[0]) * 0.5;
        }
        let a = vector.dot(normal).abs();
        let u = s.velocity.norm();
        if !a.is_finite() || !u.is_finite() || !s.pressure.is_finite() {
            return Err("Invalid section monitor sample".into());
        }
        area += a;
        pressure += a * s.pressure;
        speed += a * u;
        flux += a * s.velocity.dot(normal);
    }
    if area <= 0. || ![area, pressure, speed, flux].iter().all(|v| v.is_finite()) {
        return Err("No finite section monitor area/integrals".into());
    }
    Ok(Sample {
        iteration: snapshot.iteration,
        physical_time: snapshot.physical_time,
        area,
        mean_pressure: pressure / area,
        mean_speed: speed / area,
        volume_flux: flux,
    })
}
pub fn series(
    samples: &[Sample],
    name: &str,
    kind: usize,
) -> Result<(Series, &'static str, &'static str), String> {
    if samples.is_empty() {
        return Err("No section samples yet; enable plane recording before solving".into());
    }
    let physical = samples[0].physical_time.is_some();
    let mut previous = None;
    let mut points = Vec::with_capacity(samples.len());
    for sample in samples {
        if physical != sample.physical_time.is_some() {
            return Err("Cannot mix physical time and steady iterations".into());
        }
        let x = sample.physical_time.unwrap_or(sample.iteration as f64);
        if !x.is_finite() || previous.is_some_and(|p| x <= p) {
            return Err("Monitor coordinates must increase strictly".into());
        }
        let value = match kind {
            1 => sample.mean_pressure,
            2 => sample.volume_flux,
            _ => sample.mean_speed,
        };
        points.push((x, Some(value)));
        previous = Some(x);
    }
    Ok((
        Series {
            name: name.into(),
            points,
        },
        if physical {
            "Physical time (s)"
        } else {
            "Steady iteration (not physical time)"
        },
        match kind {
            1 => "Area mean p/rho (m2/s2)",
            2 => "Estimated section volume flux (m3/s)",
            _ => "Area mean speed (m/s)",
        },
    ))
}
pub fn csv(samples: &[Sample]) -> String {
    use std::fmt::Write;
    let mut out=String::from("iteration,time_s,section_area_m2,area_mean_p_over_rho_m2_s2,area_mean_speed_m_s,estimated_volume_flux_m3_s\n");
    for s in samples {
        let _ = writeln!(
            out,
            "{},{},{:.17e},{:.17e},{:.17e},{:.17e}",
            s.iteration,
            s.physical_time.map(|v| v.to_string()).unwrap_or_default(),
            s.area,
            s.mean_pressure,
            s.mean_speed,
            s.volume_flux
        );
    }
    out
}
