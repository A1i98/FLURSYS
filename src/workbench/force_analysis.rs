//! Reference-based loads and explicit straight-wall reattachment diagnostics.
use super::{series_plot::Series, wall_reports::*};
use crate::Vec3;
use serde::{Deserialize, Serialize};
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    /// Empty selects all wall patches. Otherwise names must match exactly.
    pub patches: Vec<String>,
    pub frequency_start_time: f64,
    pub speed: f64,
    pub measure: f64,
    pub length: f64,
    pub center: Vec3,
    pub drag_direction: Vec3,
    pub lift_direction: Vec3,
    pub reattachment: bool,
    pub wall_start: Vec3,
    pub wall_direction: Vec3,
    pub wall_tolerance: f64,
}
impl Default for Settings {
    fn default() -> Self {
        Self {
            patches: vec![],
            frequency_start_time: 0.0,
            speed: 1.0,
            measure: 1.0,
            length: 1.0,
            center: Vec3::ZERO,
            drag_direction: Vec3::new(1., 0., 0.),
            lift_direction: Vec3::new(0., 1., 0.),
            reattachment: false,
            wall_start: Vec3::ZERO,
            wall_direction: Vec3::new(1., 0., 0.),
            wall_tolerance: 1e-6,
        }
    }
}
fn unit(v: Vec3) -> Result<Vec3, String> {
    let n = v.norm();
    if !n.is_finite() || n <= 0. {
        Err("Load directions must be finite and nonzero".into())
    } else {
        Ok(v / n)
    }
}
impl Settings {
    pub fn validate(&self) -> Result<(), String> {
        if !self.frequency_start_time.is_finite() || self.frequency_start_time < 0.0 {
            return Err("Frequency start time must be finite nonnegative".into());
        }
        for x in [self.speed, self.measure, self.length, self.wall_tolerance] {
            if !x.is_finite() || x <= 0. {
                return Err(
                    "Reference speed, measure, length and wall tolerance must be finite positive"
                        .into(),
                );
            }
        }
        unit(self.drag_direction)?;
        unit(self.lift_direction)?;
        unit(self.wall_direction)?;
        if unit(self.drag_direction)?
            .dot(unit(self.lift_direction)?)
            .abs()
            > 1e-6
        {
            return Err("Drag and lift directions must be perpendicular".into());
        }
        for v in [self.center, self.wall_start] {
            if [v.x, v.y, v.z].iter().any(|x| !x.is_finite()) {
                return Err("Reference points must be finite".into());
            }
        }
        Ok(())
    }
    pub fn evaluate(&self, reports: &[WallPatchReport], density: f64) -> Result<Loads, String> {
        self.validate()?;
        for patch in &self.patches {
            if !reports.iter().any(|r| &r.patch == patch) {
                return Err(format!("No wall report for selected patch {patch:?}"));
            }
        }
        let mut force = Vec3::ZERO;
        let mut pressure = Vec3::ZERO;
        let mut viscous = Vec3::ZERO;
        let mut moment = Vec3::ZERO;
        let mut count = 0;
        let reference = ForceReference {
            density,
            speed: self.speed,
            measure: self.measure,
            length: self.length,
            center: self.center,
        };
        for r in reports
            .iter()
            .filter(|r| self.patches.is_empty() || self.patches.contains(&r.patch))
        {
            if r.invalid_faces > 0 {
                return Err(format!(
                    "Patch {} has {} invalid load faces",
                    r.patch, r.invalid_faces
                ));
            }
            let c = r.coefficients(reference)?;
            force += c.total;
            pressure += c.pressure;
            viscous += c.viscous;
            moment += c.moment;
            count += 1;
        }
        if count == 0 {
            return Err("No selected wall patches".into());
        }
        let loads = Loads {
            drag: force.dot(unit(self.drag_direction)?),
            lift: force.dot(unit(self.lift_direction)?),
            pressure,
            viscous,
            moment,
        };
        if [loads.drag, loads.lift, moment.x, moment.y, moment.z]
            .iter()
            .any(|v| !v.is_finite())
        {
            return Err("Load integration overflow".into());
        }
        Ok(loads)
    }
    pub fn summary(&self, reports: &[WallPatchReport], density: f64) -> String {
        match self.evaluate(reports, density) {
            Err(e) => format!("LOAD ANALYSIS: {e}"),
            Ok(l) => {
                let mut text=format!("REFERENCE LOADS · selected patches: {}\nUref={} · Aref={} (2D: length) · Lref={}\nCd={:.8e} · Cl={:.8e}\nCm=({:.6e}, {:.6e}, {:.6e})\nPressure C=({:.5e}, {:.5e}, {:.5e})\nViscous C=({:.5e}, {:.5e}, {:.5e})\n",if self.patches.is_empty(){"all walls".into()}else{self.patches.join(", ")},self.speed,self.measure,self.length,l.drag,l.lift,l.moment.x,l.moment.y,l.moment.z,l.pressure.x,l.pressure.y,l.pressure.z,l.viscous.x,l.viscous.y,l.viscous.z);
                if self.reattachment {
                    for r in reports
                        .iter()
                        .filter(|r| self.patches.is_empty() || self.patches.contains(&r.patch))
                    {
                        match reattachment_distances(
                            r,
                            self.wall_start,
                            self.wall_direction,
                            self.wall_tolerance,
                        ) {
                            Ok(xs) => text.push_str(&format!(
                                "{} · reattachment distances (m): {:?}\n",
                                r.patch, xs
                            )),
                            Err(e) => text.push_str(&format!("{}: {e}\n", r.patch)),
                        }
                    }
                }
                text
            }
        }
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Loads {
    pub drag: f64,
    pub lift: f64,
    pub pressure: Vec3,
    pub viscous: Vec3,
    pub moment: Vec3,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Sample {
    pub index: usize,
    pub time: Option<f64>,
    pub density: f64,
    pub reports: Vec<WallPatchReport>,
}
pub fn series(samples: &[Sample], settings: &Settings) -> Result<Vec<Series>, String> {
    settings.validate()?;
    let timed = samples.first().is_some_and(|s| s.time.is_some());
    if samples.iter().any(|s| s.time.is_some() != timed) {
        return Err("Cannot mix physical-time and iteration histories".into());
    }
    let mut curves: Vec<_> = ["Cd", "Cl", "Cm X", "Cm Y", "Cm Z"]
        .iter()
        .map(|s| Series {
            name: s.to_string(),
            points: vec![],
        })
        .collect();
    for s in samples {
        let x = s.time.unwrap_or(s.index as f64);
        let l = settings.evaluate(&s.reports, s.density)?;
        for (c, y) in curves
            .iter_mut()
            .zip([l.drag, l.lift, l.moment.x, l.moment.y, l.moment.z])
        {
            c.points.push((x, Some(y)));
        }
    }
    Ok(curves)
}
pub fn csv(samples: &[Sample], settings: &Settings) -> Result<String, String> {
    use std::fmt::Write;
    let mut text = "index,physical_time_s,Cd,Cl,Cmx,Cmy,Cmz\n".to_string();
    for s in samples {
        let l = settings.evaluate(&s.reports, s.density)?;
        writeln!(
            text,
            "{},{},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e}",
            s.index,
            s.time.map(|t| format!("{t:.16e}")).unwrap_or_default(),
            l.drag,
            l.lift,
            l.moment.x,
            l.moment.y,
            l.moment.z
        )
        .map_err(|e| e.to_string())?;
    }
    Ok(text)
}
/// Append only completed/sampled loads; raw loads permit changing references later.
pub fn append(path: &std::path::Path, sample: &Sample) -> Result<(), String> {
    use std::io::Write;
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    let mut file = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)
        .map_err(|e| e.to_string())?;
    serde_json::to_writer(&mut file, sample).map_err(|e| e.to_string())?;
    file.write_all(b"\n").map_err(|e| e.to_string())
}
pub fn load(path: &std::path::Path) -> Result<Vec<Sample>, String> {
    use std::io::BufRead;
    let file = std::fs::File::open(path).map_err(|e| e.to_string())?;
    if file.metadata().map_err(|e| e.to_string())?.len() > 128 * 1024 * 1024 {
        return Err("Force history exceeds 128 MiB; use the headless streaming reader".into());
    }
    let mut samples: Vec<Sample> = Vec::new();
    for line in std::io::BufReader::new(file).lines() {
        let sample: Sample =
            serde_json::from_str(&line.map_err(|e| e.to_string())?).map_err(|e| e.to_string())?;
        if let Some(last) = samples
            .last_mut()
            .filter(|s| s.index == sample.index && s.time == sample.time)
        {
            *last = sample;
        } else {
            samples.push(sample);
        }
    }
    Ok(samples)
}

/// Stream raw archive rows without retaining the full history in memory.
/// A final sample can replace the preceding sample at the same index/time.
pub fn visit_samples(
    path: &std::path::Path,
    mut visit: impl FnMut(Sample) -> Result<(), String>,
) -> Result<(), String> {
    use std::io::BufRead;
    for line in
        std::io::BufReader::new(std::fs::File::open(path).map_err(|e| e.to_string())?).lines()
    {
        visit(serde_json::from_str(&line.map_err(|e| e.to_string())?).map_err(|e| e.to_string())?)?;
    }
    Ok(())
}

/// Positive lift zero-crossing estimate after an explicit physical start time.
/// This reports an estimate, not a test that periodic shedding has developed.
pub fn shedding_frequency(
    samples: &[Sample],
    settings: &Settings,
    start_time: f64,
) -> Result<Option<(f64, f64)>, String> {
    if !start_time.is_finite() || start_time < 0. {
        return Err("Frequency start time must be finite nonnegative".into());
    }
    let mut points = Vec::new();
    for s in samples {
        let t = s
            .time
            .ok_or("Shedding frequency requires physical-time samples")?;
        if t >= start_time {
            points.push((t, settings.evaluate(&s.reports, s.density)?.lift));
        }
    }
    if points.windows(2).any(|w| w[1].0 <= w[0].0) {
        return Err("Frequency requires strictly increasing physical times".into());
    }
    if points.len() < 4 {
        return Ok(None);
    }
    let mean = points.iter().map(|p| p.1).sum::<f64>() / points.len() as f64;
    let mut crossings = Vec::new();
    for w in points.windows(2) {
        let (a, b) = (w[0], w[1]);
        if a.1 <= mean && b.1 > mean {
            crossings.push(a.0 + (b.0 - a.0) * (mean - a.1) / (b.1 - a.1));
        }
    }
    if crossings.len() < 3 {
        return Ok(None);
    }
    let period = (crossings.last().unwrap() - crossings[0]) / (crossings.len() - 1) as f64;
    let f = 1.0 / period;
    let st = f * settings.length / settings.speed;
    if !f.is_finite() || !st.is_finite() {
        return Err("Frequency estimate overflow".into());
    }
    Ok(Some((f, st)))
}
