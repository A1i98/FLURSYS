//! Cell-centred flow samples on arbitrary CAD planes; no time interpolation.
use crate::{MeshDimension, UnstructuredMesh, Vec3};
use serde::{Deserialize, Serialize};
use std::fmt::Write as _;
use std::{io::Write, path::Path};
/// Presentation settings do not change the recorded physical fields.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PlaneDisplay {
    /// 0 speed, 1 kinematic pressure, 2..4 global velocity, 5 normal velocity.
    pub field: usize,
    pub plot: usize,
    pub palette: usize,
    pub vector_scale: f64,
    pub vector_density: usize,
    pub levels: usize,
    pub streamlines: bool,
    pub vectors: bool,
    pub edges: bool,
    pub normalized_vectors: bool,
    pub normal_markers: bool,
    pub range: Option<[f64; 2]>,
}
impl Default for PlaneDisplay {
    fn default() -> Self {
        Self {
            field: 0,
            plot: 0,
            palette: 1,
            vector_scale: 1.0,
            vector_density: 600,
            levels: 12,
            streamlines: false,
            vectors: true,
            edges: false,
            normalized_vectors: false,
            normal_markers: false,
            range: None,
        }
    }
}
impl PlaneDisplay {
    pub fn validate(&self) -> Result<(), String> {
        if self.palette > 3
            || !self.vector_scale.is_finite()
            || self.vector_scale <= 0.0
            || self.vector_density == 0
            || self.vector_density > 2000
            || self.plot > 3
            || !(3..=32).contains(&self.levels)
            || self.field > 5
            || self
                .range
                .is_some_and(|[a, b]| !a.is_finite() || !b.is_finite() || b <= a)
        {
            return Err("Select a supported field and a finite increasing range".into());
        }
        Ok(())
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FlowPlaneMonitor {
    pub id: String,
    pub name: String,
    pub origin: Vec3,
    pub u: Vec3,
    pub v: Vec3,
    pub enabled: bool,
    pub every: usize,
    #[serde(default)]
    pub display: PlaneDisplay,
}
impl FlowPlaneMonitor {
    pub fn validate(&self) -> Result<(), String> {
        self.display.validate()?;
        if self.id.is_empty() || self.every == 0 || self.every > 1_000_000 {
            return Err("Plane needs an ID and a sampling interval between 1 and 1000000".into());
        }
        if [self.origin, self.u, self.v]
            .iter()
            .any(|v| ![v.x, v.y, v.z].iter().all(|x| x.is_finite()))
            || (self.u.norm() - 1.).abs() > 1e-7
            || (self.v.norm() - 1.).abs() > 1e-7
            || self.u.dot(self.v).abs() > 1e-7
        {
            return Err("Plane axes must be finite, orthonormal vectors".into());
        }
        Ok(())
    }
    pub fn normal(&self) -> Vec3 {
        self.u.cross(self.v)
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PlaneCell {
    pub cell: usize,
    pub point: Vec3,
    pub polygon: Vec<Vec3>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PlaneSample {
    pub geometry: PlaneCell,
    pub velocity: Vec3,
    pub pressure: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PlaneSnapshot {
    #[serde(default)]
    pub physical_time: Option<f64>,
    pub plane: FlowPlaneMonitor,
    pub iteration: usize,
    pub samples: Vec<PlaneSample>,
    pub path: Option<std::path::PathBuf>,
}
pub fn intersect(
    mesh: &UnstructuredMesh,
    plane: &FlowPlaneMonitor,
) -> Result<Vec<PlaneCell>, String> {
    plane.validate()?;
    if mesh.dimension() != MeshDimension::ThreeD {
        return Err("Flow-plane monitors require a 3D volume mesh".into());
    }
    let normal = plane.normal();
    let mut span = 0_f64;
    let mut location = plane.origin.norm();
    for p in mesh.points() {
        span = span.max((p.position - plane.origin).norm());
        location = location.max(p.position.norm());
    }
    let eps = (span * 1e-10).max(f64::EPSILON * location.max(1.) * 32.);
    let mut result = Vec::new();
    for (index, cell) in mesh.cells().iter().enumerate() {
        // If the plane coincides with an internal face, take its owner once.
        // Both adjacent cell values must not be drawn on the same polygon.
        if cell.faces.iter().any(|id| {
            let f = &mesh.faces()[*id];
            f.neighbour.is_some()
                && f.owner != index
                && f.vertices.iter().all(|v| {
                    (mesh.points()[*v].position - plane.origin)
                        .dot(normal)
                        .abs()
                        <= eps
                })
        }) {
            continue;
        }
        let mut points: Vec<Vec3> = Vec::new();
        for face in &cell.faces {
            let vertices = &mesh.faces()[*face].vertices;
            for i in 0..vertices.len() {
                let a = mesh.points()[vertices[i]].position;
                let b = mesh.points()[vertices[(i + 1) % vertices.len()]].position;
                let da = (a - plane.origin).dot(normal);
                let db = (b - plane.origin).dot(normal);
                let mut add = |p: Vec3| {
                    if !points.iter().any(|q| (*q - p).norm() <= eps) {
                        points.push(p);
                    }
                };
                if da.abs() <= eps {
                    add(a);
                }
                if (da < -eps && db > eps) || (db < -eps && da > eps) {
                    add(a + (b - a) * (da / (da - db)));
                }
            }
        }
        if points.len() < 3 {
            continue;
        }
        let center = points.iter().fold(Vec3::ZERO, |a, b| a + *b) * (1. / points.len() as f64);
        points.sort_by(|a, b| {
            let a = *a - center;
            let b = *b - center;
            a.dot(plane.v)
                .atan2(a.dot(plane.u))
                .total_cmp(&b.dot(plane.v).atan2(b.dot(plane.u)))
        });
        let area = (1..points.len() - 1)
            .map(|i| {
                (points[i] - points[0])
                    .cross(points[i + 1] - points[0])
                    .norm()
                    * 0.5
            })
            .sum::<f64>();
        if area > eps * eps {
            result.push(PlaneCell {
                cell: index,
                point: center,
                polygon: points,
            });
        }
    }
    Ok(result)
}
pub fn sample(
    plane: &FlowPlaneMonitor,
    cells: &[PlaneCell],
    iteration: usize,
    velocity: &[Vec3],
    pressure: &[f64],
) -> Result<PlaneSnapshot, String> {
    let samples = cells
        .iter()
        .map(|cell| {
            let velocity = *velocity
                .get(cell.cell)
                .ok_or("Plane sample velocity index out of range")?;
            let pressure = *pressure
                .get(cell.cell)
                .ok_or("Plane sample pressure index out of range")?;
            if ![velocity.x, velocity.y, velocity.z, pressure]
                .iter()
                .all(|v| v.is_finite())
            {
                return Err("Non-finite field on monitor plane".into());
            }
            Ok(PlaneSample {
                geometry: cell.clone(),
                velocity,
                pressure,
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    Ok(PlaneSnapshot {
        physical_time: None,
        plane: plane.clone(),
        iteration,
        samples,
        path: None,
    })
}
/// Scalar used by the section renderer and colour-range calculations.
pub fn sample_scalar(snapshot: &PlaneSnapshot, sample: &PlaneSample, field: usize) -> f64 {
    match field {
        1 => sample.pressure,
        2 => sample.velocity.x,
        3 => sample.velocity.y,
        4 => sample.velocity.z,
        5 => sample.velocity.dot(snapshot.plane.normal()),
        _ => sample.velocity.norm(),
    }
}

/// Geometric section statistics from piecewise-constant cell samples.
/// Flux here is an estimate from U·n, not the solver's conservative face flux.
pub fn section_statistics_csv(snapshot: &PlaneSnapshot, field: usize) -> Result<String, String> {
    snapshot.plane.validate()?;
    if field > 5 {
        return Err("Unsupported section field".into());
    }
    let mut area = 0.0;
    let mut integral = 0.0;
    let mut flux = 0.0;
    let mut positive_flux = 0.0;
    let mut negative_flux = 0.0;
    let mut flux_weighted = 0.0;
    let mut minimum = f64::INFINITY;
    let mut maximum = f64::NEG_INFINITY;
    let normal = snapshot.plane.normal();
    let mut used = 0;
    for sample in &snapshot.samples {
        let vertices = &sample.geometry.polygon;
        if vertices.len() < 3 {
            continue;
        }
        let origin = vertices[0];
        let mut area_vector = Vec3::new(0.0, 0.0, 0.0);
        for i in 1..vertices.len() - 1 {
            area_vector =
                area_vector + (vertices[i] - origin).cross(vertices[i + 1] - origin) * 0.5;
        }
        let measure = area_vector.dot(normal).abs();
        let value = sample_scalar(snapshot, sample, field);
        if !measure.is_finite() || measure <= 0.0 || !value.is_finite() {
            continue;
        }
        area += measure;
        integral += value * measure;
        let cell_flux = sample.velocity.dot(normal) * measure;
        if !cell_flux.is_finite() {
            return Err("Non-finite section velocity flux".into());
        }
        flux += cell_flux;
        positive_flux += cell_flux.max(0.0);
        negative_flux += cell_flux.min(0.0);
        flux_weighted += value * cell_flux.abs();
        minimum = minimum.min(value);
        maximum = maximum.max(value);
        used += 1;
    }
    if area <= 0.0
        || ![
            area,
            integral,
            flux,
            positive_flux,
            negative_flux,
            flux_weighted,
        ]
        .iter()
        .all(|v| v.is_finite())
    {
        return Err("No finite section statistics to integrate".into());
    }
    let mut csv=format!("metric,value\niteration,{}\nphysical_time_s,{}\nfield_index,{}\nvalid_polygons,{}\nsection_area_m2,{:.15e}\nminimum,{:.15e}\nmaximum,{:.15e}\narea_mean,{:.15e}\narea_integral,{:.15e}\nestimated_volume_flux_m3_s,{:.15e}\n", snapshot.iteration, snapshot.physical_time.map(|t|t.to_string()).unwrap_or_default(),field,used,area,minimum,maximum,integral/area,integral,flux);
    let absolute_flux = positive_flux - negative_flux;
    let weighted = if absolute_flux > 0.0 {
        format!("{:.15e}", flux_weighted / absolute_flux)
    } else {
        String::new()
    };
    let _=write!(csv,"positive_normal_volume_flux_m3_s,{positive_flux:.15e}\nnegative_normal_volume_flux_m3_s,{negative_flux:.15e}\nabsolute_volume_flux_m3_s,{absolute_flux:.15e}\nabs_flux_weighted_scalar,{weighted}\n");
    let reverse_fraction = if absolute_flux > 0.0 {
        format!("{:.15e}", -negative_flux / absolute_flux)
    } else {
        String::new()
    };
    let _ = writeln!(
        csv,
        "negative_normal_absolute_flux_fraction,{reverse_fraction}"
    );
    Ok(csv)
}

pub fn write_csv(snapshot: &PlaneSnapshot, folder: &Path) -> Result<std::path::PathBuf, String> {
    let _timer = crate::compute::StageTimer::new(crate::compute::Stage::MonitorOutput);
    let id = snapshot
        .plane
        .id
        .bytes()
        .map(|b| format!("{b:02x}"))
        .collect::<String>();
    let folder = folder.join(id);
    std::fs::create_dir_all(&folder).map_err(|e| e.to_string())?;
    let metadata = folder.join("plane.json");
    if !metadata.exists() {
        let mut f = tempfile::NamedTempFile::new_in(&folder).map_err(|e| e.to_string())?;
        serde_json::to_writer_pretty(&mut f, &snapshot.plane).map_err(|e| e.to_string())?;
        f.persist(&metadata).map_err(|e| e.to_string())?;
    }
    let path = folder.join(format!("iteration-{:08}.csv", snapshot.iteration));
    let mut file = tempfile::NamedTempFile::new_in(&folder).map_err(|e| e.to_string())?;
    file.write_all(csv_text(snapshot).as_bytes())
        .map_err(|e| e.to_string())?;
    file.flush().map_err(|e| e.to_string())?;
    file.persist(&path).map_err(|e| e.to_string())?;
    let mut archive = tempfile::NamedTempFile::new_in(&folder).map_err(|e| e.to_string())?;
    serde_json::to_writer(&mut archive, snapshot).map_err(|e| e.to_string())?;
    archive
        .persist(path.with_extension("json"))
        .map_err(|e| e.to_string())?;
    let image = path.with_extension("svg");
    let mut vector = tempfile::NamedTempFile::new_in(&folder).map_err(|e| e.to_string())?;
    vector
        .write_all(vector_svg(snapshot).as_bytes())
        .map_err(|e| e.to_string())?;
    vector.flush().map_err(|e| e.to_string())?;
    vector.persist(image).map_err(|e| e.to_string())?;
    Ok(path)
}

fn escaped(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}
pub fn vector_svg(snapshot: &PlaneSnapshot) -> String {
    section_svg(snapshot, &snapshot.plane.display)
}

/// Filled cell sections: no smoothing or invented point interpolation.
/// Camera in the section's u/v coordinates. Pan is a fraction of viewport size.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SectionCamera {
    pub zoom: f64,
    pub pan: [f64; 2],
    pub viewport: [f64; 2],
}
impl Default for SectionCamera {
    fn default() -> Self {
        Self {
            zoom: 1.0,
            pan: [0.0; 2],
            viewport: [1000.0, 650.0],
        }
    }
}
impl SectionCamera {
    fn transform(&self, snapshot: &PlaneSnapshot) -> ([f64; 2], [f64; 2], f64) {
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for sample in &snapshot.samples {
            for point in &sample.geometry.polygon {
                let d = *point - snapshot.plane.origin;
                let q = [d.dot(snapshot.plane.u), d.dot(snapshot.plane.v)];
                for k in 0..2 {
                    lo[k] = lo[k].min(q[k]);
                    hi[k] = hi[k].max(q[k]);
                }
            }
        }
        let [w, h] = self.viewport;
        let scale = ((w - 114.0).max(10.0) / (hi[0] - lo[0]).max(1e-15))
            .min((h - 48.0).max(10.0) / (hi[1] - lo[1]).max(1e-15))
            * 0.96
            * self.zoom;
        (
            [
                (w - 82.0) / 2.0 + self.pan[0] * w,
                h / 2.0 + self.pan[1] * h,
            ],
            [(lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5],
            scale,
        )
    }
    pub fn screen_at(&self, snapshot: &PlaneSnapshot, point: Vec3) -> [f64; 2] {
        let (center, mid, scale) = self.transform(snapshot);
        let d = point - snapshot.plane.origin;
        [
            center[0] + (d.dot(snapshot.plane.u) - mid[0]) * scale,
            center[1] - (d.dot(snapshot.plane.v) - mid[1]) * scale,
        ]
    }
    pub fn world_at(&self, snapshot: &PlaneSnapshot, x: f64, y: f64) -> Vec3 {
        let (center, mid, scale) = self.transform(snapshot);
        snapshot.plane.origin
            + snapshot.plane.u * (mid[0] + (x - center[0]) / scale)
            + snapshot.plane.v * (mid[1] + (center[1] - y) / scale)
    }
}
/// Return the intersected cell under the actual section camera, never an image pixel value.
pub fn probe_section<'a>(
    snapshot: &'a PlaneSnapshot,
    camera: &SectionCamera,
    x: f64,
    y: f64,
) -> Option<&'a PlaneSample> {
    if x < 16.0 || x > camera.viewport[0] - 98.0 || y < 24.0 || y > camera.viewport[1] - 24.0 {
        return None;
    }
    let point = camera.world_at(snapshot, x, y);
    probe_world(snapshot, point)
}
pub fn probe_world(snapshot: &PlaneSnapshot, point: Vec3) -> Option<&PlaneSample> {
    if (point - snapshot.plane.origin)
        .dot(snapshot.plane.normal())
        .abs()
        > 1e-8
    {
        return None;
    }
    let project = |v: Vec3| {
        let d = v - snapshot.plane.origin;
        (d.dot(snapshot.plane.u), d.dot(snapshot.plane.v))
    };
    let q = project(point);
    snapshot.samples.iter().find(|s| {
        let poly = &s.geometry.polygon;
        let mut inside = false;
        for i in 0..poly.len() {
            let a = project(poly[i]);
            let b = project(poly[(i + 1) % poly.len()]);
            if (a.1 > q.1) != (b.1 > q.1) && q.0 < (b.0 - a.0) * (q.1 - a.1) / (b.1 - a.1) + a.0 {
                inside = !inside;
            }
        }
        inside
    })
}
pub fn section_interactive_svg(
    snapshot: &PlaneSnapshot,
    style: &PlaneDisplay,
    camera: &SectionCamera,
) -> String {
    section_svg_impl(snapshot, style, None, None, false, Some(camera))
}

pub fn section_interactive_probe_svg(
    snapshot: &PlaneSnapshot,
    style: &PlaneDisplay,
    camera: &SectionCamera,
    point: Option<Vec3>,
) -> String {
    let mut svg = section_interactive_svg(snapshot, style, camera);
    if let Some(point) = point {
        let [x, y] = camera.screen_at(snapshot, point);
        if x.is_finite() && y.is_finite() {
            let marker = format!(
                r##"<g clip-path="url(#section-clip)"><circle cx="{x}" cy="{y}" r="5" stroke="#ffdb70" stroke-width="2" fill="none"/><path d="M{} {y} H{} M{x} {} V{}" stroke="#ffdb70" stroke-width="1"/></g></svg>"##,
                x - 8.,
                x + 8.,
                y - 8.,
                y + 8.
            );
            svg = svg.replace("</svg>", &marker);
        }
    }
    svg
}

pub fn section_svg(snapshot: &PlaneSnapshot, style: &PlaneDisplay) -> String {
    section_svg_with_scalar(snapshot, style, None, None)
}
/// Optional per-cell scalar field for volume sections; velocity overlays stay physical.
pub fn section_svg_with_scalar(
    snapshot: &PlaneSnapshot,
    style: &PlaneDisplay,
    scalar_override: Option<(&[f64], &str)>,
    palette: Option<&dyn Fn(f64) -> [u8; 4]>,
) -> String {
    section_svg_impl(snapshot, style, scalar_override, palette, true, None)
}
/// Workbench canvas presentation; scientific axes/titles remain in exported SVGs.
pub fn section_viewport_svg(snapshot: &PlaneSnapshot, style: &PlaneDisplay) -> String {
    section_svg_impl(snapshot, style, None, None, false, None)
}
pub fn section_viewport_with_scalar(
    snapshot: &PlaneSnapshot,
    style: &PlaneDisplay,
    scalar_override: Option<(&[f64], &str)>,
    palette: Option<&dyn Fn(f64) -> [u8; 4]>,
) -> String {
    section_svg_impl(snapshot, style, scalar_override, palette, false, None)
}
fn section_svg_impl(
    snapshot: &PlaneSnapshot,
    style: &PlaneDisplay,
    scalar_override: Option<(&[f64], &str)>,
    palette: Option<&dyn Fn(f64) -> [u8; 4]>,
    decorated: bool,
    camera: Option<&SectionCamera>,
) -> String {
    let time_caption = snapshot
        .physical_time
        .map(|t| format!("Physical time: {t:.6e} s"))
        .unwrap_or_else(|| "Steady iteration, not physical time".into());
    let p = &snapshot.plane;
    let field = style.field.min(5);
    let (mut label, mut unit) = match field {
        1 => ("Kinematic gauge pressure p/rho", "m2/s2"),
        2 => ("Velocity X", "m/s"),
        3 => ("Velocity Y", "m/s"),
        4 => ("Velocity Z", "m/s"),
        5 => ("Normal velocity", "m/s"),
        _ => ("Speed |U|", "m/s"),
    };
    if let Some((_, title)) = scalar_override {
        label = title;
        unit = "";
    }
    let scalar = |s: &PlaneSample| {
        if let Some((values, _)) = scalar_override {
            values.get(s.geometry.cell).copied().unwrap_or(f64::NAN)
        } else {
            sample_scalar(snapshot, s, field)
        }
    };
    let project = |q: Vec3| {
        let d = q - p.origin;
        (d.dot(p.u), d.dot(p.v))
    };
    let mut lo = (f64::INFINITY, f64::INFINITY);
    let mut hi = (f64::NEG_INFINITY, f64::NEG_INFINITY);
    let (mut vmin, mut vmax, mut speed) = (f64::INFINITY, f64::NEG_INFINITY, 0_f64);
    for s in &snapshot.samples {
        let v = scalar(s);
        vmin = vmin.min(v);
        vmax = vmax.max(v);
        speed = speed.max(s.velocity.norm());
        for q in &s.geometry.polygon {
            let (a, b) = project(*q);
            lo.0 = lo.0.min(a);
            lo.1 = lo.1.min(b);
            hi.0 = hi.0.max(a);
            hi.1 = hi.1.max(b);
        }
    }
    if snapshot.samples.is_empty() {
        return r##"<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="650"><rect width="100%" height="100%" fill="#20242c"/><text x="40" y="60" fill="white">No intersected cells on this plane.</text></svg>"##.into();
    }
    let data_range = (vmin, vmax);
    if let Some([a, b]) = style
        .range
        .filter(|[a, b]| a.is_finite() && b.is_finite() && b > a)
    {
        vmin = a;
        vmax = b;
    }
    let color = |v: f64| {
        let t = if vmax > vmin {
            ((v - vmin) / (vmax - vmin)).clamp(0., 1.)
        } else {
            0.5
        };
        if let Some(palette) = palette {
            let c = palette(t);
            return format!("#{:02x}{:02x}{:02x}", c[0], c[1], c[2]);
        }
        // Perceptually ordered Viridis anchors, linearly interpolated.
        let palette = match style.palette {
            0 => [
                [48., 18., 59.],
                [46., 91., 195.],
                [34., 196., 173.],
                [239., 220., 70.],
                [199., 44., 46.],
            ],
            2 => [
                [13., 8., 135.],
                [126., 3., 168.],
                [204., 71., 120.],
                [248., 149., 64.],
                [240., 249., 33.],
            ],
            3 => [
                [49., 54., 149.],
                [116., 173., 209.],
                [247., 247., 247.],
                [244., 109., 67.],
                [165., 0., 38.],
            ],
            _ => [
                [68., 1., 84.],
                [59., 82., 139.],
                [33., 145., 140.],
                [94., 201., 98.],
                [253., 231., 37.],
            ],
        };
        let x = t * 4.;
        let i = (x.floor() as usize).min(3);
        let f = x - i as f64;
        format!(
            "#{:02x}{:02x}{:02x}",
            (palette[i][0] * (1. - f) + palette[i + 1][0] * f) as u8,
            (palette[i][1] * (1. - f) + palette[i + 1][1] * f) as u8,
            (palette[i][2] * (1. - f) + palette[i + 1][2] * f) as u8
        )
    };
    let [width, height] = camera.map(|c| c.viewport).unwrap_or([1000.0, 650.0]);
    let legend_x = if camera.is_some() {
        width - 85.0
    } else {
        width - 150.0
    };
    let legend_text = if camera.is_some() {
        width - 64.0
    } else {
        width - 120.0
    };
    let (left, right, top, bottom) = if camera.is_some() {
        (16.0, width - 98.0, 24.0, height - 24.0)
    } else if decorated {
        (85., 805., 115., 555.)
    } else {
        (28., 805., 24., 620.)
    };
    let scale = ((right - left) / (hi.0 - lo.0).max(1e-15))
        .min((bottom - top) / (hi.1 - lo.1).max(1e-15))
        * 0.96;
    let (cx, cy) = ((left + right) / 2., (top + bottom) / 2.);
    let mid = ((lo.0 + hi.0) / 2., (lo.1 + hi.1) / 2.);
    let ((cx, cy), mid, scale) = if let Some(camera) = camera {
        let (center, mid, scale) = camera.transform(snapshot);
        ((center[0], center[1]), (mid[0], mid[1]), scale)
    } else {
        ((cx, cy), mid, scale)
    };
    let screen = |q: Vec3| {
        let (a, b) = project(q);
        (cx + (a - mid.0) * scale, cy - (b - mid.1) * scale)
    };
    let mut svg = if decorated {
        format!(
            r##"<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="650"><rect width="100%" height="100%" fill="#20242c"/><g font-family="DejaVu Sans,sans-serif" fill="#d4dce8" font-size="12"><text x="{left}" y="28" font-size="18">{} · iteration {}</text><text x="{left}" y="51">{} ({unit}) · cell-centred section</text><text x="{left}" y="73">{} cells · data min {:.4e} · max {:.4e}</text><text x="{left}" y="94">Origin ({:.4e}, {:.4e}, {:.4e}) m · normal ({:.3}, {:.3}, {:.3})</text>"##,
            escaped(&p.name),
            snapshot.iteration,
            label,
            snapshot.samples.len(),
            data_range.0,
            data_range.1,
            p.origin.x,
            p.origin.y,
            p.origin.z,
            p.normal().x,
            p.normal().y,
            p.normal().z
        )
    } else {
        format!(
            r##"<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}"><rect width="100%" height="100%" fill="#2d3340"/><g font-family="DejaVu Sans,sans-serif" fill="#d4dce8" font-size="12">"##
        )
    };
    if camera.is_some() {
        let _ = write!(
            svg,
            r##"<defs><clipPath id="section-clip"><rect x="{left}" y="{top}" width="{}" height="{}"/></clipPath></defs><g clip-path="url(#section-clip)">"##,
            right - left,
            bottom - top
        );
    }
    for s in &snapshot.samples {
        let points = s
            .geometry
            .polygon
            .iter()
            .map(|q| {
                let (x, y) = screen(*q);
                format!("{x:.2},{y:.2}")
            })
            .collect::<Vec<_>>()
            .join(" ");
        let c = if style.plot == 1 || style.plot == 3 {
            "none".to_owned()
        } else {
            color(scalar(s))
        };
        let edge = if style.edges { "#18212a" } else { c.as_str() };
        let _ = write!(
            svg,
            r##"<polygon points="{points}" fill="{c}" stroke="{edge}" stroke-width="0.5"/>"##
        );
    }
    if matches!(style.plot, 1 | 2) && vmax > vmin {
        let mut vertices = std::collections::HashMap::<(u64, u64, u64), (f64, usize)>::new();
        for sample in &snapshot.samples {
            for q in &sample.geometry.polygon {
                let entry = vertices
                    .entry((q.x.to_bits(), q.y.to_bits(), q.z.to_bits()))
                    .or_default();
                entry.0 += scalar(sample);
                entry.1 += 1;
            }
        }
        for sample in &snapshot.samples {
            let poly = &sample.geometry.polygon;
            for j in 0..poly.len() {
                let a = poly[j];
                let b = poly[(j + 1) % poly.len()];
                let mean = |q: Vec3| {
                    let (sum, n) = vertices[&(q.x.to_bits(), q.y.to_bits(), q.z.to_bits())];
                    sum / n as f64
                };
                let triangle = [
                    (sample.geometry.point, scalar(sample)),
                    (a, mean(a)),
                    (b, mean(b)),
                ];
                for level in 1..style.levels {
                    let target = vmin + (vmax - vmin) * level as f64 / style.levels as f64;
                    let mut cuts = Vec::with_capacity(3);
                    for edge in 0..3 {
                        let (a, va) = triangle[edge];
                        let (b, vb) = triangle[(edge + 1) % 3];
                        if (va <= target && vb > target) || (vb <= target && va > target) {
                            cuts.push(screen(a + (b - a) * ((target - va) / (vb - va))));
                        }
                    }
                    if cuts.len() == 2 {
                        let c = if style.plot == 1 {
                            color(target)
                        } else {
                            "#f2f4f8".into()
                        };
                        let _ = write!(
                            svg,
                            r##"<path d="M{:.2} {:.2} L{:.2} {:.2}" stroke="{c}" stroke-width="0.8" fill="none"/>"##,
                            cuts[0].0, cuts[0].1, cuts[1].0, cuts[1].1
                        );
                    }
                }
            }
        }
    }
    if style.streamlines {
        // Planar streamlines, piecewise constant velocity in each intersected cell.
        // Stop at the section boundary, stagnation, or a bounded integration budget.
        let polygons: Vec<_> = snapshot
            .samples
            .iter()
            .map(|s| {
                s.geometry
                    .polygon
                    .iter()
                    .map(|q| project(*q))
                    .collect::<Vec<_>>()
            })
            .collect();
        let inside = |q: (f64, f64), poly: &[(f64, f64)]| {
            let mut hit = false;
            for i in 0..poly.len() {
                let a = poly[i];
                let b = poly[(i + 1) % poly.len()];
                if (a.1 > q.1) != (b.1 > q.1) && q.0 < (b.0 - a.0) * (q.1 - a.1) / (b.1 - a.1) + a.0
                {
                    hit = !hit;
                }
            }
            hit
        };
        let step = ((hi.0 - lo.0).min(hi.1 - lo.1) / 100.0).max(1e-15);
        for seed in snapshot
            .samples
            .iter()
            .step_by(snapshot.samples.len().div_ceil(12).max(1))
        {
            for sign in [-1.0, 1.0] {
                let mut point = seed.geometry.point;
                let mut path = String::new();
                for index in 0..160 {
                    let q = project(point);
                    let Some(cell) = polygons.iter().position(|poly| inside(q, poly)) else {
                        break;
                    };
                    let sample = &snapshot.samples[cell];
                    let tangent = p.u * sample.velocity.dot(p.u) + p.v * sample.velocity.dot(p.v);
                    let magnitude = tangent.norm();
                    if magnitude < speed.max(1e-30) * 1e-7 {
                        break;
                    }
                    let (x, y) = screen(point);
                    let _ = write!(
                        path,
                        "{} {x:.2} {y:.2} ",
                        if index == 0 { "M" } else { "L" }
                    );
                    point += tangent * (sign * step / magnitude);
                }
                let _ = write!(
                    svg,
                    r##"<path d="{path}" fill="none" stroke="#ffdb70" stroke-width="1.2"/>"##
                );
            }
        }
    }
    if style.vectors {
        let stride = snapshot
            .samples
            .len()
            .div_ceil(style.vector_density.clamp(1, 2000))
            .max(1);
        let length = 24_f64
            .min(
                ((right - left) * (bottom - top) / snapshot.samples.len().max(1) as f64).sqrt()
                    * 0.8,
            )
            .max(6.);
        for s in snapshot.samples.iter().step_by(stride) {
            let (x, y) = screen(s.geometry.point);
            let v = s.velocity;
            let in_plane = v.dot(p.u).hypot(v.dot(p.v));
            let denom = if style.normalized_vectors {
                in_plane.max(speed * 1e-8).max(1e-30)
            } else {
                speed.max(1e-30)
            };
            let (dx, dy) = (
                v.dot(p.u) / denom * length * style.vector_scale,
                -v.dot(p.v) / denom * length * style.vector_scale,
            );
            let mag = dx.hypot(dy);
            if mag > 0.35 {
                let (ex, ey) = (x + dx, y + dy);
                let (ux, uy) = (dx / mag, dy / mag);
                let h = 3.5_f64.min(mag * 0.5);
                let _ = write!(
                    svg,
                    r##"<path d="M{x:.2} {y:.2} L{ex:.2} {ey:.2} M{:.2} {:.2} L{ex:.2} {ey:.2} L{:.2} {:.2}" fill="none" stroke="#f4f7ff" stroke-width="1.2"/>"##,
                    ex - h * ux + h * 0.5 * uy,
                    ey - h * uy - h * 0.5 * ux,
                    ex - h * ux - h * 0.5 * uy,
                    ey - h * uy + h * 0.5 * ux
                );
            }
            let n = v.dot(p.normal());
            if style.normal_markers && n.abs() > speed.max(1e-30) * 0.03 {
                if n > 0. {
                    let _ = write!(
                        svg,
                        r##"<circle cx="{x:.2}" cy="{y:.2}" r="1.8" fill="#fff"/>"##
                    );
                } else {
                    let _ = write!(
                        svg,
                        r##"<path d="M{} {} l4 4 m-4 0 l4 -4" stroke="#fff"/>"##,
                        x - 2.,
                        y - 2.
                    );
                }
            }
        }
    }
    if camera.is_some() {
        svg.push_str("</g>");
    }
    for i in 0..100 {
        let t = i as f64 / 99.;
        let y = top + (1. - t) * (bottom - top);
        let c = color(vmin + t * (vmax - vmin));
        let _ = write!(
            svg,
            r##"<rect x="{legend_x}" y="{y:.2}" width="20" height="{}" fill="{c}"/>"##,
            (bottom - top) / 99.0 + 1.0
        );
    }
    for i in 0..=4 {
        let t = i as f64 / 4.;
        let x = left + (right - left) * t;
        let y = bottom - (bottom - top) * t;
        let a = mid.0 + (x - cx) / scale;
        let b = mid.1 + (cy - y) / scale;
        let v = vmin + t * (vmax - vmin);
        if decorated {
            let _ = write!(
                svg,
                r##"<text x="{x}" y="580" text-anchor="middle">{a:.3e}</text><text x="75" y="{}" text-anchor="end">{b:.3e}</text><text x="880" y="{}">{v:.3e}</text>"##,
                y + 4.,
                y + 4.
            );
        } else {
            let _ = write!(
                svg,
                r##"<text x="{legend_text}" y="{}">{v:.3e}</text>"##,
                y + 4.
            );
        }
    }
    let vector_label = if style.normalized_vectors {
        "normalized in-plane directions (length is not speed)"
    } else {
        "in-plane velocity; length scaled to maximum speed"
    };
    if decorated {
        let _ = write!(
            svg,
            r##"<path d="M{left} {top} V{bottom} H{right}" fill="none" stroke="#9aabbf"/><text x="{cx}" y="607" text-anchor="middle">Plane u (m)</text><text transform="translate(18 {cy}) rotate(-90)" text-anchor="middle">Plane v (m)</text><text x="845" y="109">{unit}</text><text x="{left}" y="635">{time_caption} · vectors: {vector_label}</text></g></svg>"##
        );
    } else {
        let _ = write!(
            svg,
            r##"<text x="{legend_x}" y="18">{unit}</text></g></svg>"##
        );
    }
    svg
}

pub fn csv_text(snapshot: &PlaneSnapshot) -> String {
    let mut csv = String::new();
    let _=writeln!(csv,"iteration,cell,x_m,y_m,z_m,ux_m_s,uy_m_s,uz_m_s,u_plane_m_s,v_plane_m_s,normal_velocity_m_s,speed_m_s,kinematic_pressure_m2_s2,time_s");
    for s in &snapshot.samples {
        let p = s.geometry.point;
        let v = s.velocity;
        let _=writeln!(csv,"{},{},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e}",snapshot.iteration,s.geometry.cell,p.x,p.y,p.z,v.x,v.y,v.z,v.dot(snapshot.plane.u),v.dot(snapshot.plane.v),v.dot(snapshot.plane.normal()),v.norm(),s.pressure);
        if let Some(time) = snapshot.physical_time {
            csv.pop();
            let _ = writeln!(csv, ",{time:.16e}");
        } else {
            csv.pop();
            csv.push_str(",\n");
        }
    }
    csv
}

#[cfg(test)]
mod recording_tests {
    use super::*;

    #[test]
    fn recorded_frame_retains_geometry_and_all_fields_for_replay() {
        let plane = FlowPlaneMonitor {
            id: "mid-plane".into(),
            name: "Mid plane".into(),
            origin: Vec3::new(0.0, 0.0, 0.005),
            u: Vec3::new(1.0, 0.0, 0.0),
            v: Vec3::new(0.0, 1.0, 0.0),
            enabled: true,
            every: 10,
            display: PlaneDisplay::default(),
        };
        let snapshot = PlaneSnapshot {
            physical_time: None,
            plane,
            iteration: 20,
            path: None,
            samples: vec![PlaneSample {
                geometry: PlaneCell {
                    cell: 7,
                    point: Vec3::new(0.01, 0.01, 0.005),
                    polygon: vec![
                        Vec3::new(0.0, 0.0, 0.005),
                        Vec3::new(0.02, 0.0, 0.005),
                        Vec3::new(0.0, 0.02, 0.005),
                    ],
                },
                velocity: Vec3::new(1.25, -0.5, 0.125),
                pressure: -2.75,
            }],
        };
        let folder = tempfile::tempdir().unwrap();
        let path = write_csv(&snapshot, folder.path()).unwrap();
        let restored: PlaneSnapshot =
            serde_json::from_slice(&std::fs::read(path.with_extension("json")).unwrap()).unwrap();
        assert_eq!(csv_text(&snapshot), csv_text(&restored));
        assert_eq!(
            restored.samples[0].geometry.polygon,
            snapshot.samples[0].geometry.polygon
        );
        assert_eq!(restored.plane, snapshot.plane);
        let mut pressure_style = restored.plane.display.clone();
        pressure_style.field = 1;
        assert_ne!(
            section_svg(&restored, &pressure_style),
            section_svg(&restored, &restored.plane.display)
        );
        assert!(path.with_extension("svg").is_file());
    }
}
