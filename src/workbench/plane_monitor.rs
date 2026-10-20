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
            vectors: true,
            edges: false,
            normalized_vectors: true,
            normal_markers: false,
            range: None,
        }
    }
}
impl PlaneDisplay {
    pub fn validate(&self) -> Result<(), String> {
        if self.field > 5
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
        plane: plane.clone(),
        iteration,
        samples,
        path: None,
    })
}
pub fn write_csv(snapshot: &PlaneSnapshot, folder: &Path) -> Result<std::path::PathBuf, String> {
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
pub fn section_svg(snapshot: &PlaneSnapshot, style: &PlaneDisplay) -> String {
    let p = &snapshot.plane;
    let field = style.field.min(5);
    let (label, unit) = match field {
        1 => ("Kinematic gauge pressure p/rho", "m2/s2"),
        2 => ("Velocity X", "m/s"),
        3 => ("Velocity Y", "m/s"),
        4 => ("Velocity Z", "m/s"),
        5 => ("Normal velocity", "m/s"),
        _ => ("Speed |U|", "m/s"),
    };
    let scalar = |s: &PlaneSample| match field {
        1 => s.pressure,
        2 => s.velocity.x,
        3 => s.velocity.y,
        4 => s.velocity.z,
        5 => s.velocity.dot(p.normal()),
        _ => s.velocity.norm(),
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
        // Perceptually ordered Viridis anchors, linearly interpolated.
        let palette = [
            [68., 1., 84.],
            [59., 82., 139.],
            [33., 145., 140.],
            [94., 201., 98.],
            [253., 231., 37.],
        ];
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
    let (left, right, top, bottom) = (85., 805., 115., 555.);
    let scale = ((right - left) / (hi.0 - lo.0).max(1e-15))
        .min((bottom - top) / (hi.1 - lo.1).max(1e-15))
        * 0.96;
    let (cx, cy) = ((left + right) / 2., (top + bottom) / 2.);
    let mid = ((lo.0 + hi.0) / 2., (lo.1 + hi.1) / 2.);
    let screen = |q: Vec3| {
        let (a, b) = project(q);
        (cx + (a - mid.0) * scale, cy - (b - mid.1) * scale)
    };
    let mut svg = format!(
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
    );
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
        let c = color(scalar(s));
        let edge = if style.edges { "#18212a" } else { c.as_str() };
        let _ = write!(
            svg,
            r##"<polygon points="{points}" fill="{c}" stroke="{edge}" stroke-width="0.5"/>"##
        );
    }
    if style.vectors {
        let stride = snapshot.samples.len().div_ceil(600).max(1);
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
            let (dx, dy) = (v.dot(p.u) / denom * length, -v.dot(p.v) / denom * length);
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
    for i in 0..100 {
        let t = i as f64 / 99.;
        let y = top + (1. - t) * (bottom - top);
        let c = color(vmin + t * (vmax - vmin));
        let _ = write!(
            svg,
            r##"<rect x="850" y="{y:.2}" width="20" height="5" fill="{c}"/>"##
        );
    }
    for i in 0..=4 {
        let t = i as f64 / 4.;
        let x = left + (right - left) * t;
        let y = bottom - (bottom - top) * t;
        let a = mid.0 + (x - cx) / scale;
        let b = mid.1 + (cy - y) / scale;
        let v = vmin + t * (vmax - vmin);
        let _ = write!(
            svg,
            r##"<text x="{x}" y="580" text-anchor="middle">{a:.3e}</text><text x="75" y="{}" text-anchor="end">{b:.3e}</text><text x="880" y="{}">{v:.3e}</text>"##,
            y + 4.,
            y + 4.
        );
    }
    let vector_label = if style.normalized_vectors {
        "normalized in-plane directions (length is not speed)"
    } else {
        "in-plane velocity; length scaled to maximum speed"
    };
    let _ = write!(
        svg,
        r##"<path d="M{left} {top} V{bottom} H{right}" fill="none" stroke="#9aabbf"/><text x="{cx}" y="607" text-anchor="middle">Plane u (m)</text><text transform="translate(18 {cy}) rotate(-90)" text-anchor="middle">Plane v (m)</text><text x="845" y="109">{unit}</text><text x="{left}" y="635">Steady iteration, not physical time · vectors: {vector_label}</text></g></svg>"##
    );
    svg
}

pub fn csv_text(snapshot: &PlaneSnapshot) -> String {
    let mut csv = String::new();
    let _=writeln!(csv,"iteration,cell,x_m,y_m,z_m,ux_m_s,uy_m_s,uz_m_s,u_plane_m_s,v_plane_m_s,normal_velocity_m_s,speed_m_s,kinematic_pressure_m2_s2");
    for s in &snapshot.samples {
        let p = s.geometry.point;
        let v = s.velocity;
        let _=writeln!(csv,"{},{},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e}",snapshot.iteration,s.geometry.cell,p.x,p.y,p.z,v.x,v.y,v.z,v.dot(snapshot.plane.u),v.dot(snapshot.plane.v),v.dot(snapshot.plane.normal()),v.norm(),s.pressure);
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
