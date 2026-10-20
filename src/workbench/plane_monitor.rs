//! Cell-centred flow samples on arbitrary CAD planes; no time interpolation.
use crate::{MeshDimension, UnstructuredMesh, Vec3};
use serde::{Deserialize, Serialize};
use std::fmt::Write as _;
use std::{io::Write, path::Path};
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FlowPlaneMonitor {
    pub id: String,
    pub name: String,
    pub origin: Vec3,
    pub u: Vec3,
    pub v: Vec3,
    pub enabled: bool,
    pub every: usize,
}
impl FlowPlaneMonitor {
    pub fn validate(&self) -> Result<(), String> {
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
#[derive(Clone, Debug)]
pub struct PlaneCell {
    pub cell: usize,
    pub point: Vec3,
    pub polygon: Vec<Vec3>,
}
#[derive(Clone, Debug)]
pub struct PlaneSample {
    pub geometry: PlaneCell,
    pub velocity: Vec3,
    pub pressure: f64,
}
#[derive(Clone, Debug)]
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
    let (w, h) = (1000., 600.);
    let (left, right, top, bottom) = (85., 960., 92., 535.);
    let p = &snapshot.plane;
    let project = |point: Vec3| {
        let d = point - p.origin;
        (d.dot(p.u), d.dot(p.v))
    };
    let mut lo = (f64::INFINITY, f64::INFINITY);
    let mut hi = (f64::NEG_INFINITY, f64::NEG_INFINITY);
    let mut vmax = 0_f64;
    for s in &snapshot.samples {
        for point in &s.geometry.polygon {
            let q = project(*point);
            lo.0 = lo.0.min(q.0);
            lo.1 = lo.1.min(q.1);
            hi.0 = hi.0.max(q.0);
            hi.1 = hi.1.max(q.1);
        }
        vmax = vmax.max(s.velocity.norm());
    }
    if !lo.0.is_finite() {
        lo = (0., 0.);
        hi = (1., 1.);
    }
    let span = ((hi.0 - lo.0).max(1e-14), (hi.1 - lo.1).max(1e-14));
    let scale = ((right - left) / span.0).min((bottom - top) / span.1) * 0.94;
    let cx = (left + right) / 2.;
    let cy = (top + bottom) / 2.;
    let middle = ((lo.0 + hi.0) / 2., (lo.1 + hi.1) / 2.);
    let screen = |point: Vec3| {
        let q = project(point);
        (cx + (q.0 - middle.0) * scale, cy - (q.1 - middle.1) * scale)
    };
    let mut svg = format!(
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}"><rect width="100%" height="100%" fill="#20242c"/><g font-family="DejaVu Sans,sans-serif" fill="#d4dce8" font-size="13"><text x="{left}" y="26" font-size="17">{} · iteration {}</text><text x="{left}" y="49">Cell-centred U · arrows: in-plane velocity · dot/cross: positive/negative normal flow</text><text x="{left}" y="70">{} intersected cells · speed range 0 … {:.3e} m/s · SIMPLE iteration, not physical time</text>"##,
        escaped(&p.name),
        snapshot.iteration,
        snapshot.samples.len(),
        vmax
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
        let _ = write!(
            svg,
            r##"<polygon points="{points}" fill="#283441" stroke="#3c4a5b" stroke-width="0.65"/>"##
        );
    }
    let stride = (snapshot.samples.len() / 600).max(1);
    let length = 22_f64
        .min(((right - left) * (bottom - top) / snapshot.samples.len().max(1) as f64).sqrt() * 0.65)
        .max(5.);
    for s in snapshot.samples.iter().step_by(stride) {
        let (x, y) = screen(s.geometry.point);
        let v = s.velocity;
        let a = v.dot(p.u);
        let b = v.dot(p.v);
        let n = v.dot(p.normal());
        let t = if vmax > 0. {
            (v.norm() / vmax).clamp(0., 1.)
        } else {
            0.
        };
        let color = format!(
            "#{:02x}{:02x}{:02x}",
            (65. + 180. * t) as u8,
            (190. - 35. * t) as u8,
            (245. - 170. * t) as u8
        );
        let denom = vmax.max(1e-30);
        let dx = a / denom * length;
        let dy = -b / denom * length;
        let m = dx.hypot(dy);
        if m > 0.4 {
            let (ex, ey) = (x + dx, y + dy);
            let (ux, uy) = (dx / m, dy / m);
            let head = 3.5_f64.min(m * 0.5);
            let _ = write!(
                svg,
                r##"<path d="M{x:.2} {y:.2} L{ex:.2} {ey:.2} M{:.2} {:.2} L{ex:.2} {ey:.2} L{:.2} {:.2}" fill="none" stroke="{color}" stroke-width="1.4"/>"##,
                ex - head * ux + head * 0.5 * uy,
                ey - head * uy - head * 0.5 * ux,
                ex - head * ux - head * 0.5 * uy,
                ey - head * uy + head * 0.5 * ux
            );
        }
        if n.abs() > denom * 0.03 {
            let _ = write!(
                svg,
                r##"<circle cx="{x:.2}" cy="{y:.2}" r="3" fill="none" stroke="{color}"/>"##
            );
            if n > 0. {
                let _ = write!(
                    svg,
                    r##"<circle cx="{x:.2}" cy="{y:.2}" r="1" fill="{color}"/>"##
                );
            } else {
                let _ = write!(
                    svg,
                    r##"<path d="M{} {} l4 4 m-4 0 l4 -4" stroke="{color}"/>"##,
                    x - 2.,
                    y - 2.
                );
            }
        }
    }
    for i in 0..=4 {
        let t = i as f64 / 4.;
        let x = left + (right - left) * t;
        let y = bottom - (bottom - top) * t;
        let a = middle.0 + (x - cx) / scale;
        let b = middle.1 + (cy - y) / scale;
        let _ = write!(
            svg,
            r##"<text x="{x}" y="{}" text-anchor="middle">{a:.2e}</text><text x="{}" y="{}" text-anchor="end">{b:.2e}</text>"##,
            bottom + 23.,
            left - 10.,
            y + 4.
        );
    }
    let _ = write!(
        svg,
        r##"<path d="M{left} {top} V{bottom} H{right}" fill="none" stroke="#9aabbf"/><text x="{cx}" y="580" text-anchor="middle">Plane u (m)</text><text transform="translate(18 {cy}) rotate(-90)" text-anchor="middle">Plane v (m)</text></g></svg>"##
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
