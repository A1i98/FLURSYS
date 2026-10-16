//! Plane intersections of convex finite-volume cells for interior field inspection.
use super::results::{ResultDataError, ResultDataset};
use crate::{MeshDimension, Vec3};

impl ResultDataset {
    /// Intersect a coordinate plane with every convex volume cell. Axis 0/1/2
    /// means X/Y/Z; returned polygons retain their source cell indices.
    pub fn plane_slice(
        &self,
        axis: usize,
        coordinate: f64,
    ) -> Result<Vec<(usize, Vec<Vec3>)>, ResultDataError> {
        if axis > 2 || !coordinate.is_finite() || self.mesh().dimension() != MeshDimension::ThreeD {
            return Err(ResultDataError::Invalid(
                "A 3D slice requires axis X/Y/Z and a finite coordinate".into(),
            ));
        }
        let component = |p: Vec3| [p.x, p.y, p.z][axis];
        let mesh = self.mesh();
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for p in mesh.points() {
            lo = lo.min(component(p.position));
            hi = hi.max(component(p.position));
        }
        let epsilon =
            ((hi - lo).abs() * 1e-10).max(f64::EPSILON * coordinate.abs().max(1.0) * 32.0);
        let mut polygons = Vec::new();
        for (index, cell) in mesh.cells().iter().enumerate() {
            let mut points: Vec<Vec3> = Vec::new();
            let mut insert = |p: Vec3| {
                if !points.iter().any(|q| (*q - p).norm() <= epsilon) {
                    points.push(p);
                }
            };
            for face in &cell.faces {
                let vertices = &mesh.faces()[*face].vertices;
                for i in 0..vertices.len() {
                    let a = mesh.points()[vertices[i]].position;
                    let b = mesh.points()[vertices[(i + 1) % vertices.len()]].position;
                    let da = component(a) - coordinate;
                    let db = component(b) - coordinate;
                    if da.abs() <= epsilon {
                        insert(a);
                    }
                    if (da < -epsilon && db > epsilon) || (db < -epsilon && da > epsilon) {
                        insert(a + (b - a) * (da / (da - db)));
                    }
                }
            }
            if points.len() < 3 {
                continue;
            }
            let center =
                points.iter().fold(Vec3::ZERO, |sum, p| sum + *p) * (1.0 / points.len() as f64);
            let axes = match axis {
                0 => (1, 2),
                1 => (0, 2),
                _ => (0, 1),
            };
            let angle = |p: Vec3| {
                let d = p - center;
                let v = [d.x, d.y, d.z];
                v[axes.1].atan2(v[axes.0])
            };
            points.sort_by(|a, b| angle(*a).total_cmp(&angle(*b)));
            let area = (1..points.len() - 1)
                .map(|i| {
                    (points[i] - points[0])
                        .cross(points[i + 1] - points[0])
                        .norm()
                        * 0.5
                })
                .sum::<f64>();
            if area > epsilon * epsilon {
                polygons.push((index, points));
            }
        }
        Ok(polygons)
    }
}
