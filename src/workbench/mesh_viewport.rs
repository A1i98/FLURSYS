//! Disposable, mesh-bound inspection data for the workbench viewport.
//!
//! `UnstructuredMesh` remains authoritative. This module derives contiguous
//! display buffers, mesh-bound selection, picking, and quality values only.

use crate::{MeshDimension, MeshId, UnstructuredMesh, Vec3};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeshSelectionTarget {
    Face(usize),
    Cell(usize),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MeshSelection {
    mesh_id: MeshId,
    target: MeshSelectionTarget,
}

impl MeshSelection {
    pub const fn face(mesh_id: MeshId, index: usize) -> Self {
        Self {
            mesh_id,
            target: MeshSelectionTarget::Face(index),
        }
    }

    pub const fn cell(mesh_id: MeshId, index: usize) -> Self {
        Self {
            mesh_id,
            target: MeshSelectionTarget::Cell(index),
        }
    }

    pub const fn mesh_id(self) -> MeshId {
        self.mesh_id
    }
    pub const fn target(self) -> MeshSelectionTarget {
        self.target
    }

    pub fn resolve(self, mesh: &UnstructuredMesh) -> Option<MeshSelectionTarget> {
        if self.mesh_id != mesh.id() {
            return None;
        }
        match self.target {
            MeshSelectionTarget::Face(index) if index < mesh.face_count() => Some(self.target),
            MeshSelectionTarget::Cell(index) if index < mesh.cell_count() => Some(self.target),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeshQualityMetric {
    AspectRatio,
    NonOrthogonality,
    Skewness,
    CellMeasure,
}

impl MeshQualityMetric {
    pub const fn label(self) -> &'static str {
        match self {
            Self::AspectRatio => "Aspect ratio",
            Self::NonOrthogonality => "Non-orthogonality (deg)",
            Self::Skewness => "Skewness",
            Self::CellMeasure => "Cell measure",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct MeshQualityValues {
    aspect_ratio: Vec<f64>,
    non_orthogonality: Vec<f64>,
    skewness: Vec<f64>,
    cell_measure: Vec<f64>,
}

impl MeshQualityValues {
    pub fn aspect_ratio(&self) -> &[f64] {
        &self.aspect_ratio
    }
    pub fn non_orthogonality(&self) -> &[f64] {
        &self.non_orthogonality
    }
    pub fn skewness(&self) -> &[f64] {
        &self.skewness
    }
    pub fn cell_measure(&self) -> &[f64] {
        &self.cell_measure
    }

    pub fn values(&self, metric: MeshQualityMetric) -> &[f64] {
        match metric {
            MeshQualityMetric::AspectRatio => &self.aspect_ratio,
            MeshQualityMetric::NonOrthogonality => &self.non_orthogonality,
            MeshQualityMetric::Skewness => &self.skewness,
            MeshQualityMetric::CellMeasure => &self.cell_measure,
        }
    }

    pub fn color_range(&self, metric: MeshQualityMetric) -> Option<(f64, f64)> {
        let values = self.values(metric);
        (!values.is_empty() && values.iter().all(|value| value.is_finite())).then(|| {
            values
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), &value| {
                    (min.min(value), max.max(value))
                })
        })
    }

    /// Returns cell indices strictly above a visualization threshold. The
    /// threshold never affects mesh validity or solver readiness.
    pub fn bad_cells(&self, metric: MeshQualityMetric, threshold: f64) -> Vec<usize> {
        if !threshold.is_finite() {
            return Vec::new();
        }
        self.values(metric)
            .iter()
            .enumerate()
            .filter_map(|(index, &value)| (value.is_finite() && value > threshold).then_some(index))
            .collect()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RenderRange {
    pub start: usize,
    pub count: usize,
}

#[derive(Clone, Debug)]
pub struct MeshRenderCache {
    mesh_id: MeshId,
    dimension: MeshDimension,
    positions: Vec<Vec3>,
    edges: Vec<[usize; 2]>,
    surface_triangles: Vec<[usize; 3]>,
    triangle_faces: Vec<usize>,
    face_ranges: Vec<RenderRange>,
    cell_ranges: Vec<RenderRange>,
    cell_polygons: Vec<Vec<(f64, f64)>>,
    face_patch_indices: Vec<Option<usize>>,
    quality: MeshQualityValues,
    bounds: (Vec3, Vec3),
}

impl MeshRenderCache {
    pub fn build(mesh: &UnstructuredMesh) -> Result<Self, String> {
        let positions: Vec<Vec3> = mesh.points().iter().map(|point| point.position).collect();
        if positions.is_empty() || positions.iter().any(|point| !finite(*point)) {
            return Err("mesh render cache requires finite mesh points".to_string());
        }
        let mut face_patch_indices = vec![None; mesh.face_count()];
        for (patch_index, patch) in mesh.boundary_patches().iter().enumerate() {
            for &face in &patch.face_indices {
                if face >= mesh.face_count() {
                    return Err(format!(
                        "boundary patch {} references missing face {face}",
                        patch.name
                    ));
                }
                face_patch_indices[face] = Some(patch_index);
            }
        }
        let mut edges = Vec::new();
        let mut surface_triangles = Vec::new();
        let mut triangle_faces = Vec::new();
        let mut face_ranges = Vec::with_capacity(mesh.face_count());
        for (face_index, face) in mesh.faces().iter().enumerate() {
            let edge_start = edges.len();
            for (&a, &b) in face
                .vertices
                .iter()
                .zip(face.vertices.iter().cycle().skip(1))
                .take(face.vertices.len())
            {
                edges.push([a, b]);
            }
            let triangle_start = surface_triangles.len();
            let visible =
                matches!(mesh.dimension(), MeshDimension::TwoD) || face.neighbour.is_none();
            if visible && face.vertices.len() >= 3 {
                for index in 1..face.vertices.len() - 1 {
                    surface_triangles.push([
                        face.vertices[0],
                        face.vertices[index],
                        face.vertices[index + 1],
                    ]);
                    triangle_faces.push(face_index);
                }
            }
            face_ranges.push(RenderRange {
                start: triangle_start,
                count: surface_triangles.len() - triangle_start,
            });
            debug_assert!(edges.len() >= edge_start);
        }
        let cell_ranges = mesh
            .cells()
            .iter()
            .map(|cell| {
                let start = cell
                    .faces
                    .iter()
                    .filter_map(|&face| face_ranges.get(face))
                    .map(|range| range.start)
                    .min()
                    .unwrap_or(0);
                let count = cell
                    .faces
                    .iter()
                    .filter_map(|&face| face_ranges.get(face))
                    .map(|range| range.count)
                    .sum();
                RenderRange { start, count }
            })
            .collect();
        let cell_polygons = mesh
            .cells()
            .iter()
            .map(|cell| {
                let mut vertices = Vec::new();
                for &face_index in &cell.faces {
                    for &vertex_index in &mesh.faces()[face_index].vertices {
                        let point = positions[vertex_index];
                        if !vertices.contains(&(point.x, point.y)) {
                            vertices.push((point.x, point.y));
                        }
                    }
                }
                let center = cell.center;
                vertices.sort_by(|left, right| {
                    (left.1 - center.y)
                        .atan2(left.0 - center.x)
                        .total_cmp(&(right.1 - center.y).atan2(right.0 - center.x))
                });
                vertices
            })
            .collect();
        let bounds =
            positions
                .iter()
                .copied()
                .fold((positions[0], positions[0]), |(min, max), point| {
                    (
                        Vec3::new(min.x.min(point.x), min.y.min(point.y), min.z.min(point.z)),
                        Vec3::new(max.x.max(point.x), max.y.max(point.y), max.z.max(point.z)),
                    )
                });
        Ok(Self {
            mesh_id: mesh.id(),
            dimension: mesh.dimension(),
            positions,
            edges,
            surface_triangles,
            triangle_faces,
            face_ranges,
            cell_ranges,
            cell_polygons,
            face_patch_indices,
            quality: calculate_quality(mesh),
            bounds,
        })
    }

    pub const fn mesh_id(&self) -> MeshId {
        self.mesh_id
    }
    pub const fn dimension(&self) -> MeshDimension {
        self.dimension
    }
    pub fn positions(&self) -> &[Vec3] {
        &self.positions
    }
    pub fn edges(&self) -> &[[usize; 2]] {
        &self.edges
    }
    pub fn surface_triangles(&self) -> &[[usize; 3]] {
        &self.surface_triangles
    }
    pub fn triangle_faces(&self) -> &[usize] {
        &self.triangle_faces
    }
    pub fn face_ranges(&self) -> &[RenderRange] {
        &self.face_ranges
    }
    pub fn cell_ranges(&self) -> &[RenderRange] {
        &self.cell_ranges
    }
    /// Ordered 2D vertices derived from a cell's incident mesh faces.
    pub fn cell_polygon(&self, index: usize) -> Option<&[(f64, f64)]> {
        self.cell_polygons.get(index).map(Vec::as_slice)
    }
    pub fn face_patch_indices(&self) -> &[Option<usize>] {
        &self.face_patch_indices
    }
    pub fn quality(&self) -> &MeshQualityValues {
        &self.quality
    }
    pub const fn bounds(&self) -> (Vec3, Vec3) {
        self.bounds
    }

    pub fn pick_face_2d(&self, point: (f64, f64), tolerance: f64) -> Option<MeshSelection> {
        if self.dimension != MeshDimension::TwoD || !tolerance.is_finite() || tolerance < 0.0 {
            return None;
        }
        self.edges
            .iter()
            .enumerate()
            .filter_map(|(edge_index, edge)| {
                let face_index = self.edge_face(edge_index)?;
                let a = self.positions[edge[0]];
                let b = self.positions[edge[1]];
                let distance = segment_distance(point, (a.x, a.y), (b.x, b.y));
                (distance <= tolerance).then_some((distance, face_index))
            })
            .min_by(|left, right| left.0.total_cmp(&right.0))
            .map(|(_, index)| MeshSelection::face(self.mesh_id, index))
    }

    pub fn pick_cell_2d(&self, point: (f64, f64)) -> Option<MeshSelection> {
        if self.dimension != MeshDimension::TwoD {
            return None;
        }
        // Lowest index wins on shared boundaries, keeping repeat clicks deterministic.
        self.cell_polygons
            .iter()
            .enumerate()
            .find_map(|(index, polygon)| {
                point_in_polygon(point, polygon).then_some(MeshSelection::cell(self.mesh_id, index))
            })
    }

    pub fn pick_face_3d(&self, origin: Vec3, direction: Vec3) -> Option<MeshSelection> {
        if self.dimension != MeshDimension::ThreeD {
            return None;
        }
        self.surface_triangles
            .iter()
            .enumerate()
            .filter_map(|(index, triangle)| {
                ray_triangle(
                    origin,
                    direction,
                    self.positions[triangle[0]],
                    self.positions[triangle[1]],
                    self.positions[triangle[2]],
                )
                .map(|distance| (distance, self.triangle_faces[index]))
            })
            .min_by(|left, right| left.0.total_cmp(&right.0))
            .map(|(_, face)| MeshSelection::face(self.mesh_id, face))
    }

    fn edge_face(&self, edge_index: usize) -> Option<usize> {
        let mut current = 0;
        for (face, range) in self.face_ranges.iter().enumerate() {
            let vertices = if self.dimension == MeshDimension::TwoD {
                2
            } else {
                0
            };
            if vertices > 0 && edge_index == current {
                return Some(face);
            }
            current += vertices;
            let _ = range;
        }
        None
    }
}

fn calculate_quality(mesh: &UnstructuredMesh) -> MeshQualityValues {
    let aspect_ratio = mesh
        .cells()
        .iter()
        .map(|cell| {
            let mut vertices = Vec::new();
            for &face in &cell.faces {
                for &vertex in &mesh.faces()[face].vertices {
                    if !vertices.contains(&vertex) {
                        vertices.push(vertex);
                    }
                }
            }
            let mut shortest = f64::INFINITY;
            let mut longest = 0.0_f64;
            for (index, &a) in vertices.iter().enumerate() {
                for &b in &vertices[index + 1..] {
                    let d = (mesh.points()[a].position - mesh.points()[b].position).norm();
                    if d > 1.0e-12 {
                        shortest = shortest.min(d);
                        longest = longest.max(d);
                    }
                }
            }
            let ratio = longest / shortest;
            if ratio.is_finite() {
                ratio
            } else {
                1.0
            }
        })
        .collect();
    let mut non_orthogonality = vec![0.0_f64; mesh.cell_count()];
    let mut skewness = vec![0.0_f64; mesh.cell_count()];
    for face in mesh.faces().iter().filter(|face| face.neighbour.is_some()) {
        let neighbour = face.neighbour.expect("filtered");
        let delta = mesh.cells()[neighbour].center - mesh.cells()[face.owner].center;
        let distance = delta.norm();
        if distance <= 1.0e-12 {
            continue;
        }
        let angle = (delta.dot(face.normal) / distance)
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees();
        let projection = mesh.cells()[face.owner].center
            + delta
                * ((face.center - mesh.cells()[face.owner].center).dot(delta) / delta.dot(delta));
        let face_skewness = (face.center - projection).norm() / distance;
        non_orthogonality[face.owner] = non_orthogonality[face.owner].max(angle);
        non_orthogonality[neighbour] = non_orthogonality[neighbour].max(angle);
        skewness[face.owner] = skewness[face.owner].max(face_skewness);
        skewness[neighbour] = skewness[neighbour].max(face_skewness);
    }
    MeshQualityValues {
        aspect_ratio,
        non_orthogonality,
        skewness,
        cell_measure: mesh.cells().iter().map(|cell| cell.volume).collect(),
    }
}

fn finite(point: Vec3) -> bool {
    point.x.is_finite() && point.y.is_finite() && point.z.is_finite()
}
fn segment_distance(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let ab = (b.0 - a.0, b.1 - a.1);
    let length = ab.0 * ab.0 + ab.1 * ab.1;
    let t = if length == 0.0 {
        0.0
    } else {
        ((p.0 - a.0) * ab.0 + (p.1 - a.1) * ab.1) / length
    }
    .clamp(0.0, 1.0);
    (p.0 - (a.0 + t * ab.0)).hypot(p.1 - (a.1 + t * ab.1))
}
fn point_in_polygon(point: (f64, f64), polygon: &[(f64, f64)]) -> bool {
    if polygon.len() < 3 {
        return false;
    }
    polygon
        .iter()
        .zip(polygon.iter().cycle().skip(1))
        .take(polygon.len())
        .fold(false, |inside, (&a, &b)| {
            let crosses = (a.1 > point.1) != (b.1 > point.1)
                && point.0 <= (b.0 - a.0) * (point.1 - a.1) / (b.1 - a.1) + a.0;
            inside ^ crosses
        })
}
fn ray_triangle(origin: Vec3, direction: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Option<f64> {
    let e1 = b - a;
    let e2 = c - a;
    let p = direction.cross(e2);
    let det = e1.dot(p);
    if det.abs() <= 1.0e-12 {
        return None;
    };
    let inv = 1.0 / det;
    let t = origin - a;
    let u = t.dot(p) * inv;
    if !(0.0..=1.0).contains(&u) {
        return None;
    };
    let q = t.cross(e1);
    let v = direction.dot(q) * inv;
    if v < 0.0 || u + v > 1.0 {
        return None;
    };
    let distance = e2.dot(q) * inv;
    (distance >= 0.0).then_some(distance)
}
