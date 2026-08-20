//! Mesh geometry shared by previews, importers, and solver backends.
//!
//! The current flow kernel remains two-dimensional. `ExtrudedMesh3D` is a
//! geometric representation for inspection and pre-processing, not a 3D CFD
//! discretisation.

use serde::{Deserialize, Serialize};
use std::collections::{HashMap, VecDeque};
use std::fmt;
use std::sync::atomic::{AtomicU64, Ordering};

const GEOMETRY_EPSILON: f64 = 1.0e-12;
static NEXT_MESH_ID: AtomicU64 = AtomicU64::new(1);

/// Lightweight identity used to reject accidental field use with another mesh.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct MeshId(u64);

/// Cartesian vector used by both planar and spatial unstructured meshes.
/// Planar meshes store points with `z = 0` and use edge length as face area.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(self, other: Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }

    pub fn norm_squared(self) -> f64 {
        self.dot(self)
    }

    pub fn normalized(self) -> Option<Self> {
        let norm = self.norm();
        (norm > GEOMETRY_EPSILON).then_some(self / norm)
    }
}

impl std::ops::Add for Vec3 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl std::ops::AddAssign for Vec3 {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl std::ops::Sub for Vec3 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl std::ops::Neg for Vec3 {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl std::ops::Mul<f64> for Vec3 {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl std::ops::Div<f64> for Vec3 {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        Self::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point {
    pub position: Vec3,
}

impl Point {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            position: Vec3::new(x, y, z),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeshDimension {
    TwoD,
    ThreeD,
}

/// Input topology for supported convex finite-volume cells.
#[derive(Clone, Debug, PartialEq)]
pub enum CellDefinition {
    Polygon(Vec<usize>),
    Tetrahedron([usize; 4]),
    Hexahedron([usize; 8]),
    Prism([usize; 6]),
    Pyramid([usize; 5]),
    /// Explicit faces for future importer support. Faces must bound one convex cell.
    Polyhedron(Vec<Vec<usize>>),
}

impl CellDefinition {
    pub fn polygon(vertices: Vec<usize>) -> Self {
        Self::Polygon(vertices)
    }

    pub fn tetrahedron(vertices: [usize; 4]) -> Self {
        Self::Tetrahedron(vertices)
    }

    fn faces(&self, dimension: MeshDimension) -> Result<Vec<Vec<usize>>, MeshError> {
        match (dimension, self) {
            (MeshDimension::TwoD, Self::Polygon(vertices)) if vertices.len() >= 3 => Ok(vertices
                .iter()
                .copied()
                .zip(vertices.iter().copied().cycle().skip(1))
                .take(vertices.len())
                .map(|(a, b)| vec![a, b])
                .collect()),
            (MeshDimension::TwoD, Self::Polygon(_)) => Err(MeshError::InvalidCellDefinition(
                "a polygon requires at least three vertices".to_string(),
            )),
            (MeshDimension::ThreeD, Self::Tetrahedron([a, b, c, d])) => Ok(vec![
                vec![*a, *c, *b],
                vec![*a, *b, *d],
                vec![*b, *c, *d],
                vec![*c, *a, *d],
            ]),
            (MeshDimension::ThreeD, Self::Hexahedron([a, b, c, d, e, f, g, h])) => Ok(vec![
                vec![*a, *d, *c, *b],
                vec![*e, *f, *g, *h],
                vec![*a, *b, *f, *e],
                vec![*b, *c, *g, *f],
                vec![*c, *d, *h, *g],
                vec![*d, *a, *e, *h],
            ]),
            (MeshDimension::ThreeD, Self::Prism([a, b, c, d, e, f])) => Ok(vec![
                vec![*a, *c, *b],
                vec![*d, *e, *f],
                vec![*a, *b, *e, *d],
                vec![*b, *c, *f, *e],
                vec![*c, *a, *d, *f],
            ]),
            (MeshDimension::ThreeD, Self::Pyramid([a, b, c, d, e])) => Ok(vec![
                vec![*a, *d, *c, *b],
                vec![*a, *b, *e],
                vec![*b, *c, *e],
                vec![*c, *d, *e],
                vec![*d, *a, *e],
            ]),
            (MeshDimension::ThreeD, Self::Polyhedron(faces)) if faces.len() >= 4 => {
                Ok(faces.clone())
            }
            (MeshDimension::ThreeD, Self::Polyhedron(_)) => Err(MeshError::InvalidCellDefinition(
                "a polyhedron requires at least four faces".to_string(),
            )),
            _ => Err(MeshError::InvalidCellDefinition(
                "cell type is incompatible with mesh dimension".to_string(),
            )),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Face {
    pub vertices: Vec<usize>,
    pub owner: usize,
    pub neighbour: Option<usize>,
    pub center: Vec3,
    pub area: f64,
    pub normal: Vec3,
    /// Oriented face measure, outward from `owner`; length in 2D and area in 3D.
    pub area_vector: Vec3,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Cell {
    pub faces: Vec<usize>,
    pub neighbours: Vec<usize>,
    pub center: Vec3,
    /// Area in 2D and volume in 3D.
    pub volume: f64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BoundaryType {
    VelocityInlet,
    PressureOutlet,
    Wall,
    Symmetry,
    ZeroGradient,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BoundaryPatch {
    pub name: String,
    pub face_indices: Vec<usize>,
    pub boundary_type: BoundaryType,
}

#[derive(Clone, Debug, PartialEq)]
pub struct MeshQualityReport {
    pub min_cell_volume: f64,
    pub max_cell_volume: f64,
    pub max_aspect_ratio: f64,
    pub max_non_orthogonality_degrees: f64,
    pub average_non_orthogonality_degrees: f64,
    pub max_skewness: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct MeshStatistics {
    pub point_count: usize,
    pub face_count: usize,
    pub cell_count: usize,
    pub boundary_patches: Vec<(String, usize)>,
    pub quality: MeshQualityReport,
}

#[derive(Clone, Debug, PartialEq)]
pub struct UnstructuredMesh {
    id: MeshId,
    dimension: MeshDimension,
    points: Vec<Point>,
    faces: Vec<Face>,
    cells: Vec<Cell>,
    boundary_patches: Vec<BoundaryPatch>,
    quality: MeshQualityReport,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MeshError {
    EmptyMesh,
    NonFinitePoint { point: usize },
    InvalidPointIndex { point: usize, cell: usize },
    InvalidCellDefinition(String),
    DegenerateFace { cell: usize },
    ZeroVolumeCell { cell: usize },
    NonManifoldFace { vertices: Vec<usize> },
    InconsistentFaceOrientation { face: usize },
    DisconnectedTopology,
    InvalidBoundaryPatch(String),
}

impl fmt::Display for MeshError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyMesh => write!(formatter, "mesh must contain at least one cell"),
            Self::NonFinitePoint { point } => write!(formatter, "point {point} is not finite"),
            Self::InvalidPointIndex { point, cell } => {
                write!(formatter, "cell {cell} references missing point {point}")
            }
            Self::InvalidCellDefinition(message) => {
                write!(formatter, "invalid cell definition: {message}")
            }
            Self::DegenerateFace { cell } => write!(formatter, "cell {cell} has a zero-area face"),
            Self::ZeroVolumeCell { cell } => {
                write!(formatter, "cell {cell} has zero area or volume")
            }
            Self::NonManifoldFace { vertices } => {
                write!(formatter, "non-manifold face {vertices:?}")
            }
            Self::InconsistentFaceOrientation { face } => {
                write!(
                    formatter,
                    "face {face} has inconsistent owner/neighbour orientation"
                )
            }
            Self::DisconnectedTopology => write!(formatter, "mesh cell topology is disconnected"),
            Self::InvalidBoundaryPatch(message) => {
                write!(formatter, "invalid boundary patch: {message}")
            }
        }
    }
}

impl std::error::Error for MeshError {}

impl UnstructuredMesh {
    /// Builds cached topology and geometry once. No mesh connectivity search is needed in a solver loop.
    pub fn from_cells(
        dimension: MeshDimension,
        points: Vec<Point>,
        definitions: Vec<CellDefinition>,
    ) -> Result<Self, MeshError> {
        if definitions.is_empty() {
            return Err(MeshError::EmptyMesh);
        }
        for (index, point) in points.iter().enumerate() {
            if !point.position.x.is_finite()
                || !point.position.y.is_finite()
                || !point.position.z.is_finite()
            {
                return Err(MeshError::NonFinitePoint { point: index });
            }
        }

        let mut cells = Vec::with_capacity(definitions.len());
        let mut faces: Vec<Face> = Vec::new();
        let mut face_lookup = HashMap::<Vec<usize>, usize>::new();
        let mut references = Vec::with_capacity(definitions.len());

        for (cell_index, definition) in definitions.iter().enumerate() {
            let local_faces = definition.faces(dimension)?;
            let vertices = unique_vertices(&local_faces);
            for &point in &vertices {
                if point >= points.len() {
                    return Err(MeshError::InvalidPointIndex {
                        point,
                        cell: cell_index,
                    });
                }
            }
            let reference = average_point(&points, &vertices);
            let mut cell_faces = Vec::with_capacity(local_faces.len());
            for local_face in local_faces {
                let oriented = oriented_face(dimension, &points, local_face, reference)
                    .ok_or(MeshError::DegenerateFace { cell: cell_index })?;
                let mut key = oriented.vertices.clone();
                key.sort_unstable();
                if let Some(&face_index) = face_lookup.get(&key) {
                    let face = &mut faces[face_index];
                    if face.neighbour.is_some() {
                        return Err(MeshError::NonManifoldFace { vertices: key });
                    }
                    face.neighbour = Some(cell_index);
                    cell_faces.push(face_index);
                } else {
                    let face_index = faces.len();
                    faces.push(Face {
                        vertices: oriented.vertices,
                        owner: cell_index,
                        neighbour: None,
                        center: oriented.center,
                        area: oriented.area,
                        normal: oriented.normal,
                        area_vector: oriented.area_vector,
                    });
                    face_lookup.insert(key, face_index);
                    cell_faces.push(face_index);
                }
            }
            cells.push(Cell {
                faces: cell_faces,
                neighbours: Vec::new(),
                center: reference,
                volume: 0.0,
            });
            references.push(reference);
        }

        for (cell_index, definition) in definitions.iter().enumerate() {
            let local_faces = definition.faces(dimension)?;
            let geometry = cell_geometry(dimension, &points, &local_faces, references[cell_index])
                .ok_or(MeshError::ZeroVolumeCell { cell: cell_index })?;
            cells[cell_index].center = geometry.center;
            cells[cell_index].volume = geometry.volume;
        }

        for (face_index, face) in faces.iter().enumerate() {
            if (face.center - cells[face.owner].center).dot(face.normal) <= GEOMETRY_EPSILON {
                return Err(MeshError::InconsistentFaceOrientation { face: face_index });
            }
            if let Some(neighbour) = face.neighbour {
                if (cells[neighbour].center - face.center).dot(face.normal) <= GEOMETRY_EPSILON {
                    return Err(MeshError::InconsistentFaceOrientation { face: face_index });
                }
                cells[face.owner].neighbours.push(neighbour);
                cells[neighbour].neighbours.push(face.owner);
            }
        }
        for cell in &mut cells {
            cell.neighbours.sort_unstable();
            cell.neighbours.dedup();
        }
        ensure_connected(&cells)?;
        let quality = calculate_quality(&points, &faces, &cells);
        Ok(Self {
            id: MeshId(NEXT_MESH_ID.fetch_add(1, Ordering::Relaxed)),
            dimension,
            points,
            faces,
            cells,
            boundary_patches: Vec::new(),
            quality,
        })
    }

    pub fn dimension(&self) -> MeshDimension {
        self.dimension
    }

    pub fn id(&self) -> MeshId {
        self.id
    }

    pub fn cell_count(&self) -> usize {
        self.cells.len()
    }

    pub fn face_count(&self) -> usize {
        self.faces.len()
    }

    /// Attaches named, solver-facing boundary patches after topology construction.
    /// A boundary face belongs to at most one patch and internal faces are rejected.
    pub fn with_boundary_patches(
        mut self,
        boundary_patches: Vec<BoundaryPatch>,
    ) -> Result<Self, MeshError> {
        let mut assigned = vec![false; self.faces.len()];
        for patch in &boundary_patches {
            if patch.name.trim().is_empty() {
                return Err(MeshError::InvalidBoundaryPatch(
                    "patch name cannot be empty".to_string(),
                ));
            }
            for &face_index in &patch.face_indices {
                let Some(face) = self.faces.get(face_index) else {
                    return Err(MeshError::InvalidBoundaryPatch(format!(
                        "{} references missing face {face_index}",
                        patch.name
                    )));
                };
                if face.neighbour.is_some() {
                    return Err(MeshError::InvalidBoundaryPatch(format!(
                        "{} references internal face {face_index}",
                        patch.name
                    )));
                }
                if std::mem::replace(&mut assigned[face_index], true) {
                    return Err(MeshError::InvalidBoundaryPatch(format!(
                        "boundary face {face_index} is assigned more than once"
                    )));
                }
            }
        }
        self.boundary_patches = boundary_patches;
        Ok(self)
    }

    pub fn points(&self) -> &[Point] {
        &self.points
    }

    pub fn faces(&self) -> &[Face] {
        &self.faces
    }

    pub fn cells(&self) -> &[Cell] {
        &self.cells
    }

    pub fn boundary_patches(&self) -> &[BoundaryPatch] {
        &self.boundary_patches
    }

    pub fn quality(&self) -> &MeshQualityReport {
        &self.quality
    }

    pub fn statistics(&self) -> MeshStatistics {
        MeshStatistics {
            point_count: self.points.len(),
            face_count: self.faces.len(),
            cell_count: self.cells.len(),
            boundary_patches: self
                .boundary_patches
                .iter()
                .map(|patch| (patch.name.clone(), patch.face_indices.len()))
                .collect(),
            quality: self.quality.clone(),
        }
    }
}

struct OrientedFace {
    vertices: Vec<usize>,
    center: Vec3,
    area: f64,
    normal: Vec3,
    area_vector: Vec3,
}

struct CellGeometry {
    center: Vec3,
    volume: f64,
}

fn unique_vertices(faces: &[Vec<usize>]) -> Vec<usize> {
    let mut vertices = faces.iter().flatten().copied().collect::<Vec<_>>();
    vertices.sort_unstable();
    vertices.dedup();
    vertices
}

fn average_point(points: &[Point], vertices: &[usize]) -> Vec3 {
    vertices
        .iter()
        .fold(Vec3::ZERO, |sum, &index| sum + points[index].position)
        / vertices.len() as f64
}

fn oriented_face(
    dimension: MeshDimension,
    points: &[Point],
    mut vertices: Vec<usize>,
    reference: Vec3,
) -> Option<OrientedFace> {
    let mut center = average_point(points, &vertices);
    let mut area_vector = face_area_vector(dimension, points, &vertices)?;
    if area_vector.dot(center - reference) < 0.0 {
        vertices.reverse();
        center = average_point(points, &vertices);
        area_vector = face_area_vector(dimension, points, &vertices)?;
    }
    let area = area_vector.norm();
    Some(OrientedFace {
        vertices,
        center,
        area,
        normal: area_vector / area,
        area_vector,
    })
}

fn face_area_vector(
    dimension: MeshDimension,
    points: &[Point],
    vertices: &[usize],
) -> Option<Vec3> {
    let vector = match dimension {
        MeshDimension::TwoD if vertices.len() == 2 => {
            let edge = points[vertices[1]].position - points[vertices[0]].position;
            Vec3::new(edge.y, -edge.x, 0.0)
        }
        MeshDimension::ThreeD if vertices.len() >= 3 => {
            let mut twice_area = Vec3::ZERO;
            for (&first, &second) in vertices
                .iter()
                .zip(vertices.iter().cycle().skip(1))
                .take(vertices.len())
            {
                twice_area += points[first].position.cross(points[second].position);
            }
            twice_area * 0.5
        }
        _ => return None,
    };
    (vector.norm() > GEOMETRY_EPSILON).then_some(vector)
}

fn cell_geometry(
    dimension: MeshDimension,
    points: &[Point],
    local_faces: &[Vec<usize>],
    reference: Vec3,
) -> Option<CellGeometry> {
    match dimension {
        MeshDimension::TwoD => {
            let vertices = local_faces.iter().map(|face| face[0]).collect::<Vec<_>>();
            let mut twice_area = 0.0;
            let mut center = Vec3::ZERO;
            for (&first, &second) in vertices
                .iter()
                .zip(vertices.iter().cycle().skip(1))
                .take(vertices.len())
            {
                let a = points[first].position;
                let b = points[second].position;
                let cross = a.x * b.y - b.x * a.y;
                twice_area += cross;
                center += (a + b) * cross;
            }
            let signed_area = twice_area * 0.5;
            (signed_area.abs() > GEOMETRY_EPSILON).then_some(CellGeometry {
                center: center / (3.0 * twice_area),
                volume: signed_area.abs(),
            })
        }
        MeshDimension::ThreeD => {
            let mut total_volume = 0.0;
            let mut weighted_center = Vec3::ZERO;
            for face in local_faces {
                let oriented =
                    oriented_face(MeshDimension::ThreeD, points, face.clone(), reference)?;
                let first = points[oriented.vertices[0]].position;
                for pair in oriented.vertices[1..].windows(2) {
                    let second = points[pair[0]].position;
                    let third = points[pair[1]].position;
                    let volume = (first - reference)
                        .dot((second - reference).cross(third - reference))
                        / 6.0;
                    total_volume += volume;
                    weighted_center += (reference + first + second + third) * (volume / 4.0);
                }
            }
            (total_volume > GEOMETRY_EPSILON).then_some(CellGeometry {
                center: weighted_center / total_volume,
                volume: total_volume,
            })
        }
    }
}

fn ensure_connected(cells: &[Cell]) -> Result<(), MeshError> {
    if cells.len() <= 1 {
        return Ok(());
    }
    let mut visited = vec![false; cells.len()];
    let mut pending = VecDeque::from([0_usize]);
    visited[0] = true;
    while let Some(cell) = pending.pop_front() {
        for &neighbour in &cells[cell].neighbours {
            if !visited[neighbour] {
                visited[neighbour] = true;
                pending.push_back(neighbour);
            }
        }
    }
    visited
        .into_iter()
        .all(|seen| seen)
        .then_some(())
        .ok_or(MeshError::DisconnectedTopology)
}

fn calculate_quality(points: &[Point], faces: &[Face], cells: &[Cell]) -> MeshQualityReport {
    let min_cell_volume = cells
        .iter()
        .map(|cell| cell.volume)
        .fold(f64::INFINITY, f64::min);
    let max_cell_volume = cells.iter().map(|cell| cell.volume).fold(0.0_f64, f64::max);
    let max_aspect_ratio = cells
        .iter()
        .map(|cell| cell_aspect_ratio(points, cell, faces))
        .fold(1.0_f64, f64::max);
    let mut maximum_non_orthogonality = 0.0_f64;
    let mut non_orthogonality_sum = 0.0;
    let mut count = 0_usize;
    let mut max_skewness = 0.0_f64;
    for face in faces.iter().filter(|face| face.neighbour.is_some()) {
        let neighbour = face.neighbour.expect("filtered to internal faces");
        let delta = cells[neighbour].center - cells[face.owner].center;
        let distance = delta.norm();
        let cosine = (delta.dot(face.normal) / distance).clamp(-1.0, 1.0);
        let angle = cosine.acos().to_degrees();
        maximum_non_orthogonality = maximum_non_orthogonality.max(angle);
        non_orthogonality_sum += angle;
        count += 1;
        let projection = cells[face.owner].center
            + delta * ((face.center - cells[face.owner].center).dot(delta) / delta.dot(delta));
        max_skewness = max_skewness.max((face.center - projection).norm() / distance);
    }
    MeshQualityReport {
        min_cell_volume,
        max_cell_volume,
        max_aspect_ratio,
        max_non_orthogonality_degrees: maximum_non_orthogonality,
        average_non_orthogonality_degrees: if count > 0 {
            non_orthogonality_sum / count as f64
        } else {
            0.0
        },
        max_skewness,
    }
}

fn cell_aspect_ratio(points: &[Point], cell: &Cell, faces: &[Face]) -> f64 {
    let vertices = unique_vertices(
        &cell
            .faces
            .iter()
            .map(|&face| faces[face].vertices.clone())
            .collect::<Vec<_>>(),
    );
    let mut shortest = f64::INFINITY;
    let mut longest = 0.0_f64;
    for (index, &first) in vertices.iter().enumerate() {
        for &second in &vertices[index + 1..] {
            let distance = (points[second].position - points[first].position).norm();
            if distance > GEOMETRY_EPSILON {
                shortest = shortest.min(distance);
                longest = longest.max(distance);
            }
        }
    }
    longest / shortest
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StructuredMesh2D {
    pub nx: usize,
    pub ny: usize,
    pub length: f64,
    pub height: f64,
    pub dx: f64,
    pub dy: f64,
}

impl StructuredMesh2D {
    pub fn new(nx: usize, ny: usize, length: f64, height: f64) -> Result<Self, String> {
        if nx < 1 || ny < 1 {
            return Err("structured mesh dimensions must be positive".to_string());
        }
        if !length.is_finite() || !height.is_finite() || length <= 0.0 || height <= 0.0 {
            return Err(
                "structured mesh domain dimensions must be finite and positive".to_string(),
            );
        }
        Ok(Self {
            nx,
            ny,
            length,
            height,
            dx: length / nx as f64,
            dy: height / ny as f64,
        })
    }

    #[inline]
    pub fn node(&self, i: usize, j: usize) -> (f64, f64) {
        debug_assert!(i <= self.nx && j <= self.ny);
        (i as f64 * self.dx, j as f64 * self.dy)
    }

    #[inline]
    pub fn cell_center(&self, i: usize, j: usize) -> (f64, f64) {
        debug_assert!(i < self.nx && j < self.ny);
        ((i as f64 + 0.5) * self.dx, (j as f64 + 0.5) * self.dy)
    }

    pub fn cell_count(&self) -> usize {
        self.nx * self.ny
    }

    pub fn node_count(&self) -> usize {
        (self.nx + 1) * (self.ny + 1)
    }

    pub fn aspect_ratio(&self) -> f64 {
        self.dx.max(self.dy) / self.dx.min(self.dy)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExtrudedMesh3D {
    pub base: StructuredMesh2D,
    pub nz: usize,
    pub depth: f64,
    pub dz: f64,
}

impl ExtrudedMesh3D {
    pub fn new(base: StructuredMesh2D, nz: usize, depth: f64) -> Result<Self, String> {
        if nz < 1 {
            return Err("extruded mesh layer count must be positive".to_string());
        }
        if !depth.is_finite() || depth <= 0.0 {
            return Err("extruded mesh depth must be finite and positive".to_string());
        }
        Ok(Self {
            base,
            nz,
            depth,
            dz: depth / nz as f64,
        })
    }

    #[inline]
    pub fn node(&self, i: usize, j: usize, k: usize) -> (f64, f64, f64) {
        let (x, y) = self.base.node(i, j);
        (x, y, k as f64 * self.dz)
    }

    pub fn cell_count(&self) -> usize {
        self.base.cell_count() * self.nz
    }

    pub fn node_count(&self) -> usize {
        self.base.node_count() * (self.nz + 1)
    }

    pub fn cell_volume(&self) -> f64 {
        self.base.dx * self.base.dy * self.dz
    }

    pub fn aspect_ratio(&self) -> f64 {
        let min_spacing = self.base.dx.min(self.base.dy).min(self.dz);
        let max_spacing = self.base.dx.max(self.base.dy).max(self.dz);
        max_spacing / min_spacing
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::StructuredMeshArtifact;

    #[test]
    fn triangular_cell_has_outward_faces_and_conservative_area_vectors() {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(2.0, 0.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![CellDefinition::polygon(vec![0, 1, 2])],
        )
        .unwrap();

        assert!((mesh.cells()[0].volume - 1.0).abs() < 1.0e-12);
        assert_eq!(mesh.cells()[0].faces.len(), 3);
        let closure = mesh.cells()[0]
            .faces
            .iter()
            .map(|&face| mesh.faces()[face].area_vector)
            .fold(Vec3::ZERO, |sum, area_vector| sum + area_vector);
        assert!(closure.norm() < 1.0e-12);
        for face in mesh.faces() {
            assert!((face.normal.norm() - 1.0).abs() < 1.0e-12);
            assert!((face.center - mesh.cells()[face.owner].center).dot(face.normal) > 0.0);
            assert!(face.neighbour.is_none());
        }
    }

    #[test]
    fn tetrahedron_has_exact_volume_and_owner_oriented_normals() {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::ThreeD,
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
                Point::new(0.0, 0.0, 1.0),
            ],
            vec![CellDefinition::tetrahedron([0, 1, 2, 3])],
        )
        .unwrap();

        assert!((mesh.cells()[0].volume - 1.0 / 6.0).abs() < 1.0e-12);
        assert_eq!(mesh.cells()[0].faces.len(), 4);
        assert!(
            mesh.cells()[0]
                .faces
                .iter()
                .map(|&face| mesh.faces()[face].area_vector)
                .fold(Vec3::ZERO, |sum, area_vector| sum + area_vector)
                .norm()
                < 1.0e-12
        );
    }

    #[test]
    fn hexahedron_has_unit_volume_and_closed_surface_vectors() {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::ThreeD,
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
                Point::new(0.0, 0.0, 1.0),
                Point::new(1.0, 0.0, 1.0),
                Point::new(1.0, 1.0, 1.0),
                Point::new(0.0, 1.0, 1.0),
            ],
            vec![CellDefinition::Hexahedron([0, 1, 2, 3, 4, 5, 6, 7])],
        )
        .unwrap();

        assert!((mesh.cells()[0].volume - 1.0).abs() < 1.0e-12);
        assert_eq!(mesh.cells()[0].faces.len(), 6);
        assert!(
            mesh.cells()[0]
                .faces
                .iter()
                .map(|&face| mesh.faces()[face].area_vector)
                .fold(Vec3::ZERO, |sum, area_vector| sum + area_vector)
                .norm()
                < 1.0e-12
        );
    }

    #[test]
    fn shared_face_is_stored_once_with_owner_neighbour_connectivity() {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![
                CellDefinition::polygon(vec![0, 1, 2]),
                CellDefinition::polygon(vec![0, 2, 3]),
            ],
        )
        .unwrap();

        let shared = mesh
            .faces()
            .iter()
            .find(|face| face.neighbour.is_some())
            .unwrap();
        assert_eq!(mesh.faces().len(), 5);
        assert_eq!(shared.owner, 0);
        assert_eq!(shared.neighbour, Some(1));
        assert_eq!(mesh.cells()[0].neighbours, vec![1]);
        assert_eq!(mesh.cells()[1].neighbours, vec![0]);
    }

    #[test]
    fn named_patches_can_own_only_unique_boundary_faces() {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![CellDefinition::polygon(vec![0, 1, 2])],
        )
        .unwrap()
        .with_boundary_patches(vec![BoundaryPatch {
            name: "inlet".to_string(),
            face_indices: vec![0],
            boundary_type: BoundaryType::VelocityInlet,
        }])
        .unwrap();

        assert_eq!(mesh.boundary_patches()[0].name, "inlet");
        assert_eq!(
            mesh.statistics().boundary_patches,
            vec![("inlet".to_string(), 1)]
        );
    }

    #[test]
    fn structured_mesh_coordinates_follow_the_domain() {
        let mesh = StructuredMesh2D::new(4, 2, 8.0, 1.0).unwrap();
        assert_eq!(mesh.node(4, 2), (8.0, 1.0));
        assert_eq!(mesh.cell_center(1, 1), (3.0, 0.75));
    }

    #[test]
    fn extrusion_uses_actual_depth_and_layer_count() {
        let base = StructuredMesh2D::new(2, 3, 2.0, 3.0).unwrap();
        let mesh = ExtrudedMesh3D::new(base, 4, 0.8).unwrap();
        assert_eq!(mesh.node(2, 3, 4), (2.0, 3.0, 0.8));
        assert_eq!(mesh.cell_count(), 24);
        assert_eq!(mesh.node_count(), 60);
        assert!((mesh.cell_volume() - 0.2).abs() < 1.0e-12);
    }

    #[test]
    fn structured_mesh_artifact_reports_quality_and_provenance() {
        let mesh =
            ExtrudedMesh3D::new(StructuredMesh2D::new(4, 2, 2.0, 1.0).unwrap(), 5, 0.5).unwrap();
        let artifact = StructuredMeshArtifact::from_extruded(mesh, "geometry-rev-7").unwrap();
        assert_eq!(artifact.cell_count(), 40);
        assert_eq!(artifact.node_count(), 90);
        assert_eq!(artifact.source_revision, "geometry-rev-7");
        assert!((artifact.quality.cell_volume_min - 0.025).abs() < 1.0e-12);
        assert!(artifact.quality.max_aspect_ratio >= 1.0);
    }
}
