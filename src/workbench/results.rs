//! Read-only, run-aware CFD post-processing data and strict FLURSYS VTK loading.
//!
//! Only the legacy ASCII unstructured-grid dialect emitted by
//! `write_unstructured_legacy_vtk` is accepted. Result topology is owned by the
//! dataset, so archived runs never borrow or depend on the current workbench mesh.

use super::MeshRenderCache;
use crate::{CellDefinition, MeshDimension, Point, UnstructuredMesh, Vec3};
use std::path::Path;

#[path = "streamlines.rs"]
pub mod streamlines;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResultFieldKind {
    Pressure,
    VelocityMagnitude,
}

impl ResultFieldKind {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Pressure => "Pressure",
            Self::VelocityMagnitude => "Velocity Magnitude",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ResultProbe {
    pub run_id: String,
    pub cell_index: usize,
    pub center: Vec3,
    pub pressure: f64,
    pub velocity: Vec3,
    pub velocity_magnitude: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ResultDataset {
    run_id: String,
    mesh: UnstructuredMesh,
    pressure: Vec<f64>,
    velocity: Vec<Vec3>,
    velocity_magnitude: Vec<f64>,
}

impl ResultDataset {
    pub fn from_values(
        run_id: impl Into<String>,
        mesh: &UnstructuredMesh,
        pressure: Vec<f64>,
        velocity: Vec<Vec3>,
    ) -> Result<Self, ResultDataError> {
        let run_id = run_id.into();
        if run_id.trim().is_empty() {
            return Err(ResultDataError::Invalid("run ID must be non-empty".into()));
        }
        if pressure.len() != mesh.cell_count() || velocity.len() != mesh.cell_count() {
            return Err(ResultDataError::CountMismatch {
                cells: mesh.cell_count(),
                pressure: pressure.len(),
                velocity: velocity.len(),
            });
        }
        if pressure.iter().any(|value| !value.is_finite()) {
            return Err(ResultDataError::Invalid(
                "pressure contains a non-finite value".into(),
            ));
        }
        if velocity.iter().any(|value| !finite_vec(*value)) {
            return Err(ResultDataError::Invalid(
                "velocity contains a non-finite vector".into(),
            ));
        }
        let velocity_magnitude: Vec<f64> = velocity.iter().map(|value| value.norm()).collect();
        if velocity_magnitude.iter().any(|value| !value.is_finite()) {
            return Err(ResultDataError::Invalid(
                "velocity magnitude is non-finite".into(),
            ));
        }
        Ok(Self {
            run_id,
            mesh: mesh.clone(),
            pressure,
            velocity,
            velocity_magnitude,
        })
    }

    pub fn run_id(&self) -> &str {
        &self.run_id
    }
    pub fn mesh(&self) -> &UnstructuredMesh {
        &self.mesh
    }
    pub fn pressure(&self) -> &[f64] {
        &self.pressure
    }
    pub fn velocity(&self) -> &[Vec3] {
        &self.velocity
    }
    pub fn velocity_magnitude(&self) -> &[f64] {
        &self.velocity_magnitude
    }

    pub fn scalar_values(&self, field: ResultFieldKind) -> &[f64] {
        match field {
            ResultFieldKind::Pressure => &self.pressure,
            ResultFieldKind::VelocityMagnitude => &self.velocity_magnitude,
        }
    }

    pub fn scalar_range(&self, field: ResultFieldKind) -> Result<(f64, f64), ResultDataError> {
        let values = self.scalar_values(field);
        if values.is_empty() || values.iter().any(|value| !value.is_finite()) {
            return Err(ResultDataError::Invalid(format!(
                "{} has no finite values",
                field.label()
            )));
        }
        Ok(values
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), value| {
                (min.min(*value), max.max(*value))
            }))
    }

    pub fn probe_cell(&self, cell_index: usize) -> Option<ResultProbe> {
        self.mesh.cells().get(cell_index).map(|cell| ResultProbe {
            run_id: self.run_id.clone(),
            cell_index,
            center: cell.center,
            pressure: self.pressure[cell_index],
            velocity: self.velocity[cell_index],
            velocity_magnitude: self.velocity_magnitude[cell_index],
        })
    }
}

/// Disposable display data for exactly one archived or live result dataset.
/// It is replaced as a whole when the active run changes.
#[derive(Clone, Debug)]
pub struct ResultRenderCache {
    run_id: String,
    artifact_fingerprint: u64,
    mesh_cache: MeshRenderCache,
    pressure: Vec<f64>,
    velocity: Vec<Vec3>,
    velocity_magnitude: Vec<f64>,
    face_owners: Vec<usize>,
}

impl ResultRenderCache {
    pub fn build(dataset: &ResultDataset) -> Result<Self, ResultDataError> {
        let mesh_cache = MeshRenderCache::build(dataset.mesh()).map_err(|error| {
            ResultDataError::Invalid(format!("cannot build result render cache: {error}"))
        })?;
        Ok(Self {
            run_id: dataset.run_id.clone(),
            artifact_fingerprint: result_artifact_fingerprint(dataset),
            mesh_cache,
            pressure: dataset.pressure.clone(),
            velocity: dataset.velocity.clone(),
            velocity_magnitude: dataset.velocity_magnitude.clone(),
            face_owners: dataset.mesh.faces().iter().map(|face| face.owner).collect(),
        })
    }

    pub fn run_id(&self) -> &str {
        &self.run_id
    }

    /// Deterministic identity of the immutable archived result content used to
    /// build this cache. It prevents a same-named revised artifact from being
    /// treated as the prior cached result.
    pub const fn artifact_fingerprint(&self) -> u64 {
        self.artifact_fingerprint
    }

    pub fn mesh_cache(&self) -> &MeshRenderCache {
        &self.mesh_cache
    }

    pub fn cell_count(&self) -> usize {
        self.pressure.len()
    }

    pub fn scalar_for_cell(&self, field: ResultFieldKind, cell: usize) -> Option<f64> {
        match field {
            ResultFieldKind::Pressure => self.pressure.get(cell).copied(),
            ResultFieldKind::VelocityMagnitude => self.velocity_magnitude.get(cell).copied(),
        }
    }

    pub fn velocity_for_cell(&self, cell: usize) -> Option<Vec3> {
        self.velocity.get(cell).copied()
    }

    pub fn owner_for_face(&self, face: usize) -> Option<usize> {
        self.face_owners.get(face).copied()
    }

    /// Exterior triangles map through their source face to the owner cell.
    pub fn scalar_for_face(&self, field: ResultFieldKind, face: usize) -> Option<f64> {
        self.face_owners
            .get(face)
            .and_then(|&owner| self.scalar_for_cell(field, owner))
    }
}

fn result_artifact_fingerprint(dataset: &ResultDataset) -> u64 {
    const OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
    let mut hash = OFFSET;
    for byte in dataset.run_id.as_bytes() {
        hash = mix_fingerprint(hash, u64::from(*byte));
    }
    hash = mix_fingerprint(
        hash,
        match dataset.mesh.dimension() {
            MeshDimension::TwoD => 2,
            MeshDimension::ThreeD => 3,
        },
    );
    for point in dataset.mesh.points() {
        for value in [point.position.x, point.position.y, point.position.z] {
            hash = mix_fingerprint(hash, value.to_bits());
        }
    }
    for face in dataset.mesh.faces() {
        hash = mix_fingerprint(hash, face.owner as u64);
        hash = mix_fingerprint(hash, face.neighbour.map_or(u64::MAX, |cell| cell as u64));
        for &vertex in &face.vertices {
            hash = mix_fingerprint(hash, vertex as u64);
        }
    }
    for (&pressure, velocity) in dataset.pressure.iter().zip(&dataset.velocity) {
        hash = mix_fingerprint(hash, pressure.to_bits());
        for value in [velocity.x, velocity.y, velocity.z] {
            hash = mix_fingerprint(hash, value.to_bits());
        }
    }
    hash
}

fn mix_fingerprint(hash: u64, value: u64) -> u64 {
    (hash ^ value).wrapping_mul(0x0000_0100_0000_01b3)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ResultDataError {
    Io(String),
    Unsupported(String),
    Invalid(String),
    CountMismatch {
        cells: usize,
        pressure: usize,
        velocity: usize,
    },
}

impl std::fmt::Display for ResultDataError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(error) => write!(f, "result I/O error: {error}"),
            Self::Unsupported(error) => write!(f, "unsupported FLURSYS result artifact: {error}"),
            Self::Invalid(error) => write!(f, "invalid FLURSYS result artifact: {error}"),
            Self::CountMismatch { cells, pressure, velocity } => write!(f, "result field counts do not match {cells} cells (pressure {pressure}, velocity {velocity})"),
        }
    }
}
impl std::error::Error for ResultDataError {}

pub fn load_legacy_vtk_result(
    path: &Path,
    run_id: impl Into<String>,
) -> Result<ResultDataset, ResultDataError> {
    let text = std::fs::read_to_string(path)
        .map_err(|error| ResultDataError::Io(format!("cannot read {}: {error}", path.display())))?;
    parse_legacy_vtk_result(&text, run_id)
}

pub fn parse_legacy_vtk_result(
    text: &str,
    run_id: impl Into<String>,
) -> Result<ResultDataset, ResultDataError> {
    let mut lines = text.lines().map(str::trim).filter(|line| !line.is_empty());
    if lines.next() != Some("# vtk DataFile Version 3.0")
        || lines.next().is_none()
        || lines.next() != Some("ASCII")
        || lines.next() != Some("DATASET UNSTRUCTURED_GRID")
    {
        return Err(ResultDataError::Unsupported(
            "expected FLURSYS legacy ASCII UNSTRUCTURED_GRID".into(),
        ));
    }
    let points_header = lines
        .next()
        .ok_or_else(|| ResultDataError::Invalid("missing POINTS".into()))?;
    let points_count = header_count(points_header, "POINTS")?;
    let mut points = Vec::with_capacity(points_count);
    for _ in 0..points_count {
        points.push(parse_point(lines.next())?);
    }
    let cells_header = lines
        .next()
        .ok_or_else(|| ResultDataError::Invalid("missing CELLS".into()))?;
    let cells_count = header_count(cells_header, "CELLS")?;
    let mut cells = Vec::with_capacity(cells_count);
    for _ in 0..cells_count {
        cells.push(parse_cell(lines.next())?);
    }
    let types_header = lines
        .next()
        .ok_or_else(|| ResultDataError::Invalid("missing CELL_TYPES".into()))?;
    let types_count = header_count(types_header, "CELL_TYPES")?;
    if types_count != cells_count {
        return Err(ResultDataError::Invalid(
            "CELL_TYPES count differs from CELLS".into(),
        ));
    }
    let mut definitions = Vec::with_capacity(cells_count);
    let mut dimension = MeshDimension::TwoD;
    for vertices in cells {
        let cell_type = parse_usize(lines.next(), "cell type")?;
        match cell_type {
            7 => definitions.push(CellDefinition::polygon(vertices)),
            10 => {
                dimension = MeshDimension::ThreeD;
                definitions.push(CellDefinition::tetrahedron(vertices.try_into().map_err(
                    |_| ResultDataError::Unsupported("tetrahedron must have four vertices".into()),
                )?));
            }
            _ => {
                return Err(ResultDataError::Unsupported(format!(
                    "cell type {cell_type}"
                )))
            }
        }
    }
    if dimension == MeshDimension::ThreeD
        && definitions
            .iter()
            .any(|definition| !matches!(definition, CellDefinition::Tetrahedron(_)))
    {
        return Err(ResultDataError::Unsupported("mixed 2D/3D VTK cells".into()));
    }
    let mesh = UnstructuredMesh::from_cells(dimension, points, definitions)
        .map_err(|error| ResultDataError::Invalid(error.to_string()))?;
    let data_header = lines
        .next()
        .ok_or_else(|| ResultDataError::Invalid("missing CELL_DATA".into()))?;
    if header_count(data_header, "CELL_DATA")? != mesh.cell_count() {
        return Err(ResultDataError::Invalid(
            "CELL_DATA count differs from mesh cells".into(),
        ));
    }
    let pressure = parse_scalar(&mut lines, "pressure", mesh.cell_count())?;
    let _speed = parse_scalar(&mut lines, "velocity_magnitude", mesh.cell_count())?;
    let vector_header = lines
        .next()
        .ok_or_else(|| ResultDataError::Invalid("missing velocity field".into()))?;
    if vector_header != "VECTORS velocity double" {
        return Err(ResultDataError::Unsupported(format!(
            "expected VECTORS velocity double, got {vector_header:?}"
        )));
    }
    let mut velocity = Vec::with_capacity(mesh.cell_count());
    for _ in 0..mesh.cell_count() {
        velocity.push(parse_vec(lines.next())?);
    }
    if lines.next().is_some() {
        return Err(ResultDataError::Unsupported(
            "unexpected trailing VTK field or data".into(),
        ));
    }
    ResultDataset::from_values(run_id, &mesh, pressure, velocity)
}

fn header_count(line: &str, keyword: &str) -> Result<usize, ResultDataError> {
    let mut words = line.split_whitespace();
    if words.next() != Some(keyword) {
        return Err(ResultDataError::Invalid(format!("expected {keyword}")));
    }
    words
        .next()
        .ok_or_else(|| ResultDataError::Invalid(format!("missing {keyword} count")))?
        .parse()
        .map_err(|_| ResultDataError::Invalid(format!("invalid {keyword} count")))
}
fn parse_point(line: Option<&str>) -> Result<Point, ResultDataError> {
    let value = parse_vec(line)?;
    Ok(Point::new(value.x, value.y, value.z))
}
fn parse_vec(line: Option<&str>) -> Result<Vec3, ResultDataError> {
    let line = line.ok_or_else(|| ResultDataError::Invalid("unexpected end of VTK".into()))?;
    let values: Vec<f64> = line
        .split_whitespace()
        .map(|word| {
            word.parse::<f64>()
                .map_err(|_| ResultDataError::Invalid("invalid floating value".into()))
        })
        .collect::<Result<_, _>>()?;
    if values.len() != 3 || values.iter().any(|value| !value.is_finite()) {
        return Err(ResultDataError::Invalid(
            "expected three finite values".into(),
        ));
    }
    Ok(Vec3::new(values[0], values[1], values[2]))
}
fn parse_cell(line: Option<&str>) -> Result<Vec<usize>, ResultDataError> {
    let line =
        line.ok_or_else(|| ResultDataError::Invalid("unexpected end of VTK cells".into()))?;
    let values: Vec<usize> = line
        .split_whitespace()
        .map(|word| {
            word.parse()
                .map_err(|_| ResultDataError::Invalid("invalid cell index".into()))
        })
        .collect::<Result<_, _>>()?;
    let count = *values
        .first()
        .ok_or_else(|| ResultDataError::Invalid("empty cell".into()))?;
    if count < 3 || values.len() != count + 1 {
        return Err(ResultDataError::Invalid("invalid cell connectivity".into()));
    }
    Ok(values[1..].to_vec())
}
fn parse_usize(line: Option<&str>, label: &str) -> Result<usize, ResultDataError> {
    line.ok_or_else(|| ResultDataError::Invalid(format!("missing {label}")))?
        .parse()
        .map_err(|_| ResultDataError::Invalid(format!("invalid {label}")))
}
fn parse_scalar<'a>(
    lines: &mut impl Iterator<Item = &'a str>,
    name: &str,
    count: usize,
) -> Result<Vec<f64>, ResultDataError> {
    let header = lines
        .next()
        .ok_or_else(|| ResultDataError::Invalid(format!("missing {name}")))?;
    if header != format!("SCALARS {name} double 1") || lines.next() != Some("LOOKUP_TABLE default")
    {
        return Err(ResultDataError::Unsupported(format!(
            "expected scalar {name}"
        )));
    }
    let mut values = Vec::with_capacity(count);
    for _ in 0..count {
        let value: f64 = lines
            .next()
            .ok_or_else(|| ResultDataError::Invalid(format!("missing {name} value")))?
            .parse()
            .map_err(|_| ResultDataError::Invalid(format!("invalid {name} value")))?;
        if !value.is_finite() {
            return Err(ResultDataError::Invalid(format!("non-finite {name} value")));
        }
        values.push(value);
    }
    Ok(values)
}
fn finite_vec(value: Vec3) -> bool {
    value.x.is_finite() && value.y.is_finite() && value.z.is_finite()
}

#[cfg(test)]
mod tests {
    use super::streamlines::{StreamlineDirection, StreamlineField, StreamlineOptions};
    use super::*;
    use crate::{CellDefinition, MeshDimension, Point, UnstructuredMesh};
    fn mesh() -> UnstructuredMesh {
        UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![CellDefinition::polygon(vec![0, 1, 2])],
        )
        .unwrap()
    }

    fn square_mesh() -> UnstructuredMesh {
        UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![CellDefinition::polygon(vec![0, 1, 2, 3])],
        )
        .unwrap()
    }

    fn two_cell_channel_mesh() -> UnstructuredMesh {
        UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(2.0, 0.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(2.0, 1.0, 0.0),
            ],
            vec![
                CellDefinition::polygon(vec![0, 1, 4, 3]),
                CellDefinition::polygon(vec![1, 2, 5, 4]),
            ],
        )
        .unwrap()
    }
    #[test]
    fn dataset_from_live_solution_preserves_cell_values_and_derived_speed() {
        let mesh = mesh();
        let dataset = ResultDataset::from_values(
            "run-0001",
            &mesh,
            vec![2.0],
            vec![Vec3::new(3.0, 4.0, 0.0)],
        )
        .unwrap();
        assert_eq!(dataset.velocity_magnitude(), &[5.0]);
        assert_eq!(
            dataset.scalar_range(ResultFieldKind::Pressure).unwrap(),
            (2.0, 2.0)
        );
    }

    #[test]
    fn scalar_ranges_preserve_constant_and_negative_cell_values() {
        let mesh = mesh();
        let constant = ResultDataset::from_values(
            "constant",
            &mesh,
            vec![4.0],
            vec![Vec3::new(3.0, 4.0, 0.0)],
        )
        .unwrap();
        let negative = ResultDataset::from_values(
            "negative",
            &mesh,
            vec![-7.5],
            vec![Vec3::new(0.0, 0.0, 0.0)],
        )
        .unwrap();

        assert_eq!(
            constant
                .scalar_range(ResultFieldKind::VelocityMagnitude)
                .unwrap(),
            (5.0, 5.0)
        );
        assert_eq!(
            negative.scalar_range(ResultFieldKind::Pressure).unwrap(),
            (-7.5, -7.5)
        );
    }

    #[test]
    fn dataset_rejects_nonfinite_and_count_mismatch_fields() {
        let mesh = mesh();
        assert!(ResultDataset::from_values("run", &mesh, vec![1.0, 2.0], vec![]).is_err());
        assert!(ResultDataset::from_values(
            "run",
            &mesh,
            vec![f64::NAN],
            vec![Vec3::new(0.0, 0.0, 0.0)]
        )
        .is_err());
    }

    #[test]
    fn loader_rejects_unrecognized_trailing_vtk_fields() {
        let vtk = "# vtk DataFile Version 3.0\nFLURSYS result\nASCII\nDATASET UNSTRUCTURED_GRID\nPOINTS 3 double\n0 0 0\n1 0 0\n0 1 0\nCELLS 1 4\n3 0 1 2\nCELL_TYPES 1\n7\nCELL_DATA 1\nSCALARS pressure double 1\nLOOKUP_TABLE default\n1\nSCALARS velocity_magnitude double 1\nLOOKUP_TABLE default\n2\nVECTORS velocity double\n1 0 0\nSCALARS temperature double 1\nLOOKUP_TABLE default\n300\n";

        assert!(matches!(
            parse_legacy_vtk_result(vtk, "run-0001"),
            Err(ResultDataError::Unsupported(message)) if message.contains("unexpected trailing")
        ));
    }
    #[test]
    fn probe_is_bound_to_its_dataset_run() {
        let mesh = mesh();
        let dataset = ResultDataset::from_values(
            "run-0001",
            &mesh,
            vec![2.0],
            vec![Vec3::new(3.0, 4.0, 0.0)],
        )
        .unwrap();
        let probe = dataset.probe_cell(0).unwrap();
        assert_eq!(probe.run_id, "run-0001");
        assert_eq!(probe.velocity_magnitude, 5.0);
    }

    #[test]
    fn render_cache_maps_exterior_faces_to_their_owner_cell_values() {
        let mesh = mesh();
        let dataset = ResultDataset::from_values(
            "run-0001",
            &mesh,
            vec![2.0],
            vec![Vec3::new(3.0, 4.0, 0.0)],
        )
        .unwrap();

        let cache = ResultRenderCache::build(&dataset).unwrap();

        assert_eq!(cache.run_id(), "run-0001");
        assert_eq!(
            cache.scalar_for_cell(ResultFieldKind::Pressure, 0),
            Some(2.0)
        );
        assert_eq!(
            cache.scalar_for_face(ResultFieldKind::VelocityMagnitude, 0),
            Some(5.0)
        );
        assert_eq!(cache.velocity_for_cell(0), Some(Vec3::new(3.0, 4.0, 0.0)));
        assert_eq!(cache.owner_for_face(0), Some(0));
    }

    #[test]
    fn render_cache_fingerprint_distinguishes_revisions_of_the_same_run() {
        let mesh = mesh();
        let first = ResultDataset::from_values(
            "run-0001",
            &mesh,
            vec![1.0],
            vec![Vec3::new(1.0, 0.0, 0.0)],
        )
        .unwrap();
        let revised = ResultDataset::from_values(
            "run-0001",
            &mesh,
            vec![2.0],
            vec![Vec3::new(1.0, 0.0, 0.0)],
        )
        .unwrap();

        assert_ne!(
            ResultRenderCache::build(&first)
                .unwrap()
                .artifact_fingerprint(),
            ResultRenderCache::build(&revised)
                .unwrap()
                .artifact_fingerprint()
        );
    }

    #[test]
    fn streamline_velocity_is_continuously_blended_between_neighboring_cells() {
        let mesh = two_cell_channel_mesh();
        let dataset = ResultDataset::from_values(
            "run-0001",
            &mesh,
            vec![0.0, 0.0],
            vec![Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)],
        )
        .unwrap();
        let field = StreamlineField::from_dataset(&dataset).unwrap();

        let velocity = field.velocity_at(Vec3::new(1.0, 0.5, 0.0)).unwrap();

        assert!((velocity.x - 0.5).abs() < 1.0e-10);
        assert!((velocity.y - 0.5).abs() < 1.0e-10);
    }

    #[test]
    fn rk4_streamline_for_constant_horizontal_velocity_stays_horizontal() {
        let mesh = square_mesh();
        let dataset = ResultDataset::from_values(
            "run-0001",
            &mesh,
            vec![0.0],
            vec![Vec3::new(1.0, 0.0, 0.0)],
        )
        .unwrap();
        let field = StreamlineField::from_dataset(&dataset).unwrap();
        let path = field
            .integrate(
                Vec3::new(0.1, 0.5, 0.0),
                StreamlineDirection::Forward,
                StreamlineOptions {
                    step_size: 0.1,
                    max_steps: 32,
                    max_length: 2.0,
                    stagnation_speed: 1.0e-12,
                },
            )
            .unwrap();

        assert!(path.points.len() > 2);
        assert!(path
            .points
            .iter()
            .all(|point| (point.y - 0.5).abs() < 1.0e-10));
        assert!(path.points.windows(2).all(|pair| pair[1].x >= pair[0].x));
    }

    #[test]
    fn streamline_rejects_outside_seed_and_terminates_at_stagnation() {
        let mesh = square_mesh();
        let dataset = ResultDataset::from_values(
            "run-0001",
            &mesh,
            vec![0.0],
            vec![Vec3::new(0.0, 0.0, 0.0)],
        )
        .unwrap();
        let field = StreamlineField::from_dataset(&dataset).unwrap();
        let options = StreamlineOptions {
            step_size: 0.1,
            max_steps: 32,
            max_length: 2.0,
            stagnation_speed: 1.0e-12,
        };

        assert!(field
            .integrate(
                Vec3::new(-0.1, 0.5, 0.0),
                StreamlineDirection::Forward,
                options,
            )
            .is_err());
        assert_eq!(
            field
                .integrate(
                    Vec3::new(0.5, 0.5, 0.0),
                    StreamlineDirection::Forward,
                    options,
                )
                .unwrap()
                .points
                .len(),
            1
        );
    }

    #[test]
    fn rk4_streamline_respects_step_limit_for_diagonal_velocity() {
        let mesh = square_mesh();
        let dataset = ResultDataset::from_values(
            "run-0001",
            &mesh,
            vec![0.0],
            vec![Vec3::new(1.0, 1.0, 0.0)],
        )
        .unwrap();
        let field = StreamlineField::from_dataset(&dataset).unwrap();
        let path = field
            .integrate(
                Vec3::new(0.1, 0.1, 0.0),
                StreamlineDirection::Forward,
                StreamlineOptions {
                    step_size: 0.1,
                    max_steps: 3,
                    max_length: 2.0,
                    stagnation_speed: 1.0e-12,
                },
            )
            .unwrap();

        assert_eq!(path.points.len(), 4);
        assert!(path
            .points
            .iter()
            .all(|point| (point.x - point.y).abs() < 1.0e-10));
    }

    #[test]
    fn streamline_rake_generates_evenly_spaced_paths() {
        let mesh = square_mesh();
        let dataset = ResultDataset::from_values(
            "run-0001",
            &mesh,
            vec![0.0],
            vec![Vec3::new(1.0, 0.0, 0.0)],
        )
        .unwrap();
        let field = StreamlineField::from_dataset(&dataset).unwrap();
        let paths = field
            .integrate_rake(
                Vec3::new(0.1, 0.2, 0.0),
                Vec3::new(0.1, 0.8, 0.0),
                3,
                StreamlineDirection::Forward,
                StreamlineOptions {
                    step_size: 0.1,
                    max_steps: 2,
                    max_length: 1.0,
                    stagnation_speed: 1.0e-12,
                },
            )
            .unwrap();

        assert_eq!(paths.len(), 3);
        assert_eq!(
            paths
                .iter()
                .map(|path| path.points[0].y)
                .collect::<Vec<_>>(),
            vec![0.2, 0.5, 0.8]
        );
    }
}
