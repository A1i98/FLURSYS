//! Gmsh 4.1 ASCII importer. Imported topology is validated by `UnstructuredMesh`.

use crate::{BoundaryPatch, BoundaryType, CellDefinition, MeshDimension, Point, UnstructuredMesh};
use std::collections::{BTreeMap, HashMap, HashSet};
use std::fmt;
use std::fs;
use std::path::Path;

const PLANAR_TOLERANCE: f64 = 1.0e-10;
const MAX_DECLARED_ITEMS: usize = 100_000_000;

type EntityKey = (i32, i64);

#[derive(Debug)]
pub enum GmshError {
    Io(std::io::Error),
    MissingSection(&'static str),
    InvalidMeshFormat(String),
    UnsupportedBinary,
    UnsupportedVersion(String),
    Parse {
        line: usize,
        section: String,
        message: String,
    },
    UnsupportedElementType {
        element_type: i32,
        entity_tag: i64,
        element_tag: i64,
    },
    MissingNode {
        node_tag: i64,
        element_tag: i64,
    },
    InvalidEntity {
        dimension: i32,
        tag: i64,
    },
    UnknownPhysicalTag {
        dimension: i32,
        tag: i64,
    },
    BoundaryFaceNotFound {
        element_tag: i64,
    },
    BoundaryFaceIsInternal {
        element_tag: i64,
    },
    AmbiguousBoundaryAssignment {
        element_tag: i64,
        groups: Vec<i64>,
    },
    UntaggedBoundaryFace {
        face: usize,
    },
    NonXyPlanarMesh,
    MeshValidation(String),
}

impl fmt::Display for GmshError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(f, "cannot read Gmsh mesh: {error}"),
            Self::MissingSection(section) => write!(f, "Gmsh mesh is missing ${section}"),
            Self::InvalidMeshFormat(message) => write!(f, "invalid $MeshFormat: {message}"),
            Self::UnsupportedBinary => write!(f, "Gmsh binary meshes are not supported yet; export as Gmsh 4.1 ASCII."),
            Self::UnsupportedVersion(version) => write!(f, "unsupported Gmsh version {version}; expected 4.x ASCII"),
            Self::Parse { line, section, message } => write!(f, "Gmsh parse error in ${section} at line {line}: {message}"),
            Self::UnsupportedElementType { element_type, entity_tag, element_tag } => write!(f, "unsupported Gmsh element type {element_type} on entity {entity_tag}, element {element_tag}"),
            Self::MissingNode { node_tag, element_tag } => write!(f, "element {element_tag} references missing node {node_tag}"),
            Self::InvalidEntity { dimension, tag } => write!(f, "element references missing entity dimension {dimension}, tag {tag}"),
            Self::UnknownPhysicalTag { dimension, tag } => write!(f, "entity references unknown physical group dimension {dimension}, tag {tag}"),
            Self::BoundaryFaceNotFound { element_tag } => write!(f, "boundary element {element_tag} does not match an exterior cell face"),
            Self::BoundaryFaceIsInternal { element_tag } => write!(f, "boundary element {element_tag} matches an internal face"),
            Self::AmbiguousBoundaryAssignment { element_tag, groups } => write!(f, "boundary element {element_tag} belongs to multiple physical groups {groups:?}"),
            Self::UntaggedBoundaryFace { face } => write!(f, "exterior face {face} has no physical boundary group"),
            Self::NonXyPlanarMesh => write!(f, "FLURSYS currently imports 2D Gmsh meshes only in the XY plane"),
            Self::MeshValidation(message) => write!(f, "imported mesh failed validation: {message}"),
        }
    }
}
impl std::error::Error for GmshError {}
impl From<std::io::Error> for GmshError {
    fn from(value: std::io::Error) -> Self {
        Self::Io(value)
    }
}

#[derive(Clone)]
struct RawElement {
    tag: i64,
    entity_dimension: i32,
    entity_tag: i64,
    element_type: i32,
    nodes: Vec<i64>,
}

pub fn load_gmsh(path: impl AsRef<Path>) -> Result<UnstructuredMesh, GmshError> {
    parse_gmsh(&fs::read_to_string(path)?)
}

/// Parses a Gmsh 4.x ASCII mesh using strict boundary coverage: every exterior
/// face must be represented by exactly one named physical boundary group.
pub fn parse_gmsh(input: &str) -> Result<UnstructuredMesh, GmshError> {
    let sections = sections(input)?;
    parse_format(require(&sections, "MeshFormat")?)?;
    let physical_names = parse_physical_names(sections.get("PhysicalNames"))?;
    let entities = parse_entities(require(&sections, "Entities")?)?;
    let nodes = parse_nodes(require(&sections, "Nodes")?)?;
    let elements = parse_elements(require(&sections, "Elements")?)?;
    convert(physical_names, entities, nodes, elements)
}

type Lines<'a> = Vec<(usize, &'a str)>;
fn sections(input: &str) -> Result<HashMap<&str, Lines<'_>>, GmshError> {
    let mut result = HashMap::new();
    let lines: Vec<_> = input
        .lines()
        .enumerate()
        .map(|(i, s)| (i + 1, s.trim()))
        .collect();
    let mut index = 0;
    while index < lines.len() {
        let (line, value) = lines[index];
        if !value.starts_with('$') || value.starts_with("$End") {
            index += 1;
            continue;
        }
        let name = &value[1..];
        let end = format!("$End{name}");
        let start = index + 1;
        index += 1;
        while index < lines.len() && lines[index].1 != end {
            index += 1;
        }
        if index == lines.len() {
            return Err(parse(line, name, "missing section terminator"));
        }
        if result.insert(name, lines[start..index].to_vec()).is_some() {
            return Err(parse(line, name, "duplicate section"));
        }
        index += 1;
    }
    Ok(result)
}
fn require<'a>(
    sections: &'a HashMap<&str, Lines<'a>>,
    name: &'static str,
) -> Result<&'a Lines<'a>, GmshError> {
    sections.get(name).ok_or(GmshError::MissingSection(name))
}
fn parse(line: usize, section: &str, message: impl Into<String>) -> GmshError {
    GmshError::Parse {
        line,
        section: section.to_string(),
        message: message.into(),
    }
}
fn words<'a>(
    lines: &'a Lines<'a>,
    _section: &str,
) -> impl Iterator<Item = Result<(usize, &'a str), GmshError>> + 'a {
    lines.iter().flat_map(|(line, text)| {
        text.split_whitespace()
            .map(move |word| Ok((*line, word)))
            .collect::<Vec<_>>()
    })
}
fn next<'a, T: std::str::FromStr>(
    it: &mut impl Iterator<Item = Result<(usize, &'a str), GmshError>>,
    section: &str,
) -> Result<T, GmshError> {
    let (line, value) = it
        .next()
        .ok_or_else(|| parse(0, section, "unexpected end of section"))??;
    value
        .parse()
        .map_err(|_| parse(line, section, format!("invalid value {value}")))
}
fn count(value: u64, line: usize, section: &str) -> Result<usize, GmshError> {
    usize::try_from(value)
        .ok()
        .filter(|n| *n <= MAX_DECLARED_ITEMS)
        .ok_or_else(|| parse(line, section, "declared count is too large"))
}

fn parse_format(lines: &Lines<'_>) -> Result<(), GmshError> {
    let (line, text) = lines
        .first()
        .ok_or_else(|| parse(0, "MeshFormat", "missing format line"))?;
    let values: Vec<_> = text.split_whitespace().collect();
    if values.len() != 3 {
        return Err(parse(
            *line,
            "MeshFormat",
            "expected version file-type data-size",
        ));
    }
    if !values[0].starts_with('4') {
        return Err(GmshError::UnsupportedVersion(values[0].to_string()));
    }
    match values[1] {
        "0" => Ok(()),
        "1" => Err(GmshError::UnsupportedBinary),
        _ => Err(GmshError::InvalidMeshFormat(
            "file type must be 0 or 1".to_string(),
        )),
    }
}

fn parse_physical_names(
    lines: Option<&Lines<'_>>,
) -> Result<HashMap<(i32, i64), String>, GmshError> {
    let Some(lines) = lines else {
        return Ok(HashMap::new());
    };
    let (line, first) = lines
        .first()
        .ok_or_else(|| parse(0, "PhysicalNames", "missing count"))?;
    let expected: usize = first
        .parse()
        .map_err(|_| parse(*line, "PhysicalNames", "invalid count"))?;
    if expected > MAX_DECLARED_ITEMS {
        return Err(parse(*line, "PhysicalNames", "declared count is too large"));
    }
    if lines.len().saturating_sub(1) != expected {
        return Err(parse(
            *line,
            "PhysicalNames",
            "count does not match entries",
        ));
    }
    let mut names = HashMap::new();
    for (line, entry) in &lines[1..] {
        let quote = entry
            .find('"')
            .ok_or_else(|| parse(*line, "PhysicalNames", "physical name must be quoted"))?;
        let mut values = entry[..quote].split_whitespace();
        let dimension = values
            .next()
            .ok_or_else(|| parse(*line, "PhysicalNames", "missing dimension"))?
            .parse()
            .map_err(|_| parse(*line, "PhysicalNames", "invalid dimension"))?;
        let tag = values
            .next()
            .ok_or_else(|| parse(*line, "PhysicalNames", "missing tag"))?
            .parse()
            .map_err(|_| parse(*line, "PhysicalNames", "invalid tag"))?;
        if values.next().is_some() {
            return Err(parse(
                *line,
                "PhysicalNames",
                "unexpected physical-name prefix",
            ));
        }
        let closing = entry
            .rfind('"')
            .filter(|closing| *closing > quote)
            .ok_or_else(|| parse(*line, "PhysicalNames", "physical name must be quoted"))?;
        if !entry[closing + 1..].trim().is_empty() {
            return Err(parse(
                *line,
                "PhysicalNames",
                "unexpected trailing physical-name data",
            ));
        }
        names.insert((dimension, tag), entry[quote + 1..closing].to_string());
    }
    Ok(names)
}

fn parse_entities(lines: &Lines<'_>) -> Result<HashMap<EntityKey, Vec<i64>>, GmshError> {
    let mut it = words(lines, "Entities");
    let point_count: u64 = next(&mut it, "Entities")?;
    let curve_count: u64 = next(&mut it, "Entities")?;
    let surface_count: u64 = next(&mut it, "Entities")?;
    let volume_count: u64 = next(&mut it, "Entities")?;
    let counts = [point_count, curve_count, surface_count, volume_count];
    let mut entities = HashMap::new();
    for (dimension, raw_count) in counts.into_iter().enumerate() {
        let entity_count = count(raw_count, 0, "Entities")?;
        for _ in 0..entity_count {
            let tag: i64 = next(&mut it, "Entities")?;
            let coordinates = if dimension == 0 { 3 } else { 6 };
            for _ in 0..coordinates {
                let _: f64 = next(&mut it, "Entities")?;
            }
            let physical_count: u64 = next(&mut it, "Entities")?;
            let physical_count = count(physical_count, 0, "Entities")?;
            let mut physical = Vec::with_capacity(physical_count);
            for _ in 0..physical_count {
                physical.push(next(&mut it, "Entities")?);
            }
            if dimension > 0 {
                let bounds: u64 = next(&mut it, "Entities")?;
                for _ in 0..count(bounds, 0, "Entities")? {
                    let _: i64 = next(&mut it, "Entities")?;
                }
            }
            entities.insert((dimension as i32, tag), physical);
        }
    }
    if it.next().is_some() {
        return Err(parse(0, "Entities", "unexpected trailing values"));
    }
    Ok(entities)
}

fn parse_nodes(lines: &Lines<'_>) -> Result<HashMap<i64, Point>, GmshError> {
    let mut it = words(lines, "Nodes");
    let blocks: u64 = next(&mut it, "Nodes")?;
    let total: u64 = next(&mut it, "Nodes")?;
    let _: i64 = next(&mut it, "Nodes")?;
    let _: i64 = next(&mut it, "Nodes")?;
    let mut nodes = HashMap::with_capacity(count(total, 0, "Nodes")?);
    for _ in 0..count(blocks, 0, "Nodes")? {
        let entity_dimension: i32 = next(&mut it, "Nodes")?;
        let _: i64 = next(&mut it, "Nodes")?;
        let parametric: i32 = next(&mut it, "Nodes")?;
        let node_count: u64 = next(&mut it, "Nodes")?;
        if !(0..=3).contains(&entity_dimension) || !(0..=1).contains(&parametric) {
            return Err(parse(0, "Nodes", "invalid node block header"));
        }
        let node_count = count(node_count, 0, "Nodes")?;
        let mut tags = Vec::with_capacity(node_count);
        for _ in 0..node_count {
            tags.push(next(&mut it, "Nodes")?);
        }
        for tag in tags {
            let x: f64 = next(&mut it, "Nodes")?;
            let y: f64 = next(&mut it, "Nodes")?;
            let z: f64 = next(&mut it, "Nodes")?;
            for _ in 0..if parametric == 1 { entity_dimension } else { 0 } {
                let _: f64 = next(&mut it, "Nodes")?;
            }
            if nodes.insert(tag, Point::new(x, y, z)).is_some() {
                return Err(parse(0, "Nodes", "duplicate node tag"));
            }
        }
    }
    if nodes.len() != count(total, 0, "Nodes")? {
        return Err(parse(0, "Nodes", "node count does not match blocks"));
    }
    Ok(nodes)
}

fn node_count(element_type: i32) -> Option<usize> {
    match element_type {
        1 => Some(2),
        2 => Some(3),
        3 => Some(4),
        4 => Some(4),
        5 => Some(8),
        6 => Some(6),
        7 => Some(5),
        _ => None,
    }
}
fn element_dimension(element_type: i32) -> Option<i32> {
    match element_type {
        1 => Some(1),
        2 | 3 => Some(2),
        4..=7 => Some(3),
        _ => None,
    }
}
fn parse_elements(lines: &Lines<'_>) -> Result<Vec<RawElement>, GmshError> {
    let mut it = words(lines, "Elements");
    let blocks: u64 = next(&mut it, "Elements")?;
    let total: u64 = next(&mut it, "Elements")?;
    let _: i64 = next(&mut it, "Elements")?;
    let _: i64 = next(&mut it, "Elements")?;
    let mut elements = Vec::with_capacity(count(total, 0, "Elements")?);
    for _ in 0..count(blocks, 0, "Elements")? {
        let dimension: i32 = next(&mut it, "Elements")?;
        let entity_tag: i64 = next(&mut it, "Elements")?;
        let element_type: i32 = next(&mut it, "Elements")?;
        let block_count: u64 = next(&mut it, "Elements")?;
        let nodes = node_count(element_type).ok_or(GmshError::UnsupportedElementType {
            element_type,
            entity_tag,
            element_tag: -1,
        })?;
        if element_dimension(element_type) != Some(dimension) {
            return Err(parse(
                0,
                "Elements",
                "element type and block dimension disagree",
            ));
        }
        for _ in 0..count(block_count, 0, "Elements")? {
            let tag: i64 = next(&mut it, "Elements")?;
            let mut connectivity = Vec::with_capacity(nodes);
            for _ in 0..nodes {
                connectivity.push(next(&mut it, "Elements")?);
            }
            elements.push(RawElement {
                tag,
                entity_dimension: dimension,
                entity_tag,
                element_type,
                nodes: connectivity,
            });
        }
    }
    if elements.len() != count(total, 0, "Elements")? {
        return Err(parse(0, "Elements", "element count does not match blocks"));
    }
    Ok(elements)
}

fn convert(
    names: HashMap<(i32, i64), String>,
    entities: HashMap<EntityKey, Vec<i64>>,
    nodes: HashMap<i64, Point>,
    elements: Vec<RawElement>,
) -> Result<UnstructuredMesh, GmshError> {
    let dimension = elements
        .iter()
        .filter_map(|element| element_dimension(element.element_type))
        .max()
        .ok_or_else(|| GmshError::MeshValidation("mesh contains no elements".to_string()))?;
    if dimension < 2 {
        return Err(GmshError::MeshValidation(
            "computational mesh must contain 2D or 3D cells".to_string(),
        ));
    }
    let mesh_dimension = if dimension == 2 {
        MeshDimension::TwoD
    } else {
        MeshDimension::ThreeD
    };
    let mut ordered_tags: Vec<_> = nodes.keys().copied().collect();
    ordered_tags.sort_unstable();
    let mut compact = HashMap::with_capacity(ordered_tags.len());
    let mut points = Vec::with_capacity(ordered_tags.len());
    for tag in ordered_tags {
        let index = points.len();
        points.push(*nodes.get(&tag).ok_or(GmshError::MissingNode {
            node_tag: tag,
            element_tag: -1,
        })?);
        compact.insert(tag, index);
    }
    let mut cells = Vec::new();
    let mut boundaries = Vec::new();
    for element in elements {
        let physical = entities
            .get(&(element.entity_dimension, element.entity_tag))
            .ok_or(GmshError::InvalidEntity {
                dimension: element.entity_dimension,
                tag: element.entity_tag,
            })?;
        let compact_nodes = element
            .nodes
            .iter()
            .map(|tag| {
                compact.get(tag).copied().ok_or(GmshError::MissingNode {
                    node_tag: *tag,
                    element_tag: element.tag,
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        if element.entity_dimension == dimension {
            let cell =
                match element.element_type {
                    2 | 3 if dimension == 2 => CellDefinition::polygon(compact_nodes),
                    4 => CellDefinition::tetrahedron(compact_nodes.try_into().map_err(|_| {
                        GmshError::MeshValidation("invalid tetrahedron".to_string())
                    })?),
                    5 => CellDefinition::Hexahedron(compact_nodes.try_into().map_err(|_| {
                        GmshError::MeshValidation("invalid hexahedron".to_string())
                    })?),
                    6 => CellDefinition::Prism(
                        compact_nodes
                            .try_into()
                            .map_err(|_| GmshError::MeshValidation("invalid prism".to_string()))?,
                    ),
                    7 => {
                        CellDefinition::Pyramid(compact_nodes.try_into().map_err(|_| {
                            GmshError::MeshValidation("invalid pyramid".to_string())
                        })?)
                    }
                    _ => {
                        return Err(GmshError::UnsupportedElementType {
                            element_type: element.element_type,
                            entity_tag: element.entity_tag,
                            element_tag: element.tag,
                        })
                    }
                };
            cells.push(cell);
        } else if element.entity_dimension == dimension - 1 {
            boundaries.push((element, compact_nodes, physical.clone()));
        }
    }
    if dimension == 2
        && points
            .iter()
            .any(|point| point.position.z.abs() > PLANAR_TOLERANCE)
    {
        return Err(GmshError::NonXyPlanarMesh);
    }
    let mesh = UnstructuredMesh::from_cells(mesh_dimension, points, cells)
        .map_err(|error| GmshError::MeshValidation(error.to_string()))?;
    let mut faces = HashMap::new();
    for (index, face) in mesh.faces().iter().enumerate() {
        faces.insert(canonical(&face.vertices), index);
    }
    let mut patches: BTreeMap<String, Vec<usize>> = BTreeMap::new();
    let mut assigned = HashSet::new();
    for (element, nodes, physical) in boundaries {
        if physical.len() != 1 {
            return Err(GmshError::AmbiguousBoundaryAssignment {
                element_tag: element.tag,
                groups: physical,
            });
        }
        let physical_tag = physical[0];
        let name = names
            .get(&(dimension - 1, physical_tag))
            .cloned()
            .unwrap_or_else(|| format!("physical_{physical_tag}"));
        let key = canonical(&nodes);
        let Some(face) = faces.get(&key).copied() else {
            return Err(GmshError::BoundaryFaceNotFound {
                element_tag: element.tag,
            });
        };
        if mesh.faces()[face].neighbour.is_some() {
            return Err(GmshError::BoundaryFaceIsInternal {
                element_tag: element.tag,
            });
        }
        if !assigned.insert(face) {
            return Err(GmshError::AmbiguousBoundaryAssignment {
                element_tag: element.tag,
                groups: vec![physical_tag],
            });
        }
        patches.entry(name).or_default().push(face);
    }
    for (face, value) in mesh.faces().iter().enumerate() {
        if value.neighbour.is_none() && !assigned.contains(&face) {
            return Err(GmshError::UntaggedBoundaryFace { face });
        }
    }
    let patches = patches
        .into_iter()
        .map(|(name, face_indices)| BoundaryPatch {
            name,
            face_indices,
            boundary_type: BoundaryType::ZeroGradient,
        })
        .collect();
    mesh.with_boundary_patches(patches)
        .map_err(|error| GmshError::MeshValidation(error.to_string()))
}
fn canonical(vertices: &[usize]) -> Vec<usize> {
    let mut key = vertices.to_vec();
    key.sort_unstable();
    key
}

#[cfg(test)]
mod tests {
    use super::*;
    const GEOMETRY_TOL: f64 = 1.0e-10;

    fn assert_imported_geometry(
        mesh: &UnstructuredMesh,
        points: usize,
        cells: usize,
        faces: usize,
        internal: usize,
        boundary: usize,
    ) {
        assert_eq!(mesh.points().len(), points);
        assert_eq!(mesh.cells().len(), cells);
        assert_eq!(mesh.faces().len(), faces);
        assert_eq!(
            mesh.faces()
                .iter()
                .filter(|face| face.neighbour.is_some())
                .count(),
            internal
        );
        assert_eq!(
            mesh.faces()
                .iter()
                .filter(|face| face.neighbour.is_none())
                .count(),
            boundary
        );
        assert_eq!(mesh.boundary_patches()[0].face_indices.len(), boundary);
        for (cell_index, cell) in mesh.cells().iter().enumerate() {
            assert!(cell.volume > 0.0);
            let closure = cell
                .faces
                .iter()
                .fold(crate::Vec3::ZERO, |sum, &face_index| {
                    let face = &mesh.faces()[face_index];
                    sum + if face.owner == cell_index {
                        face.area_vector
                    } else {
                        -face.area_vector
                    }
                });
            assert!(
                closure.norm() < GEOMETRY_TOL,
                "cell {cell_index}: {closure:?}"
            );
        }
    }
    const TRIANGLE: &str = "$MeshFormat\n4.1 0 8\n$EndMeshFormat\n$PhysicalNames\n1\n1 1 \"walls\"\n$EndPhysicalNames\n$Entities\n0 1 1 0\n1 0 0 0 1 0 0 1 1 2 1 -1\n1 0 0 0 1 1 0 1 1 1 1\n$EndEntities\n$Nodes\n1 3 10 1001\n2 1 0 3\n10 42 1001\n0 0 0\n1 0 0\n0 1 0\n$EndNodes\n$Elements\n2 4 1 4\n1 1 1 3\n1 10 42\n2 42 1001\n3 1001 10\n2 1 2 1\n4 10 42 1001\n$EndElements\n";
    #[test]
    fn imports_non_contiguous_triangle_nodes_and_boundary_patch() {
        let mesh = parse_gmsh(TRIANGLE).unwrap();
        assert_eq!(mesh.points().len(), 3);
        assert_eq!(mesh.cells().len(), 1);
        assert_eq!(mesh.faces().len(), 3);
        assert_eq!(mesh.boundary_patches()[0].name, "walls");
        assert!((mesh.cells()[0].volume - 0.5).abs() < 1e-12);
    }

    #[test]
    fn deduplicates_shared_faces_orients_them_and_preserves_closed_cell_vectors() {
        let input = "$MeshFormat\n4.1 0 8\n$EndMeshFormat\n$PhysicalNames\n1\n1 7 \"walls\"\n$EndPhysicalNames\n$Entities\n0 1 1 0\n1 0 0 0 1 1 0 1 7 4 1 2 3 4\n1 0 0 0 1 1 0 0 1 1\n$EndEntities\n$Nodes\n1 4 10 50000\n2 1 0 4\n10 42 1001 50000\n0 0 0\n1 0 0\n1 1 0\n0 1 0\n$EndNodes\n$Elements\n2 6 1 6\n1 1 1 4\n1 10 42\n2 42 1001\n3 1001 50000\n4 50000 10\n2 1 2 2\n5 10 42 1001\n6 10 1001 50000\n$EndElements\n";
        let mesh = parse_gmsh(input).unwrap();
        assert_eq!(mesh.faces().len(), 5);
        let shared = mesh
            .faces()
            .iter()
            .find(|face| face.neighbour.is_some())
            .unwrap();
        let neighbour = shared.neighbour.unwrap();
        assert!(
            (mesh.cells()[neighbour].center - mesh.cells()[shared.owner].center)
                .dot(shared.area_vector)
                > 0.0
        );
        for (cell_index, cell) in mesh.cells().iter().enumerate() {
            let sum = cell
                .faces
                .iter()
                .fold(crate::Vec3::ZERO, |accumulator, &face_index| {
                    let face = &mesh.faces()[face_index];
                    accumulator
                        + if face.owner == cell_index {
                            face.area_vector
                        } else {
                            face.area_vector * -1.0
                        }
                });
            assert!(sum.norm() < 1e-12);
        }
    }

    #[test]
    fn rejects_binary_meshes() {
        let error = parse_gmsh("$MeshFormat\n4.1 1 8\n$EndMeshFormat\n").unwrap_err();
        assert!(matches!(error, GmshError::UnsupportedBinary));
    }

    #[test]
    fn imports_tetrahedron_with_four_tagged_exterior_faces() {
        let input = "$MeshFormat\n4.1 0 8\n$EndMeshFormat\n$PhysicalNames\n1\n2 9 \"walls\"\n$EndPhysicalNames\n$Entities\n0 0 1 1\n1 0 0 0 1 1 1 1 9 4 1 2 3 4\n1 0 0 0 1 1 1 0 1 1\n$EndEntities\n$Nodes\n1 4 10 50000\n3 1 0 4\n10 42 1001 50000\n0 0 0\n1 0 0\n0 1 0\n0 0 1\n$EndNodes\n$Elements\n2 5 1 5\n2 1 2 4\n1 10 42 1001\n2 10 50000 42\n3 42 50000 1001\n4 1001 50000 10\n3 1 4 1\n5 10 42 1001 50000\n$EndElements\n";
        let mesh = parse_gmsh(input).unwrap();
        assert_eq!(mesh.dimension(), MeshDimension::ThreeD);
        assert_eq!(mesh.faces().len(), 4);
        assert!(mesh.faces().iter().all(|face| face.neighbour.is_none()));
        assert!((mesh.cells()[0].volume - 1.0 / 6.0).abs() < 1e-12);
    }

    #[test]
    fn imports_quadrilateral_with_closed_boundary_patch() {
        let input = "$MeshFormat\n4.1 0 8\n$EndMeshFormat\n$PhysicalNames\n1\n1 1 \"walls\"\n$EndPhysicalNames\n$Entities\n0 1 1 0\n1 0 0 0 1 1 0 1 1 4 1 2 3 4\n1 0 0 0 1 1 0 0 1 1\n$EndEntities\n$Nodes\n1 4 10 50000\n2 1 0 4\n10 42 1001 50000\n0 0 0\n1 0 0\n1 1 0\n0 1 0\n$EndNodes\n$Elements\n2 5 1 5\n1 1 1 4\n1 10 42\n2 42 1001\n3 1001 50000\n4 50000 10\n2 1 3 1\n5 10 42 1001 50000\n$EndElements\n";
        let mesh = parse_gmsh(input).unwrap();
        assert_eq!(mesh.points().len(), 4);
        assert_eq!(mesh.cells().len(), 1);
        assert_eq!(mesh.faces().len(), 4);
        assert_eq!(mesh.boundary_patches()[0].face_indices.len(), 4);
        assert!((mesh.cells()[0].volume - 1.0).abs() < 1e-12);
    }

    #[test]
    fn imports_remaining_3d_element_types_with_closed_boundary_patches() {
        let header = "$MeshFormat\n4.1 0 8\n$EndMeshFormat\n$PhysicalNames\n1\n2 9 \"walls\"\n$EndPhysicalNames\n$Entities\n0 0 1 1\n1 0 0 0 1 1 1 1 9 4 1 2 3 4\n1 0 0 0 1 1 1 0 1 1\n$EndEntities\n";
        let fixtures = [
            (format!("{header}$Nodes\n1 8 1 8\n3 1 0 8\n1 2 3 4 5 6 7 8\n0 0 0\n1 0 0\n1 1 0\n0 1 0\n0 0 1\n1 0 1\n1 1 1\n0 1 1\n$EndNodes\n$Elements\n2 7 1 7\n2 1 3 6\n1 1 4 3 2\n2 5 6 7 8\n3 1 2 6 5\n4 2 3 7 6\n5 3 4 8 7\n6 4 1 5 8\n3 1 5 1\n7 1 2 3 4 5 6 7 8\n$EndElements\n"), 8, 6, 1.0),
            (format!("{header}$Nodes\n1 6 1 6\n3 1 0 6\n1 2 3 4 5 6\n0 0 0\n1 0 0\n0 1 0\n0 0 1\n1 0 1\n0 1 1\n$EndNodes\n$Elements\n3 6 1 6\n2 1 2 2\n1 1 3 2\n2 4 5 6\n2 1 3 3\n3 1 2 5 4\n4 2 3 6 5\n5 3 1 4 6\n3 1 6 1\n6 1 2 3 4 5 6\n$EndElements\n"), 6, 5, 0.5),
            (format!("{header}$Nodes\n1 5 1 5\n3 1 0 5\n1 2 3 4 5\n0 0 0\n1 0 0\n1 1 0\n0 1 0\n0.5 0.5 1\n$EndNodes\n$Elements\n3 6 1 6\n2 1 3 1\n1 1 4 3 2\n2 1 2 4\n2 1 2 5\n3 2 3 5\n4 3 4 5\n5 4 1 5\n3 1 7 1\n6 1 2 3 4 5\n$EndElements\n"), 5, 5, 1.0 / 3.0),
        ];
        for (input, points, faces, volume) in fixtures {
            let mesh = parse_gmsh(&input).unwrap();
            assert_imported_geometry(&mesh, points, 1, faces, 0, faces);
            assert!((mesh.cells()[0].volume - volume).abs() < GEOMETRY_TOL);
        }
    }

    #[test]
    fn imports_two_tetrahedra_with_one_shared_face() {
        let input = "$MeshFormat\n4.1 0 8\n$EndMeshFormat\n$PhysicalNames\n1\n2 9 \"walls\"\n$EndPhysicalNames\n$Entities\n0 0 1 1\n1 0 0 -1 1 1 1 1 9 4 1 2 3 4\n1 0 0 -1 1 1 1 0 1 1\n$EndEntities\n$Nodes\n1 5 1 5\n3 1 0 5\n1 2 3 4 5\n0 0 0\n1 0 0\n0 1 0\n0 0 1\n0 0 -1\n$EndNodes\n$Elements\n2 8 1 8\n2 1 2 6\n1 1 2 4\n2 2 3 4\n3 3 1 4\n4 1 3 5\n5 3 2 5\n6 2 1 5\n3 1 4 2\n7 1 2 3 4\n8 1 3 2 5\n$EndElements\n";
        let mesh = parse_gmsh(input).unwrap();
        assert_imported_geometry(&mesh, 5, 2, 7, 1, 6);
        let shared: Vec<_> = mesh
            .faces()
            .iter()
            .filter(|face| face.neighbour.is_some())
            .collect();
        assert_eq!(shared.len(), 1);
        let face = shared[0];
        let neighbour = face.neighbour.unwrap();
        assert_ne!(face.owner, neighbour);
        assert!(
            (mesh.cells()[neighbour].center - mesh.cells()[face.owner].center)
                .dot(face.area_vector)
                > 0.0
        );
    }
}
