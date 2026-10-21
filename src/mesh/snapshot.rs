//! Portable topology snapshots, validated through the normal mesh constructor.
use super::*;
#[derive(Serialize, Deserialize)]
pub(crate) struct MeshSnapshot {
    dimension: MeshDimension,
    points: Vec<Vec3>,
    faces: Vec<(Vec<usize>, usize, Option<usize>)>,
    cells: Vec<Vec<usize>>,
    patches: Vec<BoundaryPatch>,
}
impl MeshSnapshot {
    pub fn capture(mesh: &UnstructuredMesh) -> Self {
        Self {
            dimension: mesh.dimension,
            points: mesh.points.iter().map(|p| p.position).collect(),
            faces: mesh
                .faces
                .iter()
                .map(|f| (f.vertices.clone(), f.owner, f.neighbour))
                .collect(),
            cells: mesh.cells.iter().map(|c| c.faces.clone()).collect(),
            patches: mesh.boundary_patches.clone(),
        }
    }
    pub fn restore(self) -> Result<UnstructuredMesh, String> {
        let mut definitions = Vec::with_capacity(self.cells.len());
        for (cell, indices) in self.cells.iter().enumerate() {
            let faces = indices
                .iter()
                .map(|&i| {
                    let (vertices, owner, neighbour) = self
                        .faces
                        .get(i)
                        .ok_or("Mesh snapshot references a missing face")?;
                    if *owner != cell && *neighbour != Some(cell) {
                        return Err("Mesh snapshot cell/face addressing is inconsistent");
                    }
                    let mut v = vertices.clone();
                    if *owner != cell {
                        v.reverse();
                    }
                    Ok(v)
                })
                .collect::<Result<Vec<_>, _>>()?;
            if self.dimension == MeshDimension::TwoD {
                let first = faces.first().ok_or("Empty polygon")?;
                if first.len() != 2 || faces.iter().any(|f| f.len() != 2) {
                    return Err("Invalid 2D mesh edges".into());
                }
                let mut vertices = vec![first[0]];
                let mut next = first[1];
                for _ in 1..faces.len() {
                    if vertices.contains(&next) {
                        return Err("Invalid polygon loop".into());
                    }
                    vertices.push(next);
                    next = faces
                        .iter()
                        .find(|f| f[0] == next)
                        .ok_or("Open polygon loop")?[1];
                }
                if next != vertices[0] {
                    return Err("Open polygon loop".into());
                }
                definitions.push(CellDefinition::Polygon(vertices));
            } else {
                definitions.push(CellDefinition::Polyhedron(faces));
            }
        }
        let mut mesh = UnstructuredMesh::from_cells(
            self.dimension,
            self.points
                .iter()
                .map(|p| Point::new(p.x, p.y, p.z))
                .collect(),
            definitions,
        )
        .map_err(|e| e.to_string())?;
        if mesh.faces.len() != self.faces.len() {
            return Err("Mesh snapshot face count mismatch".into());
        }
        let mut lookup = HashMap::new();
        for (i, f) in mesh.faces.iter().enumerate() {
            let mut key = f.vertices.clone();
            key.sort_unstable();
            lookup.insert(key, i);
        }
        let mut old_to_new = vec![usize::MAX; mesh.faces.len()];
        let mut ordered = Vec::with_capacity(mesh.faces.len());
        for (index, (vertices, owner, neighbour)) in self.faces.iter().enumerate() {
            let mut key = vertices.clone();
            key.sort_unstable();
            let original = *lookup
                .get(&key)
                .ok_or("Mesh snapshot face missing after validation")?;
            if old_to_new[original] != usize::MAX {
                return Err("Duplicate mesh snapshot face".into());
            }
            let mut face = mesh.faces[original].clone();
            if face.owner != *owner || face.neighbour != *neighbour {
                return Err("Mesh snapshot ownership mismatch".into());
            }
            old_to_new[original] = index;
            face.vertices = vertices.clone();
            ordered.push(face);
        }
        for (cell, stored) in mesh.cells.iter_mut().zip(&self.cells) {
            let mut actual = cell
                .faces
                .iter()
                .map(|i| old_to_new[*i])
                .collect::<Vec<_>>();
            actual.sort_unstable();
            let mut expected = stored.clone();
            expected.sort_unstable();
            if actual != expected {
                return Err("Mesh snapshot connectivity mismatch".into());
            }
            cell.faces = stored.clone();
        }
        mesh.faces = ordered;
        let mesh = mesh
            .with_boundary_patches(self.patches)
            .map_err(|e| e.to_string())?;
        let exterior = mesh.faces.iter().filter(|f| f.neighbour.is_none()).count();
        if mesh
            .boundary_patches
            .iter()
            .map(|p| p.face_indices.len())
            .sum::<usize>()
            != exterior
        {
            return Err("Incomplete saved boundary coverage".into());
        }
        Ok(mesh)
    }
}
