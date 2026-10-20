//! Advisory mesh diagnostics and fatal geometry checks before solving.
use crate::UnstructuredMesh;
use serde::{Deserialize, Serialize};
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct MeshPreflight {
    pub invalid_cells: Vec<usize>,
    pub invalid_faces: Vec<usize>,
    pub severe_nonorthogonal_faces: Vec<usize>,
    pub max_adjacent_volume_ratio: f64,
    pub warnings: Vec<String>,
}
impl MeshPreflight {
    pub fn blocked(&self) -> bool {
        !self.invalid_cells.is_empty() || !self.invalid_faces.is_empty()
    }
    pub fn summary(&self) -> String {
        format!("Mesh preflight: {} invalid cells, {} invalid faces; {} internal faces above 70°; max adjacent volume ratio {:.3e}. {}",self.invalid_cells.len(),self.invalid_faces.len(),self.severe_nonorthogonal_faces.len(),self.max_adjacent_volume_ratio,self.warnings.join(" "))
    }
}
pub fn mesh_preflight(mesh: &UnstructuredMesh) -> MeshPreflight {
    let mut r = MeshPreflight {
        max_adjacent_volume_ratio: 1.0,
        ..Default::default()
    };
    for (i, c) in mesh.cells().iter().enumerate() {
        if !c.volume.is_finite()
            || c.volume <= 0.0
            || ![c.center.x, c.center.y, c.center.z]
                .iter()
                .all(|v| v.is_finite())
        {
            r.invalid_cells.push(i);
        }
    }
    for (i, f) in mesh.faces().iter().enumerate() {
        if !f.area.is_finite()
            || f.area <= 0.0
            || ![
                f.normal.x, f.normal.y, f.normal.z, f.center.x, f.center.y, f.center.z,
            ]
            .iter()
            .all(|v| v.is_finite())
        {
            r.invalid_faces.push(i);
            continue;
        }
        if let Some(n) = f.neighbour {
            let d = mesh.cells()[n].center - mesh.cells()[f.owner].center;
            let cosine = d.dot(f.normal) / d.norm();
            if !cosine.is_finite() || cosine <= 0.0 {
                r.invalid_faces.push(i);
            } else if cosine.clamp(-1.0, 1.0).acos().to_degrees() > 70.0 {
                r.severe_nonorthogonal_faces.push(i);
            }
            let a = mesh.cells()[n].volume;
            let b = mesh.cells()[f.owner].volume;
            if a > 0.0 && b > 0.0 {
                r.max_adjacent_volume_ratio = r.max_adjacent_volume_ratio.max((a / b).max(b / a));
            }
        }
    }
    if !r.severe_nonorthogonal_faces.is_empty() {
        r.warnings
            .push("High non-orthogonality: inspect mesh quality and improve alignment.".into());
    }
    if r.max_adjacent_volume_ratio > 10.0 {
        r.warnings
            .push("Abrupt cell-size transition: reduce local refinement growth.".into());
    }
    if mesh.quality().max_skewness > 0.5 {
        r.warnings.push("High centre-line displacement / centre distance. This skewness metric is not interchangeable with other tools' definitions.".into());
    }
    r
}
#[cfg(test)]
mod preflight_tests {
    use super::*;
    use crate::{CellDefinition, MeshDimension, Point};
    #[test]
    fn abrupt_refinement_is_advisory_not_a_hard_failure() {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0., 0., 0.),
                Point::new(1., 0., 0.),
                Point::new(21., 0., 0.),
                Point::new(0., 1., 0.),
                Point::new(1., 1., 0.),
                Point::new(21., 1., 0.),
            ],
            vec![
                CellDefinition::polygon(vec![0, 1, 4, 3]),
                CellDefinition::polygon(vec![1, 2, 5, 4]),
            ],
        )
        .unwrap();
        let r = mesh_preflight(&mesh);
        assert!(!r.blocked());
        assert_eq!(r.max_adjacent_volume_ratio, 20.0);
        assert!(!r.warnings.is_empty());
    }
    #[test]
    fn right_triangle_passes_without_false_quality_failures() {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0., 0., 0.),
                Point::new(1., 0., 0.),
                Point::new(0., 1., 0.),
            ],
            vec![CellDefinition::polygon(vec![0, 1, 2])],
        )
        .unwrap();
        let r = mesh_preflight(&mesh);
        assert!(!r.blocked());
        assert!(r.warnings.is_empty());
        assert_eq!(r.max_adjacent_volume_ratio, 1.0);
    }
}
