//! Versioned, solver-facing mesh artifacts and quality summaries.

use crate::ExtrudedMesh3D;

#[derive(Clone, Debug, PartialEq)]
pub struct MeshQualityReport {
    pub cell_volume_min: f64,
    pub cell_volume_max: f64,
    pub max_aspect_ratio: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StructuredMeshArtifact {
    pub mesh: ExtrudedMesh3D,
    pub source_revision: String,
    pub quality: MeshQualityReport,
}

impl StructuredMeshArtifact {
    pub fn from_extruded(
        mesh: ExtrudedMesh3D,
        source_revision: impl Into<String>,
    ) -> Result<Self, String> {
        let source_revision = source_revision.into();
        if source_revision.trim().is_empty() {
            return Err("mesh artifact source_revision cannot be empty".to_string());
        }
        let cell_volume = mesh.cell_volume();
        if !cell_volume.is_finite() || cell_volume <= 0.0 {
            return Err("structured mesh has an invalid cell volume".to_string());
        }
        let max_aspect_ratio = mesh.aspect_ratio();
        if !max_aspect_ratio.is_finite() || max_aspect_ratio < 1.0 {
            return Err("structured mesh has an invalid aspect ratio".to_string());
        }
        Ok(Self {
            mesh,
            source_revision,
            quality: MeshQualityReport {
                cell_volume_min: cell_volume,
                cell_volume_max: cell_volume,
                max_aspect_ratio,
            },
        })
    }

    pub fn cell_count(&self) -> usize {
        self.mesh.cell_count()
    }

    pub fn node_count(&self) -> usize {
        self.mesh.node_count()
    }
}
