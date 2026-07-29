//! Structured mesh geometry shared by previews and future mesh exporters.
//!
//! The current flow kernel remains two-dimensional. `ExtrudedMesh3D` is a
//! geometric representation for inspection and pre-processing, not a 3D CFD
//! discretisation.

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
}
