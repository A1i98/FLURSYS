#[derive(Clone, Copy, Debug)]
pub struct UniformGrid2D {
    pub nx: usize,
    pub ny: usize,
    pub length: f64,
    pub height: f64,
    pub dx: f64,
    pub dy: f64,
}

impl UniformGrid2D {
    pub fn new(nx: usize, ny: usize, length: f64, height: f64) -> Result<Self, String> {
        if nx < 4 || ny < 4 {
            return Err("nx and ny must both be at least 4".to_string());
        }
        if !(length > 0.0 && height > 0.0) {
            return Err("Domain dimensions must be positive".to_string());
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
    pub fn cell_x(&self, i: usize) -> f64 {
        (i as f64 + 0.5) * self.dx
    }

    #[inline]
    pub fn cell_y(&self, j: usize) -> f64 {
        (j as f64 + 0.5) * self.dy
    }

    #[inline]
    pub fn u_face_x(&self, i: usize) -> f64 {
        i as f64 * self.dx
    }

    #[inline]
    pub fn u_face_y(&self, j: usize) -> f64 {
        (j as f64 + 0.5) * self.dy
    }

    #[inline]
    pub fn v_face_x(&self, i: usize) -> f64 {
        (i as f64 + 0.5) * self.dx
    }

    #[inline]
    pub fn v_face_y(&self, j: usize) -> f64 {
        j as f64 * self.dy
    }
}
