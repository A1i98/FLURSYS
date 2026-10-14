//! Restarted flexible GCR for residual-dependent multigrid corrections.
//! Store A*p with each direction; nonlinear scaling in a V-cycle does not
//! require a symmetric or fixed preconditioner. The caller checks b-A*x.
use crate::{dot, norm_l2, LinearAlgebraError};

pub(super) struct GcrWorkspace {
    directions: Vec<Vec<f64>>,
    images: Vec<Vec<f64>>,
    used: usize,
}

impl GcrWorkspace {
    pub fn new(size: usize, dimension: usize) -> Self {
        Self {
            directions: (0..dimension).map(|_| vec![0.0; size]).collect(),
            images: (0..dimension).map(|_| vec![0.0; size]).collect(),
            used: 0,
        }
    }

    pub fn reset(&mut self) {
        self.used = 0;
    }

    pub fn correct(
        &mut self,
        direction: &mut [f64],
        image: &mut [f64],
        residual: &[f64],
        solution: &mut [f64],
    ) -> Result<(), LinearAlgebraError> {
        if self.used == self.directions.len() {
            self.reset();
        }
        // Reorthogonalized modified Gram-Schmidt keeps the small history
        // reliable when consecutive V-cycles produce nearly parallel images.
        for _ in 0..2 {
            for index in 0..self.used {
                let q = &self.images[index];
                let p = &self.directions[index];
                let beta = dot(image, q)?;
                for row in 0..direction.len() {
                    image[row] -= beta * q[row];
                    direction[row] -= beta * p[row];
                }
            }
        }
        let norm = norm_l2(image)?;
        if norm == 0.0 {
            return Err(LinearAlgebraError::Breakdown {
                reason: "zero flexible GCR correction image",
            });
        }
        let p = &mut self.directions[self.used];
        let q = &mut self.images[self.used];
        for row in 0..direction.len() {
            p[row] = direction[row] / norm;
            q[row] = image[row] / norm;
            if !p[row].is_finite() || !q[row].is_finite() {
                return Err(LinearAlgebraError::Breakdown {
                    reason: "non-finite flexible GCR direction",
                });
            }
        }
        let alpha = dot(residual, q)?;
        for (value, &direction) in solution.iter_mut().zip(p.iter()) {
            *value += alpha * direction;
        }
        self.used += 1;
        Ok(())
    }
}
