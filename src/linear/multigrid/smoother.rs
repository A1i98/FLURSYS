use super::{matrix::SymmetricMatrix, MultigridSmoother};
use crate::LinearAlgebraError;

pub(super) struct DicPreconditioner<'a> {
    pub matrix: &'a SymmetricMatrix,
    pub inverse: &'a [f64],
}

impl crate::Preconditioner for DicPreconditioner<'_> {
    fn len(&self) -> usize {
        self.inverse.len()
    }
    fn apply_into(&self, rhs: &[f64], output: &mut [f64]) -> Result<(), LinearAlgebraError> {
        for size in [rhs.len(), output.len()] {
            if size != self.len() {
                return Err(LinearAlgebraError::DimensionMismatch {
                    expected: self.len(),
                    actual: size,
                });
            }
        }
        output.copy_from_slice(rhs);
        for row in 0..rhs.len() {
            output[row] *= self.inverse[row];
            for face in self.matrix.offsets[row]..self.matrix.offsets[row + 1] {
                output[self.matrix.neighbors[face] as usize] -=
                    self.matrix.coefficients[face] * output[row];
            }
        }
        for row in (0..rhs.len()).rev() {
            let mut upper = 0.0;
            for face in self.matrix.offsets[row]..self.matrix.offsets[row + 1] {
                upper +=
                    self.matrix.coefficients[face] * output[self.matrix.neighbors[face] as usize];
            }
            output[row] -= self.inverse[row] * upper;
        }
        if output.iter().any(|x| !x.is_finite()) {
            return Err(LinearAlgebraError::Breakdown {
                reason: "non-finite DIC correction",
            });
        }
        Ok(())
    }
}

pub(super) fn dic_diagonal(matrix: &SymmetricMatrix) -> Result<Vec<f64>, LinearAlgebraError> {
    let mut diagonal = vec![0.0; matrix.diagonal.len()];
    update_dic_diagonal(matrix, &mut diagonal)?;
    Ok(diagonal)
}

pub(super) fn update_dic_diagonal(
    matrix: &SymmetricMatrix,
    diagonal: &mut [f64],
) -> Result<(), LinearAlgebraError> {
    diagonal.copy_from_slice(&matrix.diagonal);
    for row in 0..diagonal.len() {
        if !diagonal[row].is_finite() || diagonal[row] <= 0.0 {
            return Err(LinearAlgebraError::Breakdown {
                reason: "non-positive DIC smoothing pivot",
            });
        }
        for face in matrix.offsets[row]..matrix.offsets[row + 1] {
            let neighbor = matrix.neighbors[face] as usize;
            diagonal[neighbor] -= matrix.coefficients[face].powi(2) / diagonal[row];
        }
    }
    for d in diagonal {
        *d = 1.0 / *d;
        if !d.is_finite() {
            return Err(LinearAlgebraError::Breakdown {
                reason: "non-finite DIC inverse",
            });
        }
    }
    Ok(())
}

fn gauss_seidel(
    matrix: &SymmetricMatrix,
    rhs: &[f64],
    x: &mut [f64],
    work: &mut [f64],
    symmetric: bool,
) {
    work.copy_from_slice(rhs);
    for row in 0..rhs.len() {
        let mut value = work[row];
        for face in matrix.offsets[row]..matrix.offsets[row + 1] {
            value -= matrix.coefficients[face] * x[matrix.neighbors[face] as usize];
        }
        x[row] = value * matrix.inverse[row];
        for face in matrix.offsets[row]..matrix.offsets[row + 1] {
            work[matrix.neighbors[face] as usize] -= matrix.coefficients[face] * x[row];
        }
    }
    if symmetric {
        for row in (0..rhs.len()).rev() {
            let mut value = work[row];
            for face in matrix.offsets[row]..matrix.offsets[row + 1] {
                value -= matrix.coefficients[face] * x[matrix.neighbors[face] as usize];
            }
            x[row] = value * matrix.inverse[row];
        }
    }
}

pub(super) fn smooth(
    matrix: &SymmetricMatrix,
    inverse_dic: &[f64],
    kind: MultigridSmoother,
    rhs: &[f64],
    x: &mut [f64],
    work: &mut [f64],
    correction: &mut [f64],
    sweeps: usize,
) -> Result<(), LinearAlgebraError> {
    if sweeps == 0 {
        return Ok(());
    }
    if kind == MultigridSmoother::DicGaussSeidel {
        for _ in 0..sweeps {
            matrix.residual(x, rhs, correction);
            for row in 0..rhs.len() {
                let value = correction[row] * inverse_dic[row];
                correction[row] = value;
                let range = matrix.offsets[row]..matrix.offsets[row + 1];
                for (&neighbor, &a) in matrix.neighbors[range.clone()]
                    .iter()
                    .zip(&matrix.coefficients[range])
                {
                    correction[neighbor as usize] -= a * value;
                }
            }
            for row in (0..rhs.len()).rev() {
                let mut upper = 0.0;
                let range = matrix.offsets[row]..matrix.offsets[row + 1];
                for (&neighbor, &a) in matrix.neighbors[range.clone()]
                    .iter()
                    .zip(&matrix.coefficients[range])
                {
                    upper += a * correction[neighbor as usize];
                }
                correction[row] -= inverse_dic[row] * upper;
                x[row] += correction[row];
            }
        }
    }
    for _ in 0..sweeps {
        gauss_seidel(
            matrix,
            rhs,
            x,
            work,
            kind == MultigridSmoother::SymmetricGaussSeidel,
        );
    }
    if x.iter().any(|x| !x.is_finite()) {
        return Err(LinearAlgebraError::Breakdown {
            reason: "non-finite multigrid smoother output",
        });
    }
    Ok(())
}
