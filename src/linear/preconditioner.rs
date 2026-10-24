use super::{CsrMatrix, LinearAlgebraError};
use rayon::prelude::*;

pub trait Preconditioner {
    fn len(&self) -> usize;
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
    fn apply_into(&self, residual: &[f64], output: &mut [f64]) -> Result<(), LinearAlgebraError>;
}

/// Diagonal preconditioner storing reciprocal diagonal coefficients.
#[derive(Clone, Debug, PartialEq)]
pub struct JacobiPreconditioner {
    inverse_diagonal: Vec<f64>,
}

impl JacobiPreconditioner {
    pub fn new(matrix: &CsrMatrix) -> Result<Self, LinearAlgebraError> {
        let _timer = crate::compute::StageTimer::new(crate::compute::Stage::PreconditionerSetup);
        let inverse_diagonal = matrix
            .diagonal()?
            .into_iter()
            .enumerate()
            .map(|(row, diagonal)| {
                let inverse = 1.0 / diagonal;
                if inverse.is_finite() {
                    Ok(inverse)
                } else {
                    Err(LinearAlgebraError::NonFiniteValue {
                        context: "inverse diagonal",
                        index: row,
                    })
                }
            })
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self { inverse_diagonal })
    }

    pub fn len(&self) -> usize {
        self.inverse_diagonal.len()
    }

    pub fn is_empty(&self) -> bool {
        self.inverse_diagonal.is_empty()
    }

    pub fn apply_into(
        &self,
        residual: &[f64],
        output: &mut [f64],
    ) -> Result<(), LinearAlgebraError> {
        if residual.len() != self.inverse_diagonal.len() {
            return Err(LinearAlgebraError::DimensionMismatch {
                expected: self.inverse_diagonal.len(),
                actual: residual.len(),
            });
        }
        if output.len() != self.inverse_diagonal.len() {
            return Err(LinearAlgebraError::DimensionMismatch {
                expected: self.inverse_diagonal.len(),
                actual: output.len(),
            });
        }
        let _timer = crate::compute::StageTimer::new(crate::compute::Stage::PreconditionerApply);
        let apply=|(index, ((residual,inverse),output)): (usize,((&f64,&f64),&mut f64))|->Result<(),LinearAlgebraError> {
            *output=residual*inverse;
            if !output.is_finite(){return Err(LinearAlgebraError::NonFiniteValue{context:"preconditioned residual",index});}
            Ok(())
        };
        if crate::compute::parallel_rows(output.len()) {
            residual
                .par_iter()
                .zip(self.inverse_diagonal.par_iter())
                .zip(output.par_iter_mut())
                .enumerate()
                .try_for_each(apply)
        } else {
            residual
                .iter()
                .zip(&self.inverse_diagonal)
                .zip(output.iter_mut())
                .enumerate()
                .try_for_each(apply)
        }
    }
}

impl Preconditioner for JacobiPreconditioner {
    fn len(&self) -> usize {
        self.len()
    }

    fn apply_into(&self, residual: &[f64], output: &mut [f64]) -> Result<(), LinearAlgebraError> {
        self.apply_into(residual, output)
    }
}

/// Forward/backward Gauss-Seidel preconditioner for nonsymmetric CSR systems.
/// Forward and backward Gauss-Seidel sweeps provide symmetric preconditioning.
#[derive(Clone, Debug)]
pub struct SymmetricGaussSeidelPreconditioner<'a> {
    matrix: &'a CsrMatrix,
    diagonal_positions: Vec<usize>,
    inverse_diagonal: Vec<f64>,
}

impl<'a> SymmetricGaussSeidelPreconditioner<'a> {
    pub fn new(matrix: &'a CsrMatrix) -> Result<Self, LinearAlgebraError> {
        let _timer = crate::compute::StageTimer::new(crate::compute::Stage::PreconditionerSetup);
        if matrix.nrows() != matrix.ncols() {
            return Err(LinearAlgebraError::DimensionMismatch {
                expected: matrix.nrows(),
                actual: matrix.ncols(),
            });
        }
        let mut diagonal_positions = Vec::with_capacity(matrix.nrows());
        let mut inverse_diagonal = Vec::with_capacity(matrix.nrows());
        for row in 0..matrix.nrows() {
            let start = matrix.row_offsets()[row];
            let columns = &matrix.column_indices()[start..matrix.row_offsets()[row + 1]];
            let position = columns
                .binary_search(&row)
                .map_err(|_| LinearAlgebraError::MissingDiagonal { row })?
                + start;
            let diagonal = matrix.values()[position];
            if !diagonal.is_finite() || diagonal == 0.0 {
                return Err(LinearAlgebraError::ZeroDiagonal { row });
            }
            let inverse = 1.0 / diagonal;
            if !inverse.is_finite() {
                return Err(LinearAlgebraError::NonFiniteValue {
                    context: "Gauss-Seidel inverse diagonal",
                    index: row,
                });
            }
            diagonal_positions.push(position);
            inverse_diagonal.push(inverse);
        }
        Ok(Self {
            matrix,
            diagonal_positions,
            inverse_diagonal,
        })
    }
}

impl Preconditioner for SymmetricGaussSeidelPreconditioner<'_> {
    fn len(&self) -> usize {
        self.matrix.nrows()
    }

    fn apply_into(&self, residual: &[f64], output: &mut [f64]) -> Result<(), LinearAlgebraError> {
        let _timer = crate::compute::StageTimer::new(crate::compute::Stage::PreconditionerApply);
        let n = self.len();
        if residual.len() != n {
            return Err(LinearAlgebraError::DimensionMismatch {
                expected: n,
                actual: residual.len(),
            });
        }
        if output.len() != n {
            return Err(LinearAlgebraError::DimensionMismatch {
                expected: n,
                actual: output.len(),
            });
        }

        let offsets = self.matrix.row_offsets();
        let columns = self.matrix.column_indices();
        let values = self.matrix.values();
        for row in 0..n {
            let mut value = residual[row];
            for entry in offsets[row]..self.diagonal_positions[row] {
                value -= values[entry] * output[columns[entry]];
            }
            output[row] = value * self.inverse_diagonal[row];
        }
        for row in (0..n).rev() {
            let mut upper_product = 0.0;
            for entry in self.diagonal_positions[row] + 1..offsets[row + 1] {
                upper_product += values[entry] * output[columns[entry]];
            }
            output[row] -= upper_product * self.inverse_diagonal[row];
            if !output[row].is_finite() {
                return Err(LinearAlgebraError::NonFiniteValue {
                    context: "Gauss-Seidel preconditioned residual",
                    index: row,
                });
            }
        }
        Ok(())
    }
}

/// Zero-fill incomplete Cholesky factorization for symmetric positive-definite CSR matrices.
#[derive(Clone, Debug, PartialEq)]
pub struct IncompleteCholeskyPreconditioner {
    lower_rows: Vec<Vec<(usize, f64)>>,
    diagonal: Vec<f64>,
    upper_columns: Vec<Vec<(usize, f64)>>,
}

impl IncompleteCholeskyPreconditioner {
    pub fn new(matrix: &CsrMatrix) -> Result<Self, LinearAlgebraError> {
        let _timer = crate::compute::StageTimer::new(crate::compute::Stage::PreconditionerSetup);
        if matrix.nrows() != matrix.ncols() {
            return Err(LinearAlgebraError::DimensionMismatch {
                expected: matrix.nrows(),
                actual: matrix.ncols(),
            });
        }
        let n = matrix.nrows();
        let mut lower_rows: Vec<Vec<(usize, f64)>> = Vec::with_capacity(n);
        let mut diagonal = vec![0.0; n];
        let offsets = matrix.row_offsets();
        let columns = matrix.column_indices();
        let values = matrix.values();

        for row in 0..n {
            for entry in offsets[row]..offsets[row + 1] {
                let column = columns[entry];
                if column >= row {
                    continue;
                }
                let transpose = matrix.get(column, row).unwrap_or(0.0);
                let value = values[entry];
                let scale = value.abs().max(transpose.abs()).max(1.0);
                if (value - transpose).abs() > 64.0 * f64::EPSILON * scale {
                    return Err(LinearAlgebraError::Breakdown {
                        reason: "incomplete Cholesky requires a symmetric matrix",
                    });
                }
            }
        }

        for row in 0..n {
            let mut lower: Vec<(usize, f64)> = Vec::new();
            for entry in offsets[row]..offsets[row + 1] {
                let column = columns[entry];
                if column >= row {
                    continue;
                }
                let mut value = values[entry];
                let mut left = 0;
                let mut right = 0;
                while left < lower.len() && right < lower_rows[column].len() {
                    let (left_column, left_value) = lower[left];
                    let (right_column, right_value) = lower_rows[column][right];
                    if left_column >= column || right_column >= column {
                        break;
                    }
                    match left_column.cmp(&right_column) {
                        std::cmp::Ordering::Less => left += 1,
                        std::cmp::Ordering::Greater => right += 1,
                        std::cmp::Ordering::Equal => {
                            value -= left_value * right_value;
                            left += 1;
                            right += 1;
                        }
                    }
                }
                lower.push((column, value / diagonal[column]));
            }

            let mut pivot = matrix
                .get(row, row)
                .ok_or(LinearAlgebraError::MissingDiagonal { row })?;
            for &(_, coefficient) in &lower {
                pivot -= coefficient * coefficient;
            }
            // Pivot acceptance must be invariant under a positive rescaling of A.
            let scale = matrix.get(row, row).unwrap_or(0.0).abs();
            if !pivot.is_finite() || pivot <= 0.0 || pivot <= f64::EPSILON * scale {
                return Err(LinearAlgebraError::IncompleteCholeskyPivot {
                    row,
                    pivot,
                    diagonal: scale,
                });
            }
            diagonal[row] = pivot.sqrt();
            lower_rows.push(lower);
        }

        let mut upper_columns = vec![Vec::new(); n];
        for (row, lower) in lower_rows.iter().enumerate() {
            for &(column, value) in lower {
                upper_columns[column].push((row, value));
            }
        }
        Ok(Self {
            lower_rows,
            diagonal,
            upper_columns,
        })
    }
}

impl Preconditioner for IncompleteCholeskyPreconditioner {
    fn len(&self) -> usize {
        self.diagonal.len()
    }

    fn apply_into(&self, residual: &[f64], output: &mut [f64]) -> Result<(), LinearAlgebraError> {
        let _timer = crate::compute::StageTimer::new(crate::compute::Stage::PreconditionerApply);
        let n = self.diagonal.len();
        if residual.len() != n {
            return Err(LinearAlgebraError::DimensionMismatch {
                expected: n,
                actual: residual.len(),
            });
        }
        if output.len() != n {
            return Err(LinearAlgebraError::DimensionMismatch {
                expected: n,
                actual: output.len(),
            });
        }
        for row in 0..n {
            let mut value = residual[row];
            for &(column, coefficient) in &self.lower_rows[row] {
                value -= coefficient * output[column];
            }
            output[row] = value / self.diagonal[row];
        }
        for row in (0..n).rev() {
            let mut value = output[row];
            for &(upper_row, coefficient) in &self.upper_columns[row] {
                value -= coefficient * output[upper_row];
            }
            output[row] = value / self.diagonal[row];
            if !output[row].is_finite() {
                return Err(LinearAlgebraError::NonFiniteValue {
                    context: "incomplete Cholesky preconditioned residual",
                    index: row,
                });
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod ic_scaling_tests {
    use super::*;
    #[test]
    fn ic_acceptance_is_invariant_under_positive_matrix_scaling() {
        for scale in [1e-24, 1.0, 1e24] {
            let mut b = super::super::CsrBuilder::new(2, 2);
            for (i, j, v) in [(0, 0, 2.0), (0, 1, -1.0), (1, 0, -1.0), (1, 1, 2.0)] {
                b.add(i, j, v * scale).unwrap();
            }
            let a = b.finalize().unwrap();
            let p = IncompleteCholeskyPreconditioner::new(&a).unwrap();
            let mut x = [0.0; 2];
            p.apply_into(&[scale, scale], &mut x).unwrap();
            assert!((x[0] - 1.0).abs() < 1e-12);
            assert!((x[1] - 1.0).abs() < 1e-12);
        }
    }
    #[test]
    fn ic_still_rejects_a_genuinely_negative_pivot() {
        let mut b = super::super::CsrBuilder::new(2, 2);
        for (i, j, v) in [(0, 0, 1.0), (0, 1, 2.0), (1, 0, 2.0), (1, 1, 1.0)] {
            b.add(i, j, v).unwrap();
        }
        assert!(matches!(
            IncompleteCholeskyPreconditioner::new(&b.finalize().unwrap()),
            Err(LinearAlgebraError::IncompleteCholeskyPivot { row: 1, .. })
        ));
    }
}
