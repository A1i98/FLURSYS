use super::{CsrMatrix, LinearAlgebraError};

pub trait Preconditioner {
    fn len(&self) -> usize;
    fn apply_into(&self, residual: &[f64], output: &mut [f64]) -> Result<(), LinearAlgebraError>;
}

/// Diagonal preconditioner storing reciprocal diagonal coefficients.
#[derive(Clone, Debug, PartialEq)]
pub struct JacobiPreconditioner {
    inverse_diagonal: Vec<f64>,
}

impl JacobiPreconditioner {
    pub fn new(matrix: &CsrMatrix) -> Result<Self, LinearAlgebraError> {
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
        for (index, ((residual, inverse), output)) in residual
            .iter()
            .zip(&self.inverse_diagonal)
            .zip(output)
            .enumerate()
        {
            *output = residual * inverse;
            if !output.is_finite() {
                return Err(LinearAlgebraError::NonFiniteValue {
                    context: "preconditioned residual",
                    index,
                });
            }
        }
        Ok(())
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
            let scale = matrix.get(row, row).unwrap_or(1.0).abs().max(1.0);
            if !pivot.is_finite() || pivot <= f64::EPSILON * scale {
                return Err(LinearAlgebraError::Breakdown {
                    reason: "non-positive incomplete Cholesky pivot",
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

/// Structured 2x2 aggregation multigrid V-cycle for symmetric positive-definite
/// systems on a cell-centred rectangular layout. Coarse operators use Galerkin
/// products with piecewise-constant prolongation; forward and backward
/// Gauss-Seidel sweeps bracket the coarse-grid correction symmetrically.
#[derive(Clone, Debug, PartialEq)]
pub struct StructuredMultigridPreconditioner {
    levels: Vec<MultigridLevel>,
}

#[derive(Clone, Debug, PartialEq)]
struct MultigridLevel {
    matrix: CsrMatrix,
    inverse_diagonal: Vec<f64>,
    aggregates: Option<Vec<usize>>,
    coarse_entry_map: Option<Vec<usize>>,
    coarse_size: usize,
    coarse_cholesky: Option<Vec<f64>>,
}

impl StructuredMultigridPreconditioner {
    pub fn new(
        matrix: &CsrMatrix,
        width: usize,
        height: usize,
    ) -> Result<Self, LinearAlgebraError> {
        let expected = width
            .checked_mul(height)
            .ok_or_else(|| LinearAlgebraError::InvalidCsr {
                message: "multigrid grid dimensions overflow".to_string(),
            })?;
        if width < 2 || height < 2 || matrix.nrows() != expected || matrix.ncols() != expected {
            return Err(LinearAlgebraError::DimensionMismatch {
                expected,
                actual: matrix.nrows(),
            });
        }
        if !matrix.is_symmetric(1.0e-12) {
            return Err(LinearAlgebraError::Breakdown {
                reason: "structured multigrid requires a symmetric matrix",
            });
        }

        let mut levels = Vec::new();
        let mut current = matrix.clone();
        let mut current_width = width;
        let mut current_height = height;
        loop {
            let inverse_diagonal = multigrid_inverse_diagonal(&current)?;

            if current.nrows() <= 16 {
                let coarse_cholesky = dense_cholesky(&current)?;
                levels.push(MultigridLevel {
                    matrix: current,
                    inverse_diagonal,
                    aggregates: None,
                    coarse_entry_map: None,
                    coarse_size: 0,
                    coarse_cholesky: Some(coarse_cholesky),
                });
                break;
            }

            let coarse_width = current_width.div_ceil(2);
            let coarse_height = current_height.div_ceil(2);
            let coarse_size = coarse_width * coarse_height;
            if coarse_size >= current.nrows() {
                return Err(LinearAlgebraError::Breakdown {
                    reason: "multigrid coarsening did not reduce the system",
                });
            }
            let aggregates = (0..current.nrows())
                .map(|row| {
                    let x = row % current_width;
                    let y = row / current_width;
                    (x / 2) + coarse_width * (y / 2)
                })
                .collect::<Vec<_>>();
            let mut coarse_builder = super::CsrBuilder::new(coarse_size, coarse_size);
            for row in 0..current.nrows() {
                for entry in current.row_offsets()[row]..current.row_offsets()[row + 1] {
                    coarse_builder.add(
                        aggregates[row],
                        aggregates[current.column_indices()[entry]],
                        current.values()[entry],
                    )?;
                }
            }
            let coarse = coarse_builder.finalize()?;
            let mut coarse_entry_map = Vec::with_capacity(current.nnz());
            for row in 0..current.nrows() {
                let coarse_row = aggregates[row];
                for entry in current.row_offsets()[row]..current.row_offsets()[row + 1] {
                    let coarse_column = aggregates[current.column_indices()[entry]];
                    let start = coarse.row_offsets()[coarse_row];
                    let end = coarse.row_offsets()[coarse_row + 1];
                    let offset = coarse.column_indices()[start..end]
                        .binary_search(&coarse_column)
                        .map_err(|_| LinearAlgebraError::InvalidCsr {
                            message: "multigrid coarse sparsity map is incomplete".to_string(),
                        })?;
                    coarse_entry_map.push(start + offset);
                }
            }
            levels.push(MultigridLevel {
                matrix: current,
                inverse_diagonal,
                aggregates: Some(aggregates),
                coarse_entry_map: Some(coarse_entry_map),
                coarse_size,
                coarse_cholesky: None,
            });
            current = coarse;
            current_width = coarse_width;
            current_height = coarse_height;
        }
        Ok(Self { levels })
    }

    /// Update matrix coefficients while reusing the fixed aggregation and
    /// coarse sparsity maps for repeated pressure corrections.
    pub fn update_values(&mut self, matrix: &CsrMatrix) -> Result<(), LinearAlgebraError> {
        let Some(fine) = self.levels.first_mut() else {
            return Err(LinearAlgebraError::InvalidCsr {
                message: "multigrid hierarchy has no levels".to_string(),
            });
        };
        if fine.matrix.nrows() != matrix.nrows()
            || fine.matrix.ncols() != matrix.ncols()
            || fine.matrix.row_offsets() != matrix.row_offsets()
            || fine.matrix.column_indices() != matrix.column_indices()
        {
            return Err(LinearAlgebraError::InvalidCsr {
                message: "multigrid update changed the matrix sparsity pattern".to_string(),
            });
        }
        fine.matrix.values_mut().copy_from_slice(matrix.values());

        for level_index in 0..self.levels.len() {
            let (current_levels, coarse_levels) = self.levels.split_at_mut(level_index + 1);
            let level = &mut current_levels[level_index];
            level.inverse_diagonal = multigrid_inverse_diagonal(&level.matrix)?;

            if let Some(next_levels) = coarse_levels.first_mut() {
                let map = level
                    .coarse_entry_map
                    .as_ref()
                    .ok_or(LinearAlgebraError::Breakdown {
                        reason: "multigrid level is missing its coarse sparsity map",
                    })?;
                let coarse = &mut next_levels.matrix;
                coarse.values_mut().fill(0.0);
                for (&value, &coarse_entry) in level.matrix.values().iter().zip(map) {
                    coarse.values_mut()[coarse_entry] += value;
                }
            } else {
                level.coarse_cholesky = Some(dense_cholesky(&level.matrix)?);
            }
        }
        Ok(())
    }

    fn v_cycle(
        &self,
        level_index: usize,
        rhs: &[f64],
        solution: &mut [f64],
    ) -> Result<(), LinearAlgebraError> {
        let level = &self.levels[level_index];
        if let Some(factor) = &level.coarse_cholesky {
            dense_cholesky_solve(factor, rhs, solution)?;
            return Ok(());
        }

        solution.fill(0.0);
        gauss_seidel_smooth(
            &level.matrix,
            &level.inverse_diagonal,
            rhs,
            solution,
            true,
            2,
        )?;
        let mut product = vec![0.0; rhs.len()];
        level.matrix.spmv_into(solution, &mut product)?;
        let residual = rhs
            .iter()
            .zip(product)
            .map(|(right, applied)| right - applied)
            .collect::<Vec<_>>();
        let mut coarse_rhs = vec![0.0; level.coarse_size];
        let aggregates = level
            .aggregates
            .as_ref()
            .ok_or(LinearAlgebraError::Breakdown {
                reason: "multigrid level is missing its aggregate map",
            })?;
        for (row, &value) in residual.iter().enumerate() {
            coarse_rhs[aggregates[row]] += value;
        }
        let mut coarse_solution = vec![0.0; level.coarse_size];
        self.v_cycle(level_index + 1, &coarse_rhs, &mut coarse_solution)?;
        for (row, &aggregate) in aggregates.iter().enumerate() {
            solution[row] += coarse_solution[aggregate];
        }
        gauss_seidel_smooth(
            &level.matrix,
            &level.inverse_diagonal,
            rhs,
            solution,
            false,
            2,
        )
    }
}

fn multigrid_inverse_diagonal(matrix: &CsrMatrix) -> Result<Vec<f64>, LinearAlgebraError> {
    matrix
        .diagonal()?
        .into_iter()
        .enumerate()
        .map(|(index, value)| {
            if !value.is_finite() || value <= 0.0 {
                return Err(LinearAlgebraError::Breakdown {
                    reason: "multigrid requires a positive diagonal",
                });
            }
            let inverse = 1.0 / value;
            if !inverse.is_finite() {
                return Err(LinearAlgebraError::NonFiniteValue {
                    context: "multigrid inverse diagonal",
                    index,
                });
            }
            Ok(inverse)
        })
        .collect()
}

impl Preconditioner for StructuredMultigridPreconditioner {
    fn len(&self) -> usize {
        self.levels.first().map_or(0, |level| level.matrix.nrows())
    }

    fn apply_into(&self, residual: &[f64], output: &mut [f64]) -> Result<(), LinearAlgebraError> {
        let expected = self.len();
        if residual.len() != expected {
            return Err(LinearAlgebraError::DimensionMismatch {
                expected,
                actual: residual.len(),
            });
        }
        if output.len() != expected {
            return Err(LinearAlgebraError::DimensionMismatch {
                expected,
                actual: output.len(),
            });
        }
        self.v_cycle(0, residual, output)
    }
}

fn gauss_seidel_smooth(
    matrix: &CsrMatrix,
    inverse_diagonal: &[f64],
    rhs: &[f64],
    solution: &mut [f64],
    forward: bool,
    iterations: usize,
) -> Result<(), LinearAlgebraError> {
    let offsets = matrix.row_offsets();
    let columns = matrix.column_indices();
    let values = matrix.values();
    for _ in 0..iterations {
        for index in 0..rhs.len() {
            let row = if forward {
                index
            } else {
                rhs.len() - 1 - index
            };
            let mut value = rhs[row];
            for entry in offsets[row]..offsets[row + 1] {
                if columns[entry] != row {
                    value -= values[entry] * solution[columns[entry]];
                }
            }
            solution[row] = value * inverse_diagonal[row];
            if !solution[row].is_finite() {
                return Err(LinearAlgebraError::NonFiniteValue {
                    context: "multigrid Gauss-Seidel smoother",
                    index: row,
                });
            }
        }
    }
    Ok(())
}

fn dense_cholesky(matrix: &CsrMatrix) -> Result<Vec<f64>, LinearAlgebraError> {
    let n = matrix.nrows();
    let mut lower = vec![0.0; n * n];
    for row in 0..n {
        for column in 0..=row {
            let mut value = matrix.get(row, column).unwrap_or(0.0);
            for inner in 0..column {
                value -= lower[row * n + inner] * lower[column * n + inner];
            }
            if row == column {
                if !value.is_finite() || value <= 0.0 {
                    return Err(LinearAlgebraError::Breakdown {
                        reason: "multigrid coarse matrix is not positive definite",
                    });
                }
                lower[row * n + column] = value.sqrt();
            } else {
                lower[row * n + column] = value / lower[column * n + column];
            }
        }
    }
    Ok(lower)
}

fn dense_cholesky_solve(
    lower: &[f64],
    rhs: &[f64],
    solution: &mut [f64],
) -> Result<(), LinearAlgebraError> {
    let n = rhs.len();
    if lower.len() != n * n || solution.len() != n {
        return Err(LinearAlgebraError::DimensionMismatch {
            expected: lower.len(),
            actual: n * n,
        });
    }
    solution.copy_from_slice(rhs);
    for row in 0..n {
        for column in 0..row {
            solution[row] -= lower[row * n + column] * solution[column];
        }
        solution[row] /= lower[row * n + row];
    }
    for row in (0..n).rev() {
        for column in row + 1..n {
            solution[row] -= lower[column * n + row] * solution[column];
        }
        solution[row] /= lower[row * n + row];
        if !solution[row].is_finite() {
            return Err(LinearAlgebraError::NonFiniteValue {
                context: "multigrid coarse solution",
                index: row,
            });
        }
    }
    Ok(())
}
