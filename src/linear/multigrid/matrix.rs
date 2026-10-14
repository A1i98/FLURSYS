use crate::{CsrMatrix, LinearAlgebraError};

/// Symmetric face-addressed matrix. Each off-diagonal coefficient is stored once.
pub(super) struct SymmetricMatrix {
    pub diagonal: Vec<f64>,
    pub inverse: Vec<f64>,
    pub offsets: Vec<usize>,
    pub neighbors: Vec<u32>,
    pub coefficients: Vec<f64>,
    diagonal_entries: Vec<usize>,
    face_entries: Vec<usize>,
    transpose_entries: Vec<usize>,
}

impl SymmetricMatrix {
    pub fn new(matrix: &CsrMatrix) -> Result<Self, LinearAlgebraError> {
        if matrix.nrows() > u32::MAX as usize || !matrix.is_symmetric(1e-12) {
            return Err(LinearAlgebraError::Breakdown {
                reason: "multigrid requires a symmetric matrix with 32-bit addressing",
            });
        }
        let n = matrix.nrows();
        let mut result = Self {
            diagonal: vec![0.0; n],
            inverse: vec![0.0; n],
            offsets: vec![0],
            neighbors: Vec::new(),
            coefficients: Vec::new(),
            diagonal_entries: Vec::with_capacity(n),
            face_entries: Vec::new(),
            transpose_entries: Vec::new(),
        };
        for row in 0..n {
            let mut diagonal = None;
            for entry in matrix.row_offsets()[row]..matrix.row_offsets()[row + 1] {
                let column = matrix.column_indices()[entry];
                if column == row {
                    diagonal = Some(entry);
                }
                if column > row {
                    result.neighbors.push(column as u32);
                    result.face_entries.push(entry);
                    let start = matrix.row_offsets()[column];
                    let end = matrix.row_offsets()[column + 1];
                    let transpose = matrix.column_indices()[start..end]
                        .binary_search(&row)
                        .map_err(|_| LinearAlgebraError::Breakdown {
                            reason: "missing symmetric face coefficient",
                        })?;
                    result.transpose_entries.push(start + transpose);
                    result.coefficients.push(0.0);
                }
            }
            result
                .diagonal_entries
                .push(diagonal.ok_or(LinearAlgebraError::MissingDiagonal { row })?);
            result.offsets.push(result.neighbors.len());
        }
        result.update(matrix)?;
        Ok(result)
    }

    pub fn coupled_rows(&self) -> usize {
        let mut coupled = vec![false; self.diagonal.len()];
        for row in 0..coupled.len() {
            for face in self.offsets[row]..self.offsets[row + 1] {
                coupled[row] = true;
                coupled[self.neighbors[face] as usize] = true;
            }
        }
        coupled.into_iter().filter(|&value| value).count()
    }

    /// Topology has already been checked by the caller. Cached mirrored
    /// positions avoid a binary search for every entry on every time step.
    pub fn is_symmetric(&self, matrix: &CsrMatrix, tolerance: f64) -> bool {
        let values = matrix.values();
        self.face_entries
            .iter()
            .zip(&self.transpose_entries)
            .all(|(&upper, &lower)| {
                let (a, b) = (values[upper], values[lower]);
                a.is_finite() && b.is_finite() && (a - b).abs() <= tolerance
            })
    }

    pub fn update(&mut self, matrix: &CsrMatrix) -> Result<(), LinearAlgebraError> {
        for row in 0..self.diagonal.len() {
            let d = matrix.values()[self.diagonal_entries[row]];
            if !d.is_finite() || d <= 0.0 {
                return Err(LinearAlgebraError::Breakdown {
                    reason: "multigrid requires positive finite diagonal coefficients",
                });
            }
            self.diagonal[row] = d;
            self.inverse[row] = 1.0 / d;
            if !self.inverse[row].is_finite() {
                return Err(LinearAlgebraError::ZeroDiagonal { row });
            }
        }
        for (value, &entry) in self.coefficients.iter_mut().zip(&self.face_entries) {
            *value = matrix.values()[entry];
            if !value.is_finite() {
                return Err(LinearAlgebraError::NonFiniteValue {
                    context: "multigrid face coefficient",
                    index: entry,
                });
            }
        }
        Ok(())
    }

    pub fn symmetrize_coarse_values(&self, matrix: &mut CsrMatrix) {
        let values = matrix.values_mut();
        for (&upper, &lower) in self.face_entries.iter().zip(&self.transpose_entries) {
            let mean = 0.5 * values[upper] + 0.5 * values[lower];
            values[upper] = mean;
            values[lower] = mean;
        }
    }

    pub fn apply(&self, x: &[f64], output: &mut [f64]) {
        for ((value, &d), &x) in output.iter_mut().zip(&self.diagonal).zip(x) {
            *value = d * x;
        }
        for row in 0..self.diagonal.len() {
            let mut owner = output[row];
            let range = self.offsets[row]..self.offsets[row + 1];
            let owner_x = x[row];
            for (&neighbor, &a) in self.neighbors[range.clone()]
                .iter()
                .zip(&self.coefficients[range])
            {
                let neighbor = neighbor as usize;
                owner += a * x[neighbor];
                output[neighbor] += a * owner_x;
            }
            output[row] = owner;
        }
    }

    /// Fuse b - A*x for smoothing, avoiding a separate full-vector pass.
    pub fn residual(&self, x: &[f64], rhs: &[f64], output: &mut [f64]) {
        for row in 0..rhs.len() {
            output[row] = rhs[row] - self.diagonal[row] * x[row];
        }
        for row in 0..rhs.len() {
            let mut owner = output[row];
            let owner_x = x[row];
            let range = self.offsets[row]..self.offsets[row + 1];
            for (&neighbor, &a) in self.neighbors[range.clone()]
                .iter()
                .zip(&self.coefficients[range])
            {
                let neighbor = neighbor as usize;
                owner -= a * x[neighbor];
                output[neighbor] -= a * owner_x;
            }
            output[row] = owner;
        }
    }

    pub fn row_sums_into(&self, sums: &mut [f64]) {
        sums.copy_from_slice(&self.diagonal);
        for row in 0..sums.len() {
            for face in self.offsets[row]..self.offsets[row + 1] {
                let neighbor = self.neighbors[face] as usize;
                sums[row] += self.coefficients[face];
                sums[neighbor] += self.coefficients[face];
            }
        }
    }
}
