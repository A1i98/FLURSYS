use super::LinearAlgebraError;

/// Immutable compressed-sparse-row matrix with strictly ascending row columns.
#[derive(Clone, Debug, PartialEq)]
pub struct CsrMatrix {
    nrows: usize,
    ncols: usize,
    row_offsets: Vec<usize>,
    column_indices: Vec<usize>,
    values: Vec<f64>,
}

impl CsrMatrix {
    pub fn new(
        nrows: usize,
        ncols: usize,
        row_offsets: Vec<usize>,
        column_indices: Vec<usize>,
        values: Vec<f64>,
    ) -> Result<Self, LinearAlgebraError> {
        validate_csr(nrows, ncols, &row_offsets, &column_indices, &values)?;
        Ok(Self {
            nrows,
            ncols,
            row_offsets,
            column_indices,
            values,
        })
    }

    pub fn nrows(&self) -> usize {
        self.nrows
    }
    pub fn ncols(&self) -> usize {
        self.ncols
    }
    pub fn nnz(&self) -> usize {
        self.values.len()
    }
    pub fn row_offsets(&self) -> &[usize] {
        &self.row_offsets
    }
    pub fn column_indices(&self) -> &[usize] {
        &self.column_indices
    }
    pub fn values(&self) -> &[f64] {
        &self.values
    }

    pub(crate) fn values_mut(&mut self) -> &mut [f64] {
        &mut self.values
    }

    pub fn get(&self, row: usize, column: usize) -> Option<f64> {
        if row >= self.nrows || column >= self.ncols {
            return None;
        }
        let entries = &self.column_indices[self.row_offsets[row]..self.row_offsets[row + 1]];
        entries
            .binary_search(&column)
            .ok()
            .map(|entry| self.values[self.row_offsets[row] + entry])
    }

    pub fn diagonal(&self) -> Result<Vec<f64>, LinearAlgebraError> {
        (0..self.nrows)
            .map(|row| self.diagonal_entry(row))
            .collect()
    }

    pub(crate) fn diagonal_entry(&self, row: usize) -> Result<f64, LinearAlgebraError> {
        let value = self
            .get(row, row)
            .ok_or(LinearAlgebraError::MissingDiagonal { row })?;
        if !value.is_finite() {
            return Err(LinearAlgebraError::NonFiniteValue {
                context: "diagonal",
                index: row,
            });
        }
        if value == 0.0 {
            return Err(LinearAlgebraError::ZeroDiagonal { row });
        }
        Ok(value)
    }

    pub fn spmv(&self, input: &[f64]) -> Result<Vec<f64>, LinearAlgebraError> {
        let mut output = vec![0.0; self.nrows];
        self.spmv_into(input, &mut output)?;
        Ok(output)
    }

    pub fn spmv_into(&self, input: &[f64], output: &mut [f64]) -> Result<(), LinearAlgebraError> {
        ensure_len(self.ncols, input.len())?;
        ensure_len(self.nrows, output.len())?;
        for (row, output) in output.iter_mut().enumerate() {
            let start = self.row_offsets[row];
            let end = self.row_offsets[row + 1];
            let value = match end - start {
                0 => 0.0,
                1 => self.values[start] * input[self.column_indices[start]],
                2 => {
                    self.values[start] * input[self.column_indices[start]]
                        + self.values[start + 1] * input[self.column_indices[start + 1]]
                }
                3 => {
                    let mut sum = self.values[start] * input[self.column_indices[start]];
                    sum += self.values[start + 1] * input[self.column_indices[start + 1]];
                    sum += self.values[start + 2] * input[self.column_indices[start + 2]];
                    sum
                }
                4 => {
                    let mut sum = self.values[start] * input[self.column_indices[start]];
                    sum += self.values[start + 1] * input[self.column_indices[start + 1]];
                    sum += self.values[start + 2] * input[self.column_indices[start + 2]];
                    sum += self.values[start + 3] * input[self.column_indices[start + 3]];
                    sum
                }
                5 => {
                    let mut sum = self.values[start] * input[self.column_indices[start]];
                    sum += self.values[start + 1] * input[self.column_indices[start + 1]];
                    sum += self.values[start + 2] * input[self.column_indices[start + 2]];
                    sum += self.values[start + 3] * input[self.column_indices[start + 3]];
                    sum += self.values[start + 4] * input[self.column_indices[start + 4]];
                    sum
                }
                _ => {
                    let mut sum = 0.0;
                    for entry in start..end {
                        sum += self.values[entry] * input[self.column_indices[entry]];
                    }
                    sum
                }
            };
            if !value.is_finite() {
                return Err(LinearAlgebraError::NonFiniteValue {
                    context: "SpMV output",
                    index: row,
                });
            }
            *output = value;
        }
        Ok(())
    }

    pub fn is_symmetric(&self, tolerance: f64) -> bool {
        if self.nrows != self.ncols || !tolerance.is_finite() || tolerance < 0.0 {
            return false;
        }
        for row in 0..self.nrows {
            for entry in self.row_offsets[row]..self.row_offsets[row + 1] {
                let column = self.column_indices[entry];
                let Some(transpose) = self.get(column, row) else {
                    return false;
                };
                if (self.values[entry] - transpose).abs() > tolerance {
                    return false;
                }
            }
        }
        true
    }
}

/// Deterministic triplet assembler. Finalization sorts each row, sums duplicates,
/// and drops exact zero sums without tolerance pruning.
#[derive(Clone, Debug)]
pub struct CsrBuilder {
    nrows: usize,
    ncols: usize,
    entries: Vec<(usize, usize, f64)>,
    row_offsets: Vec<usize>,
    cursors: Vec<usize>,
    entries_by_row: Vec<(usize, f64)>,
}

impl CsrBuilder {
    pub fn new(nrows: usize, ncols: usize) -> Self {
        Self::with_capacity(nrows, ncols, 0)
    }

    /// Create an assembler with room for the expected number of triplets.
    pub fn with_capacity(nrows: usize, ncols: usize, capacity: usize) -> Self {
        Self {
            nrows,
            ncols,
            entries: Vec::with_capacity(capacity),
            row_offsets: vec![0; nrows + 1],
            cursors: vec![0; nrows],
            entries_by_row: Vec::with_capacity(capacity),
        }
    }

    /// Clear triplets while retaining all assembly and finalization buffers.
    pub fn clear(&mut self) {
        self.entries.clear();
    }

    pub fn add(&mut self, row: usize, column: usize, value: f64) -> Result<(), LinearAlgebraError> {
        if row >= self.nrows || column >= self.ncols {
            return Err(LinearAlgebraError::IndexOutOfBounds { row, column });
        }
        if !value.is_finite() {
            return Err(LinearAlgebraError::NonFiniteValue {
                context: "matrix",
                index: self.entries.len(),
            });
        }
        self.entries.push((row, column, value));
        Ok(())
    }

    pub fn finalize(mut self) -> Result<CsrMatrix, LinearAlgebraError> {
        let mut matrix = CsrMatrix {
            nrows: self.nrows,
            ncols: self.ncols,
            row_offsets: Vec::with_capacity(self.nrows + 1),
            column_indices: Vec::with_capacity(self.entries.len()),
            values: Vec::with_capacity(self.entries.len()),
        };
        self.finalize_into(&mut matrix)?;
        Ok(matrix)
    }

    /// Finalize into an existing matrix, reusing its storage and the builder's
    /// scratch arrays for repeated assembly of same-sized systems.
    pub fn finalize_into(&mut self, matrix: &mut CsrMatrix) -> Result<(), LinearAlgebraError> {
        if matrix.nrows != self.nrows || matrix.ncols != self.ncols {
            return Err(LinearAlgebraError::DimensionMismatch {
                expected: self.nrows,
                actual: matrix.nrows,
            });
        }
        // Group by row in linear time, then sort only each short row. The
        // previous whole-triplet sort ordered (row, column) over the complete
        // matrix every time SIMPLE assembled a new coefficient matrix.
        self.row_offsets.fill(0);
        for &(row, _, _) in &self.entries {
            self.row_offsets[row + 1] += 1;
        }
        for row in 0..self.nrows {
            self.row_offsets[row + 1] += self.row_offsets[row];
        }

        self.cursors
            .copy_from_slice(&self.row_offsets[..self.nrows]);
        self.entries_by_row.clear();
        self.entries_by_row.resize(self.entries.len(), (0, 0.0));
        for &(row, column, value) in &self.entries {
            let slot = self.cursors[row];
            self.entries_by_row[slot] = (column, value);
            self.cursors[row] += 1;
        }

        matrix.row_offsets.clear();
        matrix.column_indices.clear();
        matrix.values.clear();
        matrix.row_offsets.push(0);
        for row in 0..self.nrows {
            let start = self.row_offsets[row];
            let end = self.row_offsets[row + 1];
            let row_entries = &mut self.entries_by_row[start..end];
            row_entries.sort_unstable_by_key(|&(column, _)| column);

            let mut entry = 0;
            while entry < row_entries.len() {
                let column = row_entries[entry].0;
                let mut value = 0.0;
                while entry < row_entries.len() && row_entries[entry].0 == column {
                    value += row_entries[entry].1;
                    entry += 1;
                }
                if !value.is_finite() {
                    return Err(LinearAlgebraError::NonFiniteValue {
                        context: "assembled matrix",
                        index: matrix.values.len(),
                    });
                }
                if value != 0.0 {
                    matrix.column_indices.push(column);
                    matrix.values.push(value);
                }
            }
            matrix.row_offsets.push(matrix.values.len());
        }
        validate_csr(
            matrix.nrows,
            matrix.ncols,
            &matrix.row_offsets,
            &matrix.column_indices,
            &matrix.values,
        )
    }
}

fn validate_csr(
    nrows: usize,
    ncols: usize,
    row_offsets: &[usize],
    column_indices: &[usize],
    values: &[f64],
) -> Result<(), LinearAlgebraError> {
    if row_offsets.len() != nrows + 1 {
        return Err(LinearAlgebraError::InvalidCsr {
            message: "row_offsets length must equal nrows + 1".to_string(),
        });
    }
    if row_offsets.first() != Some(&0) {
        return Err(LinearAlgebraError::InvalidCsr {
            message: "first row offset must be zero".to_string(),
        });
    }
    if row_offsets.last() != Some(&values.len()) {
        return Err(LinearAlgebraError::InvalidCsr {
            message: "last row offset must equal value count".to_string(),
        });
    }
    if column_indices.len() != values.len() {
        return Err(LinearAlgebraError::InvalidCsr {
            message: "column and value counts must match".to_string(),
        });
    }
    for row in 0..nrows {
        if row_offsets[row] > row_offsets[row + 1] {
            return Err(LinearAlgebraError::InvalidCsr {
                message: "row offsets must be monotonic".to_string(),
            });
        }
        let columns = &column_indices[row_offsets[row]..row_offsets[row + 1]];
        if columns.windows(2).any(|pair| pair[0] >= pair[1]) {
            return Err(LinearAlgebraError::InvalidCsr {
                message: "row columns must be strictly ascending".to_string(),
            });
        }
    }
    for (index, &column) in column_indices.iter().enumerate() {
        if column >= ncols {
            return Err(LinearAlgebraError::InvalidCsr {
                message: format!("column {column} at entry {index} exceeds ncols"),
            });
        }
    }
    for (index, value) in values.iter().enumerate() {
        if !value.is_finite() {
            return Err(LinearAlgebraError::NonFiniteValue {
                context: "matrix",
                index,
            });
        }
    }
    Ok(())
}

pub(crate) fn ensure_len(expected: usize, actual: usize) -> Result<(), LinearAlgebraError> {
    if expected == actual {
        Ok(())
    } else {
        Err(LinearAlgebraError::DimensionMismatch { expected, actual })
    }
}
