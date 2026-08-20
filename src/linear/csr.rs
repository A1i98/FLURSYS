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
            let mut value = 0.0;
            for entry in self.row_offsets[row]..self.row_offsets[row + 1] {
                value += self.values[entry] * input[self.column_indices[entry]];
            }
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
}

impl CsrBuilder {
    pub fn new(nrows: usize, ncols: usize) -> Self {
        Self {
            nrows,
            ncols,
            entries: Vec::new(),
        }
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
        self.entries
            .sort_unstable_by_key(|&(row, column, _)| (row, column));
        let mut row_offsets = Vec::with_capacity(self.nrows + 1);
        let mut columns = Vec::with_capacity(self.entries.len());
        let mut values = Vec::with_capacity(self.entries.len());
        row_offsets.push(0);
        let mut entry = 0;
        for row in 0..self.nrows {
            while entry < self.entries.len() && self.entries[entry].0 == row {
                let column = self.entries[entry].1;
                let mut value = 0.0;
                while entry < self.entries.len()
                    && self.entries[entry].0 == row
                    && self.entries[entry].1 == column
                {
                    value += self.entries[entry].2;
                    entry += 1;
                }
                if !value.is_finite() {
                    return Err(LinearAlgebraError::NonFiniteValue {
                        context: "assembled matrix",
                        index: values.len(),
                    });
                }
                if value != 0.0 {
                    columns.push(column);
                    values.push(value);
                }
            }
            row_offsets.push(values.len());
        }
        CsrMatrix::new(self.nrows, self.ncols, row_offsets, columns, values)
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
