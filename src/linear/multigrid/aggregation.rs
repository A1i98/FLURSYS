use crate::{CsrMatrix, LinearAlgebraError};
use std::collections::BTreeMap;

/// Connected strength-based aggregation. Attach unmatched rows to an adjacent
/// cluster rather than carrying avoidable singleton rows through every level.
/// Alternating traversal direction limits the bias of greedy pairing.
pub(super) fn pairs(
    matrix: &CsrMatrix,
    forward: bool,
    weights: Option<&[f64]>,
) -> (Vec<usize>, usize) {
    let n = matrix.nrows();
    let mut map = vec![usize::MAX; n];
    let mut count = 0;
    for index in 0..n {
        let row = if forward { index } else { n - 1 - index };
        if map[row] != usize::MAX {
            continue;
        }
        let (mut free, mut adjacent) = (None, None);
        let (mut free_weight, mut adjacent_weight) = (0.0, 0.0);
        for entry in matrix.row_offsets()[row]..matrix.row_offsets()[row + 1] {
            let column = matrix.column_indices()[entry];
            if column == row {
                continue;
            }
            let weight =
                weights.map_or_else(|| matrix.values()[entry].abs(), |weights| weights[entry]);
            if map[column] == usize::MAX {
                if weight > free_weight {
                    free = Some(column);
                    free_weight = weight;
                }
            } else if weight > adjacent_weight {
                adjacent = Some(map[column]);
                adjacent_weight = weight;
            }
        }
        if let Some(neighbor) = free {
            map[row] = count;
            map[neighbor] = count;
            count += 1;
        } else if let Some(cluster) = adjacent {
            map[row] = cluster;
        } else {
            // No nonzero edge: keep a disconnected constraint independent.
            map[row] = count;
            count += 1;
        }
    }
    (map, count)
}

pub(super) fn galerkin(
    matrix: &CsrMatrix,
    map: &[usize],
    count: usize,
) -> Result<(CsrMatrix, Vec<usize>), LinearAlgebraError> {
    // Retain cancelled entries so coefficient updates keep the same topology.
    let mut coefficients = BTreeMap::new();
    for row in 0..matrix.nrows() {
        for entry in matrix.row_offsets()[row]..matrix.row_offsets()[row + 1] {
            *coefficients
                .entry((map[row], map[matrix.column_indices()[entry]]))
                .or_insert(0.0) += matrix.values()[entry];
        }
    }
    // Average only roundoff differences in mirrored Galerkin sums. Keeping
    // both CSR entries identical also keeps the coarse PCG operator symmetric.
    let mirrored: Vec<_> = coefficients
        .iter()
        .filter(|&(&(row, column), &_value)| row < column)
        .map(|(&(row, column), &value)| {
            (
                (row, column),
                0.5 * value + 0.5 * coefficients[&(column, row)],
            )
        })
        .collect();
    for ((row, column), value) in mirrored {
        coefficients.insert((row, column), value);
        coefficients.insert((column, row), value);
    }
    let mut offsets = vec![0; count + 1];
    let mut columns = Vec::with_capacity(coefficients.len());
    let mut values = Vec::with_capacity(coefficients.len());
    for ((row, column), value) in coefficients {
        offsets[row + 1] += 1;
        columns.push(column);
        values.push(value);
    }
    for row in 0..count {
        offsets[row + 1] += offsets[row];
    }
    let coarse = CsrMatrix::new(count, count, offsets, columns, values)?;
    let mut entries = Vec::with_capacity(matrix.nnz());
    for row in 0..matrix.nrows() {
        let start = coarse.row_offsets()[map[row]];
        let end = coarse.row_offsets()[map[row] + 1];
        for entry in matrix.row_offsets()[row]..matrix.row_offsets()[row + 1] {
            let column = map[matrix.column_indices()[entry]];
            let offset = coarse.column_indices()[start..end]
                .binary_search(&column)
                .map_err(|_| LinearAlgebraError::Breakdown {
                    reason: "missing Galerkin sparsity entry",
                })?;
            entries.push(start + offset);
        }
    }
    Ok((coarse, entries))
}
