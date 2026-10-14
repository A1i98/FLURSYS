use super::{
    aggregation, coarse::CoarseSystem, matrix::SymmetricMatrix, smoother, MultigridOptions,
    MultigridSmoother,
};
use crate::{CgWorkspace, CsrMatrix, LinearAlgebraError};

pub(super) struct Level {
    pub matrix: CsrMatrix,
    pub kernel: SymmetricMatrix,
    pub inverse_dic: Vec<f64>,
    pub aggregates: Option<Vec<usize>>,
    pub coarse_entries: Vec<usize>,
    pub terminal: Option<CoarseSystem>,
}

pub(super) struct Scratch {
    pub product: Vec<f64>,
    pub residual: Vec<f64>,
    pub work: Vec<f64>,
    pub correction: Vec<f64>,
    pub coarse_rhs: Vec<f64>,
    pub coarse_solution: Vec<f64>,
    pub cg: CgWorkspace,
}

impl Scratch {
    pub fn new(n: usize, coarse: usize) -> Self {
        Self {
            product: vec![0.0; n],
            residual: vec![0.0; n],
            work: vec![0.0; n],
            correction: vec![0.0; n],
            coarse_rhs: vec![0.0; coarse],
            coarse_solution: vec![0.0; coarse],
            cg: CgWorkspace::default(),
        }
    }
}

pub(super) fn build(
    matrix: &CsrMatrix,
    options: MultigridOptions,
    face_weights: Option<&[f64]>,
) -> Result<(Vec<Level>, Vec<Scratch>), LinearAlgebraError> {
    options.validate()?;
    if matrix.nrows() == 0 {
        return Err(LinearAlgebraError::Breakdown {
            reason: "empty multigrid system",
        });
    }
    let mut levels = Vec::new();
    let mut scratch = Vec::new();
    let mut current = matrix.clone();
    let mut weights = face_weights.map(<[f64]>::to_vec);
    let mut pairing_stage = 0usize;
    loop {
        let kernel = SymmetricMatrix::new(&current)?;
        let terminal = kernel.coupled_rows() <= options.coarsest_cells
            || kernel.neighbors.is_empty()
            || levels.len() + 1 >= options.max_levels;
        let (mut map, mut count) = if terminal {
            (Vec::new(), current.nrows())
        } else {
            let result = aggregation::pairs(&current, pairing_stage % 2 == 0, weights.as_deref());
            pairing_stage += 1;
            result
        };
        if terminal || count == current.nrows() {
            let terminal_system = CoarseSystem::new(&current, options)?;
            let compact_size = terminal_system.as_ref().map_or(0, CoarseSystem::len);
            scratch.push(Scratch::new(0, compact_size));
            levels.push(Level {
                matrix: current,
                kernel,
                inverse_dic: Vec::new(),
                aggregates: None,
                coarse_entries: Vec::new(),
                terminal: terminal_system,
            });
            break;
        }
        let inverse_dic = if options.smoother == MultigridSmoother::DicGaussSeidel {
            smoother::dic_diagonal(&kernel)?
        } else {
            Vec::new()
        };
        let (mut coarse, mut coarse_entries) = aggregation::galerkin(&current, &map, count)?;
        let mut next_weights = restrict_weights(weights.as_deref(), &coarse_entries, &coarse);
        for _ in 1..options.merge_levels {
            if SymmetricMatrix::new(&coarse)?.coupled_rows() <= options.coarsest_cells {
                break;
            }
            let (next_map, next_count) =
                aggregation::pairs(&coarse, pairing_stage % 2 == 0, next_weights.as_deref());
            pairing_stage += 1;
            if next_count == count {
                break;
            }
            let (next, next_entries) = aggregation::galerkin(&coarse, &next_map, next_count)?;
            for aggregate in &mut map {
                *aggregate = next_map[*aggregate];
            }
            next_weights = restrict_weights(next_weights.as_deref(), &next_entries, &next);
            coarse = next;
            count = next_count;
        }
        // Construct the final operator directly from the stored fine level.
        // Coefficient refreshes use the same summation map, with no temporary
        // intermediate levels in the repeated solve or update paths.
        if options.merge_levels > 1 {
            (coarse, coarse_entries) = aggregation::galerkin(&current, &map, count)?;
        }
        weights = next_weights;
        scratch.push(Scratch::new(current.nrows(), count));
        levels.push(Level {
            matrix: current,
            kernel,
            inverse_dic,
            aggregates: Some(map),
            coarse_entries,
            terminal: None,
        });
        current = coarse;
    }
    Ok((levels, scratch))
}

fn restrict_weights(
    weights: Option<&[f64]>,
    entries: &[usize],
    coarse: &CsrMatrix,
) -> Option<Vec<f64>> {
    weights.map(|weights| {
        let mut result = vec![0.0; coarse.nnz()];
        for (&weight, &entry) in weights.iter().zip(entries) {
            result[entry] += weight;
        }
        for row in 0..coarse.nrows() {
            for entry in coarse.row_offsets()[row]..coarse.row_offsets()[row + 1] {
                if coarse.column_indices()[entry] == row {
                    result[entry] = 0.0;
                }
            }
        }
        result
    })
}

pub(super) fn update(levels: &mut [Level], matrix: &CsrMatrix) -> Result<(), LinearAlgebraError> {
    let fine = &mut levels[0].matrix;
    if fine.row_offsets() != matrix.row_offsets()
        || fine.column_indices() != matrix.column_indices()
        || fine.nrows() != matrix.nrows()
        || fine.ncols() != matrix.ncols()
    {
        return Err(LinearAlgebraError::Breakdown {
            reason: "multigrid update changed matrix topology",
        });
    }
    if !levels[0].kernel.is_symmetric(matrix, 1e-12) {
        return Err(LinearAlgebraError::Breakdown {
            reason: "multigrid update is not symmetric",
        });
    }
    fine.values_mut().copy_from_slice(matrix.values());
    for index in 0..levels.len() {
        let (before, after) = levels.split_at_mut(index + 1);
        let level = &mut before[index];
        if index > 0 {
            level.kernel.symmetrize_coarse_values(&mut level.matrix);
        }
        level.kernel.update(&level.matrix)?;
        if !level.inverse_dic.is_empty() {
            smoother::update_dic_diagonal(&level.kernel, &mut level.inverse_dic)?;
        }
        if let Some(coarse) = after.first_mut() {
            coarse.matrix.values_mut().fill(0.0);
            for (&value, &entry) in level.matrix.values().iter().zip(&level.coarse_entries) {
                coarse.matrix.values_mut()[entry] += value;
            }
        } else if let Some(terminal) = &mut level.terminal {
            terminal.update(&level.matrix)?;
        }
    }
    Ok(())
}

pub(super) fn cholesky(matrix: &CsrMatrix) -> Result<Vec<f64>, LinearAlgebraError> {
    let n = matrix.nrows();
    let mut lower = vec![0.0; n * n];
    for row in 0..n {
        for column in 0..=row {
            let mut value = matrix.get(row, column).unwrap_or(0.0);
            for k in 0..column {
                value -= lower[row * n + k] * lower[column * n + k];
            }
            if row == column {
                if !value.is_finite() || value <= 0.0 {
                    return Err(LinearAlgebraError::Breakdown {
                        reason: "non-positive multigrid coarse pivot",
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

pub(super) fn solve_factor(lower: &[f64], rhs: &[f64], x: &mut [f64]) {
    let n = rhs.len();
    x.copy_from_slice(rhs);
    for row in 0..n {
        for k in 0..row {
            x[row] -= lower[row * n + k] * x[k];
        }
        x[row] /= lower[row * n + row];
    }
    for row in (0..n).rev() {
        for k in row + 1..n {
            x[row] -= lower[k * n + row] * x[k];
        }
        x[row] /= lower[row * n + row];
    }
}
