use super::{dot, norm_l2, residual_into, CsrMatrix, JacobiPreconditioner, LinearAlgebraError};

const BREAKDOWN_EPSILON: f64 = 1.0e-30;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinearSolverOptions {
    pub absolute_tolerance: f64,
    pub relative_tolerance: f64,
    pub max_iterations: usize,
}

impl Default for LinearSolverOptions {
    fn default() -> Self {
        Self {
            absolute_tolerance: 1.0e-10,
            relative_tolerance: 1.0e-8,
            max_iterations: 1_000,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinearSolverStatus {
    Converged,
    MaxIterations,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinearSolveReport {
    pub status: LinearSolverStatus,
    pub iterations: usize,
    pub initial_residual: f64,
    pub final_residual: f64,
    pub relative_residual: f64,
}

impl LinearSolveReport {
    pub fn converged(&self) -> bool {
        self.status == LinearSolverStatus::Converged
    }
}

pub fn cg(
    matrix: &CsrMatrix,
    rhs: &[f64],
    solution: &mut [f64],
    options: LinearSolverOptions,
) -> Result<LinearSolveReport, LinearAlgebraError> {
    solve_cg(matrix, rhs, solution, options, None)
}

pub fn pcg(
    matrix: &CsrMatrix,
    rhs: &[f64],
    solution: &mut [f64],
    preconditioner: &JacobiPreconditioner,
    options: LinearSolverOptions,
) -> Result<LinearSolveReport, LinearAlgebraError> {
    solve_cg(matrix, rhs, solution, options, Some(preconditioner))
}

fn solve_cg(
    matrix: &CsrMatrix,
    rhs: &[f64],
    solution: &mut [f64],
    options: LinearSolverOptions,
    preconditioner: Option<&JacobiPreconditioner>,
) -> Result<LinearSolveReport, LinearAlgebraError> {
    validate_solve_inputs(matrix, rhs, solution, options)?;
    if let Some(preconditioner) = preconditioner {
        if preconditioner.len() != matrix.nrows() {
            return Err(LinearAlgebraError::DimensionMismatch {
                expected: matrix.nrows(),
                actual: preconditioner.len(),
            });
        }
    }
    let n = matrix.nrows();
    let mut residual = vec![0.0; n];
    let mut direction = vec![0.0; n];
    let mut product = vec![0.0; n];
    let mut preconditioned = vec![0.0; n];
    residual_into(matrix, solution, rhs, &mut residual)?;
    let initial = norm_l2(&residual)?;
    let threshold = options.absolute_tolerance + options.relative_tolerance * initial;
    if initial <= threshold {
        return Ok(report(LinearSolverStatus::Converged, 0, initial, initial));
    }
    if let Some(preconditioner) = preconditioner {
        preconditioner.apply_into(&residual, &mut preconditioned)?;
        direction.copy_from_slice(&preconditioned);
    } else {
        direction.copy_from_slice(&residual);
        preconditioned.copy_from_slice(&residual);
    }
    let mut rho = dot(&residual, &preconditioned)?;
    for iteration in 1..=options.max_iterations {
        matrix.spmv_into(&direction, &mut product)?;
        let denominator = dot(&direction, &product)?;
        if !denominator.is_finite() || denominator.abs() <= BREAKDOWN_EPSILON {
            return Err(LinearAlgebraError::Breakdown {
                reason: "zero CG search denominator",
            });
        }
        let alpha = rho / denominator;
        if !alpha.is_finite() {
            return Err(LinearAlgebraError::Breakdown {
                reason: "non-finite CG step",
            });
        }
        for index in 0..n {
            solution[index] += alpha * direction[index];
            residual[index] -= alpha * product[index];
        }
        let residual_norm = norm_l2(&residual)?;
        if residual_norm <= threshold {
            return Ok(report(
                LinearSolverStatus::Converged,
                iteration,
                initial,
                residual_norm,
            ));
        }
        if let Some(preconditioner) = preconditioner {
            preconditioner.apply_into(&residual, &mut preconditioned)?;
        } else {
            preconditioned.copy_from_slice(&residual);
        }
        let rho_new = dot(&residual, &preconditioned)?;
        if !rho_new.is_finite() || rho.abs() <= BREAKDOWN_EPSILON {
            return Err(LinearAlgebraError::Breakdown {
                reason: "zero CG recurrence denominator",
            });
        }
        let beta = rho_new / rho;
        if !beta.is_finite() {
            return Err(LinearAlgebraError::Breakdown {
                reason: "non-finite CG recurrence",
            });
        }
        for index in 0..n {
            direction[index] = preconditioned[index] + beta * direction[index];
        }
        rho = rho_new;
    }
    let final_residual = norm_l2(&residual)?;
    Ok(report(
        LinearSolverStatus::MaxIterations,
        options.max_iterations,
        initial,
        final_residual,
    ))
}

pub fn bicgstab(
    matrix: &CsrMatrix,
    rhs: &[f64],
    solution: &mut [f64],
    options: LinearSolverOptions,
) -> Result<LinearSolveReport, LinearAlgebraError> {
    validate_solve_inputs(matrix, rhs, solution, options)?;
    let n = matrix.nrows();
    let mut r = vec![0.0; n];
    residual_into(matrix, solution, rhs, &mut r)?;
    let r_hat = r.clone();
    let initial = norm_l2(&r)?;
    let threshold = options.absolute_tolerance + options.relative_tolerance * initial;
    if initial <= threshold {
        return Ok(report(LinearSolverStatus::Converged, 0, initial, initial));
    }
    let mut p = vec![0.0; n];
    let mut v = vec![0.0; n];
    let mut s = vec![0.0; n];
    let mut t = vec![0.0; n];
    let mut rho_previous = 1.0;
    let mut alpha = 1.0;
    let mut omega = 1.0;
    for iteration in 1..=options.max_iterations {
        let rho = dot(&r_hat, &r)?;
        if !rho.is_finite() || rho.abs() <= BREAKDOWN_EPSILON {
            return Err(LinearAlgebraError::Breakdown {
                reason: "zero BiCGSTAB rho",
            });
        }
        let beta = (rho / rho_previous) * (alpha / omega);
        if !beta.is_finite() {
            return Err(LinearAlgebraError::Breakdown {
                reason: "non-finite BiCGSTAB beta",
            });
        }
        for index in 0..n {
            p[index] = r[index] + beta * (p[index] - omega * v[index]);
        }
        matrix.spmv_into(&p, &mut v)?;
        let denominator = dot(&r_hat, &v)?;
        if !denominator.is_finite() || denominator.abs() <= BREAKDOWN_EPSILON {
            return Err(LinearAlgebraError::Breakdown {
                reason: "zero BiCGSTAB alpha denominator",
            });
        }
        alpha = rho / denominator;
        if !alpha.is_finite() {
            return Err(LinearAlgebraError::Breakdown {
                reason: "non-finite BiCGSTAB alpha",
            });
        }
        for index in 0..n {
            s[index] = r[index] - alpha * v[index];
        }
        let s_norm = norm_l2(&s)?;
        if s_norm <= threshold {
            for index in 0..n {
                solution[index] += alpha * p[index];
            }
            return Ok(report(
                LinearSolverStatus::Converged,
                iteration,
                initial,
                s_norm,
            ));
        }
        matrix.spmv_into(&s, &mut t)?;
        let tt = dot(&t, &t)?;
        if !tt.is_finite() || tt.abs() <= BREAKDOWN_EPSILON {
            return Err(LinearAlgebraError::Breakdown {
                reason: "zero BiCGSTAB omega denominator",
            });
        }
        omega = dot(&t, &s)? / tt;
        if !omega.is_finite() || omega.abs() <= BREAKDOWN_EPSILON {
            return Err(LinearAlgebraError::Breakdown {
                reason: "zero BiCGSTAB omega",
            });
        }
        for index in 0..n {
            solution[index] += alpha * p[index] + omega * s[index];
            r[index] = s[index] - omega * t[index];
        }
        let residual_norm = norm_l2(&r)?;
        if residual_norm <= threshold {
            return Ok(report(
                LinearSolverStatus::Converged,
                iteration,
                initial,
                residual_norm,
            ));
        }
        rho_previous = rho;
    }
    Ok(report(
        LinearSolverStatus::MaxIterations,
        options.max_iterations,
        initial,
        norm_l2(&r)?,
    ))
}

fn validate_solve_inputs(
    matrix: &CsrMatrix,
    rhs: &[f64],
    solution: &[f64],
    options: LinearSolverOptions,
) -> Result<(), LinearAlgebraError> {
    if matrix.nrows() != matrix.ncols() {
        return Err(LinearAlgebraError::DimensionMismatch {
            expected: matrix.nrows(),
            actual: matrix.ncols(),
        });
    }
    if rhs.len() != matrix.nrows() {
        return Err(LinearAlgebraError::DimensionMismatch {
            expected: matrix.nrows(),
            actual: rhs.len(),
        });
    }
    if solution.len() != matrix.ncols() {
        return Err(LinearAlgebraError::DimensionMismatch {
            expected: matrix.ncols(),
            actual: solution.len(),
        });
    }
    if !options.absolute_tolerance.is_finite()
        || options.absolute_tolerance < 0.0
        || !options.relative_tolerance.is_finite()
        || options.relative_tolerance < 0.0
    {
        return Err(LinearAlgebraError::Breakdown {
            reason: "invalid solver tolerance",
        });
    }
    for (index, value) in rhs.iter().chain(solution).enumerate() {
        if !value.is_finite() {
            return Err(LinearAlgebraError::NonFiniteValue {
                context: "solver vector",
                index,
            });
        }
    }
    Ok(())
}

fn report(
    status: LinearSolverStatus,
    iterations: usize,
    initial: f64,
    final_residual: f64,
) -> LinearSolveReport {
    LinearSolveReport {
        status,
        iterations,
        initial_residual: initial,
        final_residual,
        relative_residual: if initial == 0.0 {
            0.0
        } else {
            final_residual / initial
        },
    }
}
