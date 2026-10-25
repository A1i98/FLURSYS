use super::vector::{dot_with_scratch, norm_l2_with_scratch};
use super::{residual_into, CsrMatrix, LinearAlgebraError, Preconditioner};

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

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
pub enum LinearSolverStatus {
    Converged,
    MaxIterations,
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize)]
pub struct LinearSolveReport {
    pub status: LinearSolverStatus,
    pub iterations: usize,
    pub initial_residual: f64,
    pub final_residual: f64,
    pub relative_residual: f64,
}

/// Reusable work vectors for repeated conjugate-gradient pressure solves.
#[derive(Default)]
pub struct CgWorkspace {
    residual: Vec<f64>,
    direction: Vec<f64>,
    product: Vec<f64>,
    preconditioned: Vec<f64>,
    partials: Vec<f64>,
}

impl CgWorkspace {
    fn resize(&mut self, size: usize) {
        self.residual.resize(size, 0.0);
        self.direction.resize(size, 0.0);
        self.product.resize(size, 0.0);
        self.preconditioned.resize(size, 0.0);
    }
}

/// Reusable vectors for consecutive nonsymmetric momentum solves.
#[derive(Default)]
pub struct BiCgStabWorkspace {
    r: Vec<f64>,
    r_hat: Vec<f64>,
    p: Vec<f64>,
    v: Vec<f64>,
    s: Vec<f64>,
    t: Vec<f64>,
    p_preconditioned: Vec<f64>,
    s_preconditioned: Vec<f64>,
    partials: Vec<f64>,
}

impl BiCgStabWorkspace {
    fn resize(&mut self, size: usize, preconditioned: bool) {
        for vector in [
            &mut self.r,
            &mut self.r_hat,
            &mut self.p,
            &mut self.v,
            &mut self.s,
            &mut self.t,
        ] {
            vector.resize(size, 0.0);
        }
        if preconditioned {
            self.p_preconditioned.resize(size, 0.0);
            self.s_preconditioned.resize(size, 0.0);
        }
    }
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

/// Unpreconditioned conjugate gradients with caller-retained scratch vectors.
pub fn cg_with_workspace(
    matrix: &CsrMatrix,
    rhs: &[f64],
    solution: &mut [f64],
    options: LinearSolverOptions,
    workspace: &mut CgWorkspace,
) -> Result<LinearSolveReport, LinearAlgebraError> {
    solve_cg_with_workspace(matrix, rhs, solution, options, None, workspace)
}

pub fn pcg<P: Preconditioner>(
    matrix: &CsrMatrix,
    rhs: &[f64],
    solution: &mut [f64],
    preconditioner: &P,
    options: LinearSolverOptions,
) -> Result<LinearSolveReport, LinearAlgebraError> {
    pcg_with_preconditioner(matrix, rhs, solution, preconditioner, options)
}

pub fn pcg_with_preconditioner<P: Preconditioner>(
    matrix: &CsrMatrix,
    rhs: &[f64],
    solution: &mut [f64],
    preconditioner: &P,
    options: LinearSolverOptions,
) -> Result<LinearSolveReport, LinearAlgebraError> {
    solve_cg(matrix, rhs, solution, options, Some(preconditioner))
}

pub fn pcg_with_preconditioner_and_workspace<P: Preconditioner>(
    matrix: &CsrMatrix,
    rhs: &[f64],
    solution: &mut [f64],
    preconditioner: &P,
    options: LinearSolverOptions,
    workspace: &mut CgWorkspace,
) -> Result<LinearSolveReport, LinearAlgebraError> {
    solve_cg_with_workspace(
        matrix,
        rhs,
        solution,
        options,
        Some(preconditioner),
        workspace,
    )
}

fn solve_cg(
    matrix: &CsrMatrix,
    rhs: &[f64],
    solution: &mut [f64],
    options: LinearSolverOptions,
    preconditioner: Option<&dyn Preconditioner>,
) -> Result<LinearSolveReport, LinearAlgebraError> {
    let mut workspace = CgWorkspace::default();
    solve_cg_with_workspace(
        matrix,
        rhs,
        solution,
        options,
        preconditioner,
        &mut workspace,
    )
}

fn solve_cg_with_workspace(
    matrix: &CsrMatrix,
    rhs: &[f64],
    solution: &mut [f64],
    options: LinearSolverOptions,
    preconditioner: Option<&dyn Preconditioner>,
    workspace: &mut CgWorkspace,
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
    if let Some(result) = crate::compute::try_gpu(matrix, rhs, solution, options) {
        return result;
    }
    let _timer = crate::compute::Timer::linear();
    let mut profile = super::profile::CgProfileSession::new();
    let n = matrix.nrows();
    workspace.resize(n);
    let CgWorkspace {
        residual,
        direction,
        product,
        preconditioned,
        partials,
    } = workspace;
    profile.measure(false, || residual_into(matrix, solution, rhs, residual))?;
    let initial = norm_l2_with_scratch(residual, partials)?;
    let threshold = options.absolute_tolerance + options.relative_tolerance * initial;
    if initial <= threshold {
        return Ok(report(LinearSolverStatus::Converged, 0, initial, initial));
    }
    if let Some(preconditioner) = preconditioner {
        profile.measure(true, || preconditioner.apply_into(residual, preconditioned))?;
        direction.copy_from_slice(preconditioned);
    } else {
        direction.copy_from_slice(residual);
        preconditioned.copy_from_slice(residual);
    }
    let mut rho = dot_with_scratch(residual, preconditioned, partials)?;
    for iteration in 1..=options.max_iterations {
        crate::compute::checkpoint()?;
        profile.iteration();
        profile.measure(false, || matrix.spmv_into(direction, product))?;
        let denominator = dot_with_scratch(direction, product, partials)?;
        if !denominator.is_finite() || denominator <= BREAKDOWN_EPSILON {
            return Err(LinearAlgebraError::Breakdown {
                reason: "non-positive CG search denominator; pressure operator must be positive definite",
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
        let recursive_norm = norm_l2_with_scratch(residual, partials)?;
        // The recurrence can drift from b-Ax on ill-conditioned pressure
        // systems. Certify convergence using the actual operator and refresh
        // when the recursive estimate reaches tolerance. Restart conjugacy after replacement, never enlarge
        // tolerances or reset the iteration budget.
        if recursive_norm <= threshold {
            residual_into(matrix, solution, rhs, residual)?;
            let true_norm = norm_l2_with_scratch(residual, partials)?;
            if true_norm <= threshold {
                return Ok(report(
                    LinearSolverStatus::Converged,
                    iteration,
                    initial,
                    true_norm,
                ));
            }
            if let Some(preconditioner) = preconditioner {
                profile.measure(true, || preconditioner.apply_into(residual, preconditioned))?;
            } else {
                preconditioned.copy_from_slice(residual);
            }
            rho = dot_with_scratch(residual, preconditioned, partials)?;
            direction.copy_from_slice(preconditioned);
            continue;
        }
        if let Some(preconditioner) = preconditioner {
            profile.measure(true, || preconditioner.apply_into(residual, preconditioned))?;
        } else {
            preconditioned.copy_from_slice(residual);
        }
        let rho_new = dot_with_scratch(residual, preconditioned, partials)?;
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
    residual_into(matrix, solution, rhs, residual)?;
    let final_residual = norm_l2_with_scratch(residual, partials)?;
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
    solve_bicgstab(matrix, rhs, solution, options, None)
}

/// Unpreconditioned BiCGStab with caller-retained scratch vectors.
pub fn bicgstab_with_workspace(
    matrix: &CsrMatrix,
    rhs: &[f64],
    solution: &mut [f64],
    options: LinearSolverOptions,
    workspace: &mut BiCgStabWorkspace,
) -> Result<LinearSolveReport, LinearAlgebraError> {
    solve_bicgstab_with_workspace(matrix, rhs, solution, options, None, workspace)
}

pub fn bicgstab_with_preconditioner<P: Preconditioner>(
    matrix: &CsrMatrix,
    rhs: &[f64],
    solution: &mut [f64],
    preconditioner: &P,
    options: LinearSolverOptions,
) -> Result<LinearSolveReport, LinearAlgebraError> {
    solve_bicgstab(matrix, rhs, solution, options, Some(preconditioner))
}

pub fn bicgstab_with_preconditioner_and_workspace<P: Preconditioner>(
    matrix: &CsrMatrix,
    rhs: &[f64],
    solution: &mut [f64],
    preconditioner: &P,
    options: LinearSolverOptions,
    workspace: &mut BiCgStabWorkspace,
) -> Result<LinearSolveReport, LinearAlgebraError> {
    solve_bicgstab_with_workspace(
        matrix,
        rhs,
        solution,
        options,
        Some(preconditioner),
        workspace,
    )
}

fn solve_bicgstab(
    matrix: &CsrMatrix,
    rhs: &[f64],
    solution: &mut [f64],
    options: LinearSolverOptions,
    preconditioner: Option<&dyn Preconditioner>,
) -> Result<LinearSolveReport, LinearAlgebraError> {
    solve_bicgstab_with_workspace(
        matrix,
        rhs,
        solution,
        options,
        preconditioner,
        &mut BiCgStabWorkspace::default(),
    )
}

fn solve_bicgstab_with_workspace(
    matrix: &CsrMatrix,
    rhs: &[f64],
    solution: &mut [f64],
    options: LinearSolverOptions,
    preconditioner: Option<&dyn Preconditioner>,
    workspace: &mut BiCgStabWorkspace,
) -> Result<LinearSolveReport, LinearAlgebraError> {
    validate_solve_inputs(matrix, rhs, solution, options)?;
    let n = matrix.nrows();
    if let Some(preconditioner) = preconditioner {
        if preconditioner.len() != n {
            return Err(LinearAlgebraError::DimensionMismatch {
                expected: n,
                actual: preconditioner.len(),
            });
        }
    }
    if let Some(result) = crate::compute::try_gpu(matrix, rhs, solution, options) {
        return result;
    }
    let _timer = crate::compute::Timer::linear();
    workspace.resize(n, preconditioner.is_some());
    let BiCgStabWorkspace {
        r,
        r_hat,
        p,
        v,
        s,
        t,
        p_preconditioned,
        s_preconditioned,
        partials,
    } = workspace;
    residual_into(matrix, solution, rhs, r)?;
    r_hat.copy_from_slice(r);
    let initial = norm_l2_with_scratch(r, partials)?;
    let threshold = options.absolute_tolerance + options.relative_tolerance * initial;
    if initial <= threshold {
        return Ok(report(LinearSolverStatus::Converged, 0, initial, initial));
    }
    // These enter the first recurrence; all other work vectors are overwritten.
    p.fill(0.0);
    v.fill(0.0);
    let mut rho_previous = 1.0;
    let mut alpha = 1.0;
    let mut omega = 1.0;
    for iteration in 1..=options.max_iterations {
        crate::compute::checkpoint()?;
        let rho = dot_with_scratch(r_hat, r, partials)?;
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
        if let Some(preconditioner) = preconditioner {
            preconditioner.apply_into(p, p_preconditioned)?;
        }
        let p_direction = if preconditioner.is_some() {
            p_preconditioned.as_slice()
        } else {
            p.as_slice()
        };
        matrix.spmv_into(p_direction, v)?;
        let denominator = dot_with_scratch(r_hat, v, partials)?;
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
        let s_norm = norm_l2_with_scratch(s, partials)?;
        if s_norm <= threshold {
            for (solution_value, &direction_value) in solution.iter_mut().zip(p_direction) {
                *solution_value += alpha * direction_value;
            }
            return Ok(report(
                LinearSolverStatus::Converged,
                iteration,
                initial,
                s_norm,
            ));
        }
        if let Some(preconditioner) = preconditioner {
            preconditioner.apply_into(s, s_preconditioned)?;
        }
        let s_direction = if preconditioner.is_some() {
            s_preconditioned.as_slice()
        } else {
            s.as_slice()
        };
        matrix.spmv_into(s_direction, t)?;
        let tt = dot_with_scratch(t, t, partials)?;
        if !tt.is_finite() || tt.abs() <= BREAKDOWN_EPSILON {
            return Err(LinearAlgebraError::Breakdown {
                reason: "zero BiCGSTAB omega denominator",
            });
        }
        omega = dot_with_scratch(t, s, partials)? / tt;
        if !omega.is_finite() || omega.abs() <= BREAKDOWN_EPSILON {
            return Err(LinearAlgebraError::Breakdown {
                reason: "zero BiCGSTAB omega",
            });
        }
        for ((solution_value, &p_value), &s_value) in
            solution.iter_mut().zip(p_direction).zip(s_direction)
        {
            *solution_value += alpha * p_value + omega * s_value;
        }
        for index in 0..n {
            r[index] = s[index] - omega * t[index];
        }
        let residual_norm = norm_l2_with_scratch(r, partials)?;
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
        norm_l2_with_scratch(r, partials)?,
    ))
}

pub(crate) fn validate_solve_inputs(
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

#[cfg(test)]
mod reduction_workspace_tests {
    use super::*;
    use crate::compute::{Backend, ComputeOptions, Execution, Scope};
    use crate::CsrBuilder;

    #[test]
    fn retained_solver_reductions_match_serial_across_parallel_threshold_and_resizes() {
        let mut cg_workspace = CgWorkspace::default();
        let mut bicg_workspace = BiCgStabWorkspace::default();
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(2)
            .build()
            .unwrap();
        for n in [4097, 17, 8193, 4095, 4097] {
            let mut builder = CsrBuilder::new(n, n);
            for i in 0..n {
                builder.add(i, i, 2.0 + (i % 4) as f64 * 0.1).unwrap();
            }
            let matrix = builder.finalize().unwrap();
            let rhs: Vec<_> = (0..n).map(|i| (i as f64 * 0.1).cos()).collect();
            let options = LinearSolverOptions {
                absolute_tolerance: 1e-9,
                relative_tolerance: 0.0,
                max_iterations: 100,
            };
            let serial = Execution::new(ComputeOptions {
                backend: Backend::Serial,
                ..Default::default()
            })
            .unwrap();
            let (expected_cg, expected_bicg) = pool.install(|| {
                let _scope = Scope::enter(serial);
                let mut a = vec![0.0; n];
                let mut b = a.clone();
                (
                    (cg(&matrix, &rhs, &mut a, options).unwrap(), a),
                    (bicgstab(&matrix, &rhs, &mut b, options).unwrap(), b),
                )
            });
            let parallel = Execution::new(ComputeOptions {
                backend: Backend::Parallel,
                ..Default::default()
            })
            .unwrap();
            pool.install(|| {
                let _scope = Scope::enter(parallel);
                let mut a = vec![0.0; n];
                let mut b = a.clone();
                let a_report =
                    cg_with_workspace(&matrix, &rhs, &mut a, options, &mut cg_workspace).unwrap();
                let b_report =
                    bicgstab_with_workspace(&matrix, &rhs, &mut b, options, &mut bicg_workspace)
                        .unwrap();
                assert!(a_report.converged());
                assert!(b_report.converged());
                assert_eq!((a_report, a), expected_cg);
                assert_eq!((b_report, b), expected_bicg);
            });
        }
    }
}
