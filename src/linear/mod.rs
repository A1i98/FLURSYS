//! Solver-independent serial sparse linear algebra primitives.
//!
//! The final matrix storage is compact CSR with strictly ascending column indices
//! in every row. `CsrBuilder` accepts arbitrary insertion order, sums duplicate
//! entries deterministically at finalization, and omits only exact zero sums.

mod csr;
mod preconditioner;
mod solver;
mod vector;

pub use csr::{CsrBuilder, CsrMatrix};
pub use preconditioner::JacobiPreconditioner;
pub use solver::{bicgstab, cg, pcg, LinearSolveReport, LinearSolverOptions, LinearSolverStatus};
pub use vector::{axpy, copy_into, dot, norm_inf, norm_l2, residual, residual_into, scale};

#[derive(Clone, Debug, PartialEq)]
pub enum LinearAlgebraError {
    DimensionMismatch { expected: usize, actual: usize },
    InvalidCsr { message: String },
    IndexOutOfBounds { row: usize, column: usize },
    NonFiniteValue { context: &'static str, index: usize },
    MissingDiagonal { row: usize },
    ZeroDiagonal { row: usize },
    Breakdown { reason: &'static str },
}

impl std::fmt::Display for LinearAlgebraError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::DimensionMismatch { expected, actual } => {
                write!(
                    formatter,
                    "dimension mismatch: expected {expected}, got {actual}"
                )
            }
            Self::InvalidCsr { message } => write!(formatter, "invalid CSR matrix: {message}"),
            Self::IndexOutOfBounds { row, column } => {
                write!(formatter, "matrix index ({row}, {column}) is out of bounds")
            }
            Self::NonFiniteValue { context, index } => {
                write!(formatter, "non-finite {context} value at index {index}")
            }
            Self::MissingDiagonal { row } => {
                write!(formatter, "missing diagonal entry in row {row}")
            }
            Self::ZeroDiagonal { row } => write!(formatter, "zero diagonal entry in row {row}"),
            Self::Breakdown { reason } => write!(formatter, "iterative solver breakdown: {reason}"),
        }
    }
}

impl std::error::Error for LinearAlgebraError {}

#[cfg(test)]
mod tests {
    use super::{
        bicgstab, cg, pcg, residual_into, CsrBuilder, CsrMatrix, JacobiPreconditioner,
        LinearAlgebraError, LinearSolverOptions, LinearSolverStatus,
    };

    const OPTIONS: LinearSolverOptions = LinearSolverOptions {
        absolute_tolerance: 1.0e-12,
        relative_tolerance: 1.0e-12,
        max_iterations: 1_000,
    };

    fn matrix(rows: &[&[f64]]) -> CsrMatrix {
        let mut builder = CsrBuilder::new(rows.len(), rows.len());
        for (row, values) in rows.iter().enumerate() {
            for (column, &value) in values.iter().enumerate() {
                if value != 0.0 {
                    builder.add(row, column, value).unwrap();
                }
            }
        }
        builder.finalize().unwrap()
    }

    fn dense_matvec(rows: &[&[f64]], input: &[f64]) -> Vec<f64> {
        rows.iter()
            .map(|row| row.iter().zip(input).map(|(a, x)| a * x).sum())
            .collect()
    }

    fn assert_close(actual: &[f64], expected: &[f64]) {
        assert_eq!(actual.len(), expected.len());
        for (&actual, &expected) in actual.iter().zip(expected) {
            assert!(
                (actual - expected).abs() < 1.0e-10,
                "{actual} != {expected}"
            );
        }
    }

    #[test]
    fn csr_builder_finalizes_a_sorted_duplicate_free_matrix() {
        let mut builder = CsrBuilder::new(3, 3);
        builder.add(0, 1, -1.0).unwrap();
        builder.add(0, 0, 4.0).unwrap();
        builder.add(1, 2, -1.0).unwrap();
        builder.add(1, 1, 4.0).unwrap();
        builder.add(1, 0, -1.0).unwrap();
        builder.add(2, 2, 3.0).unwrap();
        builder.add(2, 1, -1.0).unwrap();
        let matrix = builder.finalize().unwrap();

        assert_eq!(matrix.row_offsets(), &[0, 2, 5, 7]);
        assert_eq!(matrix.column_indices(), &[0, 1, 0, 1, 2, 1, 2]);
        assert_eq!(matrix.values(), &[4.0, -1.0, -1.0, 4.0, -1.0, -1.0, 3.0]);
        assert_eq!(matrix.diagonal().unwrap(), vec![4.0, 4.0, 3.0]);
        assert!(matrix.is_symmetric(0.0));
    }

    #[test]
    fn duplicate_assembly_spmv_and_residual_overwrite_output() {
        let mut builder = CsrBuilder::new(2, 2);
        builder.add(0, 0, 2.0).unwrap();
        builder.add(0, 0, 3.0).unwrap();
        builder.add(0, 1, -1.0).unwrap();
        builder.add(1, 0, 1.0).unwrap();
        builder.add(1, 1, 4.0).unwrap();
        let matrix = builder.finalize().unwrap();
        assert_eq!(matrix.nnz(), 4);
        assert_eq!(matrix.get(0, 0), Some(5.0));

        let mut output = vec![99.0, 99.0];
        matrix.spmv_into(&[1.0, 2.0], &mut output).unwrap();
        assert_eq!(output, vec![3.0, 9.0]);
        matrix.spmv_into(&[3.0, -1.0], &mut output).unwrap();
        assert_eq!(output, vec![16.0, -1.0]);

        residual_into(&matrix, &[1.0, 2.0], &[5.0, 10.0], &mut output).unwrap();
        assert_eq!(output, vec![2.0, 1.0]);
    }

    #[test]
    fn malformed_csr_dimensions_and_non_finite_values_are_rejected() {
        assert!(matches!(
            CsrMatrix::new(1, 1, vec![1, 1], vec![], vec![]),
            Err(LinearAlgebraError::InvalidCsr { .. })
        ));
        assert!(matches!(
            CsrMatrix::new(1, 1, vec![0, 1], vec![1], vec![1.0]),
            Err(LinearAlgebraError::InvalidCsr { .. })
        ));
        assert!(matches!(
            CsrMatrix::new(1, 1, vec![0, 1], vec![0], vec![f64::NAN]),
            Err(LinearAlgebraError::NonFiniteValue { .. })
        ));
        let matrix = matrix(&[&[1.0, 0.0], &[0.0, 1.0]]);
        assert!(matches!(
            matrix.spmv_into(&[1.0], &mut [0.0, 0.0]),
            Err(LinearAlgebraError::DimensionMismatch { .. })
        ));
        assert!(matches!(
            matrix.spmv_into(&[1.0, 2.0], &mut [0.0]),
            Err(LinearAlgebraError::DimensionMismatch { .. })
        ));
    }

    #[test]
    fn jacobi_rejects_missing_and_zero_diagonals() {
        let missing = matrix(&[&[0.0, 1.0], &[1.0, 1.0]]);
        assert!(matches!(
            JacobiPreconditioner::new(&missing),
            Err(LinearAlgebraError::MissingDiagonal { row: 0 })
        ));
        let zero = CsrMatrix::new(
            2,
            2,
            vec![0, 2, 4],
            vec![0, 1, 0, 1],
            vec![0.0, 1.0, 1.0, 2.0],
        )
        .unwrap();
        assert!(matches!(
            JacobiPreconditioner::new(&zero),
            Err(LinearAlgebraError::ZeroDiagonal { row: 0 })
        ));
    }

    #[test]
    fn cg_pcg_and_bicgstab_recover_known_solutions() {
        let spd_rows = [&[4.0, 1.0][..], &[1.0, 3.0][..]];
        let spd = matrix(&spd_rows);
        let exact = [1.0 / 11.0, 7.0 / 11.0];
        let rhs = dense_matvec(&spd_rows, &exact);
        let mut cg_solution = vec![0.0; 2];
        let cg_report = cg(&spd, &rhs, &mut cg_solution, OPTIONS).unwrap();
        assert_eq!(cg_report.status, LinearSolverStatus::Converged);
        assert!(
            cg_report.final_residual
                <= OPTIONS.absolute_tolerance
                    + OPTIONS.relative_tolerance * cg_report.initial_residual
        );
        assert_close(&cg_solution, &exact);

        let mut pcg_solution = vec![0.0; 2];
        let preconditioner = JacobiPreconditioner::new(&spd).unwrap();
        let pcg_report = pcg(&spd, &rhs, &mut pcg_solution, &preconditioner, OPTIONS).unwrap();
        assert!(pcg_report.converged());
        assert!(pcg_report.iterations <= cg_report.iterations);
        assert_close(&pcg_solution, &exact);

        let nonsymmetric_rows = [
            &[4.0, 1.0, 0.0][..],
            &[2.0, 3.0, 1.0][..],
            &[0.0, 1.0, 2.0][..],
        ];
        let nonsymmetric = matrix(&nonsymmetric_rows);
        assert!(!nonsymmetric.is_symmetric(0.0));
        let expected = [1.0, -2.0, 3.0];
        let rhs = dense_matvec(&nonsymmetric_rows, &expected);
        let mut solution = vec![0.0; 3];
        let report = bicgstab(&nonsymmetric, &rhs, &mut solution, OPTIONS).unwrap();
        assert!(report.converged());
        assert_close(&solution, &expected);
    }

    #[test]
    fn iterative_solvers_handle_large_initial_and_failure_cases() {
        let size = 32;
        let mut builder = CsrBuilder::new(size, size);
        for row in 0..size {
            builder.add(row, row, 2.0).unwrap();
            if row > 0 {
                builder.add(row, row - 1, -1.0).unwrap();
            }
            if row + 1 < size {
                builder.add(row, row + 1, -1.0).unwrap();
            }
        }
        let poisson = builder.finalize().unwrap();
        let exact: Vec<_> = (0..size)
            .map(|index| (index as f64 + 1.0) / size as f64)
            .collect();
        let rhs = poisson.spmv(&exact).unwrap();
        let mut solution = vec![0.0; size];
        let report = cg(&poisson, &rhs, &mut solution, OPTIONS).unwrap();
        assert!(report.converged());
        assert_close(&solution, &exact);

        let zero = vec![0.0; size];
        let report = bicgstab(&poisson, &zero, &mut solution.clone(), OPTIONS).unwrap();
        assert!(
            report.iterations > 0,
            "nonzero initial solution must be honored"
        );
        let mut exact_initial = exact.clone();
        let report = cg(&poisson, &rhs, &mut exact_initial, OPTIONS).unwrap();
        assert_eq!(report.iterations, 0);

        let mut zero_initial = vec![0.0; size];
        let report = bicgstab(&poisson, &zero, &mut zero_initial, OPTIONS).unwrap();
        assert!(report.converged());
        assert_eq!(report.iterations, 0);

        let mut limited = vec![0.0; size];
        let report = cg(
            &poisson,
            &rhs,
            &mut limited,
            LinearSolverOptions {
                max_iterations: 1,
                ..OPTIONS
            },
        )
        .unwrap();
        assert_eq!(report.status, LinearSolverStatus::MaxIterations);

        let singular = matrix(&[&[0.0]]);
        let error = bicgstab(&singular, &[1.0], &mut [0.0], OPTIONS).unwrap_err();
        assert!(matches!(error, LinearAlgebraError::Breakdown { .. }));

        assert!(matches!(
            cg(
                &poisson,
                &[f64::INFINITY; 32],
                &mut vec![0.0; size],
                OPTIONS
            ),
            Err(LinearAlgebraError::NonFiniteValue { .. })
        ));
    }
}
