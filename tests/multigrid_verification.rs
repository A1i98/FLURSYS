use flursys::{
    dot, pcg_with_preconditioner_and_workspace, residual, CgWorkspace, CsrBuilder,
    LinearSolverOptions, Preconditioner, StructuredMultigridPreconditioner,
};

// Variable-coefficient pressure operator with odd dimensions, an internal solid
// mask and a Dirichlet outlet. This exercises aggregation across wall cells.
fn pressure_matrix(nx: usize, ny: usize, scale: f64) -> flursys::CsrMatrix {
    let solid = |i: usize, j: usize| (4..7).contains(&i) && (3..6).contains(&j);
    let mut builder = CsrBuilder::new(nx * ny, nx * ny);
    for j in 0..ny {
        for i in 0..nx {
            let row = i + nx * j;
            if solid(i, j) {
                builder.add(row, row, scale).unwrap();
                continue;
            }
            let mut diagonal = if i + 1 == nx { 2.0 * scale } else { 0.0 };
            for (di, dj) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                let x = i as isize + di;
                let y = j as isize + dj;
                if x < 0 || x >= nx as isize || y < 0 || y >= ny as isize {
                    continue;
                }
                let (x, y) = (x as usize, y as usize);
                if solid(x, y) {
                    continue;
                }
                let column = x + nx * y;
                let value = scale * (1.0 + (row.min(column) % 7) as f64 * 0.1);
                diagonal += value;
                builder.add(row, column, -value).unwrap();
            }
            builder.add(row, row, diagonal).unwrap();
        }
    }
    builder.finalize().unwrap()
}

#[test]
fn masked_multigrid_is_symmetric_positive_and_recovers_a_manufactured_solution() {
    let (nx, ny) = (13, 9);
    let matrix = pressure_matrix(nx, ny, 1.0);
    let size = nx * ny;
    let exact: Vec<_> = (0..size).map(|i| (i as f64 * 0.13).sin()).collect();
    let mut rhs = vec![0.0; size];
    matrix.spmv_into(&exact, &mut rhs).unwrap();
    let y: Vec<_> = (0..size).map(|i| (i as f64 * 0.21).cos()).collect();
    for sweeps in [1, 2, 3, 8] {
        let mg = StructuredMultigridPreconditioner::with_smoothing_steps(&matrix, nx, ny, sweeps)
            .unwrap();
        let mut mx = vec![0.0; size];
        let mut my = vec![0.0; size];
        mg.apply_into(&exact, &mut mx).unwrap();
        mg.apply_into(&y, &mut my).unwrap();
        let xy = dot(&exact, &my).unwrap();
        let yx = dot(&y, &mx).unwrap();
        assert!((xy - yx).abs() < 1.0e-10 * xy.abs().max(yx.abs()).max(1.0));
        assert!(dot(&exact, &mx).unwrap() > 0.0);
        let mut solution = vec![0.0; size];
        let report = pcg_with_preconditioner_and_workspace(
            &matrix,
            &rhs,
            &mut solution,
            &mg,
            LinearSolverOptions {
                absolute_tolerance: 1.0e-11,
                relative_tolerance: 0.0,
                max_iterations: 500,
            },
            &mut CgWorkspace::default(),
        )
        .unwrap();
        assert!(report.converged());
        assert!(
            residual(&matrix, &solution, &rhs)
                .unwrap()
                .iter()
                .map(|x| x * x)
                .sum::<f64>()
                .sqrt()
                < 2.0e-11
        );
        assert!(solution
            .iter()
            .zip(&exact)
            .all(|(a, b)| (a - b).abs() < 1.0e-10));
    }
}

#[test]
fn reused_hierarchy_matches_a_fresh_hierarchy_after_coefficient_updates() {
    let (nx, ny) = (13, 9);
    let initial = pressure_matrix(nx, ny, 1.0);
    let mut reused = StructuredMultigridPreconditioner::new(&initial, nx, ny).unwrap();
    let rhs: Vec<_> = (0..nx * ny).map(|i| (i as f64).sin()).collect();
    for scale in [0.5, 1.5, 3.0, 1.0] {
        let updated = pressure_matrix(nx, ny, scale);
        reused.update_values(&updated).unwrap();
        let fresh = StructuredMultigridPreconditioner::new(&updated, nx, ny).unwrap();
        let mut actual = vec![0.0; nx * ny];
        let mut expected = actual.clone();
        reused.apply_into(&rhs, &mut actual).unwrap();
        fresh.apply_into(&rhs, &mut expected).unwrap();
        assert!(actual
            .iter()
            .zip(expected)
            .all(|(a, b)| (a - b).abs() < 1.0e-12));
    }
    assert!(StructuredMultigridPreconditioner::with_smoothing_steps(&initial, nx, ny, 0).is_err());
    assert!(StructuredMultigridPreconditioner::with_smoothing_steps(&initial, nx, ny, 9).is_err());
}

#[test]
fn algebraic_multigrid_recovers_manufactured_fields_with_all_smoothers_and_updates() {
    use flursys::{
        AlgebraicMultigrid, MultigridCoarseSolver, MultigridOptions, MultigridResidualNorm,
        MultigridSmoother,
    };
    let (nx, ny) = (13, 9);
    let exact: Vec<_> = (0..nx * ny).map(|i| (i as f64 * 0.13).sin()).collect();
    for smoother in [
        MultigridSmoother::GaussSeidel,
        MultigridSmoother::SymmetricGaussSeidel,
        MultigridSmoother::DicGaussSeidel,
    ] {
        for coarse_solver in [
            MultigridCoarseSolver::PcgDic,
            MultigridCoarseSolver::Cholesky,
        ] {
            for pre_sweeps in [0, 1] {
                for residual_norm in [
                    MultigridResidualNorm::AbsoluteL2,
                    MultigridResidualNorm::NormalizedL1,
                ] {
                    let options = MultigridOptions {
                        smoother,
                        coarse_solver,
                        pre_sweeps,
                        residual_norm,
                        ..Default::default()
                    };
                    let mut solver =
                        AlgebraicMultigrid::new(&pressure_matrix(nx, ny, 1.0), options).unwrap();
                    for scale in [1.0, 0.5, 3.0] {
                        let original = pressure_matrix(nx, ny, scale);
                        let mut values = original.values().to_vec();
                        // Change relative coefficients, not just a common scale.
                        for row in 0..nx * ny {
                            let start = original.row_offsets()[row];
                            let end = original.row_offsets()[row + 1];
                            let entry = start
                                + original.column_indices()[start..end]
                                    .binary_search(&row)
                                    .unwrap();
                            values[entry] += 0.03 * (row % 5) as f64;
                        }
                        let matrix = flursys::CsrMatrix::new(
                            nx * ny,
                            nx * ny,
                            original.row_offsets().to_vec(),
                            original.column_indices().to_vec(),
                            values,
                        )
                        .unwrap();
                        solver.update_values(&matrix).unwrap();
                        let mut rhs = vec![0.0; exact.len()];
                        matrix.spmv_into(&exact, &mut rhs).unwrap();
                        // Exercise a nonzero starting iterate as well as recycled scratch.
                        let mut actual = vec![0.1; exact.len()];
                        let report = solver
                            .solve(
                                &rhs,
                                &mut actual,
                                LinearSolverOptions {
                                    absolute_tolerance: 1e-12,
                                    relative_tolerance: 0.0,
                                    max_iterations: 500,
                                },
                            )
                            .unwrap();
                        assert!(report.converged(), "{options:?}: {report:?}");
                        let actual_residual = residual(&matrix, &actual, &rhs).unwrap();
                        assert!(actual_residual.iter().map(|x| x * x).sum::<f64>().sqrt() < 1e-8);
                        assert!(actual.iter().zip(&exact).all(|(a, b)| (a - b).abs() < 1e-7));
                    }
                }
            }
        }
    }
}

#[test]
fn algebraic_coarse_solve_separates_many_independent_constraints() {
    use flursys::{
        AlgebraicMultigrid, MultigridCoarseSolver, MultigridOptions, MultigridResidualNorm,
    };
    // 64 independent constraints and a 7-cell SPD tridiagonal block.
    // Only the 7 coupled unknowns should determine the coarsest-level size.
    let n = 71;
    let mut builder = CsrBuilder::new(n, n);
    for row in 0..n {
        builder.add(row, row, 4.0).unwrap();
        if row >= 64 && row + 1 < n {
            builder.add(row, row + 1, -1.0).unwrap();
            builder.add(row + 1, row, -1.0).unwrap();
        }
    }
    let matrix = builder.finalize().unwrap();
    let exact: Vec<_> = (0..n).map(|i| 0.2 + (i as f64).sin()).collect();
    let mut rhs = vec![0.0; n];
    matrix.spmv_into(&exact, &mut rhs).unwrap();
    for coarse_solver in [
        MultigridCoarseSolver::PcgDic,
        MultigridCoarseSolver::Cholesky,
    ] {
        let mut solver = AlgebraicMultigrid::new(
            &matrix,
            MultigridOptions {
                coarse_solver,
                residual_norm: MultigridResidualNorm::AbsoluteL2,
                ..Default::default()
            },
        )
        .unwrap();
        assert_eq!(solver.statistics().len(), 1);
        let mut actual = vec![0.0; n];
        let report = solver
            .solve(
                &rhs,
                &mut actual,
                LinearSolverOptions {
                    absolute_tolerance: 1e-11,
                    relative_tolerance: 0.0,
                    max_iterations: 100,
                },
            )
            .unwrap();
        assert!(report.converged());
        assert!(actual
            .iter()
            .zip(&exact)
            .all(|(a, b)| (a - b).abs() < 1e-10));
        // Cached symmetry checks must still reject an invalid coefficient update.
        let mut values = matrix.values().to_vec();
        let entry = matrix.row_offsets()[64 + 1] - 1;
        values[entry] += 0.1;
        let asymmetric = flursys::CsrMatrix::new(
            n,
            n,
            matrix.row_offsets().to_vec(),
            matrix.column_indices().to_vec(),
            values,
        )
        .unwrap();
        assert!(solver.update_values(&asymmetric).is_err());
    }
}

#[test]
fn connected_singletons_join_a_neighbor_cluster_without_absorbing_constraints() {
    use flursys::{AlgebraicMultigrid, MultigridOptions, MultigridResidualNorm};
    // A star graph exposes the greedy pairing failure: once its center is
    // paired, all other leaves still need to join that connected aggregate.
    let n = 81;
    let center = 64;
    let mut builder = CsrBuilder::new(n, n);
    for row in 0..n {
        builder
            .add(row, row, if row == center { 32.0 } else { 4.0 })
            .unwrap();
        if row > center {
            builder.add(center, row, -1.0).unwrap();
            builder.add(row, center, -1.0).unwrap();
        }
    }
    let matrix = builder.finalize().unwrap();
    let exact: Vec<_> = (0..n).map(|i| (i as f64 * 0.17).cos()).collect();
    let mut rhs = vec![0.0; n];
    matrix.spmv_into(&exact, &mut rhs).unwrap();
    let mut solver = AlgebraicMultigrid::new(
        &matrix,
        MultigridOptions {
            coarsest_cells: 2,
            residual_norm: MultigridResidualNorm::AbsoluteL2,
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(solver.statistics().len(), 2);
    // 64 isolated constraints plus one connected aggregate, with no bridging.
    assert_eq!(solver.statistics()[1].cells, 65);
    let mut actual = vec![0.0; n];
    let report = solver
        .solve(
            &rhs,
            &mut actual,
            LinearSolverOptions {
                absolute_tolerance: 1e-11,
                relative_tolerance: 0.0,
                max_iterations: 100,
            },
        )
        .unwrap();
    assert!(report.converged());
    assert!(actual
        .iter()
        .zip(&exact)
        .all(|(a, b)| (a - b).abs() < 1e-10));
}

#[test]
fn geometric_aggregation_preserves_a_masked_pressure_solution_and_rejects_bad_weights() {
    use flursys::{AlgebraicMultigrid, MultigridOptions, MultigridResidualNorm};
    let (nx, ny) = (13, 9);
    let matrix = pressure_matrix(nx, ny, 1.0);
    let mut weights = vec![0.0; matrix.nnz()];
    for row in 0..matrix.nrows() {
        for (entry, weight) in weights
            .iter_mut()
            .enumerate()
            .take(matrix.row_offsets()[row + 1])
            .skip(matrix.row_offsets()[row])
        {
            let column = matrix.column_indices()[entry];
            if column != row {
                *weight = if column.abs_diff(row) == nx { 0.2 } else { 0.1 };
            }
        }
    }
    let options = MultigridOptions {
        residual_norm: MultigridResidualNorm::AbsoluteL2,
        ..Default::default()
    };
    for merge_levels in [1, 2, 4] {
        let mut solver = AlgebraicMultigrid::new_with_face_weights(
            &matrix,
            &weights,
            MultigridOptions {
                merge_levels,
                ..options
            },
        )
        .unwrap();
        let exact: Vec<_> = (0..nx * ny).map(|i| (i as f64 * 0.13).sin()).collect();
        for scale in [1.0, 0.5, 3.0] {
            let updated = pressure_matrix(nx, ny, scale);
            solver.update_values(&updated).unwrap();
            let mut rhs = vec![0.0; nx * ny];
            updated.spmv_into(&exact, &mut rhs).unwrap();
            let mut actual = vec![0.0; nx * ny];
            assert!(solver
                .solve(
                    &rhs,
                    &mut actual,
                    LinearSolverOptions {
                        absolute_tolerance: 1e-11,
                        relative_tolerance: 0.0,
                        max_iterations: 500
                    }
                )
                .unwrap()
                .converged());
            assert!(actual.iter().zip(&exact).all(|(a, b)| (a - b).abs() < 1e-9));
        }
    }
    assert!(AlgebraicMultigrid::new_with_face_weights(
        &matrix,
        &weights[..weights.len() - 1],
        options
    )
    .is_err());
    for merge_levels in [0, 5] {
        assert!(AlgebraicMultigrid::new_with_face_weights(
            &matrix,
            &weights,
            MultigridOptions {
                merge_levels,
                ..options
            }
        )
        .is_err());
    }
    weights[0] = f64::NAN;
    assert!(AlgebraicMultigrid::new_with_face_weights(&matrix, &weights, options).is_err());
}

#[test]
fn flexible_gcr_checks_true_residual_across_restarts_reuse_and_matrix_updates() {
    use flursys::{AlgebraicMultigrid, MultigridOptions, MultigridResidualNorm};
    let (nx, ny) = (25, 17);
    let n = nx * ny;
    let exact: Vec<_> = (0..n).map(|i| (i as f64 * 0.17).sin()).collect();
    for dimension in [0, 1, 2, 4] {
        for norm in [
            MultigridResidualNorm::AbsoluteL2,
            MultigridResidualNorm::NormalizedL1,
        ] {
            let mut solver = AlgebraicMultigrid::new(
                &pressure_matrix(nx, ny, 1.0),
                MultigridOptions {
                    krylov_dimension: dimension,
                    residual_norm: norm,
                    ..Default::default()
                },
            )
            .unwrap();
            for scale in [1.0, 0.4, 2.0] {
                let matrix = pressure_matrix(nx, ny, scale);
                solver.update_values(&matrix).unwrap();
                let mut rhs = vec![0.0; n];
                matrix.spmv_into(&exact, &mut rhs).unwrap();
                let mut actual = vec![0.2; n];
                let mut ax = vec![0.0; n];
                matrix.spmv_into(&actual, &mut ax).unwrap();
                let mut row_sums = vec![0.0; n];
                matrix.spmv_into(&vec![1.0; n], &mut row_sums).unwrap();
                let normalization = rhs
                    .iter()
                    .zip(&ax)
                    .zip(&row_sums)
                    .map(|((&b, &a), &sum)| (a - 0.2 * sum).abs() + (b - 0.2 * sum).abs())
                    .sum::<f64>();
                let report = solver
                    .solve(
                        &rhs,
                        &mut actual,
                        LinearSolverOptions {
                            absolute_tolerance: 1e-12,
                            relative_tolerance: 0.0,
                            max_iterations: 500,
                        },
                    )
                    .unwrap();
                assert!(report.converged());
                let r = residual(&matrix, &actual, &rhs).unwrap();
                let measured = match norm {
                    MultigridResidualNorm::AbsoluteL2 => {
                        r.iter().map(|x| x * x).sum::<f64>().sqrt()
                    }
                    MultigridResidualNorm::NormalizedL1 => {
                        r.iter().map(|x| x.abs()).sum::<f64>() / normalization
                    }
                };
                assert!(measured < 1.01e-12, "{dimension} {norm:?}: {measured}");
                assert!((measured - report.final_residual).abs() < 1e-14);
                assert!(actual.iter().zip(&exact).all(|(a, b)| (a - b).abs() < 1e-8));
                if dimension == 1 || dimension == 2 {
                    assert!(report.iterations > dimension);
                }
                // An early return must reset history just as a completed solve does.
                assert_eq!(
                    solver
                        .solve(
                            &vec![0.0; n],
                            &mut vec![0.0; n],
                            LinearSolverOptions::default()
                        )
                        .unwrap()
                        .iterations,
                    0
                );
            }
        }
    }
    assert!(AlgebraicMultigrid::new(
        &pressure_matrix(nx, ny, 1.0),
        MultigridOptions {
            krylov_dimension: 17,
            ..Default::default()
        }
    )
    .is_err());
}
