use flursys::{
    bicgstab, bicgstab_with_preconditioner, bicgstab_with_preconditioner_and_workspace,
    bicgstab_with_workspace, cg, cg_with_workspace, residual, BiCgStabWorkspace, CgWorkspace,
    CsrBuilder, JacobiPreconditioner, LinearSolverOptions,
};

#[test]
fn unpreconditioned_workspaces_match_fresh_with_changing_sizes_and_rhs() {
    let mut cg_workspace = CgWorkspace::default();
    let mut bicg_workspace = BiCgStabWorkspace::default();
    for (n, scale) in [(31, 1.0), (17, 0.4), (37, 2.0), (31, 1.5)] {
        let mut builder = CsrBuilder::new(n, n);
        for i in 0..n {
            builder.add(i, i, 4.0 * scale).unwrap();
            if i > 0 {
                builder.add(i, i - 1, -scale).unwrap();
            }
            if i + 1 < n {
                builder.add(i, i + 1, -scale).unwrap();
            }
        }
        let matrix = builder.finalize().unwrap();
        for zero in [false, true] {
            let rhs: Vec<_> = (0..n)
                .map(|i| if zero { 0.0 } else { (i as f64 * 0.13).sin() })
                .collect();
            let options = LinearSolverOptions::default();
            let mut fresh = vec![0.0; n];
            let mut reused = fresh.clone();
            let expected = cg(&matrix, &rhs, &mut fresh, options).unwrap();
            let actual =
                cg_with_workspace(&matrix, &rhs, &mut reused, options, &mut cg_workspace).unwrap();
            assert_eq!(actual, expected);
            assert_eq!(reused, fresh);
            fresh.fill(0.0);
            reused.fill(0.0);
            let expected = bicgstab(&matrix, &rhs, &mut fresh, options).unwrap();
            let actual =
                bicgstab_with_workspace(&matrix, &rhs, &mut reused, options, &mut bicg_workspace)
                    .unwrap();
            assert_eq!(actual, expected);
            assert_eq!(reused, fresh);
        }
    }
}

#[test]
fn recycled_momentum_workspace_matches_fresh_solves_when_sizes_and_rhs_change() {
    let mut workspace = BiCgStabWorkspace::default();
    let options = LinearSolverOptions {
        absolute_tolerance: 1e-11,
        relative_tolerance: 0.0,
        max_iterations: 200,
    };
    for (n, scale) in [(31, 1.0), (17, 0.4), (37, 2.0), (31, 1.5)] {
        let mut builder = CsrBuilder::new(n, n);
        for row in 0..n {
            builder.add(row, row, 4.0 * scale).unwrap();
            if row > 0 {
                builder.add(row, row - 1, -scale).unwrap();
            }
            if row + 1 < n {
                builder.add(row, row + 1, -0.5 * scale).unwrap();
            }
        }
        let matrix = builder.finalize().unwrap();
        let preconditioner = JacobiPreconditioner::new(&matrix).unwrap();
        let exact: Vec<_> = (0..n).map(|i| (i as f64 * 0.13).sin()).collect();
        let mut rhs = vec![0.0; n];
        matrix.spmv_into(&exact, &mut rhs).unwrap();
        let mut fresh = vec![0.2; n];
        let mut reused = fresh.clone();
        let expected =
            bicgstab_with_preconditioner(&matrix, &rhs, &mut fresh, &preconditioner, options)
                .unwrap();
        let actual = bicgstab_with_preconditioner_and_workspace(
            &matrix,
            &rhs,
            &mut reused,
            &preconditioner,
            options,
            &mut workspace,
        )
        .unwrap();
        assert!(actual.converged());
        assert_eq!(actual, expected);
        assert_eq!(reused, fresh);
        assert!(
            residual(&matrix, &reused, &rhs)
                .unwrap()
                .iter()
                .map(|x| x * x)
                .sum::<f64>()
                .sqrt()
                < 2e-11
        );
        assert!(reused
            .iter()
            .zip(&exact)
            .all(|(a, b)| (a - b).abs() < 1e-10));
        let zero = bicgstab_with_preconditioner_and_workspace(
            &matrix,
            &vec![0.0; n],
            &mut vec![0.0; n],
            &preconditioner,
            options,
            &mut workspace,
        )
        .unwrap();
        assert_eq!(zero.iterations, 0);
    }
}
