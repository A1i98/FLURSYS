use flursys::{
    bicgstab_with_preconditioner, bicgstab_with_preconditioner_and_workspace, residual,
    BiCgStabWorkspace, CsrBuilder, JacobiPreconditioner, LinearSolverOptions,
};

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
