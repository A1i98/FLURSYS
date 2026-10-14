use super::*;

impl IncompressibleSolver {
    /// Measure the momentum equations on the pressure-corrected field, rather
    /// than returning the residual of the preceding momentum predictor solve.
    pub(super) fn corrected_implicit_momentum_residual(
        &self,
        workspace: &mut ImplicitAssemblyWorkspace,
    ) -> Result<f64, String> {
        let nx = self.grid.nx;
        let ny = self.grid.ny;
        let rho = self.cfg.case.density();
        workspace
            .u_matrix
            .spmv_into(self.u.as_slice(), &mut workspace.u_product)
            .map_err(|e| e.to_string())?;
        workspace
            .v_matrix
            .spmv_into(self.v.as_slice(), &mut workspace.v_product)
            .map_err(|e| e.to_string())?;
        let mut u_residual_squared = 0.0;
        let mut u_rhs_squared = 0.0;
        for j in 0..ny {
            for i in 1..nx {
                if !u_face_open(&self.solid, nx, i, j) {
                    continue;
                }
                let face = i + (nx + 1) * j;
                let rhs = workspace.u_pressure_free_rhs[face]
                    - (self.p[(i, j)] - self.p[(i - 1, j)]) * self.grid.dy / rho;
                let residual = rhs - workspace.u_product[face];
                u_residual_squared += residual * residual;
                u_rhs_squared += rhs * rhs;
            }
        }
        let mut v_residual_squared = 0.0;
        let mut v_rhs_squared = 0.0;
        for j in 1..ny {
            for i in 0..nx {
                if !v_face_open(&self.solid, ny, i, j) {
                    continue;
                }
                let face = i + nx * j;
                let rhs = workspace.v_pressure_free_rhs[face]
                    - (self.p[(i, j)] - self.p[(i, j - 1)]) * self.grid.dx / rho;
                let residual = rhs - workspace.v_product[face];
                v_residual_squared += residual * residual;
                v_rhs_squared += rhs * rhs;
            }
        }
        let normalized = |r: f64, b: f64| r.sqrt() / b.sqrt().max(f64::MIN_POSITIVE);
        Ok(normalized(u_residual_squared, u_rhs_squared)
            .max(normalized(v_residual_squared, v_rhs_squared)))
    }
}
