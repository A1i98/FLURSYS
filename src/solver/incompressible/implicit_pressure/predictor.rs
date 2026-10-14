use super::*;

impl IncompressibleSolver {
    /// Jacobi momentum reconstruction at fixed coefficients before the next
    /// pressure correction. Unlike a repeated divergence projection, this
    /// includes the off-diagonal momentum response to the corrected velocity.
    pub(super) fn rebuild_transient_pressure_predictor(
        &mut self,
        workspace: &mut ImplicitAssemblyWorkspace,
        u_diagonal: &[f64],
        v_diagonal: &[f64],
    ) -> Result<(), String> {
        let _profile = ProfileScope::new(ProfileStage::PressurePredictor);
        let nx = self.grid.nx;
        let ny = self.grid.ny;
        let rho = self.cfg.case.density();
        workspace
            .u_matrix
            .spmv_into(self.u_star.as_slice(), &mut workspace.u_product)
            .map_err(|e| e.to_string())?;
        workspace
            .v_matrix
            .spmv_into(self.v_star.as_slice(), &mut workspace.v_product)
            .map_err(|e| e.to_string())?;
        for j in 0..ny {
            for i in 1..nx {
                if u_face_open(&self.solid, nx, i, j) {
                    let face = i + (nx + 1) * j;
                    let pressure_force = (self.p[(i, j)] - self.p[(i - 1, j)]) * self.grid.dy / rho;
                    self.u_star.as_mut_slice()[face] += (workspace.u_pressure_free_rhs[face]
                        - pressure_force
                        - workspace.u_product[face])
                        / u_diagonal[face];
                }
            }
        }
        for j in 1..ny {
            for i in 0..nx {
                if v_face_open(&self.solid, ny, i, j) {
                    let face = i + nx * j;
                    let pressure_force = (self.p[(i, j)] - self.p[(i, j - 1)]) * self.grid.dx / rho;
                    self.v_star.as_mut_slice()[face] += (workspace.v_pressure_free_rhs[face]
                        - pressure_force
                        - workspace.v_product[face])
                        / v_diagonal[face];
                }
            }
        }
        if matches!(
            self.cfg
                .boundary_overrides
                .kind(&self.cfg.case, Side::Right),
            BoundaryKind::PressureOutlet { .. }
        ) {
            for j in 0..ny {
                if u_face_open(&self.solid, nx, nx, j) {
                    self.u_star[(nx, j)] = self.u_star[(nx - 1, j)];
                }
            }
        }
        Ok(())
    }
}
