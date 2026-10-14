use super::*;

impl IncompressibleSolver {
    /// Momentum coefficients and topology are fixed across pressure correctors.
    pub(super) fn assemble_implicit_pressure_matrix(
        &self,
        workspace: &mut ImplicitAssemblyWorkspace,
        u_pressure_diagonal: &[f64],
        v_pressure_diagonal: &[f64],
        transient_pressure_outlet: bool,
    ) -> Result<Option<(usize, usize)>, String> {
        let nx = self.grid.nx;
        let ny = self.grid.ny;
        let rho = self.cfg.case.density();
        let reference = self
            .pressure_reference_required()
            .then(|| self.find_reference_cell())
            .flatten();
        let pressure_assembly_profile = ProfileScope::new(ProfileStage::PressureAssembly);
        workspace.pressure_builder.clear();
        let pressure_builder = &mut workspace.pressure_builder;
        for j in 0..ny {
            for i in 0..nx {
                let row = i + nx * j;
                if self.solid[(i, j)]
                    || (!transient_pressure_outlet && self.is_pressure_outlet_cell(i, j))
                    || reference == Some((i, j))
                {
                    pressure_builder
                        .add(row, row, 1.0)
                        .map_err(|e| e.to_string())?;
                    continue;
                }
                let neighbors = [
                    (
                        i + 1 < nx,
                        (i + 1).min(nx - 1),
                        j,
                        self.grid.dy / (rho * self.grid.dx),
                        1.0 / u_pressure_diagonal[i + 1 + (nx + 1) * j],
                    ),
                    (
                        i > 0,
                        i.saturating_sub(1),
                        j,
                        self.grid.dy / (rho * self.grid.dx),
                        1.0 / u_pressure_diagonal[i + (nx + 1) * j],
                    ),
                    (
                        j + 1 < ny,
                        i,
                        (j + 1).min(ny - 1),
                        self.grid.dx / (rho * self.grid.dy),
                        1.0 / v_pressure_diagonal[i + nx * (j + 1)],
                    ),
                    (
                        j > 0,
                        i,
                        j.saturating_sub(1),
                        self.grid.dx / (rho * self.grid.dy),
                        1.0 / v_pressure_diagonal[i + nx * j],
                    ),
                ];
                let mut diagonal = 0.0;
                for (exists, ni, nj, metric, reciprocal) in neighbors {
                    if !exists || self.solid[(ni, nj)] {
                        continue;
                    }
                    let coefficient = metric * reciprocal;
                    diagonal += coefficient;
                    let neighbor = ni + nx * nj;
                    if (transient_pressure_outlet || !self.is_pressure_outlet_cell(ni, nj))
                        && reference != Some((ni, nj))
                    {
                        pressure_builder
                            .add(row, neighbor, -coefficient)
                            .map_err(|e| e.to_string())?;
                    }
                }
                if transient_pressure_outlet && i + 1 == nx {
                    // Dirichlet pressure is prescribed on the outlet face,
                    // half a cell from the last pressure unknown.
                    diagonal += 2.0 * self.grid.dy
                        / (rho * self.grid.dx * u_pressure_diagonal[nx + (nx + 1) * j]);
                }
                pressure_builder
                    .add(row, row, diagonal.max(f64::MIN_POSITIVE))
                    .map_err(|e| e.to_string())?;
            }
        }
        workspace
            .pressure_builder
            .finalize_into(&mut workspace.pressure_matrix)
            .map_err(|e| e.to_string())?;
        drop(pressure_assembly_profile);
        Ok(reference)
    }
}
