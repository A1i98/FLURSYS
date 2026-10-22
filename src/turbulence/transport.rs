//! Positivity-preserving source splitting for k/omega. Upwind scalar transport.
use super::*;
impl Closure {
    pub(super) fn advance_rans(
        &mut self,
        mesh: &UnstructuredMesh,
        state: &SimpleState,
        bc: &ResolvedVelocityBoundaryConditions,
        g: &[[[f64; 3]; 3]],
        nu: f64,
        dt: Option<f64>,
        linear: LinearSolverOptions,
    ) -> Result<(), SimpleError> {
        let n = mesh.cell_count();
        let wall = self.wall_values(mesh, state, bc, nu)?;
        let omega_walls = self.config.wall_treatment == WallTreatment::EquilibriumKOmega;
        let mut kb = vec![None; mesh.face_count()];
        let mut wb = kb.clone();
        for (i, f) in mesh.faces().iter().enumerate() {
            if f.neighbour.is_none() {
                let incoming = state.face_flux[i] < 0.0;
                let (in_k, in_w, _in_e) = if incoming {
                    self.incoming_values(mesh, state, bc, i)?
                } else {
                    (self.config.k, self.config.omega, self.config.epsilon)
                };
                kb[i] = Some(if self.walls[i] {
                    if omega_walls {
                        ScalarBoundaryCondition::ZeroGradient
                    } else {
                        ScalarBoundaryCondition::FixedValue(0.0)
                    }
                } else if incoming {
                    ScalarBoundaryCondition::FixedValue(in_k)
                } else {
                    ScalarBoundaryCondition::ZeroGradient
                });
                wb[i] = Some(if self.walls[i] && omega_walls {
                    ScalarBoundaryCondition::ZeroGradient
                } else if self.walls[i] {
                    let y = (f.center - mesh.cells()[f.owner].center)
                        .dot(f.area_vector / f.area_vector.norm())
                        .abs();
                    ScalarBoundaryCondition::FixedValue(60.0 * nu / (0.075 * y * y))
                } else if incoming {
                    ScalarBoundaryCondition::FixedValue(in_w)
                } else {
                    ScalarBoundaryCondition::ZeroGradient
                });
            }
        }
        let kb = ResolvedScalarBoundaryConditions::from_face_conditions(mesh, kb)?;
        let wb = ResolvedScalarBoundaryConditions::from_face_conditions(mesh, wb)?;
        let gradk = scalar_gradient(mesh, &self.fields.k, &kb)?;
        let gradw = scalar_gradient(mesh, &self.fields.omega, &wb)?;
        let mut dk = CellField::filled(mesh, nu);
        let mut dw = dk.clone();
        let mut sk = CellField::filled(mesh, 0.0);
        let mut sw = sk.clone();
        let mut sinkk = vec![0.0; n];
        let mut sinkw = sinkk.clone();
        for i in 0..n {
            let k = self.fields.k[i].max(1e-14);
            let w = self.fields.omega[i].max(1e-12);
            let y = self.distance[i];
            let nut = self.fields.nut[i];
            let tr = (0..3).map(|a| g[i][a][a]).sum::<f64>();
            let mut s2 = 0.0;
            for a in 0..3 {
                for b in 0..3 {
                    s2 += 0.5 * (g[i][a][b] + g[i][b][a]).powi(2);
                }
            }
            let gnu = (s2 - 2.0 / 3.0 * tr * tr).max(0.0);
            let production = wall.production[i].unwrap_or(nut * gnu);
            let (ak, aw, beta, gamma, cross) = if self.config.model == TurbulenceModel::KOmegaSst {
                let cd = 2.0 * 0.856 * gradk[i].dot(gradw[i]) / w;
                let arg = (k.sqrt() / (0.09 * w * y))
                    .max(500.0 * nu / (y * y * w))
                    .min(4.0 * 0.856 * k / (cd.max(1e-10) * y * y))
                    .min(10.0);
                let f1 = arg.powi(4).tanh();
                (
                    0.85 * f1 + 1.0 * (1.0 - f1),
                    0.5 * f1 + 0.856 * (1.0 - f1),
                    0.075 * f1 + 0.0828 * (1.0 - f1),
                    (5.0 / 9.0) * f1 + 0.44 * (1.0 - f1),
                    (1.0 - f1) * cd,
                )
            } else {
                (0.5, 0.5, 0.075, 5.0 / 9.0, 0.0)
            };
            dk[i] += ak * nut;
            dw[i] += aw * nut;
            let pk = if self.config.model == TurbulenceModel::KOmegaSst {
                production.min(10.0 * 0.09 * k * w)
            } else {
                production
            };
            sk[i] = pk;
            sinkk[i] = 0.09 * w;
            let omega_production = if self.config.model == TurbulenceModel::KOmegaSst {
                let f2 = ((2.0 * k.sqrt() / (0.09 * w * y))
                    .max(500.0 * nu / (y * y * w))
                    .min(100.0)
                    .powi(2))
                .tanh();
                gamma * gnu.min(10.0 / 0.31 * 0.09 * w * (0.31 * w).max(f2 * s2.sqrt()))
            } else {
                gamma * production * w / k
            };
            sw[i] = omega_production + cross.max(0.0);
            sinkw[i] = beta * w + (-cross).max(0.0) / w;
            // Dilatation source split; zero for an exactly solenoidal field.
            let a = 2.0 / 3.0 * tr;
            sinkk[i] += a.max(0.0);
            sk[i] += (-a).max(0.0) * k;
            sinkw[i] += (gamma * a).max(0.0);
            sw[i] += (-gamma * a).max(0.0) * w;
        }
        let old = self.old.as_ref();
        let report = scalar_solve(
            mesh,
            &state.face_flux,
            &wb,
            &dw,
            &sw,
            &sinkw,
            &mut self.fields.omega,
            old.map(|o| &o.omega),
            dt,
            self.config.relaxation,
            self.linear.as_ref().map_or(
                crate::equations::EquationControl::from_options(linear),
                |e| e.omega,
            ),
            &self.walls,
            nu,
            if omega_walls { Some(&wall.omega) } else { None },
        )?;
        self.last_linear.insert("omega".into(), report);
        // Use the newly solved bounded omega in k destruction and SST limiter.
        for i in 0..n {
            let k = self.fields.k[i].max(1e-14);
            self.fields.omega[i] = self.fields.omega[i].max(1e-12).max(k / (1e5 * nu));
            let w = self.fields.omega[i];
            let tr = (0..3).map(|a| g[i][a][a]).sum::<f64>();
            sinkk[i] = 0.09 * w + (2.0 / 3.0 * tr).max(0.0);
            if self.config.model == TurbulenceModel::KOmegaSst {
                let mut strain = 0.0;
                for a in 0..3 {
                    for b in 0..3 {
                        strain += 0.5 * (g[i][a][b] + g[i][b][a]).powi(2);
                    }
                }
                let production = self.fields.nut[i] * (strain - 2.0 / 3.0 * tr * tr).max(0.0);
                sk[i] = wall.production[i].unwrap_or(production.min(10.0 * 0.09 * k * w))
                    + (-2.0 / 3.0 * tr).max(0.0) * k;
            }
        }
        let report = scalar_solve(
            mesh,
            &state.face_flux,
            &kb,
            &dk,
            &sk,
            &sinkk,
            &mut self.fields.k,
            old.map(|o| &o.k),
            dt,
            self.config.relaxation,
            self.linear.as_ref().map_or(
                crate::equations::EquationControl::from_options(linear),
                |e| e.k,
            ),
            &self.walls,
            nu,
            None,
        )?;
        self.last_linear.insert("k".into(), report);
        for i in 0..n {
            self.fields.k[i] = self.fields.k[i].max(1e-14);
            self.fields.omega[i] = self.fields.omega[i]
                .max(1e-12)
                .max(self.fields.k[i] / (1e5 * nu));
        }
        Ok(())
    }
}
#[allow(clippy::too_many_arguments)]
pub(super) fn scalar_solve(
    mesh: &UnstructuredMesh,
    phi: &FaceField<f64>,
    bc: &ResolvedScalarBoundaryConditions,
    d: &CellField<f64>,
    source: &CellField<f64>,
    sink: &[f64],
    field: &mut CellField<f64>,
    old: Option<&CellField<f64>>,
    dt: Option<f64>,
    alpha: f64,
    linear: crate::equations::EquationControl,
    walls: &[bool],
    nu: f64,
    prescribed: Option<&[Option<f64>]>,
) -> Result<LinearSolveReport, SimpleError> {
    let mut face_diffusion = interpolate_diffusivity(
        mesh,
        Diffusivity::CellField(d),
        DiffusivityInterpolation::Harmonic,
    )?;
    for (i, wall) in walls.iter().enumerate() {
        if *wall {
            face_diffusion[i] = nu;
        }
    }
    let mut options = MomentumOptions::steady(Diffusivity::Faces(&face_diffusion), bc, phi);
    options.old = old;
    options.time_step = dt;
    options.source = Some(source);
    // Relax the complete matrix, including the implicit destruction term.
    options.diffusion.non_orthogonal_correction = NonOrthogonalCorrection::Limited;
    options.correction_field = Some(field);
    options.allow_iterative_outlet_backflow = true;
    let system = assemble_momentum_component(mesh, options)?;
    let matrix = system.matrix();
    let mut builder = CsrBuilder::with_capacity(
        mesh.cell_count(),
        mesh.cell_count(),
        matrix.nnz() + mesh.cell_count(),
    );
    let mut rhs = system.rhs().to_vec();
    for (i, cell) in mesh.cells().iter().enumerate() {
        if let Some(v) = prescribed.and_then(|p| p[i]) {
            builder.add(i, i, 1.0)?;
            rhs[i] = v;
            continue;
        }
        let diag = matrix.get(i, i).unwrap_or(0.0) + sink[i] * cell.volume;
        for j in matrix.row_offsets()[i]..matrix.row_offsets()[i + 1] {
            let col = matrix.column_indices()[j];
            if let Some(v) = prescribed.and_then(|p| p[col]) {
                rhs[i] -= matrix.values()[j] * v;
            } else {
                builder.add(i, col, matrix.values()[j])?;
            }
        }
        builder.add(i, i, sink[i] * cell.volume + (1.0 / alpha - 1.0) * diag)?;
        rhs[i] += (1.0 / alpha - 1.0) * diag * field[i];
    }
    let matrix = builder.finalize()?;
    let report = linear.solve(&matrix, &rhs, field.values_mut(), false)?;
    if report.status != LinearSolverStatus::Converged {
        return Err(SimpleError::PhysicalBoundary(
            "Turbulence scalar linear solve did not converge".into(),
        ));
    }
    if field.values().iter().any(|x| !x.is_finite()) {
        return Err(SimpleError::PhysicalBoundary(
            "Non-finite turbulence transport solution".into(),
        ));
    }
    Ok(report)
}
