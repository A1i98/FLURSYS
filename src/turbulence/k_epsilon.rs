//! Independent finite-volume implementation of the standard and RNG constant-density
//! k-epsilon equations (Launder & Spalding, 1974). Optional smooth equilibrium wall treatment.
use super::*;
impl Closure {
    pub(super) fn advance_epsilon(
        &mut self,
        mesh: &UnstructuredMesh,
        state: &SimpleState,
        bc: &ResolvedVelocityBoundaryConditions,
        g: &[[[f64; 3]; 3]],
        nu: f64,
        dt: Option<f64>,
        linear: LinearSolverOptions,
    ) -> Result<(), SimpleError> {
        let realizable = self.config.model == TurbulenceModel::RealizableKEpsilon;
        let rng = self.config.model == TurbulenceModel::RngKEpsilon;
        let (c1, c2, sigma_k, sigma_e) = if rng {
            (1.42, 1.68, 0.71942, 0.71942)
        } else if realizable {
            (0.0, 1.9, 1.0, 1.2)
        } else {
            (1.44, 1.92, 1.0, 1.3)
        };
        let cmu = self.config.model.epsilon_cmu();
        let n = mesh.cell_count();
        let wall = self.wall_values(mesh, state, bc, nu)?;
        let mut kb = vec![None; mesh.face_count()];
        let mut eb = kb.clone();
        for (i, f) in mesh.faces().iter().enumerate() {
            if f.neighbour.is_none() {
                let incoming = state.face_flux[i] < 0.0;
                let (in_k, _in_w, in_e) = if incoming {
                    self.incoming_values(mesh, state, bc, i)?
                } else {
                    (self.config.k, self.config.omega, self.config.epsilon)
                };
                kb[i] = Some(if incoming {
                    ScalarBoundaryCondition::FixedValue(in_k)
                } else {
                    ScalarBoundaryCondition::ZeroGradient
                });
                eb[i] = Some(if incoming {
                    ScalarBoundaryCondition::FixedValue(in_e)
                } else {
                    ScalarBoundaryCondition::ZeroGradient
                });
            }
        }
        let kb = ResolvedScalarBoundaryConditions::from_face_conditions(mesh, kb)?;
        let eb = ResolvedScalarBoundaryConditions::from_face_conditions(mesh, eb)?;
        let mut dk = CellField::filled(mesh, nu);
        let mut de = dk.clone();
        let mut sk = CellField::filled(mesh, 0.0);
        let mut se = sk.clone();
        let mut sinkk = vec![0.0; n];
        let mut sinke = sinkk.clone();
        let epsilon = self.fields.epsilon.as_ref().unwrap();
        for i in 0..n {
            let k = self.fields.k[i].max(1e-14);
            let e = epsilon[i].max(1e-16);
            let nut = self.fields.nut[i];
            let tr = (0..3).map(|a| g[i][a][a]).sum::<f64>();
            let mut strain = 0.0;
            for a in 0..3 {
                for b in 0..3 {
                    strain += 0.5 * (g[i][a][b] + g[i][b][a]).powi(2);
                }
            }
            let production =
                wall.production[i].unwrap_or(nut * (strain - 2.0 / 3.0 * tr * tr).max(0.0));
            dk[i] += nut / sigma_k;
            de[i] += nut / sigma_e;
            sk[i] = production + (-2.0 / 3.0 * tr).max(0.0) * k;
            sinkk[i] = e / k + (2.0 / 3.0 * tr).max(0.0);
            // RNG strain correction. Split signed production to preserve a positive sink.
            let eta = (strain - 2.0 / 3.0 * tr * tr).max(0.0).sqrt() * k / e;
            let r = if rng {
                // Algebraically equivalent form avoids cubing very large eta.
                if eta > 1.0 {
                    let inv = eta.recip();
                    (inv * inv - inv / 4.38) / (inv.powi(3) + 0.012)
                } else {
                    eta * (1.0 - eta / 4.38) / (1.0 + 0.012 * eta.powi(3))
                }
            } else {
                0.0
            };
            if realizable {
                let mag_s = (strain - 2.0 / 3.0 * tr * tr).max(0.0).sqrt();
                let c1_local = (1.0 / (1.0 + 5.0 / eta.max(1e-300))).max(0.43);
                se[i] = c1_local * mag_s * e;
                sinke[i] = c2 * e / (k + (nu * e).sqrt());
                continue;
            }
            let epsilon_production = (c1 - r) * production * e / k;
            se[i] = epsilon_production.max(0.0) + (-2.0 / 3.0 * c1 * tr).max(0.0) * e;
            sinke[i] =
                c2 * e / k + (2.0 / 3.0 * c1 * tr).max(0.0) + (-epsilon_production).max(0.0) / e;
        }
        let old = self.old.as_ref();
        let report = transport::scalar_solve(
            mesh,
            &state.face_flux,
            &eb,
            &de,
            &se,
            &sinke,
            self.fields.epsilon.as_mut().unwrap(),
            old.and_then(|o| o.epsilon.as_ref()),
            dt,
            self.config.relaxation,
            self.linear.as_ref().map_or(
                crate::equations::EquationControl::from_options(linear),
                |e| e.epsilon,
            ),
            &self.walls,
            nu,
            Some(&wall.epsilon),
        )?;
        self.last_linear.insert("epsilon".into(), report);
        // Couple k destruction to the newly solved dissipation, not its lagged value.
        for i in 0..n {
            let e = self.fields.epsilon.as_mut().unwrap();
            if wall.epsilon[i].is_none() {
                e[i] = e[i]
                    .max(1e-16)
                    .max(cmu * self.fields.k[i].max(1e-14).powi(2) / (1e5 * nu));
            }
            let tr = (0..3).map(|a| g[i][a][a]).sum::<f64>();
            sinkk[i] = e[i] / self.fields.k[i].max(1e-14) + (2.0 / 3.0 * tr).max(0.0);
        }
        let report = transport::scalar_solve(
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
            let e = self.fields.epsilon.as_mut().unwrap();
            if wall.epsilon[i].is_some() {
                continue;
            }
            e[i] = e[i]
                .max(1e-16)
                .max(cmu * self.fields.k[i].powi(2) / (1e5 * nu));
        }
        Ok(())
    }
}
