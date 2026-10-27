//! Positive, wall-resolved SA-noft2 transport on a fixed mesh.
use super::*;
const SIGMA: f64 = 2.0 / 3.0;
const CB1: f64 = 0.1355;
const CB2: f64 = 0.622;
const KAPPA: f64 = 0.41;
pub(super) fn eddy_viscosity(nt: f64, nu: f64) -> f64 {
    let chi = nt / nu;
    let fv1 = if chi > 0.0 {
        1.0 / (1.0 + (7.1 / chi).powi(3))
    } else {
        0.0
    };
    nt * fv1
}
impl Closure {
    pub(super) fn incoming_sa(&self, face: usize) -> Result<f64, SimpleError> {
        match self.incoming[face] {
            Some(TurbulenceInlet::ModifiedViscosity { nu_tilda }) => Ok(nu_tilda),
            None => Ok(self.config.nu_tilda),
            _ => Err(SimpleError::PhysicalBoundary(
                "SA incoming data must use modified viscosity, not k/omega/intensity".into(),
            )),
        }
    }
    pub(super) fn advance_sa(
        &mut self,
        mesh: &UnstructuredMesh,
        state: &SimpleState,
        g: &[[[f64; 3]; 3]],
        nu: f64,
        dt: Option<f64>,
        linear: LinearSolverOptions,
    ) -> Result<(), SimpleError> {
        let field = self.fields.nu_tilda.as_ref().ok_or_else(|| {
            SimpleError::PhysicalBoundary("Missing SA modified-viscosity field".into())
        })?;
        let mut conditions = vec![None; mesh.face_count()];
        for (i, f) in mesh.faces().iter().enumerate() {
            if f.neighbour.is_none() {
                conditions[i] = Some(if self.walls[i] {
                    ScalarBoundaryCondition::FixedValue(0.0)
                } else if state.face_flux[i] < 0.0 {
                    ScalarBoundaryCondition::FixedValue(self.incoming_sa(i)?)
                } else {
                    ScalarBoundaryCondition::ZeroGradient
                });
            }
        }
        let bc = ResolvedScalarBoundaryConditions::from_face_conditions(mesh, conditions)?;
        let gradient = scalar_gradient(mesh, field, &bc)?;
        let n = mesh.cell_count();
        let mut diffusion = CellField::filled(mesh, nu / SIGMA);
        let mut source = CellField::filled(mesh, 0.0);
        let mut sink = vec![0.0; n];
        let cw1 = CB1 / (KAPPA * KAPPA) + (1.0 + CB2) / SIGMA;
        for i in 0..n {
            let nt = field[i].max(0.0);
            let chi = nt / nu;
            if !chi.is_finite() {
                return Err(SimpleError::PhysicalBoundary(
                    "Non-finite SA viscosity ratio".into(),
                ));
            }
            let fv1 = if chi > 0.0 {
                1.0 / (1.0 + (7.1 / chi).powi(3))
            } else {
                0.0
            };
            let fv2 = 1.0 - chi / (1.0 + chi * fv1);
            let vort = ((g[i][2][1] - g[i][1][2]).powi(2)
                + (g[i][0][2] - g[i][2][0]).powi(2)
                + (g[i][1][0] - g[i][0][1]).powi(2))
            .sqrt();
            let d2 = self.distance[i] * self.distance[i];
            let st = (vort + fv2 * nt / (KAPPA * KAPPA * d2)).max(0.3 * vort);
            let r = (nt / (st.max(1e-30) * KAPPA * KAPPA * d2)).min(10.0);
            let gw = r + 0.3 * (r.powi(6) - r);
            let fw =
                gw * ((1.0 + 2.0_f64.powi(6)) / (gw.powi(6) + 2.0_f64.powi(6))).powf(1.0 / 6.0);
            diffusion[i] = (nu + nt) / SIGMA;
            source[i] = CB1 * st * nt + CB2 / SIGMA * gradient[i].norm_squared();
            sink[i] = cw1 * fw * nt / d2;
            if !diffusion[i].is_finite()
                || !source[i].is_finite()
                || !sink[i].is_finite()
                || sink[i] < 0.0
            {
                return Err(SimpleError::PhysicalBoundary(format!(
                    "Invalid SA transport coefficient in cell {i}"
                )));
            }
        }
        let control = self
            .linear
            .as_ref()
            .map_or(crate::equations::EquationControl::transport(linear), |e| {
                e.nu_tilda
            });
        let field = self
            .fields
            .nu_tilda
            .as_mut()
            .ok_or_else(|| SimpleError::PhysicalBoundary("Missing SA state".into()))?;
        let report = transport::scalar_solve(
            "nu_tilda",
            mesh,
            &state.face_flux,
            &bc,
            &diffusion,
            &source,
            &sink,
            field,
            self.old.as_ref().and_then(|o| o.nu_tilda.as_ref()),
            dt,
            self.config.relaxation,
            control,
            &self.walls,
            nu / SIGMA,
            None,
        )?;
        for value in field.values_mut() {
            *value = value.max(0.0);
        }
        self.last_linear.insert("nu_tilda".into(), report);
        Ok(())
    }
}
