//! Fixed-mesh nested pressure corrections and conservative non-orthogonal fluxes.
use super::*;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum CouplingAlgorithm {
    #[default]
    Simple,
    Pimple,
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PressureCoupling {
    pub algorithm: CouplingAlgorithm,
    pub pressure_correctors: usize,
    /// Extra deferred correction passes: zero preserves the two-point operator.
    pub non_orthogonal_correctors: usize,
    /// 0 disables explicit correction; 1 is unlimited. Intermediate values
    /// bound the explicit flux relative to the two-point pressure flux.
    pub non_orthogonal_limiter: f64,
}
impl Default for PressureCoupling {
    fn default() -> Self {
        Self {
            algorithm: CouplingAlgorithm::Simple,
            pressure_correctors: 1,
            non_orthogonal_correctors: 0,
            non_orthogonal_limiter: 0.5,
        }
    }
}
impl PressureCoupling {
    pub fn validate(self, transient: bool) -> Result<(), String> {
        if !(1..=8).contains(&self.pressure_correctors)
            || self.non_orthogonal_correctors > 8
            || !self.non_orthogonal_limiter.is_finite()
            || !(0.0..=1.0).contains(&self.non_orthogonal_limiter)
        {
            return Err("Require 1..8 pressure correctors, 0..8 extra non-orthogonal passes and limiter in [0,1]".into());
        }
        if self.algorithm == CouplingAlgorithm::Simple && self.pressure_correctors != 1 {
            return Err("Multiple inner pressure correctors require PIMPLE".into());
        }
        if self.algorithm == CouplingAlgorithm::Pimple && !transient {
            return Err("PIMPLE requires physical-time transient integration".into());
        }
        Ok(())
    }
}

/// Deferred flux E·grad(p'), E = d_f d - rAU_f S. The same face value
/// enters both neighbouring cells through integrated divergence.
fn explicit_flux(
    mesh: &UnstructuredMesh,
    coefficients: &FaceField<f64>,
    correction: &CellField<f64>,
    gradient: &CellField<Vec3>,
    boundary: Option<&ResolvedScalarBoundaryConditions>,
    limiter: f64,
) -> Result<FaceField<f64>, SimpleError> {
    let mut flux = FaceField::filled(mesh, 0.0);
    for (i, f) in mesh.faces().iter().enumerate() {
        if coefficients[i] == 0.0 {
            continue;
        }
        let (target, delta, g) = if let Some(n) = f.neighbour {
            let a = (f.center - mesh.cells()[f.owner].center).norm();
            let b = (f.center - mesh.cells()[n].center).norm();
            (
                mesh.cells()[n].center,
                correction[n] - correction[f.owner],
                (gradient[f.owner] * b + gradient[n] * a) / (a + b),
            )
        } else if let Some((n, center)) = boundary.and_then(|b| b.cyclic_neighbour(mesh, i)) {
            let a = (f.center - mesh.cells()[f.owner].center).norm();
            let b = (f.center - center).norm();
            (
                center,
                correction[n] - correction[f.owner],
                (gradient[f.owner] * b + gradient[n] * a) / (a + b),
            )
        } else if let Some(ScalarBoundaryCondition::FixedValue(p)) =
            boundary.and_then(|b| b.condition(i))
        {
            (f.center, p - correction[f.owner], gradient[f.owner])
        } else {
            continue;
        };
        let d = target - mesh.cells()[f.owner].center;
        // coefficients already contain the over-relaxed orthogonal factor.
        let response = coefficients[i] * f.area_vector.dot(d) / f.area_vector.norm_squared();
        let explicit = (d * coefficients[i] - f.area_vector * response).dot(g);
        let scale = if limiter >= 1.0 {
            1.0
        } else if limiter <= 0.0 {
            0.0
        } else {
            (limiter * (coefficients[i] * delta).abs()
                / ((1.0 - limiter) * explicit.abs() + 1e-300))
                .min(1.0)
        };
        flux[i] = explicit * scale;
        if !flux[i].is_finite() {
            return Err(SimpleError::InvalidPressureFaceCoefficient {
                face: i,
                value: flux[i],
            });
        }
    }
    if let Some(b) = boundary {
        for i in 0..mesh.face_count() {
            if let Some(link) = b.cyclic_link(i) {
                if i < link.peer_face {
                    flux[link.peer_face] = -flux[i];
                }
            }
        }
    }
    Ok(flux)
}

pub(super) fn project(
    mesh: &UnstructuredMesh,
    predicted: &FaceField<f64>,
    coefficients: &FaceField<f64>,
    options: &SimpleOptions<'_>,
    stencil: &LeastSquaresGradientStencil,
    workspace: &mut crate::equations::PressureWorkspace,
    cancelled: &mut impl FnMut() -> bool,
) -> Result<(CellField<f64>, FaceField<f64>, LinearSolveReport), SimpleError> {
    let assemble = |flux: &FaceField<f64>| match options.pressure_correction_boundary {
        Some(b) => assemble_pressure_correction_with_reference(
            mesh,
            flux,
            coefficients,
            b,
            Some(options.reference_cell),
        ),
        None => assemble_pressure_correction(mesh, flux, coefficients, options.reference_cell),
    };
    let mut system = {
        let _timer = crate::compute::StageTimer::new(crate::compute::Stage::PressureAssembly);
        assemble(predicted)?
    };
    let base_rhs = system.rhs.clone();
    let control =
        options
            .equations
            .map(|e| e.pressure)
            .unwrap_or(crate::equations::EquationControl {
                preconditioner: crate::equations::EquationPreconditioner::Ic0,
                ..crate::equations::EquationControl::from_options(options.pressure_solver)
            });
    let mut correction = CellField::filled(mesh, 0.0);
    let mut rhs_flux = predicted.clone();
    let mut last = None;
    for pass in 0..=options.pressure_coupling.non_orthogonal_correctors {
        if cancelled() {
            return Err(SimpleError::Cancelled);
        }
        if pass > 0 {
            let gradient = pressure_coupling_gradient(
                mesh,
                &correction,
                stencil,
                options.pressure_correction_boundary,
            )?;
            let extra = explicit_flux(
                mesh,
                coefficients,
                &correction,
                &gradient,
                options.pressure_correction_boundary,
                options.pressure_coupling.non_orthogonal_limiter,
            )?;
            let divergence = crate::integrated_divergence(mesh, &extra)?;
            for i in 0..mesh.cell_count() {
                system.rhs[i] = base_rhs[i] - divergence[i];
            }
            // Symmetric reference elimination must remain unchanged.
            if !mesh.faces().iter().enumerate().any(|(i, _)| {
                matches!(
                    options
                        .pressure_correction_boundary
                        .and_then(|b| b.condition(i)),
                    Some(ScalarBoundaryCondition::FixedValue(_))
                )
            }) {
                system.rhs[options.reference_cell] = 0.0;
            }
            for i in 0..mesh.face_count() {
                rhs_flux[i] = predicted[i] + extra[i];
            }
        }
        let report = workspace.solve(
            control,
            &system.matrix,
            &system.rhs,
            correction.values_mut(),
        )?;
        if !report.converged() {
            return Err(SimpleError::PressureLinearDidNotConverge);
        }
        last = Some(report);
    }
    // Commit precisely the lagged explicit flux used in the final linear RHS.
    let flux = match options.pressure_correction_boundary {
        Some(b) => correct_face_flux_with_boundary(mesh, &rhs_flux, coefficients, &correction, b)?,
        None => correct_face_flux(mesh, &rhs_flux, coefficients, &correction)?,
    };
    Ok((
        correction,
        flux,
        last.ok_or_else(|| {
            SimpleError::PhysicalBoundary("Missing pressure correction pass".into())
        })?,
    ))
}

/// Jacobi momentum defect reconstructs H/A from the frozen momentum matrices
/// using the latest U; pressure remains the current nonlinear iterate.
pub(super) fn predictor_from_defect(
    mesh: &UnstructuredMesh,
    systems: [&crate::MomentumSystem; 3],
    velocity: &CellField<Vec3>,
    assembled_gradient: &CellField<Vec3>,
    current_gradient: &CellField<Vec3>,
    response: &CellField<f64>,
) -> Result<CellField<Vec3>, SimpleError> {
    let mut result = velocity.clone();
    for (component, system) in systems.into_iter().enumerate() {
        let values = velocity
            .values()
            .iter()
            .map(|v| [v.x, v.y, v.z][component])
            .collect::<Vec<_>>();
        let defect = crate::residual(system.matrix(), &values, system.rhs())?;
        let diag = system.matrix().diagonal()?;
        for i in 0..mesh.cell_count() {
            if !diag[i].is_finite() || diag[i] <= 0.0 {
                return Err(SimpleError::InvalidMomentumDiagonal {
                    cell: i,
                    value: diag[i],
                });
            }
            let pg = assembled_gradient[i] - current_gradient[i];
            let value =
                values[i] + defect[i] / diag[i] + response[i] * [pg.x, pg.y, pg.z][component];
            match component {
                0 => result[i].x = value,
                1 => result[i].y = value,
                _ => result[i].z = value,
            }
        }
    }
    Ok(result)
}
