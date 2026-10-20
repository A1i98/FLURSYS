//! Conservative deferred linear-upwind correction with a cell slope limiter.
use super::*;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum MomentumConvection {
    #[default]
    Upwind,
    LimitedLinear,
}

pub(super) fn correct(
    mesh: &UnstructuredMesh,
    options: MomentumOptions<'_>,
    rhs: &mut [f64],
) -> Result<(), MomentumError> {
    if options.convection == MomentumConvection::Upwind {
        return Ok(());
    }
    let value = options
        .correction_field
        .ok_or(MomentumError::MissingCorrectionField)?;
    let stencil = LeastSquaresGradientStencil::new(mesh, options.boundary)?;
    let gradient = least_squares_gradient(mesh, &stencil, value)?;
    let mut bounds = value.values().iter().map(|v| (*v, *v)).collect::<Vec<_>>();
    for (i, f) in mesh.faces().iter().enumerate() {
        if let Some(n) = f.neighbour {
            bounds[f.owner].0 = bounds[f.owner].0.min(value[n]);
            bounds[f.owner].1 = bounds[f.owner].1.max(value[n]);
            bounds[n].0 = bounds[n].0.min(value[f.owner]);
            bounds[n].1 = bounds[n].1.max(value[f.owner]);
        } else if let Some(ScalarBoundaryCondition::FixedValue(v)) = options.boundary.condition(i) {
            bounds[f.owner].0 = bounds[f.owner].0.min(v);
            bounds[f.owner].1 = bounds[f.owner].1.max(v);
        }
    }
    let mut limiter = vec![1_f64; mesh.cell_count()];
    for f in mesh.faces() {
        for cell in std::iter::once(f.owner).chain(f.neighbour) {
            let d = gradient[cell].dot(f.center - mesh.cells()[cell].center);
            let a = if d > 0.0 {
                (bounds[cell].1 - value[cell]) / d
            } else if d < 0.0 {
                (bounds[cell].0 - value[cell]) / d
            } else {
                1.0
            };
            limiter[cell] = limiter[cell].min(a.clamp(0.0, 1.0));
        }
    }
    for (i, f) in mesh.faces().iter().enumerate() {
        let phi = options.flux[i];
        // Keep physical boundary values and zero-gradient extrapolation unchanged.
        let Some(neighbour) = f.neighbour else {
            continue;
        };
        let up = if phi < 0.0 { neighbour } else { f.owner };
        let delta = limiter[up] * gradient[up].dot(f.center - mesh.cells()[up].center);
        let correction = phi * delta;
        if !correction.is_finite() {
            return Err(MomentumError::NonFiniteSource {
                cell: up,
                value: correction,
            });
        }
        rhs[f.owner] -= correction;
        if let Some(n) = f.neighbour {
            rhs[n] += correction;
        }
    }
    Ok(())
}
