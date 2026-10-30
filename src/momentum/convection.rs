//! Conservative deferred linear-upwind correction with a cell slope limiter.
use super::*;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum MomentumConvection {
    #[default]
    Upwind,
    LimitedLinear,
    Lust,
    BoundedLust,
    LimitedLinearTvd,
    BoundedLimitedLinearTvd,
    LinearUpwind,
    /// Unbounded distance-weighted central interpolation (deferred).
    Central,
    /// Van Leer directional limiter with a local extrema bound.
    VanLeer,
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
    let gradient = if matches!(
        options.convection,
        MomentumConvection::BoundedLust
            | MomentumConvection::LimitedLinearTvd
            | MomentumConvection::BoundedLimitedLinearTvd
    ) {
        let mut faces = FaceField::filled(mesh, 0.);
        for (i, face) in mesh.faces().iter().enumerate() {
            faces[i] = if let Some(n) = face.neighbour {
                let a = (face.center - mesh.cells()[face.owner].center)
                    .dot(face.normal)
                    .abs();
                let b = (mesh.cells()[n].center - face.center)
                    .dot(face.normal)
                    .abs();
                (b * value[face.owner] + a * value[n]) / (a + b)
            } else {
                match options.boundary.condition(i) {
                    Some(ScalarBoundaryCondition::FixedValue(v)) => v,
                    _ => value[face.owner],
                }
            };
        }
        crate::gauss_gradient_from_faces(mesh, &faces)?
    } else {
        let owned_stencil;
        let stencil = if let Some(stencil) = options
            .least_squares_stencil
            .filter(|s| s.matches_boundary(options.boundary))
        {
            stencil
        } else {
            owned_stencil = LeastSquaresGradientStencil::new(mesh, options.boundary)?;
            &owned_stencil
        };
        least_squares_gradient(mesh, stencil, value)?
    };
    let mut bounds = Vec::new();
    if matches!(
        options.convection,
        MomentumConvection::LimitedLinear | MomentumConvection::VanLeer
    ) {
        bounds = value.values().iter().map(|v| (*v, *v)).collect();
        for (i, f) in mesh.faces().iter().enumerate() {
            if let Some(n) = f.neighbour {
                bounds[f.owner].0 = bounds[f.owner].0.min(value[n]);
                bounds[f.owner].1 = bounds[f.owner].1.max(value[n]);
                bounds[n].0 = bounds[n].0.min(value[f.owner]);
                bounds[n].1 = bounds[n].1.max(value[f.owner]);
            } else if let Some(ScalarBoundaryCondition::FixedValue(v)) =
                options.boundary.condition(i)
            {
                bounds[f.owner].0 = bounds[f.owner].0.min(v);
                bounds[f.owner].1 = bounds[f.owner].1.max(v);
            }
        }
    }
    let mut limiter = Vec::new();
    if options.convection == MomentumConvection::LimitedLinear {
        limiter = vec![1_f64; mesh.cell_count()];
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
    }
    for (i, f) in mesh.faces().iter().enumerate() {
        let phi = options.flux[i];
        // Keep physical boundary values and zero-gradient extrapolation unchanged.
        let Some(neighbour) = f.neighbour else {
            continue;
        };
        let up = if phi < 0.0 { neighbour } else { f.owner };
        let down = if phi < 0.0 { f.owner } else { neighbour };
        let reconstruction = gradient[up].dot(f.center - mesh.cells()[up].center);
        let delta = match options.convection {
            MomentumConvection::Upwind => 0.0,
            MomentumConvection::LimitedLinear => limiter[up] * reconstruction,
            MomentumConvection::LinearUpwind => reconstruction,
            MomentumConvection::LimitedLinearTvd | MomentumConvection::BoundedLimitedLinearTvd => {
                let jump = value[down] - value[up];
                let projected =
                    gradient[up].dot(mesh.cells()[down].center - mesh.cells()[up].center);
                let r = if jump.abs() > 1e-30 {
                    2. * projected / jump - 1.
                } else {
                    0.
                };
                let a = (f.center - mesh.cells()[up].center).dot(f.normal).abs();
                let b = (mesh.cells()[down].center - f.center).dot(f.normal).abs();
                (2. * r).clamp(0., 1.) * a / (a + b) * jump
            }
            MomentumConvection::Lust | MomentumConvection::BoundedLust => {
                let a = (f.center - mesh.cells()[up].center).dot(f.normal).abs();
                let b = (f.center - mesh.cells()[down].center).dot(f.normal).abs();
                let central = if a + b > 0.0 {
                    a / (a + b) * (value[down] - value[up])
                } else {
                    0.0
                };
                0.75 * central + 0.25 * reconstruction
            }
            MomentumConvection::Central => {
                let a = (f.center - mesh.cells()[up].center).dot(f.normal).abs();
                let b = (f.center - mesh.cells()[down].center).dot(f.normal).abs();
                if a + b > 0.0 {
                    a / (a + b) * (value[down] - value[up])
                } else {
                    0.0
                }
            }
            MomentumConvection::VanLeer => {
                let jump = value[down] - value[up];
                let projected =
                    gradient[up].dot(mesh.cells()[down].center - mesh.cells()[up].center);
                let r = if jump.abs() > 1e-30 {
                    (2.0 * projected / jump - 1.0).max(0.0)
                } else {
                    0.0
                };
                let psi = 2.0 * r / (1.0 + r);
                let a = (f.center - mesh.cells()[up].center).dot(f.normal).abs();
                let b = (f.center - mesh.cells()[down].center).dot(f.normal).abs();
                let candidate = if a + b > 0.0 {
                    psi * a / (a + b) * jump
                } else {
                    0.0
                };
                candidate.clamp(bounds[up].0 - value[up], bounds[up].1 - value[up])
            }
        };
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
