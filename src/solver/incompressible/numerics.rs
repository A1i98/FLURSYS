use super::*;

pub(super) fn momentum_stability_numbers(
    dt: f64,
    nu: f64,
    dx: f64,
    dy: f64,
    max_abs_u: f64,
    max_abs_v: f64,
) -> (f64, f64) {
    let momentum_cfl = dt * (max_abs_u / dx + max_abs_v / dy);
    let viscous_diffusion_number = nu * dt * (1.0 / dx.powi(2) + 1.0 / dy.powi(2));

    (momentum_cfl, viscous_diffusion_number)
}

pub(super) fn adaptive_time_step(
    current_dt: f64,
    inputs: AdaptiveTimeStepInputs,
    settings: TimeStepSettings,
) -> Result<f64, String> {
    let advective_rate = inputs.max_abs_u / inputs.dx + inputs.max_abs_v / inputs.dy;
    let cfl_dt = if advective_rate > f64::EPSILON {
        settings.target_cfl / advective_rate
    } else {
        settings.max_dt
    };
    let inverse_spacing_sum = 1.0 / inputs.dx.powi(2) + 1.0 / inputs.dy.powi(2);
    let effective_diffusivity = inputs
        .thermal_diffusivity
        .map(|alpha| alpha.max(inputs.kinematic_viscosity))
        .unwrap_or(inputs.kinematic_viscosity);
    let diffusion_dt =
        settings.target_diffusion_number / (effective_diffusivity * inverse_spacing_sum);
    let stable_dt = settings.max_dt.min(cfl_dt).min(diffusion_dt);
    if stable_dt < settings.min_dt {
        return Err(format!(
            "required stable time step {stable_dt:.3e} is below configured min_dt {min_dt:.3e}; CFL constraint={cfl_dt:.3e}, diffusion constraint={diffusion_dt:.3e}",
            min_dt = settings.min_dt
        ));
    }
    let growth_limited_dt = stable_dt.min(current_dt * settings.max_growth_factor);

    Ok(growth_limited_dt.clamp(settings.min_dt, settings.max_dt))
}
