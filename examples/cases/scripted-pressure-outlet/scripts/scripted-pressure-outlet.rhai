// Static kinematic gauge pressure p/rho: m^2/s^2, NOT Pa or total pressure.
fn profile(x, y, z, t) {
    let f = if t < params.ramp_time { t / params.ramp_time } else { 1.0 };
    params.initial_pressure + f * (params.final_pressure - params.initial_pressure)
}
