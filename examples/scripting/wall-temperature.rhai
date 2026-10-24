// No-slip flow wall + Fixed temperature thermal condition. Temperature in K.
fn profile(x, y, z, t) {
    let f = if t < params.ramp_time { t / params.ramp_time } else { 1.0 };
    params.initial_temperature + f * (params.final_temperature - params.initial_temperature)
}
