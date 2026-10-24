// Velocity inlet: m/s. Channel y spans [0, params.height] metres.
// Steady signature: no physical-time argument.
fn profile(x, y, z) {
    let r = y / params.height;
    if r <= 0.0 || r >= 1.0 { return vec3(0.0, 0.0, 0.0); }
    vec3(4.0 * params.peak_speed * r * (1.0 - r), 0.0, 0.0)
}
