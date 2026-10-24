// Velocity inlet: m/s. t is physical time in seconds, frequency in Hz.
fn profile(x, y, z, t) {
    let u = params.base_speed + params.amplitude * sin(2.0 * PI * params.frequency * t);
    vec3(u, 0.0, 0.0)
}
