// Only static outlet pressure is scripted (m^2/s^2).
// Incoming backflow velocity remains the explicit physical boundary value.
fn profile(x, y, z, t) {
    params.reference_pressure + params.amplitude * sin(2.0 * PI * params.frequency * t)
}
