// Inlet/outlet temperature: incoming K; outgoing faces retain zero gradient.
fn profile(x, y, z, t) {
    params.base_temperature + params.amplitude * sin(2.0 * PI * params.frequency * t)
}
