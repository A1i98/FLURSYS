// Fixed temperature: K. Coordinates are metres, channel height is 1 m.
fn profile(x, y, z) {
    let r = y / params.height;
    let shape = if r <= 0.0 || r >= 1.0 { 0.0 } else { 4.0 * r * (1.0 - r) };
    params.base_temperature + params.temperature_rise * shape
}
