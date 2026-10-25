fn profile(x, y, z, t) {
    let r = (y - 4.0) / 4.0;
    let u = 1.0 - r * r;
    let v = if t < 5.0 { 0.001 * u * sin(2.0 * PI * t) } else { 0.0 };
    vec3(u, v, 0.0)
}
