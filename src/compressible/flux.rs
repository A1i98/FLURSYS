//! Normal physical flux and HLLC contact-restoring approximate Riemann flux.
use super::*;
fn physical(p: Primitive, gas: IdealGas, n: Vec3) -> Conservative {
    let q = p.conservative(gas);
    let u = p.velocity.dot(n);
    Conservative([
        q.0[0] * u,
        q.0[1] * u + p.pressure * n.x,
        q.0[2] * u + p.pressure * n.y,
        q.0[3] * u + p.pressure * n.z,
        (q.0[4] + p.pressure) * u,
    ])
}
fn rusanov(l: Primitive, r: Primitive, gas: IdealGas, n: Vec3) -> Conservative {
    let speed = (l.velocity.dot(n).abs() + gas.sound_speed(l))
        .max(r.velocity.dot(n).abs() + gas.sound_speed(r));
    physical(l, gas, n).add(physical(r, gas, n)).scale(0.5).sub(
        r.conservative(gas)
            .sub(l.conservative(gas))
            .scale(0.5 * speed),
    )
}
pub(super) fn evaluate(
    l: Primitive,
    r: Primitive,
    gas: IdealGas,
    n: Vec3,
    scheme: FluxScheme,
) -> (Conservative, bool) {
    if matches!(scheme, FluxScheme::Rusanov) {
        return (rusanov(l, r, gas, n), false);
    }
    let ul = l.velocity.dot(n);
    let ur = r.velocity.dot(n);
    let sl = (ul - gas.sound_speed(l)).min(ur - gas.sound_speed(r));
    let sr = (ul + gas.sound_speed(l)).max(ur + gas.sound_speed(r));
    if sl >= 0.0 {
        return (physical(l, gas, n), false);
    }
    if sr <= 0.0 {
        return (physical(r, gas, n), false);
    }
    if matches!(scheme, FluxScheme::CentralUpwind) {
        // Two-sided local-speed central-upwind flux, with first-order states.
        let value = physical(l, gas, n)
            .scale(sr)
            .sub(physical(r, gas, n).scale(sl))
            .add(r.conservative(gas).sub(l.conservative(gas)).scale(sl * sr))
            .scale(1.0 / (sr - sl));
        return (value, false);
    }
    let den = l.density * (sl - ul) - r.density * (sr - ur);
    let sm =
        (r.pressure - l.pressure + l.density * ul * (sl - ul) - r.density * ur * (sr - ur)) / den;
    let (p, s, u) = if sm >= 0.0 { (l, sl, ul) } else { (r, sr, ur) };
    let rho = p.density * (s - u) / (s - sm);
    let v = p.velocity + n * (sm - u);
    let energy = rho
        * (p.conservative(gas).0[4] / p.density
            + (sm - u) * (sm + p.pressure / (p.density * (s - u))));
    let star = Conservative([rho, rho * v.x, rho * v.y, rho * v.z, energy]);
    if !sm.is_finite() || star.primitive(gas).is_err() {
        return (rusanov(l, r, gas, n), true);
    }
    let f = physical(p, gas, n).add(star.sub(p.conservative(gas)).scale(s));
    if f.0.iter().all(|x| x.is_finite()) {
        (f, false)
    } else {
        (rusanov(l, r, gas, n), true)
    }
}
