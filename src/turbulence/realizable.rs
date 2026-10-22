//! Local realizable k-epsilon viscosity coefficient (Shih et al., 1995).
//! Inertial frame only; constants A0=4 and invariant-based As.
pub(super) fn cmu(g: &[[f64; 3]; 3], k: f64, epsilon: f64) -> f64 {
    // Normalize the tensor before evaluating the cubic invariant.
    let scale = g.iter().flatten().fold(0.0_f64, |m, x| m.max(x.abs()));
    if scale == 0.0 {
        return 0.25;
    }
    let trace = (0..3).map(|a| g[a][a] / scale).sum::<f64>();
    let mut s = [[0.0; 3]; 3];
    let mut s2 = 0.0;
    let mut rotation2 = 0.0;
    for a in 0..3 {
        for b in 0..3 {
            s[a][b] =
                0.5 * (g[a][b] / scale + g[b][a] / scale) - if a == b { trace / 3.0 } else { 0.0 };
            s2 += 2.0 * s[a][b].powi(2);
            rotation2 += (0.5 * (g[a][b] / scale - g[b][a] / scale)).powi(2);
        }
    }
    let mut cubic = 0.0;
    for a in 0..3 {
        for b in 0..3 {
            for c in 0..3 {
                cubic += s[a][b] * s[b][c] * s[c][a];
            }
        }
    }
    let w = if s2 > 0.0 {
        2.0 * 2.0_f64.sqrt() * cubic / (s2 * s2.sqrt())
    } else {
        0.0
    };
    let phi = (6.0_f64.sqrt() * w).clamp(-1.0, 1.0).acos() / 3.0;
    let a_s = 6.0_f64.sqrt() * phi.cos();
    let u_s = (0.5 * s2 + rotation2).sqrt() * scale;
    1.0 / (4.0 + a_s * u_s * k / epsilon)
}
