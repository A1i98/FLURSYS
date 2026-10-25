use super::*;
pub fn is_standard_sod(mesh: &UnstructuredMesh, settings: &Settings) -> bool {
    let left = Primitive {
        density: 1.,
        pressure: 1.,
        velocity: Vec3::ZERO,
    };
    let right = Primitive {
        density: 0.125,
        pressure: 0.1,
        velocity: Vec3::ZERO,
    };
    let min = mesh
        .points()
        .iter()
        .map(|p| p.position.x)
        .fold(f64::INFINITY, f64::min);
    let max = mesh
        .points()
        .iter()
        .map(|p| p.position.x)
        .fold(f64::NEG_INFINITY, f64::max);
    settings.initial == right
        && (settings.gas.gamma - 1.4).abs() < 1e-12
        && settings.viscosity == Viscosity::Inviscid
        && settings.regions.len() == 1
        && settings.regions[0].state == left
        && (settings.regions[0].maximum.x - 0.5).abs() < 1e-12
        && settings.regions[0].minimum.x <= min
        && min.abs() < 1e-12
        && (max - 1.).abs() < 1e-12
        && mesh.cells().iter().all(|c| {
            c.center.y >= settings.regions[0].minimum.y
                && c.center.y <= settings.regions[0].maximum.y
                && c.center.z >= settings.regions[0].minimum.z
                && c.center.z <= settings.regions[0].maximum.z
        })
        && settings.boundaries.get("left") == Some(&Boundary::Transmissive)
        && settings.boundaries.get("right") == Some(&Boundary::Transmissive)
        && settings
            .boundaries
            .iter()
            .all(|(name, bc)| name == "left" || name == "right" || *bc == Boundary::SlipWall)
}
pub fn sod(x: f64, time: f64) -> Primitive {
    let gamma = 1.4_f64;
    let al = gamma.sqrt();
    let ar = (gamma * 0.1 / 0.125).sqrt();
    let left = |p: f64| 2. * al / (gamma - 1.) * ((p).powf((gamma - 1.) / (2. * gamma)) - 1.);
    let right = |p: f64| {
        (p - 0.1) * (2. / ((gamma + 1.) * 0.125) / (p + (gamma - 1.) / (gamma + 1.) * 0.1)).sqrt()
    };
    let mut low = 0.1;
    let mut high = 1.;
    for _ in 0..64 {
        let mid = 0.5 * (low + high);
        if left(mid) + right(mid) > 0. {
            high = mid;
        } else {
            low = mid;
        }
    }
    let ps = 0.5 * (low + high);
    let us = 0.5 * (right(ps) - left(ps));
    let xi = if time > 0. {
        (x - 0.5) / time
    } else if x < 0.5 {
        f64::NEG_INFINITY
    } else {
        f64::INFINITY
    };
    let asl = al * ps.powf((gamma - 1.) / (2. * gamma));
    let (rho, u, p) = if xi <= -al {
        (1., 0., 1.)
    } else if xi < us - asl {
        let u = 2. / (gamma + 1.) * (al + xi);
        let a = 2. / (gamma + 1.) * (al - 0.5 * (gamma - 1.) * xi);
        (
            (a / al).powf(2. / (gamma - 1.)),
            u,
            (a / al).powf(2. * gamma / (gamma - 1.)),
        )
    } else if xi <= us {
        (ps.powf(1. / gamma), us, ps)
    } else {
        let shock =
            ar * ((gamma + 1.) / (2. * gamma) * ps / 0.1 + (gamma - 1.) / (2. * gamma)).sqrt();
        if xi < shock {
            (
                0.125 * (ps / 0.1 + (gamma - 1.) / (gamma + 1.))
                    / ((gamma - 1.) / (gamma + 1.) * ps / 0.1 + 1.),
                us,
                ps,
            )
        } else {
            (0.125, 0., 0.1)
        }
    };
    Primitive {
        density: rho,
        velocity: Vec3::new(u, 0., 0.),
        pressure: p,
    }
}
#[cfg(test)]
mod tests {
    #[test]
    fn benchmark_support_sod_star_states() {
        let p = super::sod(0.5, 0.2);
        assert!((p.pressure - 0.303130178).abs() < 1e-8);
        assert!((p.velocity.x - 0.92745262).abs() < 1e-8);
        assert_eq!(super::sod(0., 0.).density, 1.);
        assert_eq!(super::sod(1., 0.).density, 0.125);
    }
}
