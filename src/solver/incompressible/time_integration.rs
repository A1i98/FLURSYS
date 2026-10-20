/// Time derivative for the implicit transient momentum equations.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TransientTimeScheme {
    #[default]
    Euler,
    Backward,
}

impl TransientTimeScheme {
    /// a0 U[n+1] = a1 U[n] + a2 U[n-1] + dt RHS[n+1].
    /// Start BDF2 with one backward-Euler step; support a shortened final step.
    pub(super) fn coefficients(
        self,
        dt: f64,
        previous_dt: Option<f64>,
    ) -> Result<[f64; 3], String> {
        if !dt.is_finite() || dt <= 0.0 {
            return Err("time derivative requires a finite positive timestep".into());
        }
        if self == Self::Euler || previous_dt.is_none() {
            return Ok([1.0, 1.0, 0.0]);
        }
        let ratio = dt / previous_dt.unwrap();
        if !ratio.is_finite() || ratio <= 0.0 || ratio >= 1.0 + 2.0_f64.sqrt() {
            return Err("BDF2 timestep ratio must be positive and below 1+sqrt(2)".into());
        }
        Ok([
            (1.0 + 2.0 * ratio) / (1.0 + ratio),
            1.0 + ratio,
            -ratio * ratio / (1.0 + ratio),
        ])
    }
}

#[derive(Clone, Copy)]
pub(super) struct TransientMomentum<'a> {
    pub old: (&'a [f64], &'a [f64]),
    pub older: Option<(&'a [f64], &'a [f64])>,
    pub weights: [f64; 3],
}

impl TransientMomentum<'_> {
    pub fn u_source(self, row: usize) -> f64 {
        (self.weights[1] * self.old.0[row] + self.weights[2] * self.older.map_or(0.0, |x| x.0[row]))
            / self.weights[0]
    }
    pub fn v_source(self, row: usize) -> f64 {
        (self.weights[1] * self.old.1[row] + self.weights[2] * self.older.map_or(0.0, |x| x.1[row]))
            / self.weights[0]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn variable_step_backward_derivative_is_exact_for_quadratic() {
        for (dt, previous) in [(0.1, 0.1), (0.04, 0.1), (0.15, 0.1)] {
            let [a0, a1, a2] = TransientTimeScheme::Backward
                .coefficients(dt, Some(previous))
                .unwrap();
            let t = 0.8_f64;
            let derivative =
                (a0 * t * t - a1 * (t - dt).powi(2) - a2 * (t - dt - previous).powi(2)) / dt;
            assert!((derivative - 2.0 * t).abs() < 1e-13);
            assert!((a0 - a1 - a2).abs() < 1e-14);
        }
    }

    #[test]
    fn backward_with_euler_start_has_second_order_decay_accuracy() {
        fn error(scheme: TransientTimeScheme, steps: usize) -> f64 {
            let dt = 1.0 / steps as f64;
            let (mut old, mut older) = (1.0, 1.0);
            for step in 0..steps {
                let [a0, a1, a2] = scheme.coefficients(dt, (step > 0).then_some(dt)).unwrap();
                let next = (a1 * old + a2 * older) / (a0 + dt);
                older = old;
                old = next;
            }
            (old - (-1.0_f64).exp()).abs()
        }
        let b = error(TransientTimeScheme::Backward, 40) / error(TransientTimeScheme::Backward, 80);
        let e = error(TransientTimeScheme::Euler, 40) / error(TransientTimeScheme::Euler, 80);
        assert!((3.8..4.2).contains(&b), "BDF2 error ratio {b}");
        assert!((1.9..2.1).contains(&e), "Euler error ratio {e}");
    }

    #[test]
    fn backward_rejects_unstable_growth_and_uses_euler_at_startup() {
        assert_eq!(
            TransientTimeScheme::Backward
                .coefficients(0.1, None)
                .unwrap(),
            [1.0, 1.0, 0.0]
        );
        assert!(TransientTimeScheme::Backward
            .coefficients(0.3, Some(0.1))
            .is_err());
        assert!(TransientTimeScheme::Backward
            .coefficients(0.1, Some(0.0))
            .is_err());
    }
}
