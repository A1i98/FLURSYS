//! Incoming RANS data; outgoing faces retain zero-gradient conditions.
use serde::{Deserialize, Serialize};
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "kebab-case")]
pub enum TurbulenceInlet {
    Direct {
        k: f64,
        omega: f64,
        epsilon: f64,
    },
    /// I is a fraction, L in metres; local incoming velocity magnitude is used.
    IntensityLength {
        intensity: f64,
        length: f64,
    },
}
impl TurbulenceInlet {
    pub fn validate(&self) -> Result<(), String> {
        let valid = match *self {
            Self::Direct { k, omega, epsilon } => [k, omega, epsilon]
                .iter()
                .all(|v| v.is_finite() && *v > 0.0),
            Self::IntensityLength { intensity, length } => {
                intensity.is_finite()
                    && intensity > 0.0
                    && intensity <= 1.0
                    && length.is_finite()
                    && length > 0.0
            }
        };
        if valid {
            Ok(())
        } else {
            Err("Require k, omega, epsilon > 0; or 0 < intensity ≤ 1 and length > 0 m".into())
        }
    }
    pub fn values(&self, speed: f64) -> Result<(f64, f64, f64), String> {
        self.validate()?;
        if !speed.is_finite() || speed < 0.0 {
            return Err("Invalid incoming speed".into());
        }
        let values = match *self {
            Self::Direct { k, omega, epsilon } => (k, omega, epsilon),
            Self::IntensityLength { intensity, length } => {
                let k = 1.5 * (intensity * speed).powi(2);
                let omega = k.sqrt() / (0.09_f64.powf(0.25) * length);
                (k, omega, 0.09_f64.powf(0.75) * k.powf(1.5) / length)
            }
        };
        if [values.0, values.1, values.2]
            .iter()
            .any(|v| !v.is_finite() || *v <= 0.0)
        {
            return Err("Incoming turbulence is zero or non-finite; specify direct positive values for zero-speed inflow".into());
        }
        Ok(values)
    }
}
