//! Incoming RANS data; outgoing faces retain zero-gradient conditions.
use serde::{Deserialize, Serialize};
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "kebab-case")]
pub enum TurbulenceInlet {
    ModifiedViscosity {
        nu_tilda: f64,
    },
    Direct {
        k: f64,
        omega: f64,
        epsilon: f64,
    },
    IntensityViscosityRatio {
        intensity: f64,
        viscosity_ratio: f64,
    },
    IntensityDiameter {
        intensity: f64,
        hydraulic_diameter: f64,
        length_ratio: f64,
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
            Self::IntensityViscosityRatio {
                intensity,
                viscosity_ratio,
            } => {
                intensity.is_finite()
                    && intensity > 0.0
                    && intensity <= 1.0
                    && viscosity_ratio.is_finite()
                    && viscosity_ratio > 0.0
            }
            Self::IntensityDiameter {
                intensity,
                hydraulic_diameter,
                length_ratio,
            } => {
                intensity.is_finite()
                    && intensity > 0.0
                    && intensity <= 1.0
                    && hydraulic_diameter.is_finite()
                    && hydraulic_diameter > 0.0
                    && length_ratio.is_finite()
                    && length_ratio > 0.0
                    && length_ratio <= 1.0
            }
            Self::ModifiedViscosity { nu_tilda } => nu_tilda.is_finite() && nu_tilda >= 0.0,
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
            Err("Require k, omega, epsilon > 0; or 0 < intensity ≤ 1 and length/diameter/viscosity ratio > 0 (length ratio ≤ 1)".into())
        }
    }
    pub fn values_with_viscosity(&self, speed: f64, nu: f64) -> Result<(f64, f64, f64), String> {
        if let Self::IntensityViscosityRatio {
            intensity,
            viscosity_ratio,
        } = *self
        {
            self.validate()?;
            if !speed.is_finite() || speed <= 0.0 || !nu.is_finite() || nu <= 0.0 {
                return Err("Intensity/viscosity ratio requires positive incoming speed and molecular viscosity".into());
            }
            let k = 1.5 * (intensity * speed).powi(2);
            let nut = viscosity_ratio * nu;
            return Self::Direct {
                k,
                omega: k / nut,
                epsilon: 0.09 * k * k / nut,
            }
            .values(speed);
        }
        self.values(speed)
    }
    pub fn values(&self, speed: f64) -> Result<(f64, f64, f64), String> {
        self.validate()?;
        if !speed.is_finite() || speed < 0.0 {
            return Err("Invalid incoming speed".into());
        }
        let values = match *self {
            Self::IntensityViscosityRatio { .. } => {
                return Err("Molecular viscosity is required for this inlet specification".into())
            }
            Self::ModifiedViscosity { .. } => {
                return Err(
                    "Modified viscosity belongs to SA, not k/omega/epsilon conversion".into(),
                )
            }
            Self::Direct { k, omega, epsilon } => (k, omega, epsilon),
            Self::IntensityDiameter {
                intensity,
                hydraulic_diameter,
                length_ratio,
            } => {
                return Self::IntensityLength {
                    intensity,
                    length: hydraulic_diameter * length_ratio,
                }
                .values(speed);
            }
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
