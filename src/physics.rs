//! Executable physics controls for the active incompressible solver.
//!
//! Models represented here are deliberately limited to models that the solver
//! actually assembles.  Unsupported multiphase, reacting, turbulent and
//! compressible models are not exposed as runnable settings.

use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum EnergyModel {
    #[default]
    Off,
    ConstantProperties,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum ThermalBoundaryCondition {
    Adiabatic,
    FixedTemperature { temperature: f64 },
}

impl Default for ThermalBoundaryCondition {
    fn default() -> Self {
        Self::Adiabatic
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ThermalSettings {
    pub model: EnergyModel,
    /// Uniform initial temperature in kelvin.
    pub initial_temperature: f64,
    /// Thermal diffusivity alpha = k / (rho cp), in m^2/s.
    pub thermal_diffusivity: f64,
    /// Volumetric heat source expressed as a temperature rate, K/s.
    pub source_temperature_rate: f64,
    pub left: ThermalBoundaryCondition,
    pub right: ThermalBoundaryCondition,
    pub bottom: ThermalBoundaryCondition,
    pub top: ThermalBoundaryCondition,
}

impl Default for ThermalSettings {
    fn default() -> Self {
        Self {
            model: EnergyModel::Off,
            initial_temperature: 293.15,
            thermal_diffusivity: 2.2e-5,
            source_temperature_rate: 0.0,
            left: ThermalBoundaryCondition::Adiabatic,
            right: ThermalBoundaryCondition::Adiabatic,
            bottom: ThermalBoundaryCondition::Adiabatic,
            top: ThermalBoundaryCondition::Adiabatic,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum BuoyancyModel {
    #[default]
    Off,
    /// Boussinesq body force: g * beta * (T - T_ref).
    Boussinesq {
        reference_temperature: f64,
        thermal_expansion: f64,
        gravity_x: f64,
        gravity_y: f64,
    },
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PhysicsSettings {
    pub thermal: ThermalSettings,
    pub buoyancy: BuoyancyModel,
}

impl PhysicsSettings {
    pub fn validate(&self, dt: f64, dx: f64, dy: f64) -> Result<(), String> {
        if self.thermal.model == EnergyModel::ConstantProperties {
            let t = &self.thermal;
            if !t.initial_temperature.is_finite()
                || !t.thermal_diffusivity.is_finite()
                || t.thermal_diffusivity <= 0.0
                || !t.source_temperature_rate.is_finite()
            {
                return Err("thermal initial temperature, diffusivity, and source must be finite; diffusivity must be positive".to_string());
            }
            for boundary in [t.left, t.right, t.bottom, t.top] {
                if let ThermalBoundaryCondition::FixedTemperature { temperature } = boundary {
                    if !temperature.is_finite() {
                        return Err(
                            "fixed thermal boundary temperatures must be finite".to_string()
                        );
                    }
                }
            }
            let fourier_sum =
                t.thermal_diffusivity * dt * (dx.recip().powi(2) + dy.recip().powi(2));
            if fourier_sum > 0.5 {
                return Err(format!(
                    "explicit thermal diffusion is unstable: alpha*dt*(1/dx^2 + 1/dy^2)={fourier_sum:.3e} exceeds 0.5"
                ));
            }
        }
        if let BuoyancyModel::Boussinesq {
            reference_temperature,
            thermal_expansion,
            gravity_x,
            gravity_y,
        } = self.buoyancy
        {
            if self.thermal.model != EnergyModel::ConstantProperties {
                return Err(
                    "Boussinesq buoyancy requires the constant-property energy equation"
                        .to_string(),
                );
            }
            if !reference_temperature.is_finite()
                || !thermal_expansion.is_finite()
                || thermal_expansion < 0.0
                || !gravity_x.is_finite()
                || !gravity_y.is_finite()
            {
                return Err("Boussinesq reference temperature, expansion coefficient, and gravity must be finite; expansion must be non-negative".to_string());
            }
        }
        Ok(())
    }
}
