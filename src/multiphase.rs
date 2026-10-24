//! Preparatory two-fluid constitutive data, not a VOF/interface-flow solver.
use crate::materials::{MaterialSnapshot, PhaseState};
use serde::{Deserialize, Serialize};
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TwoPhaseDefinition {
    pub first: MaterialSnapshot,
    pub second: MaterialSnapshot,
    pub interfacial_tension: f64,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum ConductivityMixing {
    Arithmetic,
    Harmonic,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct MixtureProperties {
    pub density: f64,
    pub dynamic_viscosity: f64,
    pub volumetric_heat_capacity: f64,
    pub specific_heat: f64,
    pub conductivity: f64,
}
impl TwoPhaseDefinition {
    pub fn mixture(
        &self,
        alpha: f64,
        conductivity: ConductivityMixing,
    ) -> Result<MixtureProperties, String> {
        self.first.validate()?;
        self.second.validate()?;
        if self.first.phase == PhaseState::Solid || self.second.phase == PhaseState::Solid {
            return Err("Prepared mixture is for two fluids; solid/fluid conjugate transfer is a separate model".into());
        }
        if !alpha.is_finite()
            || !(0.0..=1.0).contains(&alpha)
            || !self.interfacial_tension.is_finite()
            || self.interfacial_tension < 0.0
        {
            return Err(
                "Require volume fraction in [0,1] and nonnegative finite pair surface tension"
                    .into(),
            );
        }
        let a = &self.first;
        let b = &self.second;
        let blend = |x, y| alpha * x + (1.0 - alpha) * y;
        let density = blend(a.density, b.density);
        let rho_cp = blend(a.density * a.specific_heat, b.density * b.specific_heat);
        let k = match conductivity {
            ConductivityMixing::Arithmetic => blend(a.conductivity, b.conductivity),
            ConductivityMixing::Harmonic => 1.0 / blend(1.0 / a.conductivity, 1.0 / b.conductivity),
        };
        let m = MixtureProperties {
            density,
            dynamic_viscosity: blend(a.dynamic_viscosity, b.dynamic_viscosity),
            volumetric_heat_capacity: rho_cp,
            specific_heat: rho_cp / density,
            conductivity: k,
        };
        if [
            m.density,
            m.dynamic_viscosity,
            m.volumetric_heat_capacity,
            m.specific_heat,
            m.conductivity,
        ]
        .iter()
        .any(|v| !v.is_finite() || *v <= 0.0)
        {
            return Err("Non-finite mixture property".into());
        }
        Ok(m)
    }
}

impl TwoPhaseDefinition {
    /// Pair-specific catalogue data; never substitute a liquid/vapour coefficient
    /// for an arbitrary fluid pair. This does not enable a capillary flow solver.
    pub fn from_catalog(
        db: &crate::database::Database,
        pair_id: &str,
        temperature: f64,
        pressure: f64,
    ) -> Result<Self, String> {
        let (first, second) = db.pair_materials(pair_id).map_err(|e| e.to_string())?;
        Ok(Self {
            first: db
                .evaluate(&first, temperature, pressure)
                .map_err(|e| e.to_string())?,
            second: db
                .evaluate(&second, temperature, pressure)
                .map_err(|e| e.to_string())?,
            interfacial_tension: db
                .pair_property(
                    pair_id,
                    crate::database::PairQuantity::SurfaceTension,
                    temperature,
                )
                .map_err(|e| e.to_string())?,
        })
    }
}
