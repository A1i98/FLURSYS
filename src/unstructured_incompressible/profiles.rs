//! Physical-time boundary tables. Linear interpolation, explicit endpoint hold.
use super::*;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
pub type BoundaryProfiles = BTreeMap<String, BoundaryProfile>;
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum BoundaryQuantity {
    Velocity,
    FlowRate,
    Pressure,
    Temperature,
    HeatFlux,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TimeSample {
    pub time: f64,
    pub value: Vec3,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BoundaryProfile {
    pub quantity: BoundaryQuantity,
    pub samples: Vec<TimeSample>,
}
impl BoundaryQuantity {
    pub fn is_thermal(self) -> bool {
        matches!(self, Self::Temperature | Self::HeatFlux)
    }
}
impl BoundaryProfile {
    pub fn validate(&self) -> Result<(), String> {
        if !(2..=10000).contains(&self.samples.len()) {
            return Err("Boundary table needs 2..10000 rows".into());
        }
        let mut previous = -1.0;
        for s in &self.samples {
            if !s.time.is_finite()
                || s.time < 0.0
                || s.time <= previous
                || ![s.value.x, s.value.y, s.value.z]
                    .iter()
                    .all(|v| v.is_finite())
            {
                return Err("Table times must be finite, nonnegative and strictly increasing; values must be finite".into());
            }
            if self.quantity != BoundaryQuantity::Velocity && (s.value.y != 0.0 || s.value.z != 0.0)
            {
                return Err("Scalar boundary table uses only x; y and z must be zero".into());
            }
            if self.quantity == BoundaryQuantity::FlowRate && s.value.x < 0.0 {
                return Err("Incoming flow-rate table must be nonnegative".into());
            }
            if self.quantity == BoundaryQuantity::Temperature && s.value.x <= 0.0 {
                return Err("Temperature table requires positive Kelvin values".into());
            }
            previous = s.time;
        }
        Ok(())
    }
    pub fn value(&self, time: f64) -> Result<Vec3, String> {
        self.validate()?;
        if !time.is_finite() || time < 0.0 {
            return Err("Invalid physical boundary time".into());
        }
        let j = self.samples.partition_point(|s| s.time <= time);
        let value = if j == 0 {
            self.samples[0].value
        } else if j == self.samples.len() {
            self.samples[j - 1].value
        } else {
            let a = &self.samples[j - 1];
            let b = &self.samples[j];
            let f = (time - a.time) / (b.time - a.time);
            a.value * (1.0 - f) + b.value * f
        };
        if ![value.x, value.y, value.z].iter().all(|v| v.is_finite()) {
            return Err("Boundary interpolation overflow".into());
        }
        Ok(value)
    }
    pub fn apply(
        &self,
        bc: IncompressibleBoundaryCondition,
        time: f64,
    ) -> Result<IncompressibleBoundaryCondition, String> {
        use IncompressibleBoundaryCondition as B;
        let v = self.value(time)?;
        Ok(match (self.quantity, bc) {
            (BoundaryQuantity::Velocity, B::VelocityInlet { .. }) => {
                B::VelocityInlet { velocity: v }
            }
            (
                BoundaryQuantity::Velocity,
                B::ParabolicInlet {
                    axis,
                    center,
                    half_width,
                    ..
                },
            ) => B::ParabolicInlet {
                velocity: v,
                axis,
                center,
                half_width,
            },
            (BoundaryQuantity::Velocity, B::FarField { pressure, .. }) => B::FarField {
                velocity: v,
                pressure,
            },
            (BoundaryQuantity::Velocity, B::PressureOutletBackflow { pressure, .. }) => {
                B::PressureOutletBackflow {
                    velocity: v,
                    pressure,
                }
            }
            (BoundaryQuantity::FlowRate, B::FlowRateInlet { mass, .. }) => {
                B::FlowRateInlet { rate: v.x, mass }
            }
            (BoundaryQuantity::Pressure, B::PressureOutlet { .. }) => {
                B::PressureOutlet { pressure: v.x }
            }
            (BoundaryQuantity::Pressure, B::TotalPressure { .. }) => {
                B::TotalPressure { pressure: v.x }
            }
            (BoundaryQuantity::Pressure, B::FarField { velocity, .. }) => B::FarField {
                velocity,
                pressure: v.x,
            },
            (BoundaryQuantity::Pressure, B::PressureOutletBackflow { velocity, .. }) => {
                B::PressureOutletBackflow {
                    velocity,
                    pressure: v.x,
                }
            }
            _ => return Err("Time-table quantity does not match this physical boundary".into()),
        })
    }
    pub fn apply_thermal(
        &self,
        bc: crate::thermal::Boundary,
        time: f64,
    ) -> Result<crate::thermal::Boundary, String> {
        use crate::thermal::Boundary as B;
        let v = self.value(time)?.x;
        Ok(match (self.quantity,bc){
            (BoundaryQuantity::Temperature,B::FixedTemperature{..})=>B::FixedTemperature{temperature:v},
            (BoundaryQuantity::Temperature,B::InletOutlet{..})=>B::InletOutlet{incoming_temperature:v},
            (BoundaryQuantity::HeatFlux,B::HeatFlux{..})=>B::HeatFlux{inward_flux:v},
            _=>return Err("Thermal time table must match Fixed temperature, Inlet/outlet temperature or Heat flux condition".into()),
        })
    }
    /// CSV rows: t,Ux,Uy,Uz for velocity; t,value for scalar quantities.
    pub fn from_csv(quantity: BoundaryQuantity, text: &str) -> Result<Self, String> {
        let mut samples = Vec::new();
        for (line, row) in text.lines().enumerate() {
            let row = row.trim();
            if row.is_empty() || row.starts_with('#') {
                continue;
            }
            let values = row
                .split(',')
                .map(|s| s.trim().parse::<f64>())
                .collect::<Result<Vec<_>, _>>()
                .map_err(|_| format!("Invalid number on table line {}", line + 1))?;
            let columns = if quantity == BoundaryQuantity::Velocity {
                4
            } else {
                2
            };
            if values.len() != columns {
                return Err(format!(
                    "Table line {} requires {columns} comma-separated numbers",
                    line + 1
                ));
            }
            samples.push(TimeSample {
                time: values[0],
                value: if columns == 4 {
                    Vec3::new(values[1], values[2], values[3])
                } else {
                    Vec3::new(values[1], 0.0, 0.0)
                },
            });
            if samples.len() > 10000 {
                return Err("Boundary table exceeds 10000 rows".into());
            }
        }
        let profile = Self { quantity, samples };
        profile.validate()?;
        Ok(profile)
    }
    pub fn csv(&self) -> String {
        self.samples
            .iter()
            .map(|s| {
                if self.quantity == BoundaryQuantity::Velocity {
                    format!("{},{},{},{}", s.time, s.value.x, s.value.y, s.value.z)
                } else {
                    format!("{},{}", s.time, s.value.x)
                }
            })
            .collect::<Vec<_>>()
            .join("\n")
    }
}
pub(super) fn apply(
    case: &mut IncompressibleCase,
    time: f64,
) -> Result<(), IncompressibleSolveError> {
    case.solver.scripting.evaluation_time = Some(time);
    for (name, p) in &case.solver.boundary_profiles {
        if p.quantity.is_thermal() {
            if !case.solver.thermal.enabled {
                return Err(SimpleError::PhysicalBoundary(
                    "Thermal time tables require enabled temperature transport".into(),
                )
                .into());
            }
            let bc = case
                .solver
                .thermal
                .boundaries
                .get_mut(name)
                .ok_or_else(|| {
                    SimpleError::PhysicalBoundary(format!(
                        "Missing thermal boundary for table {name}"
                    ))
                })?;
            *bc = p
                .apply_thermal(*bc, time)
                .map_err(SimpleError::PhysicalBoundary)?;
            continue;
        }

        let (_, bc) = case
            .boundaries
            .iter_mut()
            .find(|(n, _)| n == name)
            .ok_or_else(|| {
                SimpleError::PhysicalBoundary(format!("Unknown time-table patch {name}"))
            })?;
        *bc = p.apply(*bc, time).map_err(SimpleError::PhysicalBoundary)?;
    }
    Ok(())
}
