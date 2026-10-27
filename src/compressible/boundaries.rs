use super::*;
use crate::scripting::{Binding, Compiled, Quantity};
use std::sync::Arc;
pub(super) type Programs = Vec<Option<Arc<Vec<(Binding, Arc<Compiled>)>>>>;
pub(super) fn compile(mesh: &UnstructuredMesh, settings: &Settings) -> Result<Programs, String> {
    let mut programs = vec![None; mesh.face_count()];
    for patch in mesh.boundary_patches() {
        if let Some(bindings) = settings.scripting.bindings.get(&patch.name) {
            let compiled = Arc::new(
                bindings
                    .iter()
                    .map(|b| Ok((b.clone(), settings.scripting.compiled(b)?)))
                    .collect::<Result<Vec<_>, String>>()?,
            );
            for &face in &patch.face_indices {
                programs[face] = Some(compiled.clone());
            }
        }
    }
    Ok(programs)
}
impl Boundary {
    pub(super) fn check_script(self, quantity: Quantity) -> Result<(), String> {
        let allowed = match quantity {
            Quantity::Velocity => matches!(
                self,
                Self::FarField { .. }
                    | Self::CharacteristicFarField { .. }
                    | Self::SupersonicInlet { .. }
                    | Self::PressureOutletBackflow { .. }
                    | Self::MovingWall { .. }
            ),
            Quantity::Pressure => matches!(
                self,
                Self::FarField { .. }
                    | Self::CharacteristicFarField { .. }
                    | Self::SupersonicInlet { .. }
                    | Self::PressureOutlet { .. }
                    | Self::PressureOutletBackflow { .. }
                    | Self::TotalInlet { .. }
            ),
            Quantity::Temperature => matches!(
                self,
                Self::FarField { .. }
                    | Self::CharacteristicFarField { .. }
                    | Self::SupersonicInlet { .. }
                    | Self::PressureOutletBackflow { .. }
                    | Self::NoSlipWall { .. }
                    | Self::MovingWall { .. }
                    | Self::TotalInlet { .. }
            ),
        };
        if allowed {
            Ok(())
        } else {
            Err(format!(
                "Script quantity {quantity:?} is incompatible with {self:?}"
            ))
        }
    }
    pub(super) fn scripted(
        self,
        program: &[(Binding, Arc<Compiled>)],
        point: Vec3,
        time: f64,
        gas: IdealGas,
    ) -> Result<Self, String> {
        let (mut velocity, mut pressure, mut temperature) = (None, None, None);
        for (binding, compiled) in program {
            let value = compiled.value(binding, point, Some(time))?;
            match binding.quantity {
                Quantity::Velocity => velocity = Some(value),
                Quantity::Pressure => pressure = Some(value.x),
                Quantity::Temperature => temperature = Some(value.x),
            }
        }
        if pressure.is_some_and(|p| p <= 0.) || temperature.is_some_and(|t| t <= 0.) {
            return Err("Scripted absolute pressure and temperature must be positive".into());
        }
        let state = |mut p: Primitive| {
            if let Some(v) = velocity {
                p.velocity = v;
            }
            if let Some(v) = pressure {
                p.pressure = v;
            }
            if let Some(t) = temperature {
                p.density = p.pressure / (gas.gas_constant * t);
            }
            p
        };
        Ok(match self {
            Self::FarField { state: p } => Self::FarField { state: state(p) },
            Self::CharacteristicFarField { state: p } => {
                Self::CharacteristicFarField { state: state(p) }
            }
            Self::SupersonicInlet { state: p } => Self::SupersonicInlet { state: state(p) },
            Self::PressureOutlet { pressure: p } => Self::PressureOutlet {
                pressure: pressure.unwrap_or(p),
            },
            Self::PressureOutletBackflow {
                pressure: p,
                state: q,
            } => Self::PressureOutletBackflow {
                pressure: pressure.unwrap_or(p),
                state: state(q),
            },
            Self::TotalInlet {
                pressure: p,
                temperature: t,
                direction,
            } => Self::TotalInlet {
                pressure: pressure.unwrap_or(p),
                temperature: temperature.unwrap_or(t),
                direction,
            },
            Self::NoSlipWall { temperature: t } => Self::NoSlipWall {
                temperature: temperature.or(t),
            },
            Self::MovingWall {
                velocity: v,
                temperature: t,
            } => Self::MovingWall {
                velocity: velocity.unwrap_or(v),
                temperature: temperature.or(t),
            },
            other => other,
        })
    }
}
