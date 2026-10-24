use crate::{materials::PhaseState, unstructured_incompressible::BoundaryProfile};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Source {
    pub id: String,
    pub citation: String,
    pub url: Option<String>,
    pub revision: Option<String>,
    pub license: Option<String>,
    pub uncertainty: Option<String>,
    pub note: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Material {
    pub id: String,
    pub name: String,
    pub phase: PhaseState,
    pub composition: String,
    pub source_id: String,
    pub revision: String,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Quantity {
    Density,
    DynamicViscosity,
    SpecificHeat,
    Conductivity,
    Expansion,
}
impl Quantity {
    pub fn key(self) -> &'static str {
        match self {
            Self::Density => "density",
            Self::DynamicViscosity => "dynamic-viscosity",
            Self::SpecificHeat => "specific-heat",
            Self::Conductivity => "conductivity",
            Self::Expansion => "expansion",
        }
    }
    pub fn unit(self) -> &'static str {
        match self {
            Self::Density => "kg/m3",
            Self::DynamicViscosity => "Pa.s",
            Self::SpecificHeat => "J/(kg.K)",
            Self::Conductivity => "W/(m.K)",
            Self::Expansion => "1/K",
        }
    }
    pub fn validate_value(self, v: f64) -> bool {
        v.is_finite() && (self == Self::Expansion || v > 0.0)
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sample {
    pub temperature: f64,
    pub pressure: f64,
    pub value: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "kebab-case", deny_unknown_fields)]
pub enum Model {
    Constant { value: f64 },
    Table { samples: Vec<Sample> },
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Property {
    pub id: String,
    pub material_id: String,
    pub quantity: Quantity,
    pub unit: String,
    pub temperature_range: [f64; 2],
    pub pressure_range: [f64; 2],
    pub model: Model,
    pub source_id: String,
    pub revision: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Solid {
    pub material_id: String,
    pub conductivity_tensor: Option<[[f64; 3]; 3]>,
    pub note: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FluidPair {
    pub id: String,
    pub first_material: String,
    pub second_material: String,
    pub source_id: String,
    pub revision: String,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum PairQuantity {
    SurfaceTension,
    ContactAngle,
}
impl PairQuantity {
    pub fn key(self) -> &'static str {
        match self {
            Self::SurfaceTension => "surface-tension",
            Self::ContactAngle => "contact-angle",
        }
    }
    pub fn unit(self) -> &'static str {
        match self {
            Self::SurfaceTension => "N/m",
            Self::ContactAngle => "rad",
        }
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PairProperty {
    pub id: String,
    pub pair_id: String,
    pub quantity: PairQuantity,
    pub unit: String,
    pub temperature_range: [f64; 2],
    pub value: f64,
    pub source_id: String,
    pub revision: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Contact {
    pub id: String,
    pub first_material: String,
    pub second_material: String,
    pub resistance_m2_k_w: f64,
    pub temperature_range: [f64; 2],
    pub source_id: String,
    pub revision: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Radiation {
    pub id: String,
    pub material_id: String,
    pub emissivity: f64,
    pub temperature_range: [f64; 2],
    pub source_id: String,
    pub revision: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Profile {
    pub id: String,
    pub patch: String,
    pub unit: String,
    pub profile: BoundaryProfile,
    pub source_id: String,
    pub revision: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReferenceCase {
    pub id: String,
    pub name: String,
    pub status: String,
    pub descriptor: serde_json::Value,
    pub source_id: String,
    pub revision: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReferenceValue {
    pub id: String,
    pub case_id: String,
    pub quantity: String,
    pub unit: String,
    pub data: Vec<[f64; 2]>,
    pub source_id: String,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Pack {
    pub sources: Vec<Source>,
    pub materials: Vec<Material>,
    pub properties: Vec<Property>,
    pub solids: Vec<Solid>,
    pub fluid_pairs: Vec<FluidPair>,
    pub pair_properties: Vec<PairProperty>,
    pub thermal_contacts: Vec<Contact>,
    pub surface_radiation: Vec<Radiation>,
    pub boundary_profiles: Vec<Profile>,
    pub reference_cases: Vec<ReferenceCase>,
    pub reference_values: Vec<ReferenceValue>,
}
#[derive(Clone, Debug, Serialize)]
pub struct Entry {
    pub id: String,
    pub name: String,
    pub phase: PhaseState,
    pub source_id: String,
    pub revision: String,
}
pub type Counts = BTreeMap<String, u64>;
