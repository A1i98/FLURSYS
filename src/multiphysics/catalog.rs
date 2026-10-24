//! Resolve optional catalogue selections into frozen, portable numerical input.
use super::*;
use crate::database::Database;
use std::collections::BTreeMap;
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Selection {
    pub database: Option<std::path::PathBuf>,
    pub temperature: f64,
    pub pressure: f64,
    pub first: Option<String>,
    pub second: Option<String>,
    #[serde(default)]
    pub solid_regions: BTreeMap<usize, String>,
    #[serde(default)]
    pub face_contacts: BTreeMap<usize, String>,
    #[serde(default)]
    pub radiation_patches: BTreeMap<String, String>,
}
impl Options {
    pub fn resolve_catalog(&self) -> Result<(Self, serde_json::Value), String> {
        let mut resolved = self.clone();
        let mut history = Vec::new();
        if let Some(selection) = &self.catalog {
            if !selection.temperature.is_finite()
                || selection.temperature <= 0.0
                || !selection.pressure.is_finite()
                || selection.pressure <= 0.0
            {
                return Err(
                    "Catalogue evaluation requires finite positive Kelvin and pascals".into(),
                );
            }
            let db = if let Some(path) = &selection.database {
                Database::open(path)
            } else {
                Database::open_default()
            }
            .map_err(|e| e.to_string())?;
            for (id, target, label) in [
                (&selection.first, &mut resolved.first, "first"),
                (&selection.second, &mut resolved.second, "second"),
            ] {
                if let Some(id) = id {
                    let snapshot = db
                        .evaluate(id, selection.temperature, selection.pressure)
                        .map_err(|e| e.to_string())?;
                    *target = Properties::from_snapshot(&snapshot)?;
                    history.push(serde_json::json!({"target":label,"snapshot":snapshot,"provenance":db.provenance("materials",id).map_err(|e|e.to_string())?}));
                }
            }
            for (&index, id) in &selection.solid_regions {
                let region = resolved
                    .regions
                    .get_mut(index)
                    .ok_or("Catalogue solid selection names a missing region index")?;
                let material = db
                    .solid(id, selection.temperature, selection.pressure)
                    .map_err(|e| e.to_string())?;
                region.solid = Some(material);
                history.push(serde_json::json!({"region":index,"properties":material,"provenance":db.provenance("materials",id).map_err(|e|e.to_string())?}));
            }
            for (&face, id) in &selection.face_contacts {
                let resistance = db
                    .contact(id, selection.temperature)
                    .map_err(|e| e.to_string())?;
                resolved.energy.contact_resistance.insert(face, resistance);
                history.push(serde_json::json!({"face":face,"resistance_m2_k_w":resistance,"provenance":db.provenance("thermal_contacts",id).map_err(|e|e.to_string())?}));
            }
            for (patch, id) in &selection.radiation_patches {
                let e = db
                    .emissivity(id, selection.temperature)
                    .map_err(|e| e.to_string())?;
                match resolved.energy.boundaries.get_mut(patch) {
                    Some(energy::Boundary::Radiation{emissivity,..})=>*emissivity=e,
                    _=>return Err("Catalogue radiation selection needs an existing Radiation boundary with explicit ambient temperature".into()),
                }
                history.push(serde_json::json!({"patch":patch,"emissivity":e,"provenance":db.provenance("surface_radiation",id).map_err(|e|e.to_string())?}));
            }
        }
        if self.catalog.is_some() {
            resolved
                .provenance
                .push(serde_json::json!({"evaluation":self.catalog,"applied":history}));
        }
        resolved.catalog = None;
        Ok((
            resolved,
            serde_json::json!({"evaluation":self.catalog,"applied":history,"note":"Properties evaluated once. No temperature-dependent material update, tensor conduction or momentum feedback is enabled by catalogue data."}),
        ))
    }
}
