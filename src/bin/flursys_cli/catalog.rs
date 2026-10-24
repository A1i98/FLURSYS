//! Explicit catalogue import/application. Never changes properties during a solve.
use super::{emit, parse};
use flursys::{
    database::{Database, Pack},
    workbench::{load_workspace, save_workspace, WorkbenchSession},
};
use std::{collections::BTreeMap, path::Path};
fn value(opts: &BTreeMap<String, String>, key: &str) -> Result<f64, String> {
    let v = opts
        .get(key)
        .ok_or_else(|| format!("Missing --{key}"))?
        .parse::<f64>()
        .map_err(|_| format!("Invalid --{key}"))?;
    if !v.is_finite() {
        return Err(format!("--{key} must be finite"));
    }
    Ok(v)
}
pub fn run(args: &[String]) -> Result<(), String> {
    let action = args.get(1).map(String::as_str).unwrap_or("status");
    let operand = match action {
        "import" | "inspect" | "reference" | "evaluate" | "apply-material" | "apply-profile"
        | "apply-radiation" => Some(args.get(2).ok_or("Missing catalogue operand")?),
        _ => None,
    };
    let flags = if operand.is_some() {
        &args[3..]
    } else {
        args.get(2..).unwrap_or(&[])
    };
    let allowed:&[&str]=match action{
  "status"|"materials"=>&["db","json"],"import"|"inspect"|"reference"=>&["db","json"],"backup"=>&["db","output","json"],
  "evaluate"=>&["db","temperature","pressure","json"],
  "apply-material"=>&["db","id","temperature","pressure","json"],
  "apply-profile"=>&["db","id","patch","json"],
  "apply-radiation"=>&["db","id","patch","temperature","ambient","json"],
  _=>return Err("Unknown catalogue action; use status, materials, import, inspect, reference, evaluate, apply-material, apply-profile, apply-radiation or backup".into()),
 };
    let opts = parse(flags, allowed)?;
    let path = opts
        .get("db")
        .map(std::path::PathBuf::from)
        .map(Ok)
        .unwrap_or_else(flursys::database::default_path)
        .map_err(|e| e.to_string())?;
    let mut db = Database::open(&path).map_err(|e| e.to_string())?;
    let result = (|| -> flursys::database::Result<serde_json::Value> {
        Ok(match action {
            "status" => serde_json::json!({"path":path,"schema_version":1,"counts":db.counts()?}),
            "materials" => serde_json::to_value(db.entries(true)?)?,
            "inspect" => serde_json::to_value(db.records(operand.unwrap())?)?,
            "reference" => db.reference(operand.unwrap())?,
            "import" => {
                let pack: Pack = serde_json::from_slice(&std::fs::read(operand.unwrap())?)?;
                db.import(&pack)?;
                serde_json::json!({"imported":true,"counts":db.counts()?})
            }
            "backup" => {
                let output = Path::new(opts.get("output").ok_or("Missing --output")?);
                db.backup(output)?;
                serde_json::json!({"backup":output})
            }
            "evaluate" => serde_json::to_value(db.evaluate(
                operand.unwrap(),
                value(&opts, "temperature")?,
                value(&opts, "pressure")?,
            )?)?,
            "apply-material" | "apply-profile" | "apply-radiation" => {
                let folder = Path::new(operand.unwrap());
                let mut project = load_workspace(folder)?;
                let id = opts.get("id").ok_or("Missing --id")?;
                match action {
                    "apply-material" => {
                        let m = db.evaluate(
                            id,
                            value(&opts, "temperature")?,
                            value(&opts, "pressure")?,
                        )?;
                        let mut session = WorkbenchSession::from_project(&project)?;
                        session.apply_catalog_material(m)?;
                        let draft = session.to_project(project.name.clone());
                        project.material = draft.material;
                        project.solver.thermal = draft.solver.thermal;
                        project.catalog_history = draft.catalog_history;
                    }
                    "apply-profile" => {
                        let (default_patch, profile) = db.profile(id)?;
                        let patch = opts.get("patch").unwrap_or(&default_patch);
                        let mut session = WorkbenchSession::from_project(&project)?;
                        if profile.quantity
                            == flursys::unstructured_incompressible::BoundaryQuantity::FlowRate
                        {
                            let expected = match session.boundary_assignment(patch) {
                                Some(flursys::IncompressibleBoundaryCondition::FlowRateInlet {
                                    mass: true,
                                    ..
                                }) => "kg/s",
                                Some(flursys::IncompressibleBoundaryCondition::FlowRateInlet {
                                    mass: false,
                                    ..
                                }) => "m3/s",
                                _ => {
                                    return Err(
                                        "Flow-rate profile requires an assigned flow-rate inlet"
                                            .into(),
                                    )
                                }
                            };
                            if db.stored_profile_unit(id)? != expected {
                                return Err("Stored flow-rate profile unit does not match mass/volume inlet units".into());
                            }
                        }
                        session.set_boundary_profile(patch, Some(profile.clone()))?;
                        project.solver.boundary_profiles = session
                            .to_project(project.name.clone())
                            .solver
                            .boundary_profiles;
                        project.catalog_history.push(serde_json::json!({"kind":"boundary-profile","patch":patch,"profile":profile,"provenance":db.provenance("boundary_profiles",id)?}));
                    }
                    _ => {
                        let patch = opts.get("patch").ok_or("Missing --patch")?;
                        if !project.boundaries.iter().any(|b| &b.selection == patch) {
                            return Err("Assign a physical boundary on this patch first".into());
                        }
                        let at = value(&opts, "temperature")?;
                        let ambient = value(&opts, "ambient")?;
                        let emissivity = db.emissivity(id, at)?;
                        project.solver.thermal.boundaries.insert(
                            patch.clone(),
                            flursys::thermal::Boundary::Radiation {
                                emissivity,
                                ambient_temperature: ambient,
                            },
                        );
                        let mut checked = project.solver.thermal.clone();
                        checked.enabled = true;
                        checked.validate()?;
                        project.catalog_history.push(serde_json::json!({"kind":"surface-radiation","patch":patch,"evaluation_temperature":at,"emissivity":emissivity,"ambient_temperature":ambient,"provenance":db.provenance("surface_radiation",id)?}));
                    }
                }
                save_workspace(folder, &project)?;
                serde_json::json!({"saved":folder,"project":project.name,"applied_id":id,"note":"Evaluated values copied into project; existing runs retained. Restart/initialization is an explicit separate action."})
            }
            _ => unreachable!(),
        })
    })();
    emit(
        result.map_err(|e| e.to_string())?,
        opts.contains_key("json"),
    );
    Ok(())
}
