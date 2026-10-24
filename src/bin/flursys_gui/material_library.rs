//! Human-readable catalogue browsing, property curves and immutable revisions.
use super::*;
use flursys::database::{self, Database, Quantity};
fn number(s: &SharedString, label: &str) -> Result<f64, String> {
    let v = s
        .trim()
        .parse::<f64>()
        .map_err(|_| format!("Invalid {label}"))?;
    if !v.is_finite() {
        return Err(format!("{label} must be finite"));
    }
    Ok(v)
}
fn id(ui: &MainWindow) -> Result<String, String> {
    ui.get_library_ids()
        .row_data(ui.get_library_selected().max(0) as usize)
        .map(|s| s.to_string())
        .ok_or_else(|| "Select a material".into())
}
fn choose(ui: &MainWindow) -> Result<(), String> {
    let id = id(ui)?;
    let db = Database::open_default().map_err(|e| e.to_string())?;
    let data = db.provenance("materials", &id).map_err(|e| e.to_string())?;
    let record = &data["record"];
    let source = &data["source"];
    let mut details = format!(
        "{}\nID: {} · phase: {} · revision: {}\n\n",
        record["name"].as_str().unwrap_or("Material"),
        id,
        record["phase"].as_str().unwrap_or(""),
        record["revision"].as_str().unwrap_or("")
    );
    let mut tmin: f64 = 0.0;
    let mut tmax = f64::INFINITY;
    let mut pmin: f64 = 0.0;
    let mut pmax = f64::INFINITY;
    for p in db
        .records("material_properties")
        .map_err(|e| e.to_string())?
        .into_iter()
        .filter(|v| v["material_id"].as_str() == Some(&id))
    {
        details.push_str(&format!(
            "{} · {} · {}\nT [{}, {}] K · p [{}, {}] Pa\nSource ID: {} · property revision: {}\n\n",
            p["quantity"].as_str().unwrap_or(""),
            p["unit"].as_str().unwrap_or(""),
            p["model"].as_str().unwrap_or(""),
            p["temperature_min"],
            p["temperature_max"],
            p["pressure_min"],
            p["pressure_max"],
            p["source_id"].as_str().unwrap_or(""),
            p["revision"].as_str().unwrap_or("")
        ));
        tmin = tmin.max(p["temperature_min"].as_f64().unwrap_or(0.0));
        tmax = tmax.min(p["temperature_max"].as_f64().unwrap_or(f64::INFINITY));
        pmin = pmin.max(p["pressure_min"].as_f64().unwrap_or(0.0));
        pmax = pmax.min(p["pressure_max"].as_f64().unwrap_or(f64::INFINITY));
    }
    for (key, label) in [
        ("citation", "Citation"),
        ("url", "URL"),
        ("revision", "Source revision"),
        ("license", "Licence"),
        ("uncertainty", "Uncertainty"),
        ("note", "Notes"),
    ] {
        details.push_str(&format!(
            "{label}: {}\n",
            source[key].as_str().unwrap_or("Unknown / not supplied")
        ));
    }
    ui.set_library_details(details.into());
    if tmin > 0.0 && tmax.is_finite() && tmax >= tmin {
        ui.set_library_tmin(tmin.to_string().into());
        ui.set_library_tmax(tmax.to_string().into());
    }
    if pmin > 0.0 && pmax.is_finite() && pmax >= pmin {
        ui.set_library_pressure(if pmin <= 101325.0 && pmax >= 101325.0 {
            "101325".into()
        } else {
            ((pmin + pmax) * 0.5).to_string().into()
        });
    }
    ui.set_library_curve(slint::Image::default());
    Ok(())
}
fn refresh(ui: &MainWindow) -> Result<(), String> {
    let selected = id(ui).ok();
    let filter = ui.get_library_filter().to_lowercase();
    let entries = Database::open_default()
        .and_then(|db| db.entries(true))
        .map_err(|e| e.to_string())?
        .into_iter()
        .filter(|e| {
            e.name.to_lowercase().contains(&filter) || e.id.to_lowercase().contains(&filter)
        })
        .collect::<Vec<_>>();
    ui.set_library_entries(ModelRc::from(Rc::new(VecModel::from(
        entries
            .iter()
            .map(|e| SharedString::from(format!("{} · {:?}", e.name, e.phase)))
            .collect::<Vec<_>>(),
    ))));
    ui.set_library_ids(ModelRc::from(Rc::new(VecModel::from(
        entries
            .iter()
            .map(|e| SharedString::from(e.id.as_str()))
            .collect::<Vec<_>>(),
    ))));
    ui.set_library_selected(
        selected
            .and_then(|id| entries.iter().position(|e| e.id == id))
            .unwrap_or(0) as i32,
    );
    if entries.is_empty() {
        ui.set_library_details("No matching materials.".into());
    } else {
        choose(ui)?;
    }
    Ok(())
}
fn feedback(ui: &MainWindow, result: Result<(), String>) {
    if let Err(e) = result {
        ui.set_library_feedback(e.into());
    }
}
fn quantity(i: i32) -> Quantity {
    match i {
        1 => Quantity::DynamicViscosity,
        2 => Quantity::SpecificHeat,
        3 => Quantity::Conductivity,
        4 => Quantity::Expansion,
        _ => Quantity::Density,
    }
}
fn escape(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}
fn curve_svg(points: &[(f64, f64)], label: &str, unit: &str) -> String {
    let (t0, t1) = (points.first().unwrap().0, points.last().unwrap().0);
    let lo = points.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
    let hi = points.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
    let path = points
        .iter()
        .map(|(t, v)| {
            format!(
                "{:.2},{:.2}",
                70.0 + if t1 > t0 {
                    (t - t0) / (t1 - t0) * 580.0
                } else {
                    290.0
                },
                260.0
                    - if hi > lo {
                        (v - lo) / (hi - lo) * 210.0
                    } else {
                        105.0
                    }
            )
        })
        .collect::<Vec<_>>()
        .join(" ");
    format!(
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="720" height="330" viewBox="0 0 720 330"><rect width="720" height="330" fill="#20252d"/><g fill="#d3dae4" font-family="sans-serif" font-size="12"><text x="70" y="25">{} ({})</text><text x="12" y="56">{hi:.3e}</text><text x="12" y="260">{lo:.3e}</text><text x="70" y="285">{t0:.5}</text><text x="600" y="285">{t1:.5}</text><text x="290" y="315">Temperature (K)</text></g><path d="M70 45V260H650" fill="none" stroke="#8091a5"/><polyline points="{path}" fill="none" stroke="#55cdf5" stroke-width="2"/></svg>"##,
        escape(label),
        escape(unit)
    )
}
pub(super) fn bind(ui: &MainWindow) {
    let weak = ui.as_weak();
    ui.on_open_library(move || {
        if let Some(ui) = weak.upgrade() {
            if ui.get_library_draft().revision.is_empty() {
                ui.set_library_draft(LibraryMaterialDraft {
                    tmin: "293.15".into(),
                    tmax: "293.15".into(),
                    pmin: "101325".into(),
                    pmax: "101325".into(),
                    revision: "1".into(),
                    ..Default::default()
                });
            }
            feedback(&ui, refresh(&ui));
            ui.set_library_open(true);
        }
    });
    let weak = ui.as_weak();
    ui.on_refresh_library(move || {
        if let Some(ui) = weak.upgrade() {
            feedback(&ui, refresh(&ui));
        }
    });
    let weak = ui.as_weak();
    ui.on_select_library(move || {
        if let Some(ui) = weak.upgrade() {
            feedback(&ui, choose(&ui));
        }
    });
    let weak = ui.as_weak();
    ui.on_evaluate_library(move||{let Some(ui)=weak.upgrade()else{return};if ui.get_library_busy(){return;}
  let input=(||->Result<_,String>{let id=id(&ui)?;let t0=number(&ui.get_library_tmin(),"minimum temperature")?;let t1=number(&ui.get_library_tmax(),"maximum temperature")?;let pressure=number(&ui.get_library_pressure(),"pressure")?;if t0<=0.0||t1<t0||pressure<=0.0{return Err("Require positive T/p and Tmin <= Tmax".into());}Ok((id,t0,t1,pressure,quantity(ui.get_library_quantity())))})();
  let(id,t0,t1,p,q)=match input{Ok(v)=>v,Err(e)=>{ui.set_library_feedback(e.into());return}};ui.set_library_busy(true);let weak=ui.as_weak();
  std::thread::spawn(move||{let result=(||->Result<_,String>{let db=Database::open_default().map_err(|e|e.to_string())?;let points=(0..64).map(|i|{let t=t0+(t1-t0)*i as f64/63.0;db.property(&id,q,t,p).map(|v|(t,v)).map_err(|e|e.to_string())}).collect::<Result<Vec<_>,_>>()?;Ok(curve_svg(&points,q.key(),q.unit()))})();let _=weak.upgrade_in_event_loop(move|ui|{ui.set_library_busy(false);match result{Ok(svg)=>match slint::Image::load_from_svg_data(svg.as_bytes()){Ok(image)=>{ui.set_library_curve(image);ui.set_library_feedback("Property model evaluated at 64 temperature points; no extrapolation.".into());},Err(e)=>ui.set_library_feedback(e.to_string().into())},Err(e)=>ui.set_library_feedback(e.into())}});});
 });
    let weak = ui.as_weak();
    ui.on_copy_library_material(move||{let Some(ui)=weak.upgrade()else{return};feedback(&ui,(||{let id=id(&ui)?;let db=Database::open_default().map_err(|e|e.to_string())?;let t=number(&ui.get_library_tmin(),"temperature")?;let p=number(&ui.get_library_pressure(),"pressure")?;let entry=db.entries(true).map_err(|e|e.to_string())?.into_iter().find(|m|m.id==id).ok_or("Material no longer exists")?;let solid=entry.phase==flursys::materials::PhaseState::Solid;let property=|q|db.property(&id,q,t,p).map(|v|v.to_string().into()).map_err(|e|e.to_string());let provenance=db.provenance("materials",&id).map_err(|e|e.to_string())?;
 ui.set_library_draft(LibraryMaterialDraft{name:format!("{} · new revision",entry.name).into(),phase:match entry.phase{flursys::materials::PhaseState::Gas=>0,flursys::materials::PhaseState::Liquid=>1,_=>2},rho:property(Quantity::Density)?,cp:property(Quantity::SpecificHeat)?,k:property(Quantity::Conductivity)?,mu:if solid{"".into()}else{property(Quantity::DynamicViscosity)?},beta:if solid{"".into()}else{property(Quantity::Expansion)?},tmin:t.to_string().into(),tmax:t.to_string().into(),pmin:p.to_string().into(),pmax:p.to_string().into(),citation:provenance["source"]["citation"].as_str().unwrap_or("").into(),url:provenance["source"]["url"].as_str().unwrap_or("").into(),revision:"1".into(),note:format!("Constant values copied at T={t} K, p={p} Pa from {id}. Declare the actual valid range before reuse.").into()});Ok(())})());});
    let weak = ui.as_weak();
    ui.on_create_library_material(move||{let Some(ui)=weak.upgrade()else{return};if ui.get_library_busy(){return;}let pack=make_pack(ui.get_library_draft());let pack=match pack{Ok(v)=>v,Err(e)=>{ui.set_library_feedback(e.into());return}};ui.set_library_busy(true);let weak=ui.as_weak();std::thread::spawn(move||{let result=Database::open_default().and_then(|mut db|db.import(&pack)).map_err(|e|e.to_string());let _=weak.upgrade_in_event_loop(move|ui|{ui.set_library_busy(false);match result{Ok(())=>{ui.invoke_refresh_library();ui.set_library_feedback("New immutable material revision created. Reopen the fluid selector to use it.".into());},Err(e)=>ui.set_library_feedback(e.into())}});});});
}
fn make_pack(d: LibraryMaterialDraft) -> Result<database::Pack, String> {
    let phase = match d.phase {
        0 => flursys::materials::PhaseState::Gas,
        1 => flursys::materials::PhaseState::Liquid,
        2 => flursys::materials::PhaseState::Solid,
        _ => return Err("Invalid phase".into()),
    };
    let nonce = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_err(|e| e.to_string())?
        .as_nanos();
    let id = format!("user-material-{nonce}");
    let source_id = format!("user-source-{nonce}");
    let tr = [number(&d.tmin, "Tmin")?, number(&d.tmax, "Tmax")?];
    let pr = [number(&d.pmin, "pmin")?, number(&d.pmax, "pmax")?];
    let mut values = vec![
        (Quantity::Density, number(&d.rho, "density")?),
        (Quantity::SpecificHeat, number(&d.cp, "Cp")?),
        (Quantity::Conductivity, number(&d.k, "conductivity")?),
    ];
    if phase != flursys::materials::PhaseState::Solid {
        values.push((
            Quantity::DynamicViscosity,
            number(&d.mu, "dynamic viscosity")?,
        ));
        values.push((Quantity::Expansion, number(&d.beta, "expansion")?));
    }
    let properties = values
        .into_iter()
        .map(|(q, value)| database::Property {
            id: format!("{id}:{}", q.key()),
            material_id: id.clone(),
            quantity: q,
            unit: q.unit().into(),
            temperature_range: tr,
            pressure_range: pr,
            model: database::Model::Constant { value },
            source_id: source_id.clone(),
            revision: d.revision.to_string(),
        })
        .collect();
    let mut pack = database::Pack {
        sources: vec![database::Source {
            id: source_id.clone(),
            citation: d.citation.to_string(),
            url: if d.url.trim().is_empty() {
                None
            } else {
                Some(d.url.to_string())
            },
            revision: Some(d.revision.to_string()),
            license: None,
            uncertainty: None,
            note: d.note.to_string(),
        }],
        materials: vec![database::Material {
            id: id.clone(),
            name: d.name.to_string(),
            phase,
            composition: "User-defined; see source".into(),
            source_id,
            revision: d.revision.to_string(),
        }],
        properties,
        ..Default::default()
    };
    if phase == flursys::materials::PhaseState::Solid {
        pack.solids.push(database::Solid {
            material_id: id,
            conductivity_tensor: None,
            note: "Isotropic constant material entered in GUI".into(),
        });
    }
    Ok(pack)
}
