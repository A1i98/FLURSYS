//! Versioned SQLite catalogue and rebuildable workspace metadata index.
//! Project JSON/evaluated snapshots and external result files remain authoritative.
mod model;
use crate::materials::{MaterialSnapshot, PhaseState};
pub use model::*;
use rusqlite::{params, Connection, OptionalExtension};
use std::{
    path::{Path, PathBuf},
    time::Duration,
};
pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error + Send + Sync>>;
const APPLICATION_ID: i64 = 1179407699;
const TABLES: &[&str] = &[
    "sources",
    "materials",
    "material_properties",
    "property_samples",
    "solid_properties",
    "fluid_pairs",
    "fluid_pair_properties",
    "thermal_contacts",
    "surface_radiation",
    "boundary_profiles",
    "reference_cases",
    "reference_values",
    "projects",
    "project_materials",
    "runs",
    "artifacts",
];
pub struct Database {
    connection: Connection,
}
pub fn default_path() -> Result<PathBuf> {
    if let Some(p) = std::env::var_os("FLURSYS_DATABASE") {
        if p.is_empty() {
            return Err("FLURSYS_DATABASE is empty".into());
        }
        return Ok(p.into());
    }
    if let Some(p) = std::env::var_os("XDG_DATA_HOME") {
        let p = PathBuf::from(p);
        if p.is_absolute() {
            return Ok(p.join("flursys/catalog.sqlite3"));
        }
    }
    if cfg!(target_os = "windows") {
        if let Some(p) = std::env::var_os("LOCALAPPDATA") {
            return Ok(PathBuf::from(p).join("FLURSYS/catalog.sqlite3"));
        }
    }
    let home =
        std::env::var_os("HOME").ok_or("Set FLURSYS_DATABASE or an application data directory")?;
    Ok(PathBuf::from(home).join(if cfg!(target_os = "macos") {
        "Library/Application Support/FLURSYS/catalog.sqlite3"
    } else {
        ".local/share/flursys/catalog.sqlite3"
    }))
}
fn phase(p: PhaseState) -> &'static str {
    match p {
        PhaseState::Gas => "gas",
        PhaseState::Liquid => "liquid",
        PhaseState::Solid => "solid",
    }
}
fn decode_phase(s: &str) -> Result<PhaseState> {
    match s {
        "gas" => Ok(PhaseState::Gas),
        "liquid" => Ok(PhaseState::Liquid),
        "solid" => Ok(PhaseState::Solid),
        _ => Err("Invalid stored material phase".into()),
    }
}
fn text(s: &str) -> Result<()> {
    if s.trim().is_empty() || s.len() > 4096 || s.chars().any(|c| c == '\0') {
        Err("Database identifiers/metadata must be nonempty and at most 4096 bytes".into())
    } else {
        Ok(())
    }
}
fn range(r: [f64; 2]) -> Result<()> {
    if r.iter().any(|v| !v.is_finite() || *v <= 0.0) || r[0] > r[1] {
        Err("Invalid positive finite property validity range".into())
    } else {
        Ok(())
    }
}
fn inside(x: f64, r: [f64; 2]) -> bool {
    x.is_finite() && x >= r[0] && x <= r[1]
}
fn json<T: serde::Serialize>(v: &T) -> Result<String> {
    Ok(serde_json::to_string(v)?)
}
impl Database {
    pub fn open_default() -> Result<Self> {
        Self::open(&default_path()?)
    }
    pub fn open(path: &Path) -> Result<Self> {
        if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
            std::fs::create_dir_all(parent)?;
        }
        let mut connection = Connection::open_with_flags(
            path,
            rusqlite::OpenFlags::SQLITE_OPEN_READ_WRITE
                | rusqlite::OpenFlags::SQLITE_OPEN_CREATE
                | rusqlite::OpenFlags::SQLITE_OPEN_NO_MUTEX,
        )?;
        connection.busy_timeout(Duration::from_secs(3))?;
        connection.pragma_update(None, "foreign_keys", "ON")?;
        let id: i64 = connection.pragma_query_value(None, "application_id", |r| r.get(0))?;
        let version: i64 = connection.pragma_query_value(None, "user_version", |r| r.get(0))?;
        if version > 1 || (id != 0 && id != APPLICATION_ID) {
            return Err("Unsupported or foreign database; no migration performed".into());
        }
        if version == 0 {
            let count:i64=connection.query_row("SELECT count(*) FROM sqlite_master WHERE type='table' AND name NOT LIKE 'sqlite_%'",[],|r|r.get(0))?;
            if count != 0 {
                return Err("Refusing to initialize a nonempty unrecognized database".into());
            }
            let tx =
                connection.transaction_with_behavior(rusqlite::TransactionBehavior::Immediate)?;
            tx.execute_batch(include_str!("schema.sql"))?;
            Self::seed(&tx)?;
            tx.commit()?;
        } else if id != APPLICATION_ID {
            return Err("Database application identity is missing".into());
        }
        connection.pragma_update(None, "journal_mode", "WAL")?;
        Ok(Self { connection })
    }
    fn seed(c: &Connection) -> Result<()> {
        for id in crate::materials::IDS {
            let m = crate::materials::evaluate_builtin(id, 293.15, 101325.0)?;
            let source = format!("builtin:{id}");
            c.execute(
                "INSERT INTO sources VALUES(?1,?2,?3,?4,NULL,NULL,?5)",
                params![
                    source,
                    m.name,
                    m.source,
                    "421b7a1693061ead99502f0e607122e0e57c0513",
                    m.note
                ],
            )?;
            c.execute(
                "INSERT INTO materials VALUES(?1,?2,?3,?4,?5,'1',1)",
                params![
                    m.id,
                    m.name,
                    phase(m.phase),
                    "See source; no invented composition",
                    source
                ],
            )?;
            for quantity in [
                Quantity::Density,
                Quantity::DynamicViscosity,
                Quantity::SpecificHeat,
                Quantity::Conductivity,
                Quantity::Expansion,
            ] {
                c.execute("INSERT INTO material_properties VALUES(?1,?2,?3,?4,'builtin',NULL,280,330,95000,105000,?5,'1')",params![format!("builtin:{id}:{}",quantity.key()),id,quantity.key(),quantity.unit(),source])?;
            }
        }
        c.execute("INSERT INTO sources VALUES('builtin:cht-heater','Illustrative constant solid properties',NULL,'1',NULL,NULL,'rho=8000 kg/m3, Cp=450 J/(kg K), k=80 W/(m K). Not identified as a certified steel grade; application evaluation restricted to 293.15 K, 101325 Pa.')",[])?;
        c.execute("INSERT INTO materials VALUES('cht-heater-tutorial','Heater solid · illustrative properties','solid','Unspecified tutorial solid','builtin:cht-heater','1',1)",[])?;
        for (quantity, value) in [
            (Quantity::Density, 8000.0),
            (Quantity::SpecificHeat, 450.0),
            (Quantity::Conductivity, 80.0),
        ] {
            c.execute("INSERT INTO material_properties VALUES(?1,'cht-heater-tutorial',?2,?3,'constant',?4,293.15,293.15,101325,101325,'builtin:cht-heater','1')",params![format!("builtin:heater:{}",quantity.key()),quantity.key(),quantity.unit(),value])?;
        }
        c.execute("INSERT INTO solid_properties VALUES('cht-heater-tutorial',NULL,'Isotropic constant tutorial solid; no measured anisotropic tensor')",[])?;
        c.execute("INSERT INTO sources VALUES('builtin:idealizations','Explicit mathematical idealizations',NULL,'1',NULL,NULL,'Perfect contact and black surface are ideal boundary models, not measured properties of the tutorial solid. Temperature window is an application numerical restriction, not material validation.')",[])?;
        c.execute("INSERT INTO thermal_contacts VALUES('ideal-perfect-contact','cht-heater-tutorial','cht-heater-tutorial',0,1,10000,'builtin:idealizations','1')",[])?;
        c.execute("INSERT INTO surface_radiation VALUES('ideal-black-surface','cht-heater-tutorial',1,1,10000,'builtin:idealizations','1')",[])?;
        c.execute("INSERT INTO sources VALUES('builtin:dam-break','Illustrative air-water surface tension',NULL,'1',NULL,NULL,'sigma=0.07 N/m. Not a measured pair database; application evaluation restricted to 293.15 K.')",[])?;
        c.execute("INSERT INTO fluid_pairs VALUES('air-water-tutorial','air-tutorial','water-nsrds','builtin:dam-break','1')",[])?;
        c.execute("INSERT INTO fluid_pair_properties VALUES('builtin:air-water-sigma','air-water-tutorial','surface-tension','N/m',293.15,293.15,0.07,'builtin:dam-break','1')",[])?;
        c.execute("INSERT INTO sources VALUES('builtin:jet-profile','FLURSYS illustrative jet-temperature ramp',NULL,'1',NULL,NULL,'User-selectable example boundary input, not experimental data. Does not configure gravity, the inlet velocity or the reference temperature automatically.')",[])?;
        let jet = crate::unstructured_incompressible::BoundaryProfile {
            quantity: crate::unstructured_incompressible::BoundaryQuantity::Temperature,
            samples: vec![
                crate::unstructured_incompressible::TimeSample {
                    time: 0.0,
                    value: crate::Vec3::new(293.15, 0.0, 0.0),
                },
                crate::unstructured_incompressible::TimeSample {
                    time: 1.0,
                    value: crate::Vec3::new(303.15, 0.0, 0.0),
                },
            ],
        };
        c.execute("INSERT INTO boundary_profiles VALUES('jet-temperature-ramp','in','Temperature','K',?1,'builtin:jet-profile','1')",[json(&jet)?])?;
        c.execute("INSERT INTO sources VALUES('builtin:examples','FLURSYS executable example definitions',NULL,NULL,NULL,NULL,'Examples are not validated reference measurements')",[])?;
        for case in crate::workbench::catalogue::builtin_templates()? {
            c.execute(
                "INSERT INTO reference_cases VALUES(?1,?2,'unvalidated',?3,'builtin:examples','1')",
                params![format!("builtin:{}", case.id), case.name, json(&case)?],
            )?;
        }
        Ok(())
    }
    pub fn counts(&self) -> Result<Counts> {
        let mut counts = Counts::new();
        for name in TABLES {
            let n: i64 =
                self.connection
                    .query_row(&format!("SELECT count(*) FROM {name}"), [], |r| r.get(0))?;
            counts.insert((*name).into(), n as u64);
        }
        Ok(counts)
    }
    pub fn entries(&self, include_solids: bool) -> Result<Vec<Entry>> {
        let mut query=self.connection.prepare("SELECT id,name,phase,source_id,revision FROM materials WHERE (?1 OR phase<>'solid') ORDER BY builtin DESC,name,id")?;
        let rows = query.query_map([include_solids], |r| {
            Ok((
                r.get::<_, String>(0)?,
                r.get::<_, String>(1)?,
                r.get::<_, String>(2)?,
                r.get::<_, String>(3)?,
                r.get::<_, String>(4)?,
            ))
        })?;
        rows.map(|r| {
            let (id, name, p, source_id, revision) = r?;
            Ok(Entry {
                id,
                name,
                phase: decode_phase(&p)?,
                source_id,
                revision,
            })
        })
        .collect()
    }
    pub fn import(&mut self, pack: &Pack) -> Result<()> {
        for id in pack
            .sources
            .iter()
            .map(|r| r.id.as_str())
            .chain(pack.materials.iter().map(|r| r.id.as_str()))
            .chain(pack.properties.iter().map(|r| r.id.as_str()))
            .chain(pack.fluid_pairs.iter().map(|r| r.id.as_str()))
            .chain(pack.pair_properties.iter().map(|r| r.id.as_str()))
            .chain(pack.thermal_contacts.iter().map(|r| r.id.as_str()))
            .chain(pack.surface_radiation.iter().map(|r| r.id.as_str()))
            .chain(pack.boundary_profiles.iter().map(|r| r.id.as_str()))
            .chain(pack.reference_cases.iter().map(|r| r.id.as_str()))
            .chain(pack.reference_values.iter().map(|r| r.id.as_str()))
        {
            text(id)?;
            if id.starts_with("builtin:") {
                return Err("Builtin catalogue IDs are reserved".into());
            }
        }

        let tx = self
            .connection
            .transaction_with_behavior(rusqlite::TransactionBehavior::Immediate)?;
        for s in &pack.sources {
            text(&s.id)?;
            text(&s.citation)?;
            if s.id.starts_with("builtin:") {
                return Err("Builtin source IDs are reserved".into());
            }
            tx.execute(
                "INSERT INTO sources VALUES(?1,?2,?3,?4,?5,?6,?7)",
                params![
                    s.id,
                    s.citation,
                    s.url,
                    s.revision,
                    s.license,
                    s.uncertainty,
                    s.note
                ],
            )?;
        }
        for m in &pack.materials {
            text(&m.id)?;
            text(&m.name)?;
            text(&m.revision)?;
            tx.execute(
                "INSERT INTO materials VALUES(?1,?2,?3,?4,?5,?6,0)",
                params![
                    m.id,
                    m.name,
                    phase(m.phase),
                    m.composition,
                    m.source_id,
                    m.revision
                ],
            )?;
        }
        for p in &pack.properties {
            text(&p.id)?;
            text(&p.revision)?;
            range(p.temperature_range)?;
            range(p.pressure_range)?;
            if p.unit != p.quantity.unit() {
                return Err(
                    format!("Property {} requires SI unit {}", p.id, p.quantity.unit()).into(),
                );
            }
            let builtin: bool = tx.query_row(
                "SELECT builtin FROM materials WHERE id=?1",
                [&p.material_id],
                |r| r.get(0),
            )?;
            if builtin || p.id.starts_with("builtin:") {
                return Err("Builtin property models are immutable".into());
            }
            let (kind, value) = match &p.model {
                Model::Constant { value } => {
                    if !p.quantity.validate_value(*value) {
                        return Err("Invalid constant property".into());
                    }
                    ("constant", Some(*value))
                }
                Model::Table { samples } => {
                    if samples.is_empty() || samples.len() > 100000 {
                        return Err("Property table requires 1..100000 points".into());
                    }
                    for s in samples {
                        if !inside(s.temperature, p.temperature_range)
                            || !inside(s.pressure, p.pressure_range)
                            || !p.quantity.validate_value(s.value)
                        {
                            return Err("Invalid or out-of-range property sample".into());
                        }
                    }
                    ("table", None)
                }
            };
            tx.execute(
                "INSERT INTO material_properties VALUES(?1,?2,?3,?4,?5,?6,?7,?8,?9,?10,?11,?12)",
                params![
                    p.id,
                    p.material_id,
                    p.quantity.key(),
                    p.unit,
                    kind,
                    value,
                    p.temperature_range[0],
                    p.temperature_range[1],
                    p.pressure_range[0],
                    p.pressure_range[1],
                    p.source_id,
                    p.revision
                ],
            )?;
            if let Model::Table { samples } = &p.model {
                for s in samples {
                    tx.execute(
                        "INSERT INTO property_samples VALUES(?1,?2,?3,?4)",
                        params![p.id, s.temperature, s.pressure, s.value],
                    )?;
                }
            }
        }
        for s in &pack.solids {
            let p: String = tx.query_row(
                "SELECT phase FROM materials WHERE id=?1",
                [&s.material_id],
                |r| r.get(0),
            )?;
            if p != "solid" {
                return Err("Solid properties require a solid material".into());
            }
            if let Some(t) = s.conductivity_tensor {
                validate_tensor(t)?;
            }
            tx.execute(
                "INSERT INTO solid_properties VALUES(?1,?2,?3)",
                params![
                    s.material_id,
                    s.conductivity_tensor.map(|v| json(&v)).transpose()?,
                    s.note
                ],
            )?;
        }
        for p in &pack.fluid_pairs {
            text(&p.id)?;
            for m in [&p.first_material, &p.second_material] {
                let phase: String =
                    tx.query_row("SELECT phase FROM materials WHERE id=?1", [m], |r| r.get(0))?;
                if phase == "solid" {
                    return Err("Fluid pair cannot include a solid".into());
                }
            }
            tx.execute(
                "INSERT INTO fluid_pairs VALUES(?1,?2,?3,?4,?5)",
                params![
                    p.id,
                    p.first_material,
                    p.second_material,
                    p.source_id,
                    p.revision
                ],
            )?;
        }
        for p in &pack.pair_properties {
            range(p.temperature_range)?;
            if !p.value.is_finite()
                || p.value < 0.0
                || p.unit != p.quantity.unit()
                || matches!(p.quantity, PairQuantity::ContactAngle)
                    && p.value > std::f64::consts::PI
            {
                return Err("Invalid pair property, SI unit or contact angle [0,pi]".into());
            }
            tx.execute(
                "INSERT INTO fluid_pair_properties VALUES(?1,?2,?3,?4,?5,?6,?7,?8,?9)",
                params![
                    p.id,
                    p.pair_id,
                    p.quantity.key(),
                    p.unit,
                    p.temperature_range[0],
                    p.temperature_range[1],
                    p.value,
                    p.source_id,
                    p.revision
                ],
            )?;
        }
        for c in &pack.thermal_contacts {
            range(c.temperature_range)?;
            if !c.resistance_m2_k_w.is_finite() || c.resistance_m2_k_w < 0.0 {
                return Err("Invalid contact resistance".into());
            }
            tx.execute(
                "INSERT INTO thermal_contacts VALUES(?1,?2,?3,?4,?5,?6,?7,?8)",
                params![
                    c.id,
                    c.first_material,
                    c.second_material,
                    c.resistance_m2_k_w,
                    c.temperature_range[0],
                    c.temperature_range[1],
                    c.source_id,
                    c.revision
                ],
            )?;
        }
        for s in &pack.surface_radiation {
            range(s.temperature_range)?;
            if !s.emissivity.is_finite() || !(0.0..=1.0).contains(&s.emissivity) {
                return Err("Emissivity must be in [0,1]".into());
            }
            tx.execute(
                "INSERT INTO surface_radiation VALUES(?1,?2,?3,?4,?5,?6,?7)",
                params![
                    s.id,
                    s.material_id,
                    s.emissivity,
                    s.temperature_range[0],
                    s.temperature_range[1],
                    s.source_id,
                    s.revision
                ],
            )?;
        }
        for p in &pack.boundary_profiles {
            p.profile.validate()?;
            text(&p.id)?;
            text(&p.patch)?;
            if !(p.unit == profile_unit(p.profile.quantity)
                || (p.profile.quantity
                    == crate::unstructured_incompressible::BoundaryQuantity::FlowRate
                    && p.unit == "kg/s"))
            {
                return Err("Boundary profile unit does not match its quantity".into());
            }
            tx.execute(
                "INSERT INTO boundary_profiles VALUES(?1,?2,?3,?4,?5,?6,?7)",
                params![
                    p.id,
                    p.patch,
                    format!("{:?}", p.profile.quantity),
                    p.unit,
                    json(&p.profile)?,
                    p.source_id,
                    p.revision
                ],
            )?;
        }
        for r in &pack.reference_cases {
            if r.id.starts_with("builtin:") {
                return Err("Builtin example references are immutable".into());
            }
            text(&r.name)?;
            tx.execute(
                "INSERT INTO reference_cases VALUES(?1,?2,?3,?4,?5,?6)",
                params![
                    r.id,
                    r.name,
                    r.status,
                    json(&r.descriptor)?,
                    r.source_id,
                    r.revision
                ],
            )?;
        }
        for v in &pack.reference_values {
            text(&v.quantity)?;
            text(&v.unit)?;
            if v.data.is_empty() || v.data.iter().flatten().any(|v| !v.is_finite()) {
                return Err("Reference values require finite data".into());
            }
            tx.execute(
                "INSERT INTO reference_values VALUES(?1,?2,?3,?4,?5,?6)",
                params![
                    v.id,
                    v.case_id,
                    v.quantity,
                    v.unit,
                    json(&v.data)?,
                    v.source_id
                ],
            )?;
        }
        tx.commit()?;
        Ok(())
    }
    pub fn property(&self, material: &str, quantity: Quantity, t: f64, p: f64) -> Result<f64> {
        let (id,unit,model,value,tmin,tmax,pmin,pmax):(String,String,String,Option<f64>,f64,f64,f64,f64)=self.connection.query_row("SELECT id,unit,model,constant_value,temperature_min,temperature_max,pressure_min,pressure_max FROM material_properties WHERE material_id=?1 AND quantity=?2",params![material,quantity.key()],|r|Ok((r.get(0)?,r.get(1)?,r.get(2)?,r.get(3)?,r.get(4)?,r.get(5)?,r.get(6)?,r.get(7)?)))?;
        if unit != quantity.unit() || !inside(t, [tmin, tmax]) || !inside(p, [pmin, pmax]) {
            return Err(format!("{} evaluation outside validity [{tmin},{tmax}] K / [{pmin},{pmax}] Pa or wrong stored SI unit",quantity.key()).into());
        }
        let v = match model.as_str() {
            "builtin" => {
                let m = crate::materials::evaluate_builtin(material, t, p)?;
                match quantity {
                    Quantity::Density => m.density,
                    Quantity::DynamicViscosity => m.dynamic_viscosity,
                    Quantity::SpecificHeat => m.specific_heat,
                    Quantity::Conductivity => m.conductivity,
                    Quantity::Expansion => m.expansion,
                }
            }
            "constant" => value.ok_or("Constant model lacks a value")?,
            "table" => {
                let mut query=self.connection.prepare("SELECT temperature,pressure,value FROM property_samples WHERE property_id=?1 ORDER BY pressure,temperature")?;
                let samples = query
                    .query_map([id], |r| {
                        Ok(Sample {
                            temperature: r.get(0)?,
                            pressure: r.get(1)?,
                            value: r.get(2)?,
                        })
                    })?
                    .collect::<std::result::Result<Vec<_>, _>>()?;
                interpolate(&samples, t, p)?
            }
            _ => return Err("Unsupported material model".into()),
        };
        if !quantity.validate_value(v) {
            return Err("Evaluated material property is nonphysical".into());
        }
        Ok(v)
    }
    pub fn evaluate(&self, id: &str, t: f64, p: f64) -> Result<MaterialSnapshot> {
        let (name, phase_name, source_id, revision): (String, String, String, String) =
            self.connection.query_row(
                "SELECT name,phase,source_id,revision FROM materials WHERE id=?1",
                [id],
                |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?, r.get(3)?)),
            )?;
        let state = decode_phase(&phase_name)?;
        if state == PhaseState::Solid {
            return Err(
                "Select a fluid for the flow solver; evaluate solid thermal properties separately"
                    .into(),
            );
        }
        let (url, citation, note): (Option<String>, String, String) = self.connection.query_row(
            "SELECT url,citation,note FROM sources WHERE id=?1",
            [source_id],
            |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?)),
        )?;
        let m=MaterialSnapshot{id:id.into(),name,phase:state,temperature:t,pressure:p,density:self.property(id,Quantity::Density,t,p)?,dynamic_viscosity:self.property(id,Quantity::DynamicViscosity,t,p)?,specific_heat:self.property(id,Quantity::SpecificHeat,t,p)?,conductivity:self.property(id,Quantity::Conductivity,t,p)?,expansion:self.property(id,Quantity::Expansion,t,p)?,liquid_vapour_surface_tension:if crate::materials::IDS.contains(&id){crate::materials::evaluate_builtin(id,t,p)?.liquid_vapour_surface_tension}else{None},source:url.unwrap_or(citation),note:format!("{note} Material revision: {revision}; evaluated once, properties frozen during solve.")};
        m.validate()?;
        Ok(m)
    }
    pub fn solid(&self, id: &str, t: f64, p: f64) -> Result<crate::multiphysics::Properties> {
        let state: String =
            self.connection
                .query_row("SELECT phase FROM materials WHERE id=?1", [id], |r| {
                    r.get(0)
                })?;
        if state != "solid" {
            return Err("Requested solid properties from a non-solid".into());
        }
        let conductivity = self.property(id, Quantity::Conductivity, t, p)?;
        let tensor: Option<String> = self
            .connection
            .query_row(
                "SELECT conductivity_tensor_json FROM solid_properties WHERE material_id=?1",
                [id],
                |r| r.get(0),
            )
            .optional()?
            .flatten();
        if let Some(t) = tensor {
            let t: [[f64; 3]; 3] = serde_json::from_str(&t)?;
            validate_tensor(t)?;
            let k = t[0][0];
            if (k - conductivity).abs() > 1e-12 * k.abs().max(conductivity.abs()) {
                return Err("Stored isotropic tensor and scalar conductivity disagree".into());
            }

            if (0..3).any(|i| {
                (0..3).any(|j| (t[i][j] - if i == j { k } else { 0.0 }).abs() > 1e-12 * k.abs())
            }) {
                return Err(
                    "Anisotropic data is stored but the scalar thermal solver cannot consume it"
                        .into(),
                );
            }
        }
        Ok(crate::multiphysics::Properties {
            density: self.property(id, Quantity::Density, t, p)?,
            specific_heat: self.property(id, Quantity::SpecificHeat, t, p)?,
            conductivity,
        })
    }
    pub fn profile(
        &self,
        id: &str,
    ) -> Result<(String, crate::unstructured_incompressible::BoundaryProfile)> {
        let (p, j): (String, String) = self.connection.query_row(
            "SELECT patch,samples_json FROM boundary_profiles WHERE id=?1",
            [id],
            |r| Ok((r.get(0)?, r.get(1)?)),
        )?;
        let v: crate::unstructured_incompressible::BoundaryProfile = serde_json::from_str(&j)?;
        v.validate()?;
        Ok((p, v))
    }
    pub fn contact(&self, id: &str, t: f64) -> Result<f64> {
        let(v,a,b):(f64,f64,f64)=self.connection.query_row("SELECT resistance_m2_k_w,temperature_min,temperature_max FROM thermal_contacts WHERE id=?1",[id],|r|Ok((r.get(0)?,r.get(1)?,r.get(2)?)))?;
        if !inside(t, [a, b]) {
            return Err("Contact evaluation outside valid temperature range".into());
        }
        Ok(v)
    }
    pub fn emissivity(&self, id: &str, t: f64) -> Result<f64> {
        let (v, a, b): (f64, f64, f64) = self.connection.query_row(
            "SELECT emissivity,temperature_min,temperature_max FROM surface_radiation WHERE id=?1",
            [id],
            |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?)),
        )?;
        if !inside(t, [a, b]) {
            return Err("Surface evaluation outside valid temperature range".into());
        }
        Ok(v)
    }
    pub fn pair_property(&self, id: &str, quantity: PairQuantity, t: f64) -> Result<f64> {
        let(v,a,b):(f64,f64,f64)=self.connection.query_row("SELECT value,temperature_min,temperature_max FROM fluid_pair_properties WHERE pair_id=?1 AND quantity=?2",params![id,quantity.key()],|r|Ok((r.get(0)?,r.get(1)?,r.get(2)?)))?;
        if !inside(t, [a, b]) {
            return Err("Pair property outside valid temperature range".into());
        }
        Ok(v)
    }
    pub fn reference(&self, id: &str) -> Result<serde_json::Value> {
        let mut q = self.connection.prepare(
            "SELECT quantity,unit,data_json FROM reference_values WHERE case_id=?1 ORDER BY id",
        )?;
        let values=q.query_map([id],|r|Ok((r.get::<_,String>(0)?,r.get::<_,String>(1)?,r.get::<_,String>(2)?)))?.map(|v|->Result<_>{let(q,u,j)=v?;Ok(serde_json::json!({"quantity":q,"unit":u,"data":serde_json::from_str::<serde_json::Value>(&j)?}))}).collect::<Result<Vec<_>>>()?;
        let (name, status, j): (String, String, String) = self.connection.query_row(
            "SELECT name,status,descriptor_json FROM reference_cases WHERE id=?1",
            [id],
            |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?)),
        )?;
        Ok(
            serde_json::json!({"id":id,"name":name,"status":status,"descriptor":serde_json::from_str::<serde_json::Value>(&j)?,"values":values}),
        )
    }
}
fn interpolate(samples: &[Sample], t: f64, p: f64) -> Result<f64> {
    let mut pressures = samples.iter().map(|s| s.pressure).collect::<Vec<_>>();
    pressures.sort_by(f64::total_cmp);
    pressures.dedup();
    let bracket = |x: f64, v: &[f64]| -> Result<(f64, f64)> {
        let lo = v
            .iter()
            .rev()
            .find(|&&q| q <= x)
            .copied()
            .ok_or("Property table cannot extrapolate")?;
        let hi = v
            .iter()
            .find(|&&q| q >= x)
            .copied()
            .ok_or("Property table cannot extrapolate")?;
        Ok((lo, hi))
    };
    let (lo, hi) = bracket(p, &pressures)?;
    let at = |pressure: f64| -> Result<f64> {
        let mut row = samples
            .iter()
            .filter(|s| s.pressure == pressure)
            .collect::<Vec<_>>();
        row.sort_by(|a, b| a.temperature.total_cmp(&b.temperature));
        let temps = row.iter().map(|s| s.temperature).collect::<Vec<_>>();
        let (a, b) = bracket(t, &temps)?;
        let va = row.iter().find(|s| s.temperature == a).unwrap().value;
        let vb = row.iter().find(|s| s.temperature == b).unwrap().value;
        Ok(if a == b {
            va
        } else {
            va + (vb - va) * (t - a) / (b - a)
        })
    };
    let a = at(lo)?;
    Ok(if lo == hi {
        a
    } else {
        a + (at(hi)? - a) * (p - lo) / (hi - lo)
    })
}
fn validate_tensor(t: [[f64; 3]; 3]) -> Result<()> {
    if t.iter().flatten().any(|v| !v.is_finite())
        || (0..3)
            .any(|i| (0..3).any(|j| (t[i][j] - t[j][i]).abs() > 1e-12 * t[i][i].abs().max(1.0)))
    {
        return Err("Conductivity tensor must be finite symmetric positive definite".into());
    }
    let mut l = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..=i {
            let mut v = t[i][j];
            for k in 0..j {
                v -= l[i][k] * l[j][k];
            }
            if i == j {
                if v <= 0.0 || !v.is_finite() {
                    return Err("Conductivity tensor is not positive definite".into());
                }
                l[i][j] = v.sqrt();
            } else {
                l[i][j] = v / l[j][j];
            }
        }
    }
    Ok(())
}
pub fn profile_unit(q: crate::unstructured_incompressible::BoundaryQuantity) -> &'static str {
    use crate::unstructured_incompressible::BoundaryQuantity as Q;
    match q {
        Q::Velocity => "m/s",
        Q::FlowRate => "m3/s",
        Q::Pressure => "m2/s2",
        Q::Temperature => "K",
        Q::HeatFlux => "W/m2",
    }
}

impl Database {
    /// Rebuildable index; never stores field arrays or changes the project snapshot.
    pub fn index_project(
        &mut self,
        workspace: &Path,
        project: &crate::workbench::WorkbenchProject,
    ) -> Result<()> {
        project.validate()?;
        let tx = self
            .connection
            .transaction_with_behavior(rusqlite::TransactionBehavior::Immediate)?;
        tx.execute("INSERT INTO projects(id,name,workspace,snapshot_json) VALUES(?1,?2,?3,?4) ON CONFLICT(id) DO UPDATE SET name=excluded.name,workspace=excluded.workspace,snapshot_json=excluded.snapshot_json,updated_at=CURRENT_TIMESTAMP",params![project.project_id,project.name,workspace.to_string_lossy(),json(project)?])?;
        tx.execute("INSERT INTO project_materials VALUES(?1,?2,?3) ON CONFLICT(project_id) DO UPDATE SET material_id=excluded.material_id,evaluated_snapshot_json=excluded.evaluated_snapshot_json",params![project.project_id,project.material.catalog.as_ref().map(|m|&m.id),json(&project.material)?])?;
        tx.execute(
            "DELETE FROM runs WHERE project_id=?1",
            [&project.project_id],
        )?;
        for run in &project.runs {
            tx.execute(
                "INSERT INTO runs VALUES(?1,?2,?3,?4)",
                params![project.project_id, run.id, json(&run.status)?, json(run)?],
            )?;
            for (kind, path) in [
                ("report", Some(&run.report_path)),
                ("solution", run.solution_path.as_ref()),
            ] {
                if let Some(path) = path {
                    tx.execute(
                        "INSERT INTO artifacts VALUES(?1,?2,?3,?4)",
                        params![project.project_id, run.id, kind, path.to_string_lossy()],
                    )?;
                }
            }
        }
        tx.commit()?;
        Ok(())
    }
    /// Query registered metadata, never an arbitrary SQL endpoint.
    pub fn records(&self, table: &str) -> Result<Vec<serde_json::Value>> {
        if !TABLES.contains(&table) {
            return Err("Unknown catalogue table".into());
        }
        let mut statement = self.connection.prepare(&format!("SELECT * FROM {table}"))?;
        let names = statement
            .column_names()
            .into_iter()
            .map(String::from)
            .collect::<Vec<_>>();
        let rows = statement.query_map([], |r| {
            let mut value = serde_json::Map::new();
            for (i, name) in names.iter().enumerate() {
                let item = match r.get_ref(i)? {
                    rusqlite::types::ValueRef::Null => serde_json::Value::Null,
                    rusqlite::types::ValueRef::Integer(v) => v.into(),
                    rusqlite::types::ValueRef::Real(v) => serde_json::json!(v),
                    rusqlite::types::ValueRef::Text(v) => {
                        String::from_utf8_lossy(v).to_string().into()
                    }
                    rusqlite::types::ValueRef::Blob(_) => "[binary]".into(),
                };
                value.insert(name.clone(), item);
            }
            Ok(serde_json::Value::Object(value))
        })?;
        Ok(rows.collect::<std::result::Result<Vec<_>, _>>()?)
    }
}
impl Database {
    pub fn backup(&self, path: &Path) -> Result<()> {
        if path.exists() {
            return Err("Backup destination already exists".into());
        }
        self.connection.execute(
            "VACUUM INTO ?1",
            [path.to_str().ok_or("Backup path must be UTF-8")?],
        )?;
        Ok(())
    }
    pub fn provenance(&self, table: &str, id: &str) -> Result<serde_json::Value> {
        if ![
            "materials",
            "thermal_contacts",
            "surface_radiation",
            "fluid_pairs",
            "boundary_profiles",
        ]
        .contains(&table)
        {
            return Err("Unsupported provenance record".into());
        }
        let source: String = self.connection.query_row(
            &format!("SELECT source_id FROM {table} WHERE id=?1"),
            [id],
            |r| r.get(0),
        )?;
        let record = self
            .records(table)?
            .into_iter()
            .find(|v| v.get("id").and_then(|v| v.as_str()) == Some(id))
            .ok_or("Missing catalogue record")?;
        let source = self
            .records("sources")?
            .into_iter()
            .find(|v| v.get("id").and_then(|v| v.as_str()) == Some(&source));
        Ok(serde_json::json!({"record":record,"source":source}))
    }
}
impl Database {
    pub fn pair_materials(&self, id: &str) -> Result<(String, String)> {
        Ok(self.connection.query_row(
            "SELECT first_material,second_material FROM fluid_pairs WHERE id=?1",
            [id],
            |r| Ok((r.get(0)?, r.get(1)?)),
        )?)
    }
}
impl Database {
    pub fn stored_profile_unit(&self, id: &str) -> Result<String> {
        Ok(self.connection.query_row(
            "SELECT unit FROM boundary_profiles WHERE id=?1",
            [id],
            |r| r.get(0),
        )?)
    }
}
