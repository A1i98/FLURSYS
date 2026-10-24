//! Source-traceable fluid property models. SI units and restricted evaluation ranges.
//! Coefficients are scientific data; correlation evaluation is implemented in Rust.
use serde::{Deserialize, Serialize};
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum PhaseState {
    Gas,
    Liquid,
    Solid,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MaterialSnapshot {
    pub id: String,
    pub name: String,
    pub phase: PhaseState,
    pub temperature: f64,
    pub pressure: f64,
    pub density: f64,
    pub dynamic_viscosity: f64,
    pub specific_heat: f64,
    pub conductivity: f64,
    pub expansion: f64,
    pub liquid_vapour_surface_tension: Option<f64>,
    pub source: String,
    pub note: String,
}
impl MaterialSnapshot {
    pub fn validate(&self) -> Result<(), String> {
        if [
            self.temperature,
            self.pressure,
            self.density,
            self.dynamic_viscosity,
            self.specific_heat,
            self.conductivity,
        ]
        .iter()
        .any(|v| !v.is_finite() || *v <= 0.0)
            || !self.expansion.is_finite()
        {
            return Err("Material requires positive finite T, p, rho, mu, cp, conductivity and finite expansion".into());
        }
        if self
            .liquid_vapour_surface_tension
            .is_some_and(|v| !v.is_finite() || v < 0.0)
        {
            return Err("Invalid surface tension".into());
        }
        Ok(())
    }
    pub fn kinematic_viscosity(&self) -> f64 {
        self.dynamic_viscosity / self.density
    }
    pub fn thermal_diffusivity(&self) -> f64 {
        self.conductivity / (self.density * self.specific_heat)
    }
}
pub const IDS: [&str; 3] = ["air-tutorial", "water-nsrds", "ethanol-nsrds"];
pub const NAMES: [&str; 3] = [
    "Air · idealized constant-property model",
    "Water · liquid correlation",
    "Ethanol · liquid correlation",
];
/// Fluid choices from the versioned database, including imported custom data.
pub fn catalog_entries() -> Result<Vec<crate::database::Entry>, String> {
    crate::database::Database::open_default()
        .and_then(|db| db.entries(false))
        .map_err(|e| e.to_string())
}
/// Evaluate from database models; no extrapolation and no silent fallback.
pub fn evaluate(id: &str, t: f64, p: f64) -> Result<MaterialSnapshot, String> {
    crate::database::Database::open_default()
        .and_then(|db| db.evaluate(id, t, p))
        .map_err(|e| e.to_string())
}
const ROOT: &str =
    "https://github.com/OpenFOAM/OpenFOAM-dev/blob/421b7a1693061ead99502f0e607122e0e57c0513/";
fn polynomial(t: f64, c: [f64; 6]) -> f64 {
    c.into_iter().rev().fold(0.0, |v, a| v * t + a)
}
fn density(t: f64, c: [f64; 4]) -> (f64, f64) {
    let [a, b, tc, d] = c;
    let theta = 1.0 - t / tc;
    (
        a / b.powf(1.0 + theta.powf(d)),
        -b.ln() * d / tc * theta.powf(d - 1.0),
    )
}
fn viscosity(t: f64, c: [f64; 5]) -> f64 {
    let [a, b, c, d, e] = c;
    (a + b / t + c * t.ln() + d * t.powf(e)).exp()
}
/// Application-restricted window 280–330 K and 0.095–0.105 MPa. These
/// restrictions do not claim that the source's full correlation range is validated.
pub(crate) fn evaluate_builtin(id: &str, t: f64, p: f64) -> Result<MaterialSnapshot, String> {
    if !t.is_finite()
        || !(280.0..=330.0).contains(&t)
        || !p.is_finite()
        || !(95000.0..=105000.0).contains(&p)
    {
        return Err("Catalog evaluation is restricted to 280–330 K and 95000–105000 Pa; use explicit user properties outside this window".into());
    }
    let (name, phase, rho, mu, cp, k, beta, sigma, path, note) = match id {
        "air-tutorial" => {
            let rho = p / ((8314.46261815324 / 28.9) * t);
            let mu = 1.8e-5;
            let cp = 1005.0;
            (NAMES[0],PhaseState::Gas,rho,mu,cp,mu*cp/0.7,1.0/t,None,"tutorials/fluid/blockedChannel/constant/physicalProperties","Idealized tutorial gas: molecular weight 28.9, cp 1005, constant mu 1.8e-5, Pr 0.7. Not a measured air database.")
        }
        "water-nsrds" => {
            let (rho, beta) = density(t, [98.343885, 0.30542, 647.13, 0.081]);
            let tr = t / 647.13;
            (NAMES[1],PhaseState::Liquid,rho,viscosity(t,[-51.964,3670.6,5.7331,-5.3495e-29,10.0]),polynomial(t,[15341.1046350264,-116.019983347211,0.451013044684985,-0.000783569247849015,5.20127671384957e-7,0.0]),polynomial(t,[-0.4267,0.0056903,-8.0065e-6,1.815e-9,0.0,0.0]),beta,Some(0.18548*(1.0-tr).powf(2.717-3.554*tr+2.047*tr*tr)),"src/thermophysicalModels/thermophysicalProperties/liquidProperties/H2O/H2O.C","NSRDS liquid fit; pressure dependence neglected. Not IAPWS. Surface tension is liquid/vapour, not an arbitrary fluid-pair coefficient.")
        }
        "ethanol-nsrds" => {
            let (rho, beta) = density(t, [70.1308387, 0.26395, 516.25, 0.2367]);
            (NAMES[2],PhaseState::Liquid,rho,viscosity(t,[8.049,776.0,-3.068,0.0,0.0]),polynomial(t,[2052.57331394213,-0.121990926653498,-0.00714146172046278,5.20523562482363e-5,0.0,0.0]),0.253-0.000281*t,beta,Some(polynomial(t,[0.037640,-2.157e-5,-1.025e-7,0.0,0.0,0.0])),"src/thermophysicalModels/thermophysicalProperties/liquidProperties/C2H5OH/C2H5OH.C","NSRDS liquid fit; pressure dependence neglected. Liquid/vapour surface tension is not automatically a two-fluid interfacial coefficient.")
        }
        _ => return Err(format!("Unknown material catalog ID {id:?}")),
    };
    let data = MaterialSnapshot {
        id: id.into(),
        name: name.into(),
        phase,
        temperature: t,
        pressure: p,
        density: rho,
        dynamic_viscosity: mu,
        specific_heat: cp,
        conductivity: k,
        expansion: beta,
        liquid_vapour_surface_tension: sigma,
        source: format!("{ROOT}{path}"),
        note: note.into(),
    };
    data.validate()?;
    Ok(data)
}
