use flursys::{
    multiphysics::{
        self,
        metrics::Coordinates,
        species::{self, SpeciesOptions},
    },
    UnstructuredMesh,
};
use serde::{Deserialize, Serialize};
use std::{fs, io::Write, path::Path};

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    mesh_signature: String,
    coordinates: Coordinates,
    face_flux: Vec<f64>,
    initial_fraction: Vec<f64>,
    dt: f64,
    steps: usize,
    options: SpeciesOptions,
}

fn write_new(path: &Path, value: &impl Serialize) -> Result<(), String> {
    let data = serde_json::to_vec_pretty(value).map_err(|e| e.to_string())?;
    let mut file = fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .map_err(|e| e.to_string())?;
    file.write_all(&data).map_err(|e| e.to_string())
}

pub fn template(mesh: &UnstructuredMesh, path: &Path) -> Result<serde_json::Value, String> {
    let config = Config {
        mesh_signature: multiphysics::mesh_signature(mesh),
        coordinates: Coordinates::Cartesian,
        face_flux: vec![0.0; mesh.face_count()],
        initial_fraction: vec![0.0; mesh.cell_count()],
        dt: 0.001,
        steps: 1,
        options: SpeciesOptions::default(),
    };
    write_new(path, &config)?;
    Ok(
        serde_json::json!({"summary":format!("Species transport template saved to {}", path.display())}),
    )
}

pub fn execute(
    mesh: &UnstructuredMesh,
    config: &Path,
    output: &Path,
) -> Result<serde_json::Value, String> {
    let config: Config = serde_json::from_slice(&fs::read(config).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())?;
    if config.mesh_signature != multiphysics::mesh_signature(mesh) {
        return Err("Species configuration mesh signature mismatch".into());
    }
    if config.steps == 0
        || config.steps > 1_000_000
        || !config.dt.is_finite()
        || config.dt <= 0.0
        || !(config.dt * config.steps as f64).is_finite()
    {
        return Err("Invalid species time step or step count".into());
    }
    let measures = config.coordinates.measures(mesh)?;
    let mut fraction = config.initial_fraction;
    let mut history = Vec::with_capacity(config.steps);
    for _ in 0..config.steps {
        history.push(species::advance(
            mesh,
            &measures,
            &config.face_flux,
            &mut fraction,
            config.dt,
            &config.options,
        )?);
    }
    let result = serde_json::json!({"mesh_signature":config.mesh_signature,"time":config.dt * config.steps as f64,"fraction":fraction,"history":history});
    write_new(output, &result)?;
    Ok(
        serde_json::json!({"summary":format!("Species transport completed: {} steps, output {}", config.steps, output.display())}),
    )
}
