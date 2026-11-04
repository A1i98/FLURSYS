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
    restart: Option<&Path>,
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
    let checkpoint = restart.map(species::SpeciesCheckpoint::load).transpose()?;
    if let Some(checkpoint) = &checkpoint {
        checkpoint.validate(mesh, config.coordinates, &config.face_flux, &config.options)?;
    }
    let initial_step = checkpoint.as_ref().map_or(0, |c| c.step);
    let initial_time = checkpoint.as_ref().map_or(0.0, |c| c.time);
    let final_step = initial_step
        .checked_add(config.steps)
        .ok_or("Species step overflow")?;
    let final_time = initial_time + config.dt * config.steps as f64;
    if !final_time.is_finite() {
        return Err("Species time overflow".into());
    }
    let mut fraction = checkpoint.map_or(config.initial_fraction, |c| c.fraction);
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
    let checkpoint = species::SpeciesCheckpoint::capture(
        mesh,
        config.coordinates,
        &config.face_flux,
        &config.options,
        final_step,
        final_time,
        fraction,
    )?;
    let result = serde_json::json!({"mesh_signature":config.mesh_signature,"time":final_time,"fraction":checkpoint.fraction,"history":history,"checkpoint":checkpoint});
    write_new(output, &result)?;
    Ok(
        serde_json::json!({"summary":format!("Species transport completed: {} steps, output {}", config.steps, output.display())}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use flursys::{BoundaryPatch, BoundaryType, CellDefinition, MeshDimension, Point};

    #[test]
    fn checkpoint_result_resumes_without_changing_the_solution() {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0., 0., 0.),
                Point::new(1., 0., 0.),
                Point::new(2., 0., 0.),
                Point::new(0., 1., 0.),
                Point::new(1., 1., 0.),
                Point::new(2., 1., 0.),
            ],
            vec![
                CellDefinition::polygon(vec![0, 1, 4, 3]),
                CellDefinition::polygon(vec![1, 2, 5, 4]),
            ],
        )
        .unwrap();
        let faces = mesh
            .faces()
            .iter()
            .enumerate()
            .filter_map(|(i, f)| f.neighbour.is_none().then_some(i))
            .collect();
        let mesh = mesh
            .with_boundary_patches(vec![BoundaryPatch {
                name: "wall".into(),
                face_indices: faces,
                boundary_type: BoundaryType::Wall,
            }])
            .unwrap();
        let dir = tempfile::tempdir().unwrap();
        let config = dir.path().join("config.json");
        template(&mesh, &config).unwrap();
        let mut settings: Config = serde_json::from_slice(&fs::read(&config).unwrap()).unwrap();
        settings.initial_fraction = vec![1.0, 0.0];
        settings.options.diffusivity = 0.1;
        settings.dt = 0.1;
        settings.steps = 2;
        fs::write(&config, serde_json::to_vec(&settings).unwrap()).unwrap();
        let first = dir.path().join("first.json");
        let continued = dir.path().join("continued.json");
        let full = dir.path().join("full.json");
        execute(&mesh, &config, &first, None).unwrap();
        execute(&mesh, &config, &continued, Some(&first)).unwrap();
        settings.steps = 4;
        fs::write(&config, serde_json::to_vec(&settings).unwrap()).unwrap();
        execute(&mesh, &config, &full, None).unwrap();
        let resumed: serde_json::Value =
            serde_json::from_slice(&fs::read(continued).unwrap()).unwrap();
        let uninterrupted: serde_json::Value =
            serde_json::from_slice(&fs::read(full).unwrap()).unwrap();
        assert_eq!(resumed["fraction"], uninterrupted["fraction"]);
        assert_eq!(resumed["checkpoint"]["step"], 4);
        assert_eq!(
            resumed["checkpoint"]["time"],
            uninterrupted["checkpoint"]["time"]
        );
    }
}
