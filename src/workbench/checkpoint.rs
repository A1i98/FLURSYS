use crate::{CellField, IncompressibleInitialConditions as Initial, UnstructuredMesh, Vec3};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    io::Write,
    path::{Path, PathBuf},
};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MeshSignature {
    dimension: String,
    points: Vec<Vec3>,
    faces: Vec<(Vec<usize>, usize, Option<usize>)>,
    cells: Vec<Vec<usize>>,
}
impl MeshSignature {
    pub fn new(mesh: &UnstructuredMesh) -> Self {
        Self {
            dimension: format!("{:?}", mesh.dimension()),
            points: mesh.points().iter().map(|p| p.position).collect(),
            faces: mesh
                .faces()
                .iter()
                .map(|f| (f.vertices.clone(), f.owner, f.neighbour))
                .collect(),
            cells: mesh.cells().iter().map(|c| c.faces.clone()).collect(),
        }
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct RecoveryPolicy {
    pub every_iterations: usize,
    pub every_seconds: u64,
    pub stop_on_error: bool,
    pub max_history_frames: usize,
    pub max_history_bytes: u64,
}
impl Default for RecoveryPolicy {
    fn default() -> Self {
        Self {
            every_iterations: 100,
            every_seconds: 60,
            stop_on_error: true,
            max_history_frames: 500,
            max_history_bytes: 1024 * 1024 * 1024,
        }
    }
}
impl RecoveryPolicy {
    pub fn validate(&self) -> Result<(), String> {
        if self.every_iterations > 1_000_000
            || self.every_seconds > 86_400
            || self.max_history_frames > 1_000_000
        {
            return Err("Recovery interval must be 0..1000000 iterations, 0..86400 seconds; history limit 0..1000000 frames".into());
        }
        Ok(())
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CheckpointInfo {
    pub path: PathBuf,
    pub format: u32,
    pub iteration: usize,
    pub physical_time: Option<f64>,
    pub bytes: u64,
    pub complete: bool,
    pub checksum_verified: bool,
    pub backward_history: bool,
}
pub struct RecoveredCheckpoint {
    pub info: CheckpointInfo,
    pub fields: Initial,
    pub warnings: Vec<String>,
}
#[derive(Serialize, Deserialize)]
struct StoredTurbulence {
    #[serde(default)]
    model: crate::turbulence::TurbulenceModel,
    k: Vec<f64>,
    omega: Vec<f64>,
    nut: Vec<f64>,
    #[serde(default)]
    epsilon: Option<Vec<f64>>,
    #[serde(default)]
    nu_tilda: Option<Vec<f64>>,
}
#[derive(Serialize, Deserialize)]
struct StoredState {
    velocity: Vec<Vec3>,
    pressure: Vec<f64>,
    #[serde(default)]
    time: Option<f64>,
    #[serde(default)]
    last_dt: Option<f64>,
    #[serde(default)]
    face_flux: Option<Vec<f64>>,
    #[serde(default)]
    turbulence: Option<StoredTurbulence>,
    #[serde(default)]
    temperature: Option<Vec<f64>>,
}
#[derive(Serialize, Deserialize)]
struct FieldCheckpoint {
    format: u32,
    mesh: MeshSignature,
    iteration: usize,
    context: String,
    #[serde(flatten)]
    state: StoredState,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    previous: Option<StoredState>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    checksum: Option<String>,
}
struct HashWriter(Sha256);
impl Write for HashWriter {
    fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
        self.0.update(b);
        Ok(b.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}
fn digest(record: &FieldCheckpoint) -> Result<String, String> {
    let mut writer = HashWriter(Sha256::new());
    serde_json::to_writer(&mut writer, record).map_err(|e| e.to_string())?;
    Ok(format!("{:x}", writer.0.finalize()))
}
fn validate_state(s: &StoredState, cells: usize, faces: usize) -> Result<(), String> {
    if s.velocity.len() != cells
        || s.pressure.len() != cells
        || s.pressure.iter().any(|v| !v.is_finite())
        || s.velocity
            .iter()
            .any(|v| ![v.x, v.y, v.z].iter().all(|v| v.is_finite()))
    {
        return Err(
            "Checkpoint velocity/pressure fields are invalid or do not match the mesh".into(),
        );
    }
    if s.time.is_some_and(|v| !v.is_finite() || v < 0.)
        || s.last_dt.is_some_and(|v| !v.is_finite() || v <= 0.)
    {
        return Err("Checkpoint physical time or step size is invalid".into());
    }
    if let Some(phi) = &s.face_flux {
        if phi.len() != faces || phi.iter().any(|v| !v.is_finite()) {
            return Err("Invalid checkpoint face flux".into());
        }
    }
    if let Some(t) = &s.temperature {
        if t.len() != cells || t.iter().any(|v| !v.is_finite() || *v <= 0.) {
            return Err("Invalid checkpoint temperature".into());
        }
    }
    if let Some(t) = &s.turbulence {
        for field in [&t.k, &t.omega, &t.nut] {
            if field.len() != cells || field.iter().any(|v| !v.is_finite() || *v < 0.) {
                return Err("Invalid checkpoint turbulence field".into());
            }
        }
        for field in [&t.epsilon, &t.nu_tilda].into_iter().flatten() {
            if field.len() != cells || field.iter().any(|v| !v.is_finite() || *v < 0.) {
                return Err("Invalid checkpoint turbulence field".into());
            }
        }
        if t.model == crate::turbulence::TurbulenceModel::SpalartAllmaras && t.nu_tilda.is_none() {
            return Err("SA checkpoint requires nu_tilda".into());
        }
        if t.model.uses_epsilon() && t.epsilon.is_none() {
            return Err("k-epsilon checkpoint requires epsilon".into());
        }
    }
    Ok(())
}
fn validate(record: &FieldCheckpoint) -> Result<(), String> {
    validate_state(
        &record.state,
        record.mesh.cells.len(),
        record.mesh.faces.len(),
    )?;
    if let Some(previous) = &record.previous {
        validate_state(previous, record.mesh.cells.len(), record.mesh.faces.len())?;
        if previous.face_flux.is_none()
            || !matches!((previous.time,record.state.time),(Some(a),Some(b)) if a<b)
            || record.state.last_dt.is_none()
            || previous.temperature.is_some() != record.state.temperature.is_some()
            || previous.turbulence.as_ref().map(|t| t.model)
                != record.state.turbulence.as_ref().map(|t| t.model)
        {
            return Err("Checkpoint contains incompatible BDF2 history".into());
        }
        let dt = record.state.last_dt.unwrap();
        let elapsed = record.state.time.unwrap() - previous.time.unwrap();
        if (elapsed - dt).abs() > 1e-9 * elapsed.abs().max(dt).max(1e-30) {
            return Err("BDF2 history does not match the last physical step size".into());
        }
    }
    Ok(())
}
fn read(path: &Path) -> Result<FieldCheckpoint, String> {
    let file = std::fs::File::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut record: FieldCheckpoint = serde_json::from_reader(std::io::BufReader::new(file))
        .map_err(|e| format!("{}: {e}", path.display()))?;
    match record.format {
        1 => {}
        2 => {
            let sum = record
                .checksum
                .take()
                .ok_or("Checkpoint checksum is missing")?;
            if sum != digest(&record)? {
                return Err(format!("Checkpoint checksum mismatch: {}", path.display()));
            }
            record.checksum = Some(sum);
        }
        n => return Err(format!("Unsupported checkpoint format {n}")),
    }
    validate(&record)?;
    Ok(record)
}
fn info(path: &Path, record: &FieldCheckpoint) -> Result<CheckpointInfo, String> {
    Ok(CheckpointInfo {
        path: path.into(),
        format: record.format,
        iteration: record.iteration,
        physical_time: record.state.time,
        bytes: std::fs::metadata(path).map_err(|e| e.to_string())?.len(),
        complete: record.state.face_flux.is_some(),
        checksum_verified: record.format == 2,
        backward_history: record.previous.is_some(),
    })
}
pub fn inspect(path: &Path) -> Result<CheckpointInfo, String> {
    let record = read(path)?;
    info(path, &record)
}
pub fn backup_paths(path: &Path) -> [PathBuf; 2] {
    [1, 2].map(|i| {
        let mut name = path.as_os_str().to_owned();
        name.push(format!(".previous-{i}"));
        PathBuf::from(name)
    })
}
fn durable_copy(from: &Path, to: &Path) -> Result<(), String> {
    let parent = to.parent().unwrap_or(Path::new("."));
    let mut temp = tempfile::NamedTempFile::new_in(parent).map_err(|e| e.to_string())?;
    std::io::copy(
        &mut std::fs::File::open(from).map_err(|e| e.to_string())?,
        &mut temp,
    )
    .map_err(|e| e.to_string())?;
    temp.as_file().sync_all().map_err(|e| e.to_string())?;
    temp.persist(to).map_err(|e| e.to_string())?;
    Ok(())
}
fn persist(
    path: &Path,
    mut record: FieldCheckpoint,
    limit: Option<u64>,
) -> Result<CheckpointInfo, String> {
    validate(&record)?;
    record.checksum = Some(digest(&record)?);
    let parent = path.parent().unwrap_or(Path::new("."));
    std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    let mut temp = tempfile::NamedTempFile::new_in(parent).map_err(|e| e.to_string())?;
    {
        let mut writer = std::io::BufWriter::new(temp.as_file_mut());
        serde_json::to_writer(&mut writer, &record).map_err(|e| e.to_string())?;
        writer.flush().map_err(|e| e.to_string())?;
    }
    if limit.is_some_and(|max| max > 0 && temp.as_file().metadata().map_or(true, |m| m.len() > max))
    {
        return Err(
            "A field-history frame exceeds the storage budget; recovery checkpoints remain enabled"
                .into(),
        );
    }
    temp.as_file().sync_all().map_err(|e| e.to_string())?;
    if path
        .file_name()
        .is_some_and(|n| n.to_string_lossy().ends_with("checkpoint.json"))
        && path.exists()
        && read(path).is_ok()
    {
        let [first, second] = backup_paths(path);
        if first.exists() && read(&first).is_ok() {
            durable_copy(&first, &second)?;
        }
        durable_copy(path, &first)?;
    }
    temp.persist(path).map_err(|e| e.to_string())?;
    #[cfg(unix)]
    std::fs::File::open(parent)
        .and_then(|f| f.sync_all())
        .map_err(|e| e.to_string())?;
    info(path, &record)
}
fn store(initial: &Initial, physical: bool) -> Result<StoredState, String> {
    let Initial::Restart {
        velocity,
        pressure,
        face_flux,
        time,
        last_dt,
        temperature,
        turbulence,
        ..
    } = initial
    else {
        return Err("A complete restart state is required".into());
    };
    if velocity.mesh_id() != pressure.mesh_id() || velocity.mesh_id() != face_flux.mesh_id() {
        return Err("Restart fields belong to different meshes".into());
    }
    if temperature
        .as_ref()
        .is_some_and(|t| t.mesh_id() != velocity.mesh_id())
        || turbulence.as_ref().is_some_and(|t| {
            t.k.mesh_id() != velocity.mesh_id()
                || t.omega.mesh_id() != velocity.mesh_id()
                || t.nut.mesh_id() != velocity.mesh_id()
                || t.epsilon
                    .as_ref()
                    .is_some_and(|f| f.mesh_id() != velocity.mesh_id())
                || t.nu_tilda
                    .as_ref()
                    .is_some_and(|f| f.mesh_id() != velocity.mesh_id())
        })
    {
        return Err("Restart auxiliary fields belong to a different mesh".into());
    }
    Ok(StoredState {
        velocity: velocity.values().to_vec(),
        pressure: pressure.values().to_vec(),
        face_flux: Some(face_flux.values().to_vec()),
        time: physical.then_some(*time),
        last_dt: if physical { *last_dt } else { None },
        temperature: temperature.as_ref().map(|t| t.values().to_vec()),
        turbulence: turbulence.as_ref().map(|t| StoredTurbulence {
            model: t.model,
            k: t.k.values().to_vec(),
            omega: t.omega.values().to_vec(),
            nut: t.nut.values().to_vec(),
            epsilon: t.epsilon.as_ref().map(|v| v.values().to_vec()),
            nu_tilda: t.nu_tilda.as_ref().map(|v| v.values().to_vec()),
        }),
    })
}
fn record(
    mesh: &MeshSignature,
    iteration: usize,
    initial: &Initial,
    physical: bool,
    context: &str,
) -> Result<FieldCheckpoint, String> {
    let previous = if let Initial::Restart { previous, .. } = initial {
        previous.as_deref().map(|v| store(v, true)).transpose()?
    } else {
        None
    };
    Ok(FieldCheckpoint {
        format: 2,
        mesh: mesh.clone(),
        iteration,
        context: context.into(),
        state: store(initial, physical)?,
        previous,
        checksum: None,
    })
}
pub fn write_solution(
    path: &Path,
    mesh: &MeshSignature,
    iteration: usize,
    solution: &crate::IncompressibleSolution,
    context: &str,
) -> Result<(), String> {
    write_restart(
        path,
        mesh,
        iteration,
        &solution.restart(),
        solution.physical_time.is_some(),
        context,
    )
}
pub fn write_restart(
    path: &Path,
    mesh: &MeshSignature,
    iteration: usize,
    initial: &Initial,
    physical: bool,
    context: &str,
) -> Result<(), String> {
    let _timer = crate::compute::StageTimer::new(crate::compute::Stage::MonitorOutput);
    persist(
        path,
        record(mesh, iteration, initial, physical, context)?,
        None,
    )
    .map(|_| ())
}
pub fn write_history(
    path: &Path,
    mesh: &MeshSignature,
    iteration: usize,
    initial: &Initial,
    physical: bool,
    context: &str,
    policy: &RecoveryPolicy,
) -> Result<(), String> {
    persist(
        path,
        record(mesh, iteration, initial, physical, context)?,
        Some(policy.max_history_bytes),
    )?;
    prune_history(path.parent().unwrap_or(Path::new(".")), policy)
}
pub fn write(
    path: &Path,
    mesh: &MeshSignature,
    iteration: usize,
    velocity: &[Vec3],
    pressure: &[f64],
    context: &str,
) -> Result<(), String> {
    persist(
        path,
        FieldCheckpoint {
            format: 2,
            mesh: mesh.clone(),
            iteration,
            context: context.into(),
            state: StoredState {
                velocity: velocity.into(),
                pressure: pressure.into(),
                time: None,
                last_dt: None,
                face_flux: None,
                turbulence: None,
                temperature: None,
            },
            previous: None,
            checksum: None,
        },
        None,
    )
    .map(|_| ())
}
fn restore(
    s: StoredState,
    mesh: &UnstructuredMesh,
    previous: Option<Box<Initial>>,
) -> Result<Initial, String> {
    let velocity = CellField::from_values(mesh, s.velocity).map_err(|e| format!("{e:?}"))?;
    let pressure = CellField::from_values(mesh, s.pressure).map_err(|e| format!("{e:?}"))?;
    if let Some(phi) = s.face_flux {
        let turbulence = s
            .turbulence
            .map(|t| -> Result<_, String> {
                Ok(crate::turbulence::TurbulenceFields {
                    model: t.model,
                    k: CellField::from_values(mesh, t.k).map_err(|e| format!("{e:?}"))?,
                    omega: CellField::from_values(mesh, t.omega).map_err(|e| format!("{e:?}"))?,
                    nut: CellField::from_values(mesh, t.nut).map_err(|e| format!("{e:?}"))?,
                    epsilon: t
                        .epsilon
                        .map(|v| CellField::from_values(mesh, v).map_err(|e| format!("{e:?}")))
                        .transpose()?,
                    nu_tilda: t
                        .nu_tilda
                        .map(|v| CellField::from_values(mesh, v).map_err(|e| format!("{e:?}")))
                        .transpose()?,
                })
            })
            .transpose()?;
        Ok(Initial::Restart {
            velocity,
            pressure,
            face_flux: crate::FaceField::from_values(mesh, phi).map_err(|e| format!("{e:?}"))?,
            time: s.time.unwrap_or(0.),
            last_dt: s.last_dt,
            turbulence,
            temperature: s
                .temperature
                .map(|v| CellField::from_values(mesh, v).map_err(|e| format!("{e:?}")))
                .transpose()?,
            previous,
        })
    } else {
        Ok(Initial::Fields { velocity, pressure })
    }
}
pub fn load(path: &Path, mesh: &UnstructuredMesh) -> Result<(usize, Initial), String> {
    let record = read(path)?;
    if record.mesh != MeshSignature::new(mesh) {
        return Err(
            "Checkpoint geometry, topology or cell ordering does not match the current mesh".into(),
        );
    }
    let previous = record
        .previous
        .map(|s| restore(s, mesh, None).map(Box::new))
        .transpose()?;
    Ok((record.iteration, restore(record.state, mesh, previous)?))
}
pub fn load_recoverable(
    path: &Path,
    mesh: &UnstructuredMesh,
) -> Result<RecoveredCheckpoint, String> {
    let mut warnings = Vec::new();
    for candidate in std::iter::once(path.to_path_buf()).chain(backup_paths(path)) {
        if !candidate.exists() {
            continue;
        }
        match load(&candidate, mesh) {
            Ok((_, fields)) => {
                return Ok(RecoveredCheckpoint {
                    info: inspect(&candidate)?,
                    fields,
                    warnings,
                });
            }
            Err(e) => warnings.push(e),
        }
    }
    Err(format!(
        "No valid recovery checkpoint at {}: {}",
        path.display(),
        warnings.join("; ")
    ))
}
pub fn recover(folder: &Path, mesh: &UnstructuredMesh) -> Result<RecoveredCheckpoint, String> {
    let mut candidates = Vec::new();
    let mut errors = Vec::new();
    for name in ["checkpoint.json", "retained-checkpoint.json"] {
        if name == "retained-checkpoint.json" && folder.join("failed-last-sample.json").exists() {
            continue;
        }
        let path = folder.join(name);
        if !path.exists() && !backup_paths(&path).iter().any(|p| p.exists()) {
            continue;
        }
        match load_recoverable(&path, mesh) {
            Ok(c) => candidates.push(c),
            Err(e) => errors.push(e),
        }
    }
    candidates.sort_by_key(|c| c.info.iteration);
    let mut result = candidates.pop().ok_or_else(|| errors.join("; "))?;
    result.warnings.extend(errors);
    Ok(result)
}
pub fn discover(workspace: &Path, mesh: &UnstructuredMesh) -> Result<RecoveredCheckpoint, String> {
    let mut folders = std::fs::read_dir(workspace.join("monitors"))
        .map_err(|e| e.to_string())?
        .filter_map(Result::ok)
        .filter(|e| e.file_type().is_ok_and(|t| t.is_dir()))
        .map(|e| e.path())
        .collect::<Vec<_>>();
    folders
        .sort_by_key(|p| std::cmp::Reverse(std::fs::metadata(p).and_then(|m| m.modified()).ok()));
    let mut errors = Vec::new();
    for folder in folders {
        match recover(&folder, mesh) {
            Ok(mut r) => {
                r.warnings.extend(errors);
                return Ok(r);
            }
            Err(e) => errors.push(e),
        }
    }
    Err(format!(
        "No compatible recovery checkpoint: {}",
        errors.join("; ")
    ))
}
pub fn prune_history(folder: &Path, policy: &RecoveryPolicy) -> Result<(), String> {
    let mut files = std::fs::read_dir(folder)
        .map_err(|e| e.to_string())?
        .filter_map(Result::ok)
        .filter(|e| {
            e.file_type().is_ok_and(|t| t.is_file())
                && e.file_name().to_string_lossy().starts_with("iteration-")
                && e.path().extension().is_some_and(|e| e == "json")
        })
        .map(|e| {
            e.metadata()
                .map(|m| (e.path(), m.len()))
                .map_err(|e| e.to_string())
        })
        .collect::<Result<Vec<_>, _>>()?;
    files.sort_by(|a, b| a.0.cmp(&b.0));
    let mut bytes = files.iter().map(|(_, n)| n).sum::<u64>();
    let mut count = files.len();
    for (path, n) in files {
        if count <= 1
            || ((policy.max_history_frames == 0 || count <= policy.max_history_frames)
                && (policy.max_history_bytes == 0 || bytes <= policy.max_history_bytes))
        {
            break;
        }
        std::fs::remove_file(path).map_err(|e| e.to_string())?;
        bytes -= n;
        count -= 1;
    }
    Ok(())
}
pub struct RecoveryWriter {
    path: PathBuf,
    signature: std::sync::OnceLock<std::sync::Arc<MeshSignature>>,
    context: String,
    physical: bool,
    policy: RecoveryPolicy,
    state: std::sync::Mutex<WriterState>,
}
struct WriterState {
    last: Option<CheckpointInfo>,
    saved_at: std::time::Instant,
    error: Option<(String, std::time::Instant)>,
}
impl RecoveryWriter {
    pub fn new(path: PathBuf, context: String, physical: bool, policy: RecoveryPolicy) -> Self {
        Self {
            path,
            context,
            physical,
            policy,
            signature: Default::default(),
            state: std::sync::Mutex::new(WriterState {
                last: None,
                saved_at: std::time::Instant::now(),
                error: None,
            }),
        }
    }
    pub fn set_mesh(&self, signature: std::sync::Arc<MeshSignature>) {
        let _ = self.signature.set(signature);
    }
    pub fn save(
        &self,
        index: usize,
        fields: &Initial,
        force: bool,
    ) -> Result<Option<CheckpointInfo>, String> {
        let mut state = self
            .state
            .lock()
            .map_err(|_| "Checkpoint writer lock failed")?;
        if let Some((error, when)) = &state.error {
            if !force && when.elapsed().as_secs() < 10 {
                return if self.policy.stop_on_error {
                    Err(error.clone())
                } else {
                    Ok(None)
                };
            }
        }
        if let Some(last) = &state.last {
            if last.iteration >= index {
                return Ok(force.then(|| last.clone()));
            }
        }
        let enabled = self.policy.every_iterations > 0 || self.policy.every_seconds > 0;
        let due = force
            || enabled
                && (state.last.is_none()
                    || self.policy.every_iterations > 0
                        && index % self.policy.every_iterations == 0
                    || self.policy.every_seconds > 0
                        && state.saved_at.elapsed().as_secs() >= self.policy.every_seconds);
        if !due {
            return Ok(None);
        }
        let signature = self
            .signature
            .get()
            .ok_or("No accepted mesh is available for checkpointing")?;
        match record(signature, index, fields, self.physical, &self.context)
            .and_then(|r| persist(&self.path, r, None))
        {
            Ok(info) => {
                state.last = Some(info.clone());
                state.saved_at = std::time::Instant::now();
                state.error = None;
                Ok(Some(info))
            }
            Err(e) => {
                state.error = Some((e.clone(), std::time::Instant::now()));
                Err(e)
            }
        }
    }
}

#[cfg(test)]
mod checkpoint_tests {
    use super::*;
    use crate::IncompressibleInitialConditions;
    use crate::{CellDefinition, MeshDimension, Point};
    fn mesh(scale: f64) -> UnstructuredMesh {
        UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0., 0., 0.),
                Point::new(scale, 0., 0.),
                Point::new(0., 1., 0.),
            ],
            vec![CellDefinition::polygon(vec![0, 1, 2])],
        )
        .unwrap()
    }
    #[test]
    fn field_restart_rebinds_fields_to_reloaded_mesh_and_rejects_changed_geometry() {
        let first = mesh(1.0);
        let reloaded = mesh(1.0);
        let folder = tempfile::tempdir().unwrap();
        let path = folder.path().join("checkpoint.json");
        write(
            &path,
            &MeshSignature::new(&first),
            100,
            &[Vec3::new(1.0, 2.0, 3.0)],
            &[-2.5],
            "fixture",
        )
        .unwrap();
        let (iteration, fields) = load(&path, &reloaded).unwrap();
        assert_eq!(iteration, 100);
        match fields {
            IncompressibleInitialConditions::Fields { velocity, pressure } => {
                assert_eq!(velocity.values(), &[Vec3::new(1.0, 2.0, 3.0)]);
                assert_eq!(pressure.values(), &[-2.5]);
            }
            _ => panic!("expected stored fields"),
        }
        assert!(load(&path, &mesh(2.0)).is_err());
    }
    #[test]
    fn atomic_replacement_preserves_last_valid_checkpoint_on_bad_fields() {
        let mesh = mesh(1.0);
        let signature = MeshSignature::new(&mesh);
        let folder = tempfile::tempdir().unwrap();
        let path = folder.path().join("checkpoint.json");
        write(&path, &signature, 1, &[Vec3::ZERO], &[0.0], "fixture").unwrap();
        write(
            &path,
            &signature,
            100,
            &[Vec3::new(1.0, 0.0, 0.0)],
            &[2.0],
            "fixture",
        )
        .unwrap();
        assert!(write(
            &path,
            &signature,
            101,
            &[Vec3::ZERO],
            &[f64::NAN],
            "fixture"
        )
        .is_err());
        assert_eq!(load(&path, &mesh).unwrap().0, 100);
        assert!(write(&path, &signature, 102, &[], &[], "fixture").is_err());
    }
    #[test]
    fn corrupted_primary_falls_back_without_using_failed_sample() {
        let mesh = mesh(1.0);
        let signature = MeshSignature::new(&mesh);
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("checkpoint.json");
        for i in 1..=3 {
            write(
                &path,
                &signature,
                i,
                &[Vec3::new(i as f64, 0., 0.)],
                &[0.],
                "fixture",
            )
            .unwrap();
        }
        write(
            &dir.path().join("failed-last-sample.json"),
            &signature,
            999,
            &[Vec3::ZERO],
            &[0.],
            "failed",
        )
        .unwrap();
        std::fs::write(&path, b"{truncated").unwrap();
        let recovered = recover(dir.path(), &mesh).unwrap();
        assert_eq!(recovered.info.iteration, 2);
        assert!(recovered.info.checksum_verified);
        assert!(!recovered.warnings.is_empty());
        assert!(load_recoverable(&path, &super::checkpoint_tests::mesh(2.)).is_err());
    }
    #[test]
    fn checksum_rejects_valid_json_with_modified_fields_and_reads_legacy() {
        let mesh = mesh(1.);
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("checkpoint.json");
        write(
            &path,
            &MeshSignature::new(&mesh),
            7,
            &[Vec3::new(0.12345678901234567, 0., 0.)],
            &[2.],
            "fixture",
        )
        .unwrap();
        assert!(load(&path, &mesh).is_ok());
        let mut value: serde_json::Value =
            serde_json::from_slice(&std::fs::read(&path).unwrap()).unwrap();
        value["pressure"][0] = serde_json::json!(100.);
        std::fs::write(&path, serde_json::to_vec(&value).unwrap()).unwrap();
        assert!(load(&path, &mesh).unwrap_err().contains("checksum"));
        value["format"] = serde_json::json!(1);
        value.as_object_mut().unwrap().remove("checksum");
        std::fs::write(&path, serde_json::to_vec(&value).unwrap()).unwrap();
        assert!(load(&path, &mesh).is_ok());
        assert!(!inspect(&path).unwrap().checksum_verified);
    }
    #[test]
    fn playback_limits_keep_latest_frames_and_reject_oversized_write() {
        let mesh = mesh(1.);
        let dir = tempfile::tempdir().unwrap();
        let signature = MeshSignature::new(&mesh);
        let fields = Initial::Restart {
            velocity: CellField::from_values(&mesh, vec![Vec3::ZERO]).unwrap(),
            pressure: CellField::from_values(&mesh, vec![0.]).unwrap(),
            face_flux: crate::FaceField::from_values(&mesh, vec![0.; mesh.faces().len()]).unwrap(),
            time: 0.,
            last_dt: None,
            temperature: None,
            turbulence: None,
            previous: None,
        };
        let mut policy = RecoveryPolicy {
            max_history_frames: 2,
            ..Default::default()
        };
        for i in 1..=5 {
            write_history(
                &dir.path().join(format!("iteration-{i:08}.json")),
                &signature,
                i,
                &fields,
                false,
                "fixture",
                &policy,
            )
            .unwrap();
        }
        assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 2);
        assert!(dir.path().join("iteration-00000005.json").exists());
        policy.max_history_bytes = 1;
        assert!(write_history(
            &dir.path().join("iteration-00000006.json"),
            &signature,
            6,
            &fields,
            false,
            "fixture",
            &policy
        )
        .is_err());
        assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 2);
    }
}
