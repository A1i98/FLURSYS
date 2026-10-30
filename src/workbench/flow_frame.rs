use super::{
    checkpoint::{CheckpointInfo, MeshSignature},
    ResultDataset,
};
use crate::{compressible, mesh::MeshSnapshot, UnstructuredMesh, Vec3};
use serde::{Deserialize, Serialize};
use std::path::Path;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "model", rename_all = "kebab-case")]
pub enum FlowFields {
    Incompressible {
        pressure: Vec<f64>,
        velocity: Vec<Vec3>,
        temperature: Option<Vec<f64>>,
    },
    Compressible {
        state: compressible::Snapshot,
    },
}
#[derive(Clone, Serialize, Deserialize)]
pub struct FlowFrame {
    pub run_id: String,
    pub step: usize,
    pub time: Option<f64>,
    pub mesh: MeshSnapshot,
    pub signature: MeshSignature,
    pub fields: FlowFields,
}
#[derive(Serialize, Deserialize)]
struct Envelope {
    format: String,
    checksum: String,
    payload: FlowFrame,
}
impl FlowFrame {
    pub fn compressible(
        run_id: impl Into<String>,
        mesh: &UnstructuredMesh,
        state: compressible::Snapshot,
    ) -> Self {
        Self {
            run_id: run_id.into(),
            step: state.report.accepted_steps,
            time: Some(state.report.time),
            mesh: MeshSnapshot::capture(mesh),
            signature: MeshSignature::new(mesh),
            fields: FlowFields::Compressible { state },
        }
    }
    pub fn dataset(&self) -> Result<ResultDataset, String> {
        let mesh = self.mesh.clone().restore()?;
        self.dataset_on(&mesh)
    }
    pub fn dataset_on(&self, mesh: &UnstructuredMesh) -> Result<ResultDataset, String> {
        if self.signature != MeshSignature::new(mesh) {
            return Err("Field frame belongs to a different mesh".into());
        }
        match &self.fields {
            FlowFields::Compressible { state } => state.dataset(&self.run_id, mesh),
            FlowFields::Incompressible {
                pressure,
                velocity,
                temperature,
            } => ResultDataset::from_values(&self.run_id, mesh, pressure.clone(), velocity.clone())
                .and_then(|d| d.with_temperature(temperature.clone()))
                .map_err(|e| e.to_string()),
        }
    }
    pub fn gas_restart(
        &self,
        mesh: &UnstructuredMesh,
        gas: compressible::IdealGas,
    ) -> Result<compressible::Snapshot, String> {
        if self.signature != MeshSignature::new(mesh) {
            return Err(
                "Checkpoint belongs to a different mesh; Initialize after remeshing".into(),
            );
        }
        match &self.fields {
            FlowFields::Compressible { state } => {
                state.validate(mesh, gas)?;
                Ok(state.clone())
            }
            _ => Err("Checkpoint belongs to an incompressible model".into()),
        }
    }
    fn validate(&self) -> Result<(), String> {
        if self.run_id.trim().is_empty() || self.time.is_some_and(|t| !t.is_finite() || t < 0.) {
            return Err("Invalid saved frame identity or time".into());
        }
        match &self.fields {
            FlowFields::Compressible { state } => {
                state.gas.validate()?;
                if state.fields.len() != self.mesh.cell_count()
                    || self.step != state.report.accepted_steps
                    || self.time != Some(state.report.time)
                {
                    return Err("Saved frame field counts, step or time disagree".into());
                }
                if state.fields.iter().any(|p| {
                    !p.density.is_finite()
                        || p.density <= 0.
                        || !p.pressure.is_finite()
                        || p.pressure <= 0.
                        || ![p.velocity.x, p.velocity.y, p.velocity.z]
                            .iter()
                            .all(|v| v.is_finite())
                }) {
                    return Err("Unphysical saved compressible frame".into());
                }
            }
            FlowFields::Incompressible {
                pressure,
                velocity,
                temperature,
            } => {
                let n = self.mesh.cell_count();
                if pressure.len() != n
                    || velocity.len() != n
                    || pressure.iter().any(|v| !v.is_finite())
                    || velocity
                        .iter()
                        .any(|v| ![v.x, v.y, v.z].iter().all(|v| v.is_finite()))
                    || temperature.as_ref().is_some_and(|v| {
                        v.len() != n || v.iter().any(|v| !v.is_finite() || *v <= 0.)
                    })
                {
                    return Err("Invalid saved incompressible frame".into());
                }
            }
        }
        Ok(())
    }
    pub fn write(&self, path: &Path, limit: Option<u64>) -> Result<CheckpointInfo, String> {
        self.validate()?;
        let envelope = Envelope {
            format: "flursys-flow-frame/1".into(),
            checksum: crate::storage::digest(self)?,
            payload: self.clone(),
        };
        let bytes = crate::storage::write_json(path, &envelope, limit, || {
            if path
                .file_name()
                .is_some_and(|p| p.to_string_lossy().ends_with("checkpoint.json"))
                && Self::read(path).is_ok()
            {
                let [first, second] = super::checkpoint::backup_paths(path);
                if Self::read(&first).is_ok() {
                    super::checkpoint::durable_copy(&first, &second)?;
                }
                super::checkpoint::durable_copy(path, &first)?;
            }
            Ok(())
        })?;
        Ok(self.info(path, bytes))
    }
    pub fn read(path: &Path) -> Result<Self, String> {
        let envelope: Envelope =
            serde_json::from_reader(std::fs::File::open(path).map_err(|e| e.to_string())?)
                .map_err(|e| e.to_string())?;
        if envelope.format != "flursys-flow-frame/1"
            || envelope.checksum != crate::storage::digest(&envelope.payload)?
        {
            return Err("Frame format or checksum mismatch".into());
        }
        envelope.payload.validate()?;
        Ok(envelope.payload)
    }
    pub fn info(&self, path: &Path, bytes: u64) -> CheckpointInfo {
        CheckpointInfo {
            path: path.into(),
            format: 3,
            iteration: self.step,
            physical_time: self.time,
            bytes,
            complete: true,
            checksum_verified: true,
            backward_history: false,
        }
    }
}
pub fn discover(
    workspace: &Path,
    mesh: &UnstructuredMesh,
    gas: compressible::IdealGas,
) -> Result<std::path::PathBuf, String> {
    for folder in super::checkpoint::recovery_folders(workspace)? {
        let path = folder.join("checkpoint.json");
        for candidate in std::iter::once(path.clone()).chain(super::checkpoint::backup_paths(&path))
        {
            if let Ok(frame) = FlowFrame::read(&candidate) {
                if frame.gas_restart(mesh, gas).is_ok() {
                    return Ok(candidate);
                }
            }
        }
    }
    Err("No compatible recovery checkpoint".into())
}
pub fn is_frame(path: &Path) -> bool {
    use std::io::Read;
    let mut prefix = [0; 80];
    std::fs::File::open(path)
        .and_then(|mut f| f.read(&mut prefix))
        .ok()
        .is_some_and(|n| String::from_utf8_lossy(&prefix[..n]).contains("flursys-flow-frame/1"))
}

pub fn finalize_compressible(
    workspace: &Path,
    project: &mut super::WorkbenchProject,
    mesh: &UnstructuredMesh,
    state: &compressible::Snapshot,
    recording: &Path,
    continuation: Option<&str>,
    histories: &[Vec<(f64, Option<f64>)>; 4],
    failure: Option<String>,
) -> Result<super::RunRecord, String> {
    use super::RunStatus;
    let status = if failure.is_some() {
        RunStatus::Failed
    } else if state.report.stopped {
        RunStatus::Stopped
    } else if state.report.reached_end_time {
        RunStatus::Completed
    } else {
        RunStatus::MaxIterations
    };
    let mut record = match continuation {
        Some(id) => project
            .runs
            .iter()
            .find(|r| r.id == id)
            .cloned()
            .ok_or("Continuation run is missing")?,
        None => project.next_run(status.clone()),
    };
    record.status = status;
    record.failure_diagnostic = failure.clone();
    record.iterations = Some(state.report.accepted_steps);
    record.time_steps = state.report.accepted_steps;
    record.physical_time = Some(state.report.time);
    record.continuity_residual = Some(state.report.residuals[0]);
    record.mesh_identity = Some(mesh.id().get());
    let folder = std::path::PathBuf::from(format!("runs/run-{:04}", record.ordinal));
    let solution = folder.join("fields.json");
    FlowFrame::compressible(&record.id, mesh, state.clone())
        .write(&workspace.join(&solution), None)?;
    record.solution_path = Some(solution);
    record.report_path = folder.join("report.json");
    let recording = recording
        .strip_prefix(workspace)
        .map_err(|_| "Recording path must be inside the workspace")?;
    let mut settings = project
        .solver
        .compressible
        .clone()
        .ok_or("Missing gas model")?;
    settings.scripting = project.solver.scripting.clone();
    let walls = compressible::wall_reports(mesh, &settings, state, true)?;
    let report = serde_json::json!({"run":record,"model":"compressible","solver_settings":project.solver,"compressible_report":state.report,"wall_reports":walls,"indexed_convergence_history":histories,"plane_recording_folder":recording,"failure":failure});
    crate::storage::write_json(&workspace.join(&record.report_path), &report, None, || {
        Ok(())
    })?;
    if let Some(old) = project.runs.iter_mut().find(|r| r.id == record.id) {
        *old = record.clone();
    } else {
        project.runs.push(record.clone());
    }
    super::save_workspace(workspace, project).map_err(|e| e.to_string())?;
    Ok(record)
}
