//! Shared GUI/CLI scalar history and restart publication.
use super::*;
use std::{
    fs,
    io::{BufWriter, Write},
    path::{Path, PathBuf},
};
pub struct Archive {
    folder: PathBuf,
    options: Options,
    history: BufWriter<fs::File>,
    frames: Vec<serde_json::Value>,
    last: Option<checkpoint::Checkpoint>,
    last_report: Option<serde_json::Value>,
}
impl Archive {
    pub fn create(
        folder: &Path,
        options: &Options,
        provenance: &serde_json::Value,
    ) -> Result<Self, String> {
        fs::create_dir(folder).map_err(|e| format!("Choose a new run directory: {e}"))?;
        write_json(&folder.join("resolved-transport.json"), options)?;
        write_json(&folder.join("catalog-provenance.json"), provenance)?;
        let history = BufWriter::new(
            fs::OpenOptions::new()
                .create_new(true)
                .write(true)
                .open(folder.join("history.jsonl"))
                .map_err(|e| e.to_string())?,
        );
        Ok(Self {
            folder: folder.into(),
            options: options.clone(),
            history,
            frames: Vec::new(),
            last: None,
            last_report: None,
        })
    }
    pub fn last_checkpoint(&self) -> Option<&checkpoint::Checkpoint> {
        self.last.as_ref()
    }
    pub fn record(
        &mut self,
        mesh: &UnstructuredMesh,
        state: &State,
        report: &StepReport,
    ) -> Result<(), String> {
        serde_json::to_writer(&mut self.history, report).map_err(|e| e.to_string())?;
        writeln!(self.history).map_err(|e| e.to_string())?;
        if self.last.is_none() {
            write_json(
                &self.folder.join("mesh.json"),
                &crate::mesh::MeshSnapshot::capture(mesh),
            )?;
        }
        self.last = Some(checkpoint::Checkpoint::capture(mesh, &self.options, state)?);
        self.last_report = Some(serde_json::to_value(report).map_err(|e| e.to_string())?);
        if state.step % self.options.write_every == 0 {
            self.frame(mesh)?;
        }
        Ok(())
    }
    fn frame(&mut self, mesh: &UnstructuredMesh) -> Result<(), String> {
        let Some(checkpoint) = &self.last else {
            return Ok(());
        };
        let state = &checkpoint.state;
        if self
            .frames
            .last()
            .and_then(|v| v.get("step"))
            .and_then(|v| v.as_u64())
            == Some(state.step as u64)
        {
            return Ok(());
        }
        let name = format!("step-{:08}", state.step);
        let vtk = format!("{name}.vtk");
        let data = format!("{name}.json");
        crate::output::write_transport_vtk(&self.folder.join(&vtk), mesh, state)?;
        write_json(&self.folder.join(&data), checkpoint)?;
        let mut csv = BufWriter::new(
            fs::OpenOptions::new()
                .create_new(true)
                .write(true)
                .open(self.folder.join(format!("{name}.csv")))
                .map_err(|e| e.to_string())?,
        );
        writeln!(
            csv,
            "cell,x_m,y_m,z_m,time_s,phase_fraction,temperature_K,density_kg_m3,solid"
        )
        .map_err(|e| e.to_string())?;
        for (i, c) in mesh.cells().iter().enumerate() {
            writeln!(
                csv,
                "{i},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{}",
                c.center.x,
                c.center.y,
                c.center.z,
                state.time,
                state.phase_fraction[i],
                state.temperature[i],
                state.density[i],
                u8::from(state.solid[i])
            )
            .map_err(|e| e.to_string())?;
        }
        csv.flush().map_err(|e| e.to_string())?;
        let range = |v: &[f64]| {
            [
                v.iter().copied().fold(f64::INFINITY, f64::min),
                v.iter().copied().fold(f64::NEG_INFINITY, f64::max),
            ]
        };
        self.frames.push(serde_json::json!({"name":vtk,"state":data,"time":state.time,"step":state.step,"ranges":[range(&state.temperature),range(&state.phase_fraction),range(&state.density),[0.0,1.0]]}));
        write_json(&self.folder.join("frames.json"), &self.frames)?;
        write_json(
            &self.folder.join("transport.vtk.series"),
            &serde_json::json!({"file-series-version":"1.0","files":self.frames.iter().map(|f|serde_json::json!({"name":f["name"],"time":f["time"]})).collect::<Vec<_>>()}),
        )?;
        checkpoint.save(&self.folder.join("checkpoint.json"))?;
        self.history.flush().map_err(|e| e.to_string())
    }
    pub fn finish(
        &mut self,
        mesh: &UnstructuredMesh,
        status: &str,
        message: &str,
    ) -> Result<serde_json::Value, String> {
        if let Some(c) = &self.last {
            c.save(&self.folder.join("checkpoint.json"))?;
        }
        self.history.flush().map_err(|e| e.to_string())?;
        self.frame(mesh)?;
        if let Some(c) = &self.last {
            c.save(&self.folder.join("checkpoint.json"))?;
            if status == "completed" {
                write_json(&self.folder.join("final-state.json"), &c.state)?;
            }
        }
        let report = serde_json::json!({"status":status,"message":message,"last_step":self.last_report,"coordinates":self.options.coordinates,"mesh_signature":mesh_signature(mesh),"model":"prescribed-flux phase and sensible-energy transport","capabilities":{"variable_density_momentum":false,"capillary_force":false,"axisymmetric_momentum":false},"frames":self.frames.len()});
        write_json(&self.folder.join("report.json"), &report)?;
        Ok(report)
    }
}
fn write_json(path: &Path, value: &impl Serialize) -> Result<(), String> {
    let temp = path.with_extension("tmp");
    fs::write(&temp, serde_json::to_vec(value).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())?;
    fs::rename(temp, path).map_err(|e| e.to_string())
}

pub fn load_mesh(folder: &Path) -> Result<UnstructuredMesh, String> {
    let snapshot: crate::mesh::MeshSnapshot =
        serde_json::from_slice(&fs::read(folder.join("mesh.json")).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
    snapshot.restore()
}
