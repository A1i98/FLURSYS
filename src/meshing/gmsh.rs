//! Command-line Gmsh backend configuration and invocation.

use super::{GmshGeoDocument, MeshingError};
use crate::{load_gmsh, MeshDimension, UnstructuredMesh};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::thread;
use std::time::{Duration, SystemTime};

const TEMP_PREFIX: &str = "flursys-gmsh-";
const STALE_WORKSPACE_AGE: Duration = Duration::from_secs(24 * 60 * 60);

/// Ensure an interrupted meshing job cannot outlive its temporary input files.
struct GmshChild {
    process: Child,
    reaped: bool,
}

impl Drop for GmshChild {
    fn drop(&mut self) {
        if !self.reaped {
            let _ = self.process.kill();
            let _ = self.process.wait();
        }
    }
}

/// Only remove this application's old, process-owned scratch directories. The
/// project workspace and saved run artifacts are never considered here.
#[cfg(target_os = "linux")]
fn prune_stale_gmsh_workspaces(root: &Path, now: SystemTime) {
    let Ok(entries) = fs::read_dir(root) else {
        return;
    };
    for entry in entries.flatten() {
        let name = entry.file_name();
        let Some(name) = name.to_str() else { continue };
        let Some((pid, _random)) = name
            .strip_prefix(TEMP_PREFIX)
            .and_then(|name| name.split_once('-'))
        else {
            continue;
        };
        let Ok(pid) = pid.parse::<u32>() else {
            continue;
        };
        if Path::new("/proc").join(pid.to_string()).exists() {
            continue;
        }
        let path = entry.path();
        let Ok(metadata) = fs::symlink_metadata(&path) else {
            continue;
        };
        if !metadata.is_dir()
            || !metadata
                .modified()
                .ok()
                .and_then(|modified| now.duration_since(modified).ok())
                .is_some_and(|age| age >= STALE_WORKSPACE_AGE)
        {
            continue;
        }
        let Ok(files) = fs::read_dir(&path) else {
            continue;
        };
        let mut safe = true;
        for file in files {
            let Ok(file) = file else {
                safe = false;
                break;
            };
            let file_name = file.file_name();
            let Some(file_name) = file_name.to_str() else {
                safe = false;
                break;
            };
            if !matches!(
                file_name,
                "case.geo" | "case.msh" | "gmsh.stdout" | "gmsh.stderr" | "gmsh.pid"
            ) || !fs::symlink_metadata(file.path()).is_ok_and(|metadata| metadata.is_file())
            {
                safe = false;
                break;
            }
            if file_name == "gmsh.pid" {
                let Some(child_pid) = fs::read_to_string(file.path())
                    .ok()
                    .and_then(|text| text.trim().parse::<u32>().ok())
                else {
                    safe = false;
                    break;
                };
                if Path::new("/proc").join(child_pid.to_string()).exists() {
                    safe = false;
                    break;
                }
            }
        }
        if safe {
            let _ = fs::remove_dir_all(path);
        }
    }
}

#[cfg(not(target_os = "linux"))]
fn prune_stale_gmsh_workspaces(_root: &Path, _now: SystemTime) {}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GmshExecutable {
    Auto,
    Path(PathBuf),
}

#[derive(Clone, Debug, PartialEq)]
pub struct GmshMeshOptions {
    pub dimension: MeshDimension,
    pub characteristic_length: f64,
    pub min_size: Option<f64>,
    pub max_size: Option<f64>,
    pub element_order: u8,
}

impl GmshMeshOptions {
    pub fn two_d(characteristic_length: f64) -> Result<Self, MeshingError> {
        Self::new(MeshDimension::TwoD, characteristic_length)
    }

    pub fn three_d(characteristic_length: f64) -> Result<Self, MeshingError> {
        Self::new(MeshDimension::ThreeD, characteristic_length)
    }

    pub fn new(dimension: MeshDimension, characteristic_length: f64) -> Result<Self, MeshingError> {
        let options = Self {
            dimension,
            characteristic_length,
            min_size: Some(characteristic_length),
            max_size: Some(characteristic_length),
            element_order: 1,
        };
        options.validate()?;
        Ok(options)
    }

    pub fn validate(&self) -> Result<(), MeshingError> {
        validate_positive_finite("characteristic length", self.characteristic_length)?;
        if let Some(minimum) = self.min_size {
            validate_positive_finite("minimum mesh size", minimum)?;
        }
        if let Some(maximum) = self.max_size {
            validate_positive_finite("maximum mesh size", maximum)?;
        }
        if let (Some(minimum), Some(maximum)) = (self.min_size, self.max_size) {
            if minimum > maximum {
                return Err(MeshingError::InvalidOptions {
                    message: "minimum mesh size must not exceed maximum mesh size".into(),
                });
            }
        }
        if self.element_order != 1 {
            return Err(MeshingError::InvalidOptions {
                message: "only first-order Gmsh elements are supported by the current importer"
                    .into(),
            });
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GmshVersion {
    pub value: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GmshMeshingReport {
    pub version: GmshVersion,
    pub dimension: MeshDimension,
    pub mesh_format: String,
    pub node_count: usize,
    pub cell_count: usize,
    pub patch_count: usize,
    pub stdout: String,
    pub stderr: String,
}

#[derive(Debug)]
pub struct GeneratedMesh {
    pub mesh: UnstructuredMesh,
    pub report: GmshMeshingReport,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GmshMesher {
    executable: GmshExecutable,
}

impl GmshMesher {
    pub fn auto() -> Self {
        Self {
            executable: GmshExecutable::Auto,
        }
    }

    pub fn from_executable(path: impl Into<PathBuf>) -> Self {
        Self {
            executable: GmshExecutable::Path(path.into()),
        }
    }

    pub fn executable(&self) -> &GmshExecutable {
        &self.executable
    }

    pub fn version(&self) -> Result<GmshVersion, MeshingError> {
        let output = self.command().arg("--version").output().map_err(|error| {
            MeshingError::GmshExecutableNotFound {
                executable: self.executable_label(),
                message: error.to_string(),
            }
        })?;
        if !output.status.success() {
            return Err(MeshingError::GmshProcessFailed {
                status: output.status.code(),
                stdout: text(&output.stdout),
                stderr: text(&output.stderr),
            });
        }
        let value = text(&output.stdout).trim().to_owned();
        if value.is_empty() {
            return Err(MeshingError::InvalidGmshVersion {
                stdout: text(&output.stdout),
                stderr: text(&output.stderr),
            });
        }
        Ok(GmshVersion { value })
    }

    pub fn command_arguments(
        &self,
        geo_path: impl AsRef<Path>,
        mesh_path: impl AsRef<Path>,
        options: &GmshMeshOptions,
    ) -> Result<Vec<String>, MeshingError> {
        options.validate()?;
        let dimension = match options.dimension {
            MeshDimension::TwoD => "-2",
            MeshDimension::ThreeD => "-3",
        };
        let minimum = options.min_size.unwrap_or(options.characteristic_length);
        let maximum = options.max_size.unwrap_or(options.characteristic_length);
        Ok(vec![
            geo_path.as_ref().display().to_string(),
            dimension.into(),
            "-format".into(),
            "msh4".into(),
            "-setnumber".into(),
            "Mesh.Binary".into(),
            "0".into(),
            "-o".into(),
            mesh_path.as_ref().display().to_string(),
            "-clscale".into(),
            "1".into(),
            "-clmin".into(),
            minimum.to_string(),
            "-clmax".into(),
            maximum.to_string(),
            "-order".into(),
            options.element_order.to_string(),
        ])
    }

    pub fn generate(
        &self,
        geometry: &GmshGeoDocument,
        options: &GmshMeshOptions,
    ) -> Result<GeneratedMesh, MeshingError> {
        self.generate_with_cancellation(geometry, options, || false)
    }

    /// Generates a mesh while allowing a caller to cooperatively terminate the
    /// owned Gmsh child process. The callback is intentionally UI-agnostic.
    pub fn generate_cancellable(
        &self,
        geometry: &GmshGeoDocument,
        options: &GmshMeshOptions,
        is_cancelled: impl Fn() -> bool,
    ) -> Result<GeneratedMesh, MeshingError> {
        self.generate_with_cancellation(geometry, options, is_cancelled)
    }

    fn generate_with_cancellation(
        &self,
        geometry: &GmshGeoDocument,
        options: &GmshMeshOptions,
        is_cancelled: impl Fn() -> bool,
    ) -> Result<GeneratedMesh, MeshingError> {
        if geometry.dimension() != options.dimension {
            return Err(MeshingError::InvalidOptions {
                message: "geometry and mesh option dimensions differ".into(),
            });
        }
        let version = self.version()?;
        let temporary_root = std::env::temp_dir();
        prune_stale_gmsh_workspaces(&temporary_root, SystemTime::now());
        let workspace = tempfile::Builder::new()
            .prefix(&format!("{TEMP_PREFIX}{}-", std::process::id()))
            .tempdir_in(&temporary_root)
            .map_err(|error| MeshingError::Io {
                message: error.to_string(),
            })?;
        let geo_path = workspace.path().join("case.geo");
        let mesh_path = workspace.path().join("case.msh");
        let stdout_path = workspace.path().join("gmsh.stdout");
        let stderr_path = workspace.path().join("gmsh.stderr");
        fs::write(&geo_path, geometry.to_geo_string()?).map_err(|error| MeshingError::Io {
            message: error.to_string(),
        })?;
        let arguments = self.command_arguments(&geo_path, &mesh_path, options)?;
        let stdout_file = fs::File::create(&stdout_path).map_err(|error| MeshingError::Io {
            message: error.to_string(),
        })?;
        let stderr_file = fs::File::create(&stderr_path).map_err(|error| MeshingError::Io {
            message: error.to_string(),
        })?;
        let process = self
            .command()
            .args(arguments)
            .stdout(Stdio::from(stdout_file))
            .stderr(Stdio::from(stderr_file))
            .spawn()
            .map_err(|error| MeshingError::GmshExecutableNotFound {
                executable: self.executable_label(),
                message: error.to_string(),
            })?;
        let mut child = GmshChild {
            process,
            reaped: false,
        };
        let _ = fs::write(
            workspace.path().join("gmsh.pid"),
            child.process.id().to_string(),
        );
        let status = loop {
            if is_cancelled() {
                return Err(MeshingError::Cancelled);
            }
            if let Some(status) = child.process.try_wait().map_err(|error| MeshingError::Io {
                message: error.to_string(),
            })? {
                child.reaped = true;
                break status;
            }
            thread::sleep(Duration::from_millis(20));
        };
        let stdout = text(&fs::read(&stdout_path).map_err(|error| MeshingError::Io {
            message: error.to_string(),
        })?);
        let stderr = text(&fs::read(&stderr_path).map_err(|error| MeshingError::Io {
            message: error.to_string(),
        })?);
        if !status.success() {
            return Err(MeshingError::GmshProcessFailed {
                status: status.code(),
                stdout,
                stderr,
            });
        }
        if !mesh_path.is_file() {
            return Err(MeshingError::MissingOutputMesh {
                path: mesh_path,
                stdout,
                stderr,
            });
        }
        let mesh_format = verify_ascii_msh4(&mesh_path)?;
        let mesh = load_gmsh(&mesh_path).map_err(|error| MeshingError::GmshImportError {
            message: error.to_string(),
        })?;
        let report = GmshMeshingReport {
            version,
            dimension: options.dimension,
            mesh_format,
            node_count: mesh.points().len(),
            cell_count: mesh.cell_count(),
            patch_count: mesh.boundary_patches().len(),
            stdout,
            stderr,
        };
        Ok(GeneratedMesh { mesh, report })
    }

    fn command(&self) -> Command {
        match &self.executable {
            GmshExecutable::Auto => Command::new("gmsh"),
            GmshExecutable::Path(path) => Command::new(path),
        }
    }

    fn executable_label(&self) -> PathBuf {
        match &self.executable {
            GmshExecutable::Auto => PathBuf::from("gmsh"),
            GmshExecutable::Path(path) => path.clone(),
        }
    }
}

fn validate_positive_finite(label: &str, value: f64) -> Result<(), MeshingError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(MeshingError::InvalidOptions {
            message: format!("{label} must be finite and positive"),
        })
    }
}

fn text(bytes: &[u8]) -> String {
    String::from_utf8_lossy(bytes).into_owned()
}

fn verify_ascii_msh4(path: &Path) -> Result<String, MeshingError> {
    let input = fs::read_to_string(path).map_err(|error| MeshingError::Io {
        message: error.to_string(),
    })?;
    let mut lines = input.lines();
    if lines.next() != Some("$MeshFormat") {
        return Err(MeshingError::InvalidGeneratedMeshFormat {
            path: path.to_path_buf(),
            message: "missing $MeshFormat header".into(),
        });
    }
    let format = lines.next().unwrap_or_default();
    let words = format.split_whitespace().collect::<Vec<_>>();
    if words.len() != 3 || !words[0].starts_with('4') || words[1] != "0" {
        return Err(MeshingError::InvalidGeneratedMeshFormat {
            path: path.to_path_buf(),
            message: format!("expected Gmsh v4 ASCII MeshFormat, got {format:?}"),
        });
    }
    Ok(format.to_owned())
}

#[cfg(all(test, target_os = "linux"))]
mod tests {
    use super::*;

    #[test]
    fn stale_scratch_cleanup_ignores_live_processes_and_unfamiliar_contents() {
        let root = tempfile::tempdir().unwrap();
        let abandoned = root.path().join("flursys-gmsh-4294967295-abandoned");
        fs::create_dir(&abandoned).unwrap();
        fs::write(abandoned.join("case.geo"), "Point(1) = {0,0,0,1};").unwrap();
        let live = root
            .path()
            .join(format!("flursys-gmsh-{}-running", std::process::id()));
        fs::create_dir(&live).unwrap();
        let live_child = root.path().join("flursys-gmsh-4294967295-child-running");
        fs::create_dir(&live_child).unwrap();
        fs::write(live_child.join("gmsh.pid"), std::process::id().to_string()).unwrap();
        let project = root.path().join("flursys-gmsh-4294967295-project");
        fs::create_dir(&project).unwrap();
        fs::write(project.join("project.json"), "do not delete").unwrap();
        let cutoff = SystemTime::now() + STALE_WORKSPACE_AGE + Duration::from_secs(1);

        prune_stale_gmsh_workspaces(root.path(), cutoff);

        assert!(!abandoned.exists());
        assert!(live.exists());
        assert!(live_child.exists());
        assert!(project.join("project.json").exists());
    }
}
