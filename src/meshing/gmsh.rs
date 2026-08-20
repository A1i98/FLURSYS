//! Command-line Gmsh backend configuration and invocation.

use super::{GmshGeoDocument, MeshingError};
use crate::{load_gmsh, MeshDimension, UnstructuredMesh};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

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
        if geometry.dimension() != options.dimension {
            return Err(MeshingError::InvalidOptions {
                message: "geometry and mesh option dimensions differ".into(),
            });
        }
        let version = self.version()?;
        let workspace = tempfile::Builder::new()
            .prefix("flursys-gmsh-")
            .tempdir()
            .map_err(|error| MeshingError::Io {
                message: error.to_string(),
            })?;
        let geo_path = workspace.path().join("case.geo");
        let mesh_path = workspace.path().join("case.msh");
        fs::write(&geo_path, geometry.to_geo_string()?).map_err(|error| MeshingError::Io {
            message: error.to_string(),
        })?;
        let arguments = self.command_arguments(&geo_path, &mesh_path, options)?;
        let output = self.command().args(arguments).output().map_err(|error| {
            MeshingError::GmshExecutableNotFound {
                executable: self.executable_label(),
                message: error.to_string(),
            }
        })?;
        let stdout = text(&output.stdout);
        let stderr = text(&output.stderr);
        if !output.status.success() {
            return Err(MeshingError::GmshProcessFailed {
                status: output.status.code(),
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
