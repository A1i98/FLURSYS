//! Headless, solver-independent meshing backends.

mod geo;
mod gmsh;

pub use geo::GmshGeoDocument;
pub use gmsh::{
    GeneratedMesh, GmshExecutable, GmshMeshOptions, GmshMesher, GmshMeshingReport, GmshVersion,
};

#[derive(Clone, Debug, PartialEq)]
pub enum MeshingError {
    InvalidOptions {
        message: String,
    },
    InvalidGeometry {
        message: String,
    },
    InvalidPhysicalName {
        name: String,
    },
    GmshExecutableNotFound {
        executable: std::path::PathBuf,
        message: String,
    },
    InvalidGmshVersion {
        stdout: String,
        stderr: String,
    },
    GmshProcessFailed {
        status: Option<i32>,
        stdout: String,
        stderr: String,
    },
    MissingOutputMesh {
        path: std::path::PathBuf,
        stdout: String,
        stderr: String,
    },
    InvalidGeneratedMeshFormat {
        path: std::path::PathBuf,
        message: String,
    },
    GmshImportError {
        message: String,
    },
    Io {
        message: String,
    },
}

impl std::fmt::Display for MeshingError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidOptions { message } => {
                write!(formatter, "invalid meshing options: {message}")
            }
            Self::InvalidGeometry { message } => {
                write!(formatter, "invalid meshing geometry: {message}")
            }
            Self::InvalidPhysicalName { name } => {
                write!(formatter, "invalid Gmsh physical name: {name:?}")
            }
            Self::GmshExecutableNotFound {
                executable,
                message,
            } => {
                write!(
                    formatter,
                    "cannot start Gmsh executable {}: {message}",
                    executable.display()
                )
            }
            Self::InvalidGmshVersion { stdout, stderr } => {
                write!(formatter, "Gmsh --version produced no version text (stdout: {stdout:?}, stderr: {stderr:?})")
            }
            Self::GmshProcessFailed {
                status,
                stdout,
                stderr,
            } => {
                write!(
                    formatter,
                    "Gmsh exited with {status:?} (stdout: {stdout:?}, stderr: {stderr:?})"
                )
            }
            Self::MissingOutputMesh {
                path,
                stdout,
                stderr,
            } => {
                write!(
                    formatter,
                    "Gmsh succeeded without writing {} (stdout: {stdout:?}, stderr: {stderr:?})",
                    path.display()
                )
            }
            Self::InvalidGeneratedMeshFormat { path, message } => {
                write!(
                    formatter,
                    "generated mesh {} is not compatible Gmsh v4 ASCII: {message}",
                    path.display()
                )
            }
            Self::GmshImportError { message } => {
                write!(formatter, "cannot import generated Gmsh mesh: {message}")
            }
            Self::Io { message } => write!(formatter, "meshing I/O failure: {message}"),
        }
    }
}

impl std::error::Error for MeshingError {}
