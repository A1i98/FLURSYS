//! Versioned, portable simulation projects.
//!
//! A project is data, not compiled application code. This lets users define,
//! exchange, import, and run supported cases after FLURSYS itself is built.

use crate::cases::{BackwardStepCase, CavityCase, CylinderCase};
use crate::{
    Case, ConvectionScheme, PressureSolverKind, PressureVelocityCoupling, SimulationConfig,
};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

pub const PROJECT_FORMAT_VERSION: u32 = 1;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Project {
    pub format_version: u32,
    pub name: String,
    pub case: ProjectCase,
    #[serde(default)]
    pub solver: ProjectSolver,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum ProjectCase {
    LidDrivenCavity {
        length: f64,
        height: f64,
        density: f64,
        lid_velocity: f64,
        reynolds: f64,
    },
    Cylinder {
        length: f64,
        height: f64,
        diameter: f64,
        center_x: f64,
        center_y: f64,
        density: f64,
        freestream_velocity: f64,
        reynolds: f64,
        #[serde(default = "default_perturbation")]
        perturbation: f64,
    },
    BackwardFacingStep {
        length: f64,
        height: f64,
        step_height: f64,
        step_x: f64,
        density: f64,
        mean_velocity: f64,
        reynolds: f64,
    },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct ProjectSolver {
    pub nx: usize,
    pub ny: usize,
    pub dt: f64,
    pub max_iterations: usize,
    pub coupling: ProjectCoupling,
    pub pressure_solver: ProjectPressureSolver,
    pub pressure_tolerance: f64,
    pub pressure_iterations: usize,
    pub velocity_relaxation: f64,
    pub pressure_relaxation: f64,
    pub steady_tolerance: f64,
    pub threads: usize,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ProjectCoupling {
    Projection,
    Simple,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ProjectPressureSolver {
    Pcg,
    Sor,
}

impl Default for Project {
    fn default() -> Self {
        Self {
            format_version: PROJECT_FORMAT_VERSION,
            name: "Lid-driven cavity".to_string(),
            case: ProjectCase::from(CavityCase::default()),
            solver: ProjectSolver::default(),
        }
    }
}

impl Default for ProjectSolver {
    fn default() -> Self {
        Self {
            nx: 64,
            ny: 64,
            dt: 1.0e-3,
            max_iterations: 10_000,
            coupling: ProjectCoupling::Simple,
            pressure_solver: ProjectPressureSolver::Pcg,
            pressure_tolerance: 1.0e-5,
            pressure_iterations: 1_200,
            velocity_relaxation: 0.7,
            pressure_relaxation: 0.3,
            steady_tolerance: 1.0e-7,
            threads: 0,
        }
    }
}

impl Project {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, String> {
        let path = path.as_ref();
        let text = fs::read_to_string(path)
            .map_err(|error| format!("cannot read project {}: {error}", path.display()))?;
        let project: Self = serde_json::from_str(&text)
            .map_err(|error| format!("invalid FLURSYS project {}: {error}", path.display()))?;
        project.validate()?;
        Ok(project)
    }

    pub fn save(&self, path: impl AsRef<Path>) -> Result<(), String> {
        self.validate()?;
        let path = path.as_ref();
        let text = serde_json::to_string_pretty(self)
            .map_err(|error| format!("cannot serialize project: {error}"))?;
        fs::write(path, format!("{text}\n"))
            .map_err(|error| format!("cannot write project {}: {error}", path.display()))
    }

    pub fn simulation_config(
        &self,
        output_dir: impl Into<PathBuf>,
    ) -> Result<SimulationConfig, String> {
        self.validate()?;
        let solver = &self.solver;
        let coupling = match solver.coupling {
            ProjectCoupling::Projection => PressureVelocityCoupling::Projection,
            ProjectCoupling::Simple => PressureVelocityCoupling::Simple,
        };
        let pressure_solver = match solver.pressure_solver {
            ProjectPressureSolver::Pcg => PressureSolverKind::Pcg,
            ProjectPressureSolver::Sor => PressureSolverKind::Sor,
        };
        Ok(SimulationConfig {
            case: self.case.to_case(),
            nx: solver.nx,
            ny: solver.ny,
            dt: solver.dt,
            max_steps: solver.max_iterations,
            t_end: if matches!(coupling, PressureVelocityCoupling::Projection) {
                solver.max_iterations as f64 * solver.dt
            } else {
                0.0
            },
            convection: ConvectionScheme::FirstOrderUpwind,
            coupling,
            pressure_solver,
            pressure_max_iters: solver.pressure_iterations,
            pressure_tolerance: solver.pressure_tolerance,
            pressure_omega: 1.7,
            velocity_relaxation: solver.velocity_relaxation,
            pressure_relaxation: solver.pressure_relaxation,
            print_every: 100,
            output_every: 100,
            frame_every: 500,
            steady_tolerance: solver.steady_tolerance,
            minimum_steps: 100,
            threads: solver.threads,
            output_dir: output_dir.into(),
        })
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.format_version != PROJECT_FORMAT_VERSION {
            return Err(format!(
                "unsupported project format version {}; expected {}",
                self.format_version, PROJECT_FORMAT_VERSION
            ));
        }
        if self.name.trim().is_empty() {
            return Err("project name cannot be empty".to_string());
        }
        self.simulation_config_unvalidated(PathBuf::from("results/validation"))
            .validate()
    }

    fn simulation_config_unvalidated(&self, output_dir: PathBuf) -> SimulationConfig {
        let solver = &self.solver;
        let coupling = match solver.coupling {
            ProjectCoupling::Projection => PressureVelocityCoupling::Projection,
            ProjectCoupling::Simple => PressureVelocityCoupling::Simple,
        };
        SimulationConfig {
            case: self.case.to_case(),
            nx: solver.nx,
            ny: solver.ny,
            dt: solver.dt,
            max_steps: solver.max_iterations,
            t_end: if matches!(coupling, PressureVelocityCoupling::Projection) {
                solver.max_iterations as f64 * solver.dt
            } else {
                0.0
            },
            convection: ConvectionScheme::FirstOrderUpwind,
            coupling,
            pressure_solver: match solver.pressure_solver {
                ProjectPressureSolver::Pcg => PressureSolverKind::Pcg,
                ProjectPressureSolver::Sor => PressureSolverKind::Sor,
            },
            pressure_max_iters: solver.pressure_iterations,
            pressure_tolerance: solver.pressure_tolerance,
            pressure_omega: 1.7,
            velocity_relaxation: solver.velocity_relaxation,
            pressure_relaxation: solver.pressure_relaxation,
            print_every: 100,
            output_every: 100,
            frame_every: 500,
            steady_tolerance: solver.steady_tolerance,
            minimum_steps: 100,
            threads: solver.threads,
            output_dir,
        }
    }
}

impl ProjectCase {
    fn to_case(&self) -> Case {
        match self {
            Self::LidDrivenCavity {
                length,
                height,
                density,
                lid_velocity,
                reynolds,
            } => Case::LidDrivenCavity(CavityCase {
                length: *length,
                height: *height,
                rho: *density,
                lid_velocity: *lid_velocity,
                reynolds: *reynolds,
                nu: lid_velocity * length / reynolds,
            }),
            Self::Cylinder {
                length,
                height,
                diameter,
                center_x,
                center_y,
                density,
                freestream_velocity,
                reynolds,
                perturbation,
            } => {
                let nu = freestream_velocity * diameter / reynolds;
                Case::CylinderRe100(CylinderCase {
                    length: *length,
                    height: *height,
                    diameter: *diameter,
                    xc: *center_x,
                    yc: *center_y,
                    rho: *density,
                    u_inf: *freestream_velocity,
                    reynolds: *reynolds,
                    mu: density * nu,
                    nu,
                    perturbation: *perturbation,
                })
            }
            Self::BackwardFacingStep {
                length,
                height,
                step_height,
                step_x,
                density,
                mean_velocity,
                reynolds,
            } => Case::BackwardFacingStep(BackwardStepCase {
                length: *length,
                height: *height,
                step_height: *step_height,
                step_x: *step_x,
                rho: *density,
                u_mean: *mean_velocity,
                reynolds: *reynolds,
                nu: mean_velocity * step_height / reynolds,
            }),
        }
    }
}

impl From<CavityCase> for ProjectCase {
    fn from(case: CavityCase) -> Self {
        Self::LidDrivenCavity {
            length: case.length,
            height: case.height,
            density: case.rho,
            lid_velocity: case.lid_velocity,
            reynolds: case.reynolds,
        }
    }
}

fn default_perturbation() -> f64 {
    1.0e-3
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_project_converts_to_a_valid_simulation() {
        Project::default()
            .simulation_config("target/project-test")
            .unwrap();
    }

    #[test]
    fn project_json_round_trip_preserves_the_case() {
        let project = Project::default();
        let json = serde_json::to_string(&project).unwrap();
        let restored: Project = serde_json::from_str(&json).unwrap();
        restored.validate().unwrap();
    }

    #[test]
    fn bundled_case_file_is_importable() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/cavity.flursys.json");
        let project = Project::load(path).unwrap();
        assert_eq!(project.name, "Lid-driven cavity Re=100");
        project.simulation_config("target/import-test").unwrap();
    }
}
