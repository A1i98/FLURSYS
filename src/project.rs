//! Versioned, portable simulation projects.
//!
//! A project is data, not compiled application code. This lets users define,
//! exchange, import, and run supported cases after FLURSYS itself is built.

use crate::cases::{BackwardStepCase, CavityCase, ChannelCase, CylinderCase};
use crate::{
    AnalysisDimension, BoundaryCondition, BoundaryConditionKind, BoundaryFace, Case,
    ConvectionScheme, ExecutionPlan, LidDrivenCavity3DConfig, MeshTopology, PhysicsSettings,
    PreprocessingModel, PressureSolverKind, PressureVelocityCoupling, SimulationConfig,
    WorkbenchAnalysis,
};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

pub const PROJECT_FORMAT_VERSION: u32 = 2;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Project {
    pub format_version: u32,
    pub name: String,
    pub case: ProjectCase,
    #[serde(default)]
    pub solver: ProjectSolver,
    /// Geometry, mesh, and named boundary data retained independently from the
    /// current 2D solver.
    pub preprocessing: PreprocessingModel,
    /// Executable flow/thermal physics.
    #[serde(default)]
    pub physics: PhysicsSettings,
    /// Solver-independent intent used to select an executable backend.
    #[serde(default)]
    pub workbench: WorkbenchAnalysis,
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
    Channel {
        length: f64,
        height: f64,
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
    /// Iteration cadence for publishing convergence data to an interactive client.
    pub gui_update_every: usize,
    /// Iteration cadence for appending residual and force history to disk.
    pub history_every: usize,
    /// Iteration cadence for writing field frames to disk and the GUI animation.
    pub frame_every: usize,
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
        let case = ProjectCase::from(CavityCase::default());
        Self {
            format_version: PROJECT_FORMAT_VERSION,
            name: "Lid-driven cavity".to_string(),
            preprocessing: default_preprocessing(&case),
            physics: PhysicsSettings::default(),
            workbench: WorkbenchAnalysis::default(),
            case,
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
            gui_update_every: 10,
            history_every: 10,
            frame_every: 100,
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
            print_every: solver.gui_update_every,
            output_every: solver.history_every,
            frame_every: solver.frame_every,
            steady_tolerance: solver.steady_tolerance,
            minimum_steps: 100,
            threads: solver.threads,
            boundary_overrides: self.preprocessing.solver_overrides(),
            physics: self.physics.clone(),
            output_dir: output_dir.into(),
        })
    }

    /// Select a concrete solver only after validating the project's declared
    /// dimension, geometry, meshing intent, and physics against its backend.
    pub fn execution_plan(&self, output_dir: impl Into<PathBuf>) -> Result<ExecutionPlan, String> {
        self.validate()?;
        self.workbench.validate()?;
        let output_dir = output_dir.into();
        match self.workbench.dimension {
            AnalysisDimension::TwoD => Ok(ExecutionPlan::StructuredIncompressible2D(Box::new(
                self.simulation_config(output_dir)?,
            ))),
            AnalysisDimension::ThreeD => Ok(ExecutionPlan::StructuredCavity3D(
                self.cavity_3d_config(output_dir)?,
            )),
        }
    }

    fn cavity_3d_config(&self, output_dir: PathBuf) -> Result<LidDrivenCavity3DConfig, String> {
        let ProjectCase::LidDrivenCavity {
            length,
            height,
            density,
            lid_velocity,
            reynolds,
        } = &self.case
        else {
            return Err(
                "the current 3D backend supports only a lid-driven cavity case".to_string(),
            );
        };
        if self.preprocessing.mesh.topology != MeshTopology::Structured {
            return Err("the current 3D backend requires a structured mesh".to_string());
        }
        if !self.preprocessing.geometry.parts.is_empty()
            || !self.preprocessing.geometry.sketches.is_empty()
            || !self.preprocessing.geometry.features.is_empty()
        {
            return Err(
                "the current 3D backend cannot yet mesh CAD geometry; remove geometry parts, sketches, and features"
                    .to_string(),
            );
        }
        if self.physics != PhysicsSettings::default() {
            return Err(
                "the current 3D backend supports constant-property flow only; thermal and buoyancy models are not available"
                    .to_string(),
            );
        }
        for boundary in &self.preprocessing.boundaries {
            let supported = match boundary.face {
                BoundaryFace::Left
                | BoundaryFace::Right
                | BoundaryFace::Bottom
                | BoundaryFace::Top => matches!(boundary.kind, BoundaryConditionKind::CaseDefault),
                BoundaryFace::Front | BoundaryFace::Back => {
                    matches!(boundary.kind, BoundaryConditionKind::Symmetry)
                }
            };
            if !supported {
                return Err(format!(
                    "the current 3D cavity backend does not support the configured {} boundary; use the case defaults",
                    boundary.face.label()
                ));
            }
        }
        Ok(LidDrivenCavity3DConfig {
            nx: self.solver.nx,
            ny: self.solver.ny,
            nz: self.preprocessing.mesh.cells_z,
            length: *length,
            height: *height,
            depth: self.preprocessing.geometry.extrusion_depth,
            density: *density,
            lid_velocity: *lid_velocity,
            reynolds: *reynolds,
            dt: self.solver.dt,
            max_steps: self.solver.max_iterations,
            pressure_max_iters: self.solver.pressure_iterations,
            pressure_tolerance: self.solver.pressure_tolerance,
            pressure_omega: 1.7,
            output_dir,
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
        self.preprocessing.validate()?;
        self.validate_active_solver_boundaries()?;
        self.simulation_config_unvalidated(PathBuf::from("results/validation"))
            .validate()
    }

    fn validate_active_solver_boundaries(&self) -> Result<(), String> {
        for boundary in &self.preprocessing.boundaries {
            if boundary.face != BoundaryFace::Right
                && matches!(boundary.kind, BoundaryConditionKind::PressureOutlet { .. })
            {
                return Err(format!(
                    "the active 2D solver supports a pressure outlet only on the right boundary; {} is retained for future 3D workflows but cannot run yet",
                    boundary.face.label()
                ));
            }
        }
        Ok(())
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
            print_every: solver.gui_update_every,
            output_every: solver.history_every,
            frame_every: solver.frame_every,
            steady_tolerance: solver.steady_tolerance,
            minimum_steps: 100,
            threads: solver.threads,
            boundary_overrides: self.preprocessing.solver_overrides(),
            physics: self.physics.clone(),
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
            Self::Channel {
                length,
                height,
                density,
                mean_velocity,
                reynolds,
            } => Case::Channel(ChannelCase {
                length: *length,
                height: *height,
                rho: *density,
                u_mean: *mean_velocity,
                reynolds: *reynolds,
                nu: mean_velocity * height / reynolds,
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

impl From<CylinderCase> for ProjectCase {
    fn from(case: CylinderCase) -> Self {
        Self::Cylinder {
            length: case.length,
            height: case.height,
            diameter: case.diameter,
            center_x: case.xc,
            center_y: case.yc,
            density: case.rho,
            freestream_velocity: case.u_inf,
            reynolds: case.reynolds,
            perturbation: case.perturbation,
        }
    }
}

impl From<BackwardStepCase> for ProjectCase {
    fn from(case: BackwardStepCase) -> Self {
        Self::BackwardFacingStep {
            length: case.length,
            height: case.height,
            step_height: case.step_height,
            step_x: case.step_x,
            density: case.rho,
            mean_velocity: case.u_mean,
            reynolds: case.reynolds,
        }
    }
}

impl From<ChannelCase> for ProjectCase {
    fn from(case: ChannelCase) -> Self {
        Self::Channel {
            length: case.length,
            height: case.height,
            density: case.rho,
            mean_velocity: case.u_mean,
            reynolds: case.reynolds,
        }
    }
}

fn default_perturbation() -> f64 {
    1.0e-3
}

fn default_preprocessing(_case: &ProjectCase) -> PreprocessingModel {
    PreprocessingModel {
        boundaries: vec![
            BoundaryCondition {
                name: "inlet-left".to_string(),
                face: BoundaryFace::Left,
                kind: BoundaryConditionKind::CaseDefault,
            },
            BoundaryCondition {
                name: "outlet-right".to_string(),
                face: BoundaryFace::Right,
                kind: BoundaryConditionKind::CaseDefault,
            },
            BoundaryCondition {
                name: "bottom".to_string(),
                face: BoundaryFace::Bottom,
                kind: BoundaryConditionKind::CaseDefault,
            },
            BoundaryCondition {
                name: "top".to_string(),
                face: BoundaryFace::Top,
                kind: BoundaryConditionKind::CaseDefault,
            },
            BoundaryCondition {
                name: "front".to_string(),
                face: BoundaryFace::Front,
                kind: BoundaryConditionKind::Symmetry,
            },
            BoundaryCondition {
                name: "back".to_string(),
                face: BoundaryFace::Back,
                kind: BoundaryConditionKind::Symmetry,
            },
        ],
        ..PreprocessingModel::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cases::{BoundaryKind, Side};
    use crate::{
        AnalysisDimension, GeometryFeatureKind, GeometryPart, GeometryPartKind, GeometrySketch,
        SketchPlane, SketchProfileKind, SolverBackend,
    };

    #[test]
    fn default_project_converts_to_a_valid_simulation() {
        Project::default()
            .simulation_config("target/project-test")
            .unwrap();
    }

    #[test]
    fn three_d_cavity_execution_plan_selects_the_3d_backend() {
        let mut project = Project::default();
        project.workbench.dimension = AnalysisDimension::ThreeD;
        let plan = project.execution_plan("target/project-3d-test").unwrap();
        assert_eq!(plan.backend(), SolverBackend::StructuredCavity3D);
        assert!(plan.capability_summary().contains("3D lid-driven cavity"));
    }

    #[test]
    fn three_d_execution_rejects_cad_geometry_until_meshing_is_available() {
        let mut project = Project::default();
        project.workbench.dimension = AnalysisDimension::ThreeD;
        project.preprocessing.geometry.parts.push(GeometryPart {
            name: "future-solid".to_string(),
            kind: GeometryPartKind::Box {
                length: 0.2,
                width: 0.2,
                height: 0.2,
            },
            x: 0.5,
            y: 0.5,
            z: 0.5,
        });
        assert!(project
            .execution_plan("target/project-3d-reject")
            .unwrap_err()
            .contains("cannot yet mesh CAD geometry"));
    }

    #[test]
    fn project_json_round_trip_preserves_the_case() {
        let project = Project::default();
        let json = serde_json::to_string(&project).unwrap();
        let restored: Project = serde_json::from_str(&json).unwrap();
        restored.validate().unwrap();
    }

    #[test]
    fn project_persists_interactive_output_cadence() {
        let mut project = Project::default();
        project.solver.gui_update_every = 7;
        project.solver.history_every = 11;
        project.solver.frame_every = 29;
        let json = serde_json::to_string(&project).unwrap();
        let restored: Project = serde_json::from_str(&json).unwrap();
        let config = restored.simulation_config("target/cadence-test").unwrap();
        assert_eq!(config.print_every, 7);
        assert_eq!(config.output_every, 11);
        assert_eq!(config.frame_every, 29);
    }

    #[test]
    fn bundled_case_file_is_importable() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/cavity.flursys.json");
        let project = Project::load(path).unwrap();
        assert_eq!(project.name, "Lid-driven cavity Re=100");
        project.simulation_config("target/import-test").unwrap();
    }

    #[test]
    fn bundled_channel_file_is_importable() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/channel.flursys.json");
        let project = Project::load(path).unwrap();
        assert_eq!(project.name, "Plane Poiseuille channel Re=100");
        assert!(matches!(project.case, ProjectCase::Channel { .. }));
        project
            .simulation_config("target/channel-import-test")
            .unwrap();
    }

    #[test]
    fn bundled_three_d_cavity_file_selects_the_three_d_backend() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/cavity-3d.flursys.json");
        let project = Project::load(path).unwrap();
        let plan = project
            .execution_plan("target/three-d-project-import-test")
            .unwrap();
        assert_eq!(plan.backend(), SolverBackend::StructuredCavity3D);
    }

    #[test]
    fn thermal_buoyancy_example_is_importable() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("examples/thermal-buoyancy-cavity.flursys.json");
        let project = Project::load(path).unwrap();
        assert!(matches!(
            project.physics.buoyancy,
            crate::BuoyancyModel::Boussinesq { .. }
        ));
        project
            .simulation_config("target/thermal-project-test")
            .unwrap();
    }

    #[test]
    fn named_boundary_conditions_become_solver_overrides() {
        let mut project = Project::default();
        project
            .preprocessing
            .boundary_mut(BoundaryFace::Left)
            .unwrap()
            .kind = BoundaryConditionKind::Velocity {
            u: 2.5,
            v: 0.0,
            w: 0.0,
        };
        let config = project.simulation_config("target/boundary-test").unwrap();
        assert!(matches!(
            config.boundary_overrides.kind(&config.case, Side::Left),
            BoundaryKind::Velocity
        ));
        assert_eq!(
            config
                .boundary_overrides
                .velocity(&config.case, Side::Left, 0.0, 0.5, 0.0),
            (2.5, 0.0)
        );
    }

    #[test]
    fn project_load_rejects_schema_without_preprocessing() {
        let path = std::env::temp_dir().join(format!(
            "flursys-missing-preprocessing-{}.json",
            std::process::id()
        ));
        std::fs::write(
            &path,
            r#"{
                "format_version": 1,
                "name": "missing preprocessing",
                "case": {
                    "kind": "lid-driven-cavity",
                    "length": 1.0,
                    "height": 1.0,
                    "density": 1.0,
                    "lid_velocity": 1.0,
                    "reynolds": 100.0
                }
            }"#,
        )
        .unwrap();
        let result = Project::load(&path);
        std::fs::remove_file(path).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn parametric_geometry_parts_round_trip_with_a_project() {
        let mut project = Project::default();
        project.preprocessing.geometry.parts.push(GeometryPart {
            name: "inlet-block".to_string(),
            kind: GeometryPartKind::Box {
                length: 2.0,
                width: 1.0,
                height: 0.5,
            },
            x: 1.0,
            y: 0.0,
            z: 0.25,
        });
        let json = serde_json::to_string(&project).unwrap();
        let restored: Project = serde_json::from_str(&json).unwrap();
        restored.validate().unwrap();
        assert_eq!(restored.preprocessing.geometry.parts.len(), 1);
        assert!(matches!(
            restored.preprocessing.geometry.parts[0].kind,
            GeometryPartKind::Box { .. }
        ));
    }

    #[test]
    fn geometry_part_names_must_be_unique() {
        let part = GeometryPart {
            name: "shared-name".to_string(),
            kind: GeometryPartKind::Cylinder {
                radius: 0.5,
                height: 1.0,
                segments: 32,
            },
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mut project = Project::default();
        project.preprocessing.geometry.parts.push(part.clone());
        project.preprocessing.geometry.parts.push(part);
        assert!(project.validate().unwrap_err().contains("not unique"));
    }

    #[test]
    fn advanced_parametric_primitives_round_trip_and_validate() {
        let mut project = Project::default();
        project.preprocessing.geometry.parts = vec![
            GeometryPart {
                name: "fairing".to_string(),
                kind: GeometryPartKind::Cone {
                    radius: 0.5,
                    height: 1.2,
                    segments: 32,
                },
                x: 0.0,
                y: 0.0,
                z: 0.6,
            },
            GeometryPart {
                name: "plenum".to_string(),
                kind: GeometryPartKind::Torus {
                    major_radius: 1.0,
                    minor_radius: 0.2,
                    segments: 32,
                },
                x: 2.0,
                y: 0.0,
                z: 0.0,
            },
        ];
        let json = serde_json::to_string(&project).unwrap();
        let restored: Project = serde_json::from_str(&json).unwrap();
        restored.validate().unwrap();
        assert!(matches!(
            restored.preprocessing.geometry.parts[1].kind,
            GeometryPartKind::Torus { .. }
        ));
    }

    #[test]
    fn torus_requires_a_clear_major_radius() {
        let mut project = Project::default();
        project.preprocessing.geometry.parts.push(GeometryPart {
            name: "invalid-torus".to_string(),
            kind: GeometryPartKind::Torus {
                major_radius: 0.5,
                minor_radius: 0.5,
                segments: 32,
            },
            x: 0.0,
            y: 0.0,
            z: 0.0,
        });
        assert!(project.validate().unwrap_err().contains("major radius"));
    }

    #[test]
    fn sketch_extrude_persists_its_source_and_generated_solid() {
        let mut project = Project::default();
        let output = project
            .preprocessing
            .geometry
            .add_sketch_feature(
                GeometrySketch::from_profile(
                    "inlet-profile".to_string(),
                    SketchPlane::Xy,
                    SketchProfileKind::Rectangle {
                        width: 2.0,
                        height: 1.0,
                    },
                    0.0,
                    0.0,
                    0.0,
                ),
                "inlet-extrude".to_string(),
                GeometryFeatureKind::Extrude { depth: 0.5 },
            )
            .unwrap();
        assert_eq!(output, "inlet-extrude solid");
        project.validate().unwrap();
        let restored: Project =
            serde_json::from_str(&serde_json::to_string(&project).unwrap()).unwrap();
        assert_eq!(restored.preprocessing.geometry.sketches.len(), 1);
        assert_eq!(restored.preprocessing.geometry.features.len(), 1);
        assert!(matches!(
            restored.preprocessing.geometry.parts[0].kind,
            GeometryPartKind::Box { .. }
        ));
    }

    #[test]
    fn sketch_extrude_uses_the_drawn_rectangle_dimensions() {
        let mut sketch = GeometrySketch::from_profile(
            "drawn-profile".to_string(),
            SketchPlane::Xy,
            SketchProfileKind::Rectangle {
                width: 1.0,
                height: 1.0,
            },
            0.0,
            0.0,
            0.0,
        );
        sketch.entities.clear();
        sketch.add_rectangle(0.0, 0.0, 3.0, 2.0).unwrap();

        let mut geometry = crate::GeometryModel::default();
        geometry
            .add_sketch_feature(
                sketch,
                "drawn-extrude".to_string(),
                GeometryFeatureKind::Extrude { depth: 0.5 },
            )
            .unwrap();

        assert!(matches!(
            geometry.parts[0].kind,
            GeometryPartKind::Box {
                length: 3.0,
                width: 2.0,
                height: 0.5,
            }
        ));
    }

    #[test]
    fn sketch_extrude_uses_the_drawn_profile_position() {
        let mut sketch = GeometrySketch::from_profile(
            "offset-profile".to_string(),
            SketchPlane::Xy,
            SketchProfileKind::Rectangle {
                width: 1.0,
                height: 1.0,
            },
            1.0,
            2.0,
            0.0,
        );
        sketch.entities.clear();
        sketch.add_rectangle(4.0, -3.0, 2.0, 2.0).unwrap();

        let mut geometry = crate::GeometryModel::default();
        geometry
            .add_sketch_feature(
                sketch,
                "offset-extrude".to_string(),
                GeometryFeatureKind::Extrude { depth: 0.5 },
            )
            .unwrap();

        assert_eq!(geometry.parts[0].x, 5.0);
        assert_eq!(geometry.parts[0].y, -1.0);
    }

    #[test]
    fn circle_revolve_materializes_a_torus_but_rectangle_is_rejected() {
        let sketch = GeometrySketch::from_profile(
            "seal-profile".to_string(),
            SketchPlane::Xy,
            SketchProfileKind::Circle { radius: 0.2 },
            0.0,
            0.0,
            0.0,
        );
        let mut geometry = crate::GeometryModel::default();
        geometry
            .add_sketch_feature(
                sketch,
                "seal-revolve".to_string(),
                GeometryFeatureKind::Revolve {
                    axis_offset: 1.0,
                    angle_degrees: 360.0,
                },
            )
            .unwrap();
        assert!(matches!(
            geometry.parts[0].kind,
            GeometryPartKind::Torus { .. }
        ));
        let error = geometry
            .add_sketch_feature(
                GeometrySketch::from_profile(
                    "rect-profile".to_string(),
                    SketchPlane::Xy,
                    SketchProfileKind::Rectangle {
                        width: 1.0,
                        height: 1.0,
                    },
                    0.0,
                    0.0,
                    0.0,
                ),
                "invalid-revolve".to_string(),
                GeometryFeatureKind::Revolve {
                    axis_offset: 1.0,
                    angle_degrees: 360.0,
                },
            )
            .unwrap_err();
        assert!(error.contains("circular 2D profile"));
    }

    #[test]
    fn editable_sketch_supports_square_dimension_axis_and_trim() {
        let mut sketch = GeometrySketch::from_profile(
            "editable".to_string(),
            SketchPlane::Xy,
            SketchProfileKind::Rectangle {
                width: 1.0,
                height: 1.0,
            },
            0.0,
            0.0,
            0.0,
        );
        sketch.entities.clear();
        sketch.add_rectangle(0.0, 0.0, 2.0, 2.0).unwrap();
        sketch.add_line(-2.0, 0.0, 2.0, 0.0).unwrap();
        sketch
            .add_distance_dimension(-1.0, -1.0, 1.0, -1.0)
            .unwrap();
        sketch.selected_axis = crate::SketchAxis::Vertical;
        assert!(sketch.select_entity_near(0.0, -1.0, 0.1).is_some());
        sketch.set_selected_dimension(3.0).unwrap();
        sketch.trim_line_near(-1.8, 0.0).unwrap();
        assert_eq!(sketch.entities.len(), 5);
        assert_eq!(sketch.dimensions[0].value, 2.0);
        assert_eq!(sketch.dimensions[1].value, 3.0);
        assert_eq!(sketch.selected_axis, crate::SketchAxis::Vertical);
    }

    #[test]
    fn selected_line_accepts_a_horizontal_constraint() {
        let mut sketch = GeometrySketch::from_profile(
            "constrained".to_string(),
            SketchPlane::Xy,
            SketchProfileKind::Rectangle {
                width: 1.0,
                height: 1.0,
            },
            0.0,
            0.0,
            0.0,
        );
        sketch.entities.clear();
        sketch.add_line(0.0, 0.0, 2.0, 1.0).unwrap();
        sketch.select_entity_near(1.0, 0.5, 0.1).unwrap();

        sketch
            .apply_selected_axis_constraint(crate::SketchAxis::Horizontal)
            .unwrap();

        assert!(matches!(
            sketch.entities[0].kind,
            crate::SketchEntityKind::Line {
                x1: 0.0,
                y1: 0.0,
                x2: 2.0,
                y2: 0.0,
            }
        ));
        assert_eq!(sketch.constraints.len(), 1);
    }

    #[test]
    fn selected_dimension_is_a_single_editable_driving_value() {
        let mut sketch = GeometrySketch::from_profile(
            "dimension".to_string(),
            SketchPlane::Xy,
            SketchProfileKind::Rectangle {
                width: 2.0,
                height: 2.0,
            },
            0.0,
            0.0,
            0.0,
        );
        sketch.entities.clear();
        sketch.add_line(0.0, 0.0, 2.0, 0.0).unwrap();
        sketch.select_entity_near(1.0, 0.0, 0.1);

        sketch.set_selected_dimension(5.0).unwrap();
        sketch.set_selected_dimension(7.5).unwrap();

        assert_eq!(sketch.dimensions.len(), 1);
        assert_eq!(sketch.dimensions[0].entity, sketch.selected_entity);
        assert_eq!(sketch.dimensions[0].value, 7.5);
    }
}
