//! Constant-property temperature transport and Boussinesq forcing for SIMPLE.
use crate::*;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "kebab-case")]
pub enum Boundary {
    Adiabatic,
    FixedTemperature {
        temperature: f64,
    },
    InletOutlet {
        incoming_temperature: f64,
    },
    /// Positive into fluid, W/m².
    HeatFlux {
        inward_flux: f64,
    },
    Convection {
        coefficient: f64,
        ambient_temperature: f64,
    },
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ThermalWallTreatment {
    #[default]
    Molecular,
    EquilibriumPrt,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Options {
    pub enabled: bool,
    pub initial_temperature: f64,
    pub specific_heat: f64,
    pub conductivity: f64,
    pub source_w_m3: f64,
    pub relaxation: f64,
    pub relative_tolerance: f64,
    pub turbulent_prandtl: f64,
    pub wall_treatment: ThermalWallTreatment,
    pub buoyancy: bool,
    pub reference_temperature: f64,
    pub expansion: f64,
    pub gravity: Vec3,
    pub max_boussinesq_fraction: f64,
    pub boundaries: BTreeMap<String, Boundary>,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            enabled: false,
            initial_temperature: 293.15,
            specific_heat: 1005.0,
            conductivity: 0.025,
            source_w_m3: 0.0,
            relaxation: 0.7,
            relative_tolerance: 1e-8,
            turbulent_prandtl: 0.85,
            wall_treatment: ThermalWallTreatment::Molecular,
            buoyancy: false,
            reference_temperature: 293.15,
            expansion: 0.0,
            gravity: Vec3::new(0.0, -9.81, 0.0),
            max_boussinesq_fraction: 0.1,
            boundaries: BTreeMap::new(),
        }
    }
}
impl Options {
    pub fn validate(&self) -> Result<(), String> {
        if self.buoyancy && !self.enabled {
            return Err("Boussinesq requires temperature transport".into());
        }
        if !self.enabled {
            return Ok(());
        }
        if [
            self.initial_temperature,
            self.specific_heat,
            self.conductivity,
            self.relaxation,
            self.relative_tolerance,
            self.turbulent_prandtl,
            self.reference_temperature,
            self.max_boussinesq_fraction,
        ]
        .iter()
        .any(|v| !v.is_finite() || *v <= 0.0)
            || self.relaxation > 1.0
            || self.max_boussinesq_fraction > 0.2
            || !self.source_w_m3.is_finite()
            || !self.expansion.is_finite()
            || self.expansion < 0.0
            || ![self.gravity.x, self.gravity.y, self.gravity.z]
                .iter()
                .all(|v| v.is_finite())
        {
            return Err("Invalid thermal properties, positive Kelvin temperatures, relaxation/tolerance or Boussinesq settings".into());
        }
        for bc in self.boundaries.values() {
            match *bc {
                Boundary::FixedTemperature { temperature }
                | Boundary::InletOutlet {
                    incoming_temperature: temperature,
                } if !temperature.is_finite() || temperature <= 0.0 => {
                    return Err("Boundary temperature must be positive Kelvin".into())
                }
                Boundary::HeatFlux { inward_flux } if !inward_flux.is_finite() => {
                    return Err("Heat flux must be finite W/m²".into())
                }
                Boundary::Convection {
                    coefficient,
                    ambient_temperature,
                } if !coefficient.is_finite()
                    || coefficient <= 0.0
                    || !ambient_temperature.is_finite()
                    || ambient_temperature <= 0.0 =>
                {
                    return Err("Convection requires h > 0 and ambient T > 0 K".into())
                }
                _ => {}
            }
        }
        Ok(())
    }
}
#[derive(Debug)]
pub(crate) struct Closure {
    config: Options,
    linear: Option<crate::equations::EquationControl>,
    pub last_linear: Option<LinearSolveReport>,
    rho: f64,
    field: CellField<f64>,
    old: CellField<f64>,
    face_bc: Vec<Option<Boundary>>,
    walls: Vec<bool>,
    pub relative_change: f64,
}
fn failure(message: impl Into<String>) -> SimpleError {
    SimpleError::PhysicalBoundary(message.into())
}
impl Closure {
    pub fn new(
        case: &IncompressibleCase,
        initial: Option<CellField<f64>>,
    ) -> Result<Self, SimpleError> {
        let config = case.solver.thermal.clone();
        config.validate().map_err(failure)?;
        if config.wall_treatment == ThermalWallTreatment::EquilibriumPrt
            && (case.solver.turbulence.model != crate::turbulence::TurbulenceModel::KEpsilon
                || case.solver.turbulence.wall_treatment
                    != crate::turbulence::WallTreatment::EquilibriumKEpsilon)
        {
            return Err(failure("Equilibrium Prt thermal wall treatment requires equilibrium k-epsilon momentum walls"));
        }
        let mesh = &case.mesh;
        let mut walls = vec![false; mesh.face_count()];
        for (name, bc) in &case.boundaries {
            if matches!(
                bc,
                IncompressibleBoundaryCondition::NoSlipWall
                    | IncompressibleBoundaryCondition::MovingWall { .. }
            ) {
                if let Some(p) = mesh.boundary_patches().iter().find(|p| &p.name == name) {
                    for &f in &p.face_indices {
                        walls[f] = true;
                    }
                }
            }
        }
        let mut face_bc = vec![None; mesh.face_count()];
        for p in mesh.boundary_patches() {
            let b = *config.boundaries.get(&p.name).ok_or_else(|| {
                failure(format!("Assign a thermal condition to patch {:?}", p.name))
            })?;
            for i in &p.face_indices {
                face_bc[*i] = Some(b);
            }
        }
        if config
            .boundaries
            .keys()
            .any(|n| !mesh.boundary_patches().iter().any(|p| &p.name == n))
        {
            return Err(failure("Thermal boundary refers to missing patch"));
        }
        let field = initial.unwrap_or_else(|| CellField::filled(mesh, config.initial_temperature));
        field.ensure_mesh(mesh)?;
        if field.values().iter().any(|v| !v.is_finite() || *v <= 0.0) {
            return Err(failure(
                "Temperature state must be finite and positive Kelvin",
            ));
        }
        Ok(Self {
            linear: case.solver.equations.as_ref().map(|e| e.temperature),
            last_linear: None,
            config,
            rho: case.material.density,
            old: field.clone(),
            field,
            face_bc,
            walls,
            relative_change: f64::INFINITY,
        })
    }
    pub fn backward_old(&mut self, older: Option<&CellField<f64>>) {
        for i in 0..self.old.values().len() {
            let b = older.map_or(self.config.initial_temperature, |t| t[i]);
            self.old[i] = (4.0 * self.old[i] - b) / 3.0;
        }
    }
    pub fn field(&self) -> CellField<f64> {
        self.field.clone()
    }
    pub fn correct(
        &mut self,
        mesh: &UnstructuredMesh,
        state: &SimpleState,
        nut: Option<&CellField<f64>>,
        wall_nut: Option<&FaceField<f64>>,
        dt: Option<f64>,
        linear: LinearSolverOptions,
    ) -> Result<CellField<Vec3>, SimpleError> {
        let rho_cp = self.rho * self.config.specific_heat;
        let mut conditions = vec![None; mesh.face_count()];
        let mut d = CellField::filled(mesh, self.config.conductivity / rho_cp);
        if let Some(nut) = nut {
            for i in 0..mesh.cell_count() {
                d[i] += nut[i] / self.config.turbulent_prandtl;
            }
        }
        let mut face_d = interpolate_diffusivity(
            mesh,
            Diffusivity::CellField(&d),
            DiffusivityInterpolation::Harmonic,
        )?;
        if let Some(n) = wall_nut {
            n.ensure_mesh(mesh)?;
        }
        for (i, wall) in self.walls.iter().enumerate() {
            if *wall {
                let turbulent = if self.config.wall_treatment
                    == ThermalWallTreatment::EquilibriumPrt
                {
                    wall_nut.ok_or_else(|| failure("Missing modeled thermal wall viscosity"))?[i]
                        / self.config.turbulent_prandtl
                } else {
                    0.0
                };
                face_d[i] = self.config.conductivity / rho_cp + turbulent;
            }
        }
        let mut source = CellField::filled(mesh, self.config.source_w_m3 / rho_cp);
        let mut extra_diag = vec![0.0; mesh.cell_count()];
        for (i, f) in mesh.faces().iter().enumerate() {
            if f.neighbour.is_some() {
                continue;
            }
            let b = self.face_bc[i].ok_or_else(|| failure("Missing thermal boundary"))?;
            let incoming =
                state.face_flux[i] < -1e-12 * f.area * state.velocity[f.owner].norm().max(1.0);
            conditions[i] = Some(match b {
                Boundary::FixedTemperature { temperature } => {
                    ScalarBoundaryCondition::FixedValue(temperature)
                }
                Boundary::InletOutlet {
                    incoming_temperature,
                } => {
                    if incoming {
                        ScalarBoundaryCondition::FixedValue(incoming_temperature)
                    } else {
                        ScalarBoundaryCondition::ZeroGradient
                    }
                }
                _ => {
                    if incoming {
                        return Err(failure(format!("Thermal inflow/backflow at face {i} needs Fixed temperature or Inlet/outlet temperature")));
                    }
                    ScalarBoundaryCondition::ZeroGradient
                }
            });
            match b {
                Boundary::HeatFlux { inward_flux } => {
                    source[f.owner] +=
                        inward_flux * f.area / (rho_cp * mesh.cells()[f.owner].volume)
                }
                Boundary::Convection {
                    coefficient,
                    ambient_temperature,
                } => {
                    let y = (f.center - mesh.cells()[f.owner].center).dot(f.normal);
                    if y <= 0.0 {
                        return Err(failure("Invalid thermal wall distance"));
                    }
                    let conductance =
                        f.area / (1.0 / coefficient + y / (rho_cp * face_d[i])) / rho_cp;
                    extra_diag[f.owner] += conductance;
                    source[f.owner] +=
                        conductance * ambient_temperature / mesh.cells()[f.owner].volume;
                }
                _ => {}
            }
        }
        let bc = ResolvedScalarBoundaryConditions::from_face_conditions(mesh, conditions)?;
        let mut options =
            MomentumOptions::steady(Diffusivity::Faces(&face_d), &bc, &state.face_flux);
        options.old = dt.map(|_| &self.old);
        options.time_step = dt;
        options.source = Some(&source);
        options.correction_field = Some(&self.field);
        options.diffusion.non_orthogonal_correction = NonOrthogonalCorrection::Limited;
        options.allow_iterative_outlet_backflow = true;
        let system = assemble_momentum_component(mesh, options)?;
        let matrix = system.matrix();
        let mut builder = CsrBuilder::with_capacity(
            mesh.cell_count(),
            mesh.cell_count(),
            matrix.nnz() + mesh.cell_count(),
        );
        let mut rhs = system.rhs().to_vec();
        let previous = self.field.clone();
        let alpha = self.config.relaxation;
        for i in 0..mesh.cell_count() {
            let diag = matrix.get(i, i).unwrap_or(0.0) + extra_diag[i];
            for j in matrix.row_offsets()[i]..matrix.row_offsets()[i + 1] {
                builder.add(i, matrix.column_indices()[j], matrix.values()[j])?;
            }
            builder.add(i, i, extra_diag[i] + (1.0 / alpha - 1.0) * diag)?;
            rhs[i] += (1.0 / alpha - 1.0) * diag * self.field[i];
        }
        let matrix = builder.finalize()?;
        let report = self
            .linear
            .unwrap_or(crate::equations::EquationControl::from_options(linear))
            .solve(&matrix, &rhs, self.field.values_mut(), false)?;
        self.last_linear = Some(report);
        if report.status != LinearSolverStatus::Converged {
            return Err(failure(
                "Temperature linear equation did not reach tolerance",
            ));
        }
        if self
            .field
            .values()
            .iter()
            .any(|v| !v.is_finite() || *v <= 0.0)
        {
            return Err(failure(
                "Nonphysical temperature; no temperature clipping was applied",
            ));
        }
        self.relative_change = previous
            .values()
            .iter()
            .zip(self.field.values())
            .map(|(a, b)| (a - b).abs() / b.abs().max(1.0))
            .fold(0.0, f64::max);
        let mut force = CellField::filled(mesh, Vec3::ZERO);
        if self.config.buoyancy {
            for i in 0..mesh.cell_count() {
                let fraction =
                    self.config.expansion * (self.field[i] - self.config.reference_temperature);
                if fraction.abs() > self.config.max_boussinesq_fraction {
                    return Err(failure(format!("Boussinesq density variation {fraction:.4e} exceeds configured small-variation limit in cell {i}")));
                }
                force[i] = self.config.gravity * (1.0 - fraction);
            }
        }
        Ok(force)
    }
    pub fn converged(&self) -> bool {
        self.relative_change <= self.config.relative_tolerance
    }
}
