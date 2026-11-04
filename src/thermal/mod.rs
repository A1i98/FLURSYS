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
    /// Diffuse-grey exchange with a large isothermal enclosure; not RTE.
    Radiation {
        emissivity: f64,
        ambient_temperature: f64,
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
    pub reduced_pressure: bool,
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
            reduced_pressure: false,
            reference_temperature: 293.15,
            expansion: 0.0,
            gravity: Vec3::new(0.0, -9.81, 0.0),
            max_boussinesq_fraction: 0.1,
            boundaries: BTreeMap::new(),
        }
    }
}
impl Options {
    /// Acceleration relative to a reference-density hydrostatic state.
    /// Full-pressure runs additionally retain reference gravity.
    pub fn relative_buoyancy_acceleration(&self, temperature: f64) -> Result<Vec3, String> {
        self.validate()?;
        self.validate_temperature(temperature)?;
        Ok(if self.enabled && self.buoyancy {
            self.gravity * (-self.expansion * (temperature - self.reference_temperature))
        } else {
            Vec3::ZERO
        })
    }
    pub fn validate_temperature(&self, temperature: f64) -> Result<(), String> {
        if !temperature.is_finite() || temperature <= 0.0 {
            return Err("Temperature must be finite positive Kelvin".into());
        }
        if self.enabled && self.buoyancy {
            let fraction = self.expansion * (temperature - self.reference_temperature);
            if !fraction.is_finite() || fraction.abs() > self.max_boussinesq_fraction {
                return Err(format!(
                    "Boussinesq |beta*(T-Tref)|={:.4e} exceeds limit {:.4e} at T={temperature:.6} K; use smaller temperature differences or a variable-density model",
                    fraction.abs(),
                    self.max_boussinesq_fraction
                ));
            }
        }
        Ok(())
    }
    pub fn validate_for_case(&self, case: &IncompressibleCase) -> Result<(), String> {
        self.validate_for_mesh(&case.mesh)?;
        if self.enabled {
            for patch in case.mesh.boundary_patches() {
                if !self.boundaries.contains_key(&patch.name) {
                    return Err(format!(
                        "Assign a thermal boundary to patch {:?}",
                        patch.name
                    ));
                }
            }
            if self
                .boundaries
                .keys()
                .any(|name| !case.mesh.boundary_patches().iter().any(|p| &p.name == name))
            {
                return Err("Thermal boundary refers to a missing mesh patch".into());
            }
            for (name, profile) in &case.solver.boundary_profiles {
                if profile.quantity
                    == crate::unstructured_incompressible::BoundaryQuantity::Temperature
                {
                    profile.validate()?;
                    for sample in &profile.samples {
                        self.validate_temperature(sample.value.x)
                            .map_err(|e| format!("Patch {name}, t={} s: {e}", sample.time))?;
                    }
                }
            }
        }
        Ok(())
    }
    pub fn validate_for_mesh(&self, mesh: &UnstructuredMesh) -> Result<(), String> {
        self.validate()?;
        if self.enabled
            && self.buoyancy
            && mesh.dimension() == MeshDimension::TwoD
            && self.gravity.z.abs() > 1e-12 * self.gravity.norm().max(1.0)
        {
            return Err(
                "2D XY buoyancy requires gz=0; choose in-plane gravity or a 3D case".into(),
            );
        }
        Ok(())
    }
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
            || self.relative_tolerance >= 1.0
            || !self.gravity.norm().is_finite()
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
        self.validate_temperature(self.initial_temperature)?;
        for bc in self.boundaries.values() {
            match *bc {
                Boundary::FixedTemperature { temperature }
                | Boundary::InletOutlet {
                    incoming_temperature: temperature,
                } if !temperature.is_finite() || temperature <= 0.0 => {
                    return Err("Boundary temperature must be positive Kelvin".into());
                }
                Boundary::HeatFlux { inward_flux } if !inward_flux.is_finite() => {
                    return Err("Heat flux must be finite W/m²".into());
                }
                Boundary::Radiation {
                    emissivity,
                    ambient_temperature,
                } if !emissivity.is_finite()
                    || !(0.0..=1.0).contains(&emissivity)
                    || !ambient_temperature.is_finite()
                    || ambient_temperature <= 0.0 =>
                {
                    return Err("Radiation requires emissivity in [0,1] and positive ambient Kelvin temperature".into());
                }
                Boundary::Convection {
                    coefficient,
                    ambient_temperature,
                } if !coefficient.is_finite()
                    || coefficient <= 0.0
                    || !ambient_temperature.is_finite()
                    || ambient_temperature <= 0.0 =>
                {
                    return Err("Convection requires h > 0 and ambient T > 0 K".into());
                }
                _ => {}
            }
            if let Boundary::FixedTemperature { temperature }
            | Boundary::InletOutlet {
                incoming_temperature: temperature,
            } = *bc
            {
                self.validate_temperature(temperature)?;
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
    previous: CellField<f64>,
    pattern: crate::linear::CsrPatternCache,
    face_bc: Vec<Option<Boundary>>,
    walls: Vec<bool>,
    pub relative_change: f64,
    pub absolute_change: f64,
}
fn failure(message: impl Into<String>) -> SimpleError {
    SimpleError::PhysicalBoundary(message.into())
}

fn relax_temperature_matrix(
    mut matrix: CsrMatrix,
    rhs: &mut [f64],
    extra: &[f64],
    field: &[f64],
    alpha: f64,
) -> Result<CsrMatrix, SimpleError> {
    let n = matrix.nrows();
    let factor = 1.0 / alpha - 1.0;
    let reusable = (0..n).all(|i| {
        matrix.get(i, i).is_some_and(|old| {
            let diagonal = old + extra[i];
            let addition = extra[i] + factor * diagonal;
            addition.is_finite() && (old + addition).is_finite() && old + addition != 0.0
        })
    });
    if reusable {
        for i in 0..n {
            let start = matrix.row_offsets()[i];
            let end = matrix.row_offsets()[i + 1];
            let diagonal = start
                + matrix.column_indices()[start..end]
                    .binary_search(&i)
                    .expect("validated diagonal");
            let old = matrix.values()[diagonal];
            let diag = old + extra[i];
            matrix.values_mut()[diagonal] += extra[i] + factor * diag;
            rhs[i] += factor * diag * field[i];
        }
        return Ok(matrix);
    }
    let mut builder = CsrBuilder::with_capacity(n, n, matrix.nnz() + n);
    for i in 0..n {
        let diag = matrix.get(i, i).unwrap_or(0.0) + extra[i];
        for j in matrix.row_offsets()[i]..matrix.row_offsets()[i + 1] {
            builder.add(i, matrix.column_indices()[j], matrix.values()[j])?;
        }
        builder.add(i, i, extra[i] + factor * diag)?;
        rhs[i] += factor * diag * field[i];
    }
    Ok(builder.finalize()?)
}
impl Closure {
    pub fn new(
        case: &IncompressibleCase,
        initial: Option<CellField<f64>>,
    ) -> Result<Self, SimpleError> {
        let config = case.solver.thermal.clone();
        config.validate_for_case(case).map_err(failure)?;
        if config.wall_treatment == ThermalWallTreatment::EquilibriumPrt
            && (case.solver.turbulence.model != crate::turbulence::TurbulenceModel::KEpsilon
                || case.solver.turbulence.wall_treatment
                    != crate::turbulence::WallTreatment::EquilibriumKEpsilon)
        {
            return Err(failure(
                "Equilibrium Prt thermal wall treatment requires equilibrium k-epsilon momentum walls",
            ));
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
            let binding = case
                .solver
                .scripting
                .bindings
                .get(&p.name)
                .into_iter()
                .flatten()
                .find(|b| b.quantity == crate::scripting::Quantity::Temperature);
            let compiled = binding
                .map(|b| case.solver.scripting.compiled(b))
                .transpose()
                .map_err(failure)?;
            for i in &p.face_indices {
                let value = if let (Some(binding), Some(compiled)) = (binding, compiled.as_ref()) {
                    let t = compiled
                        .value(
                            binding,
                            mesh.faces()[*i].center,
                            case.solver.scripting.evaluation_time,
                        )
                        .map_err(|e| {
                            failure(format!("Temperature script on {}, face {i}: {e}", p.name))
                        })?
                        .x;
                    config.validate_temperature(t).map_err(failure)?;
                    match b {
                        Boundary::FixedTemperature { .. } => {
                            Boundary::FixedTemperature { temperature: t }
                        }
                        Boundary::InletOutlet { .. } => Boundary::InletOutlet {
                            incoming_temperature: t,
                        },
                        _ => {
                            return Err(failure(
                                "Temperature script requires Fixed or Inlet/outlet temperature",
                            ));
                        }
                    }
                } else {
                    b
                };
                face_bc[*i] = Some(value);
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
        for &temperature in field.values() {
            config.validate_temperature(temperature).map_err(failure)?;
        }
        Ok(Self {
            linear: case.solver.equations.as_ref().map(|e| e.temperature),
            last_linear: None,
            config,
            rho: case.material.density,
            old: field.clone(),
            previous: field.clone(),
            pattern: Default::default(),
            field,
            face_bc,
            walls,
            relative_change: f64::INFINITY,
            absolute_change: f64::INFINITY,
        })
    }
    pub fn backward_old(&mut self, older: Option<&CellField<f64>>, weights: [f64; 2]) {
        for i in 0..self.old.values().len() {
            let b = older.map_or(self.config.initial_temperature, |t| t[i]);
            self.old[i] = weights[0] * self.old[i] + weights[1] * b;
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
                        return Err(failure(format!(
                            "Thermal inflow/backflow at face {i} needs Fixed temperature or Inlet/outlet temperature"
                        )));
                    }
                    ScalarBoundaryCondition::ZeroGradient
                }
            });
            match b {
                Boundary::HeatFlux { inward_flux } => {
                    source[f.owner] +=
                        inward_flux * f.area / (rho_cp * mesh.cells()[f.owner].volume)
                }
                Boundary::Radiation {
                    emissivity,
                    ambient_temperature,
                } => {
                    let y = (f.center - mesh.cells()[f.owner].center).dot(f.normal);
                    if y <= 0.0 {
                        return Err(failure("Invalid radiative wall distance"));
                    }
                    let (slope, intercept) = crate::multiphysics::energy::radiative_linearization(
                        self.field[f.owner],
                        ambient_temperature,
                        rho_cp * face_d[i] / y,
                        emissivity,
                    )
                    .map_err(failure)?;
                    extra_diag[f.owner] += slope * f.area / rho_cp;
                    source[f.owner] += intercept * f.area / (rho_cp * mesh.cells()[f.owner].volume);
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
        let system = crate::momentum::assemble_momentum_component_with_prepared_viscosity(
            mesh,
            options,
            None,
            None,
            Some(&mut self.pattern),
        )?;
        let (matrix, mut rhs) = system.into_matrix_and_rhs();
        self.previous
            .values_mut()
            .copy_from_slice(self.field.values());
        let matrix = relax_temperature_matrix(
            matrix,
            &mut rhs,
            &extra_diag,
            self.field.values(),
            self.config.relaxation,
        )?;
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
        (self.relative_change, self.absolute_change) =
            self.previous.values().iter().zip(self.field.values()).fold(
                (0.0_f64, 0.0_f64),
                |(relative, absolute), (a, b)| {
                    let difference = (a - b).abs();
                    (
                        relative.max(difference / b.abs().max(1.0)),
                        absolute.max(difference),
                    )
                },
            );
        let mut force = CellField::filled(mesh, Vec3::ZERO);
        if self.config.buoyancy {
            for i in 0..mesh.cell_count() {
                let fraction =
                    self.config.expansion * (self.field[i] - self.config.reference_temperature);
                self.config
                    .validate_temperature(self.field[i])
                    .map_err(|e| failure(format!("{e}; cell {i}")))?;
                force[i] = self.config.gravity
                    * (if self.config.reduced_pressure {
                        0.0
                    } else {
                        1.0
                    } - fraction);
            }
        }
        Ok(force)
    }
    pub fn converged(&self) -> bool {
        self.relative_change <= self.config.relative_tolerance
    }
}

#[cfg(test)]
mod matrix_reuse_tests {
    use super::*;

    #[test]
    fn relaxation_preserves_coefficients_and_handles_missing_diagonal() {
        for missing in [false, true] {
            let mut builder = CsrBuilder::new(2, 2);
            if !missing {
                builder.add(0, 0, 2.0).unwrap();
            }
            builder.add(0, 1, -1.0).unwrap();
            builder.add(1, 0, -1.0).unwrap();
            builder.add(1, 1, 2.0).unwrap();
            let mut rhs = vec![1.0, 2.0];
            let matrix = relax_temperature_matrix(
                builder.finalize().unwrap(),
                &mut rhs,
                &[1.0, 0.0],
                &[10.0, 20.0],
                0.5,
            )
            .unwrap();
            assert_eq!(matrix.get(0, 0), Some(if missing { 2.0 } else { 6.0 }));
            assert_eq!(matrix.get(0, 1), Some(-1.0));
            assert_eq!(matrix.get(1, 0), Some(-1.0));
            assert_eq!(matrix.get(1, 1), Some(4.0));
            assert_eq!(rhs, [if missing { 11.0 } else { 31.0 }, 42.0]);
        }
    }

    #[test]
    fn relaxation_removes_an_exactly_cancelled_diagonal() {
        let mut builder = CsrBuilder::new(1, 1);
        builder.add(0, 0, 2.0).unwrap();
        let mut rhs = [1.0];
        let matrix =
            relax_temperature_matrix(builder.finalize().unwrap(), &mut rhs, &[-2.0], &[10.0], 0.5)
                .unwrap();
        assert_eq!(matrix.get(0, 0), None);
        assert_eq!(rhs, [1.0]);
    }
}
