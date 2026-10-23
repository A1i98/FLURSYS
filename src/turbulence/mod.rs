//! Incompressible eddy-viscosity closures on a fixed unstructured mesh.
//! Wall-resolved k-omega/SST and optional smooth equilibrium k-epsilon walls.
use crate::*;
use serde::{Deserialize, Serialize};
mod boundary;
mod transport;
pub use boundary::TurbulenceInlet;
mod k_epsilon;
mod realizable;
mod spalart_allmaras;
mod wall_distance;
mod wall_functions;
pub use wall_functions::{smooth_wall, WallTreatment};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum TurbulenceModel {
    #[default]
    Laminar,
    KOmega,
    KOmegaSst,
    Smagorinsky,
    Wale,
    KEpsilon,
    RngKEpsilon,
    RealizableKEpsilon,
    SpalartAllmaras,
}
impl TurbulenceModel {
    pub fn uses_epsilon(self) -> bool {
        matches!(
            self,
            Self::KEpsilon | Self::RngKEpsilon | Self::RealizableKEpsilon
        )
    }
    pub(crate) fn epsilon_cmu(self) -> f64 {
        if self == Self::RngKEpsilon {
            0.0845
        } else {
            0.09
        }
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct TurbulenceOptions {
    pub model: TurbulenceModel,
    pub wall_treatment: WallTreatment,
    pub inlets: std::collections::BTreeMap<String, TurbulenceInlet>,
    /// Uniform initial and incoming k (m²/s²) and omega (1/s).
    pub k: f64,
    pub omega: f64,
    /// k-epsilon incoming dissipation, m²/s³.
    pub epsilon: f64,
    pub nu_tilda: f64,
    pub relaxation: f64,
    pub relative_tolerance: f64,
    pub ck: f64,
    pub ce: f64,
    pub cw: f64,
}
impl Default for TurbulenceOptions {
    fn default() -> Self {
        Self {
            model: TurbulenceModel::Laminar,
            wall_treatment: WallTreatment::Resolved,
            inlets: Default::default(),
            k: 1e-4,
            omega: 1.0,
            epsilon: 1e-5,
            nu_tilda: 1e-5,
            relaxation: 0.5,
            relative_tolerance: 1e-6,
            ck: 0.094,
            ce: 1.048,
            cw: 0.325,
        }
    }
}
impl TurbulenceOptions {
    pub fn validate(&self) -> Result<(), String> {
        if self.wall_treatment == WallTreatment::EquilibriumKEpsilon && !self.model.uses_epsilon() {
            return Err("Equilibrium wall treatment requires k-epsilon".into());
        }
        if self.wall_treatment == WallTreatment::EquilibriumKOmega
            && !matches!(
                self.model,
                TurbulenceModel::KOmega | TurbulenceModel::KOmegaSst
            )
        {
            return Err("Omega wall functions require k-omega or SST".into());
        }
        for b in self.inlets.values() {
            b.validate()?;
            if matches!(b, TurbulenceInlet::ModifiedViscosity { .. })
                != (self.model == TurbulenceModel::SpalartAllmaras)
            {
                return Err("SA requires modified-viscosity incoming data; other RANS models require k/omega/epsilon or intensity/length".into());
            }
        }
        if !self.nu_tilda.is_finite() || self.nu_tilda < 0.0 {
            return Err("SA modified viscosity must be finite and nonnegative".into());
        }
        let constants = [self.k, self.omega, self.epsilon, self.ck, self.ce, self.cw];
        if (self.model != TurbulenceModel::SpalartAllmaras
            && constants.iter().any(|x| !x.is_finite() || *x <= 0.0))
            || !self.relative_tolerance.is_finite()
            || self.relative_tolerance <= 0.0
            || !self.relaxation.is_finite()
            || self.relaxation <= 0.0
            || self.relaxation > 1.0
        {
            return Err("Turbulence k, omega and model constants must be positive; relaxation must be in (0,1]".into());
        }
        Ok(())
    }
    pub fn is_rans(&self) -> bool {
        matches!(
            self.model,
            TurbulenceModel::KOmega
                | TurbulenceModel::KOmegaSst
                | TurbulenceModel::KEpsilon
                | TurbulenceModel::RngKEpsilon
                | TurbulenceModel::RealizableKEpsilon
                | TurbulenceModel::SpalartAllmaras
        )
    }
}
#[derive(Clone, Debug)]
pub struct TurbulenceFields {
    pub model: TurbulenceModel,
    pub k: CellField<f64>,
    pub omega: CellField<f64>,
    pub nut: CellField<f64>,
    pub epsilon: Option<CellField<f64>>,
    pub nu_tilda: Option<CellField<f64>>,
}
impl TurbulenceFields {
    pub fn new(mesh: &UnstructuredMesh, options: &TurbulenceOptions) -> Self {
        Self {
            model: options.model,
            k: CellField::filled(
                mesh,
                if options.model == TurbulenceModel::SpalartAllmaras {
                    0.0
                } else {
                    options.k
                },
            ),
            omega: CellField::filled(
                mesh,
                if options.model == TurbulenceModel::SpalartAllmaras {
                    0.0
                } else {
                    options.omega
                },
            ),
            nut: CellField::filled(mesh, 0.0),
            nu_tilda: (options.model == TurbulenceModel::SpalartAllmaras)
                .then(|| CellField::filled(mesh, options.nu_tilda)),
            epsilon: (options.model.uses_epsilon())
                .then(|| CellField::filled(mesh, options.epsilon)),
        }
    }
}

#[derive(Debug)]
pub(crate) struct Closure {
    config: TurbulenceOptions,
    linear: Option<crate::equations::EquationSolvers>,
    molecular_viscosity: f64,
    pub last_linear: std::collections::BTreeMap<String, LinearSolveReport>,
    fields: TurbulenceFields,
    walls: Vec<bool>,
    incoming: Vec<Option<TurbulenceInlet>>,
    distance: Vec<f64>,
    old: Option<TurbulenceFields>,
    pub relative_change: f64,
}
impl Closure {
    pub fn new(
        mesh: &UnstructuredMesh,
        case: &IncompressibleCase,
        old: Option<TurbulenceFields>,
    ) -> Result<Self, SimpleError> {
        case.solver
            .turbulence
            .validate()
            .map_err(SimpleError::PhysicalBoundary)?;
        let config = case.solver.turbulence.clone();
        if config.model != TurbulenceModel::Laminar && case.material.kinematic_viscosity <= 0.0 {
            return Err(SimpleError::PhysicalBoundary(
                "Turbulent flow requires positive molecular viscosity".into(),
            ));
        }
        if matches!(
            config.model,
            TurbulenceModel::Smagorinsky | TurbulenceModel::Wale
        ) && mesh.dimension() != MeshDimension::ThreeD
        {
            return Err(SimpleError::PhysicalBoundary(
                "LES requires a three-dimensional volume mesh".into(),
            ));
        }
        let mut incoming = vec![None; mesh.face_count()];
        for (name, b) in &config.inlets {
            let patch = mesh
                .boundary_patches()
                .iter()
                .find(|p| &p.name == name)
                .ok_or_else(|| {
                    SimpleError::PhysicalBoundary(format!("Unknown turbulence patch {name}"))
                })?;
            if matches!(
                case.boundaries
                    .iter()
                    .find(|(n, _)| n == name)
                    .map(|(_, b)| b),
                Some(
                    IncompressibleBoundaryCondition::NoSlipWall
                        | IncompressibleBoundaryCondition::MovingWall { .. }
                        | IncompressibleBoundaryCondition::Slip
                        | IncompressibleBoundaryCondition::Symmetry
                )
            ) {
                return Err(SimpleError::PhysicalBoundary(format!(
                    "Incoming turbulence cannot be assigned to impermeable patch {name}"
                )));
            }
            for &f in &patch.face_indices {
                incoming[f] = Some(*b);
            }
        }
        let mut walls = vec![false; mesh.face_count()];
        for (name, bc) in &case.boundaries {
            if matches!(
                bc,
                IncompressibleBoundaryCondition::NoSlipWall
                    | IncompressibleBoundaryCondition::MovingWall { .. }
            ) {
                if let Some(patch) = mesh.boundary_patches().iter().find(|p| &p.name == name) {
                    for &f in &patch.face_indices {
                        walls[f] = true;
                    }
                }
            }
        }
        if config.model.uses_epsilon()
            && config.wall_treatment != WallTreatment::EquilibriumKEpsilon
            && walls.iter().any(|w| *w)
        {
            return Err(SimpleError::PhysicalBoundary("Select equilibrium k-epsilon wall treatment for solid walls, or use wall-resolved SST.".into()));
        }
        let distance = if matches!(
            config.model,
            TurbulenceModel::KOmega | TurbulenceModel::KOmegaSst | TurbulenceModel::SpalartAllmaras
        ) {
            wall_distance::distances(mesh, &walls)?
        } else {
            vec![1.0; mesh.cell_count()]
        };
        let mut fields = old
            .clone()
            .filter(|f| f.model == config.model)
            .unwrap_or_else(|| TurbulenceFields::new(mesh, &config));
        if let Some(v) = &fields.nu_tilda {
            v.ensure_mesh(mesh)?;
            if v.values().iter().any(|x| !x.is_finite() || *x < 0.0) {
                return Err(SimpleError::PhysicalBoundary(
                    "Invalid SA restart field".into(),
                ));
            }
        }
        if config.model == TurbulenceModel::SpalartAllmaras && fields.nu_tilda.is_none() {
            return Err(SimpleError::PhysicalBoundary(
                "SA restart requires nu_tilda".into(),
            ));
        }
        if config.model == TurbulenceModel::SpalartAllmaras {
            fields.k.fill(0.0);
            fields.omega.fill(0.0);
        }
        fields.k.ensure_mesh(mesh)?;
        fields.omega.ensure_mesh(mesh)?;
        fields.nut.ensure_mesh(mesh)?;
        if let Some(e) = &fields.epsilon {
            e.ensure_mesh(mesh)?;
            if e.values().iter().any(|v| !v.is_finite() || *v <= 0.0) {
                return Err(SimpleError::PhysicalBoundary(
                    "Invalid epsilon restart field".into(),
                ));
            }
        }
        if config.model.uses_epsilon() && fields.epsilon.is_none() {
            return Err(SimpleError::PhysicalBoundary(
                "k-epsilon restart requires an epsilon field".into(),
            ));
        }

        if fields
            .k
            .values()
            .iter()
            .chain(fields.omega.values())
            .chain(fields.nut.values())
            .any(|x| !x.is_finite() || *x < 0.0)
        {
            return Err(SimpleError::PhysicalBoundary(
                "Invalid restart turbulence fields".into(),
            ));
        }
        Ok(Self {
            linear: case.solver.equations.clone(),
            molecular_viscosity: case.material.kinematic_viscosity,
            last_linear: Default::default(),
            config,
            old: Some(fields.clone()),
            fields,
            walls,
            incoming,
            distance,
            relative_change: 0.0,
        })
    }
    fn incoming_values(
        &self,
        mesh: &UnstructuredMesh,
        state: &SimpleState,
        bc: &ResolvedVelocityBoundaryConditions,
        face: usize,
    ) -> Result<(f64, f64, f64), SimpleError> {
        let Some(b) = self.incoming[face] else {
            return Ok((self.config.k, self.config.omega, self.config.epsilon));
        };
        let f = &mesh.faces()[face];
        let normal = f.area_vector / f.area;
        let u = state.velocity[f.owner];
        // Use imposed velocity where available, including pressure-outlet backflow.
        let fallback = u - normal * u.dot(normal) + normal * (state.face_flux[face] / f.area);
        let component = |c, v| match bc.component(c).condition(face) {
            Some(ScalarBoundaryCondition::FixedValue(x)) => x,
            _ => v,
        };
        let u = Vec3::new(
            component(MomentumComponent::X, fallback.x),
            component(MomentumComponent::Y, fallback.y),
            component(MomentumComponent::Z, fallback.z),
        );
        b.values_with_viscosity(u.norm(),self.molecular_viscosity).map_err(SimpleError::PhysicalBoundary)
    }
    pub fn backward_old(&mut self, older: Option<&TurbulenceFields>, weights: [f64;2]) {
        if let Some(old)=&mut self.old {
            for i in 0..old.k.values().len() {
                old.k[i]=weights[0]*old.k[i]+weights[1]*older.map_or(self.config.k,|v|v.k[i]);
                old.omega[i]=weights[0]*old.omega[i]+weights[1]*older.map_or(self.config.omega,|v|v.omega[i]);
                if let Some(nt)=&mut old.nu_tilda {
                    nt[i]=weights[0]*nt[i]+weights[1]*older.and_then(|v|v.nu_tilda.as_ref()).map_or(self.config.nu_tilda,|v|v[i]);
                }
                if let Some(e)=&mut old.epsilon {
                    e[i]=weights[0]*e[i]+weights[1]*older.and_then(|v|v.epsilon.as_ref()).map_or(self.config.epsilon,|v|v[i]);
                }
            }
        }
    }
    pub fn converged(&self) -> bool {
        self.relative_change <= self.config.relative_tolerance
    }
    pub fn thermal_wall_viscosity(
        &self,
        mesh: &UnstructuredMesh,
    ) -> Result<FaceField<f64>, SimpleError> {
        let mut result = FaceField::filled(mesh, 0.0);
        for (i, wall) in self.walls.iter().enumerate() {
            if *wall {
                result[i] = self.wall_nut(mesh, i, self.molecular_viscosity)?;
            }
        }
        Ok(result)
    }
    pub fn turbulent_viscosity(&self) -> &CellField<f64> {
        &self.fields.nut
    }
    pub fn fields(&self) -> TurbulenceFields {
        self.fields.clone()
    }
    pub fn correct(
        &mut self,
        mesh: &UnstructuredMesh,
        state: &SimpleState,
        bc: &ResolvedVelocityBoundaryConditions,
        nu: f64,
        dt: Option<f64>,
        linear: LinearSolverOptions,
    ) -> Result<(FaceField<f64>, CellField<Vec3>), SimpleError> {
        let gradient = velocity_gradient(mesh, &state.velocity, bc)?;
        let previous = self.fields.clone();
        if self.config.model == TurbulenceModel::SpalartAllmaras {
            self.advance_sa(mesh, state, &gradient, nu, dt, linear)?;
        } else if self.config.model.uses_epsilon() {
            self.advance_epsilon(mesh, state, bc, &gradient, nu, dt, linear)?;
        } else if self.config.is_rans() {
            self.advance_rans(mesh, state, bc, &gradient, nu, dt, linear)?;
        }
        for (i, g) in gradient.iter().enumerate() {
            let mut s2 = 0.0;
            let mut sd2 = 0.0;
            let mut g2 = [[0.0; 3]; 3];
            for a in 0..3 {
                for b in 0..3 {
                    g2[a][b] = (0..3).map(|c| g[a][c] * g[c][b]).sum();
                }
            }
            let trace = (0..3).map(|a| g[a][a]).sum::<f64>();
            let trace_g2 = (0..3).map(|a| g2[a][a]).sum::<f64>();
            for a in 0..3 {
                for b in 0..3 {
                    let strain = 0.5 * (g[a][b] + g[b][a]);
                    s2 += strain * strain;
                    let sd =
                        0.5 * (g2[a][b] + g2[b][a]) - if a == b { trace_g2 / 3.0 } else { 0.0 };
                    sd2 += sd * sd;
                }
            }
            let delta = mesh.cells()[i].volume.cbrt();
            let k = self.fields.k[i].max(1e-14);
            let w = self.fields.omega[i].max(1e-12);
            self.fields.nut[i] = match self.config.model {
                TurbulenceModel::Laminar => 0.0,
                TurbulenceModel::SpalartAllmaras => spalart_allmaras::eddy_viscosity(
                    self.fields.nu_tilda.as_ref().map_or(0.0, |v| v[i]),
                    nu,
                ),
                TurbulenceModel::KOmega => k / w,
                TurbulenceModel::KEpsilon | TurbulenceModel::RngKEpsilon => {
                    self.config.model.epsilon_cmu() * k * k
                        / self.fields.epsilon.as_ref().unwrap()[i].max(1e-16)
                }
                TurbulenceModel::RealizableKEpsilon => {
                    let e = self.fields.epsilon.as_ref().unwrap()[i].max(1e-16);
                    realizable::cmu(g, k, e) * k * k / e
                }
                TurbulenceModel::KOmegaSst => {
                    let y = self.distance[i];
                    let f2 = ((2.0 * k.sqrt() / (0.09 * w * y))
                        .max(500.0 * nu / (y * y * w))
                        .min(100.0)
                        .powi(2))
                    .tanh();
                    0.31 * k / (0.31 * w).max(f2 * (2.0 * s2).sqrt())
                }
                TurbulenceModel::Smagorinsky => {
                    let a = self.config.ce / delta;
                    let b = 2.0 / 3.0 * trace;
                    let c = 2.0 * self.config.ck * delta * (s2 - trace * trace / 3.0).max(0.0);
                    let disc = (b * b + 4.0 * a * c).sqrt();
                    let sqrt_k = if b >= 0.0 {
                        if c == 0.0 {
                            0.0
                        } else {
                            2.0 * c / (b + disc)
                        }
                    } else {
                        (-b + disc) / (2.0 * a)
                    };
                    self.fields.k[i] = sqrt_k * sqrt_k;
                    self.config.ck * delta * sqrt_k
                }
                TurbulenceModel::Wale => {
                    let denominator = s2.powf(2.5) + sd2.powf(1.25);
                    let nut = if denominator > 0.0 {
                        (self.config.cw * delta).powi(2) * sd2.powf(1.5) / denominator
                    } else {
                        0.0
                    };
                    self.fields.k[i] = (nut / (self.config.ck * delta)).powi(2);
                    nut
                }
            };
            if !self.fields.nut[i].is_finite() || self.fields.nut[i] < 0.0 {
                return Err(SimpleError::PhysicalBoundary(format!(
                    "Non-finite turbulent viscosity in cell {i}"
                )));
            }
        }
        self.relative_change = relative_change(&previous.k, &self.fields.k)
            .max(relative_change(&previous.omega, &self.fields.omega))
            .max(relative_change(&previous.nut, &self.fields.nut));
        if let (Some(a), Some(b)) = (&previous.nu_tilda, &self.fields.nu_tilda) {
            self.relative_change = self.relative_change.max(relative_change(a, b));
        }
        if let (Some(a), Some(b)) = (&previous.epsilon, &self.fields.epsilon) {
            self.relative_change = self.relative_change.max(relative_change(a, b));
        }
        if !self.config.is_rans() {
            self.relative_change = relative_change(&previous.nut, &self.fields.nut);
        }
        let mut viscosity = FaceField::filled(mesh, nu);
        let mut source = CellField::filled(mesh, Vec3::ZERO);
        for (i, f) in mesh.faces().iter().enumerate() {
            let o = f.owner;
            let n = f.neighbour.unwrap_or(o);
            let l = if f.neighbour.is_some() {
                (f.center - mesh.cells()[o].center).norm()
                    / ((f.center - mesh.cells()[o].center).norm()
                        + (f.center - mesh.cells()[n].center).norm())
            } else {
                0.0
            };
            let nut = if self.walls[i] {
                self.wall_nut(mesh, i, nu)?
            } else {
                (1.0 - l) * self.fields.nut[o] + l * self.fields.nut[n]
            };
            let inlet = if self.config.is_rans()
                && self.config.model != TurbulenceModel::SpalartAllmaras
                && f.neighbour.is_none()
                && !self.walls[i]
                && state.face_flux[i] < 0.0
            {
                Some(self.incoming_values(mesh, state, bc, i)?)
            } else {
                None
            };
            let nut = if self.config.model == TurbulenceModel::SpalartAllmaras
                && f.neighbour.is_none()
                && !self.walls[i]
                && state.face_flux[i] < 0.0
            {
                spalart_allmaras::eddy_viscosity(self.incoming_sa(i)?, nu)
            } else if let Some((k, w, e)) = inlet {
                if self.config.model == TurbulenceModel::RealizableKEpsilon {
                    // Incoming k/epsilon are prescribed; use the owner velocity gradient.
                    realizable::cmu(&gradient[o], k, e) * k * k / e
                } else if self.config.model.uses_epsilon() {
                    self.config.model.epsilon_cmu() * k * k / e
                } else {
                    k / w
                }
            } else {
                nut
            };
            if !nut.is_finite() || nut < 0.0 {
                return Err(SimpleError::PhysicalBoundary(format!(
                    "Invalid incoming turbulent viscosity on face {i}"
                )));
            }
            viscosity[i] = nu + nut;
            // Remaining transpose/deviatoric stress and isotropic SGS/RANS stress.
            let mut traction = [0.0; 3];
            let area = [f.area_vector.x, f.area_vector.y, f.area_vector.z];
            let tr = (0..3)
                .map(|a| (1.0 - l) * gradient[o][a][a] + l * gradient[n][a][a])
                .sum::<f64>();
            let k = if self.walls[i] {
                0.0
            } else {
                (1.0 - l) * self.fields.k[o] + l * self.fields.k[n]
            };
            let k = inlet.map(|(k, _, _)| k).unwrap_or(k);
            for a in 0..3 {
                for b in 0..3 {
                    let gt = (1.0 - l) * gradient[o][b][a] + l * gradient[n][b][a];
                    traction[a] += (viscosity[i] * gt
                        - if a == b {
                            2.0 / 3.0 * (viscosity[i] * tr + k)
                        } else {
                            0.0
                        })
                        * area[b];
                }
            }
            let t = Vec3::new(traction[0], traction[1], traction[2]);
            source[o] += t / mesh.cells()[o].volume;
            if let Some(n) = f.neighbour {
                source[n] = source[n] - t / mesh.cells()[n].volume;
            }
        }
        Ok((viscosity, source))
    }
}
fn relative_change(a: &CellField<f64>, b: &CellField<f64>) -> f64 {
    a.values()
        .iter()
        .zip(b.values())
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
        / b.values().iter().map(|x| x.abs()).fold(1e-30, f64::max)
}
pub(crate) fn scalar_gradient(
    mesh: &UnstructuredMesh,
    field: &CellField<f64>,
    boundary: &ResolvedScalarBoundaryConditions,
) -> Result<CellField<Vec3>, SimpleError> {
    let mut faces = FaceField::filled(mesh, 0.0);
    for (i, f) in mesh.faces().iter().enumerate() {
        faces[i] = if let Some(n) = f.neighbour {
            let a = (f.center - mesh.cells()[f.owner].center).norm();
            let b = (f.center - mesh.cells()[n].center).norm();
            (b * field[f.owner] + a * field[n]) / (a + b)
        } else {
            match boundary.condition(i) {
                Some(ScalarBoundaryCondition::FixedValue(v)) => v,
                _ => field[f.owner],
            }
        };
    }
    Ok(gauss_gradient_from_faces(mesh, &faces)?)
}
fn velocity_gradient(
    mesh: &UnstructuredMesh,
    u: &CellField<Vec3>,
    bc: &ResolvedVelocityBoundaryConditions,
) -> Result<Vec<[[f64; 3]; 3]>, SimpleError> {
    let gradients = [
        MomentumComponent::X,
        MomentumComponent::Y,
        MomentumComponent::Z,
    ]
    .map(|c| {
        scalar_gradient(
            mesh,
            &momentum_component_field(mesh, u, c)?,
            bc.component(c),
        )
    })
    .into_iter()
    .collect::<Result<Vec<_>, SimpleError>>()?;
    Ok((0..mesh.cell_count())
        .map(|i| {
            gradients
                .iter()
                .map(|v| [v[i].x, v[i].y, v[i].z])
                .collect::<Vec<_>>()
                .try_into()
                .unwrap()
        })
        .collect())
}
