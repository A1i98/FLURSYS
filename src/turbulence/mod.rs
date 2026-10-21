//! Incompressible eddy-viscosity closures on a fixed unstructured mesh.
//! RANS uses wall-resolved k-omega equations; no high-Re wall functions.
use crate::*;
use serde::{Deserialize, Serialize};
mod transport;
mod wall_distance;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum TurbulenceModel {
    #[default]
    Laminar,
    KOmega,
    KOmegaSst,
    Smagorinsky,
    Wale,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct TurbulenceOptions {
    pub model: TurbulenceModel,
    /// Uniform initial and incoming k (m²/s²) and omega (1/s).
    pub k: f64,
    pub omega: f64,
    pub relaxation: f64,
    pub ck: f64,
    pub ce: f64,
    pub cw: f64,
}
impl Default for TurbulenceOptions {
    fn default() -> Self {
        Self {
            model: TurbulenceModel::Laminar,
            k: 1e-4,
            omega: 1.0,
            relaxation: 0.5,
            ck: 0.094,
            ce: 1.048,
            cw: 0.325,
        }
    }
}
impl TurbulenceOptions {
    pub fn validate(&self) -> Result<(), String> {
        if [self.k, self.omega, self.ck, self.ce, self.cw]
            .iter()
            .any(|x| !x.is_finite() || *x <= 0.0)
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
            TurbulenceModel::KOmega | TurbulenceModel::KOmegaSst
        )
    }
}
#[derive(Clone, Debug)]
pub struct TurbulenceFields {
    pub model: TurbulenceModel,
    pub k: CellField<f64>,
    pub omega: CellField<f64>,
    pub nut: CellField<f64>,
}
impl TurbulenceFields {
    pub fn new(mesh: &UnstructuredMesh, options: &TurbulenceOptions) -> Self {
        Self {
            model: options.model,
            k: CellField::filled(mesh, options.k),
            omega: CellField::filled(mesh, options.omega),
            nut: CellField::filled(mesh, 0.0),
        }
    }
}

#[derive(Debug)]
pub(crate) struct Closure {
    config: TurbulenceOptions,
    fields: TurbulenceFields,
    walls: Vec<bool>,
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
        let distance = if config.is_rans() {
            wall_distance::distances(mesh, &walls)?
        } else {
            vec![1.0; mesh.cell_count()]
        };
        let fields = old
            .clone()
            .filter(|f| f.model == config.model)
            .unwrap_or_else(|| TurbulenceFields::new(mesh, &config));
        fields.k.ensure_mesh(mesh)?;
        fields.omega.ensure_mesh(mesh)?;
        fields.nut.ensure_mesh(mesh)?;
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
            config,
            old: Some(fields.clone()),
            fields,
            walls,
            distance,
            relative_change: 0.0,
        })
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
        if self.config.is_rans() {
            self.advance_rans(mesh, state, &gradient, nu, dt, linear)?;
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
                TurbulenceModel::KOmega => k / w,
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
                0.0
            } else {
                (1.0 - l) * self.fields.nut[o] + l * self.fields.nut[n]
            };
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
