//! Workbench data shared by geometry, meshing, and boundary setup.
//!
//! The current numerical kernel is a two-dimensional structured-grid solver.
//! This module deliberately keeps the pre-processing model independent from
//! that kernel, so projects can retain named boundaries and 3D extrusion data
//! while the solver grows into true 3D and unstructured support.

use crate::cases::{BoundaryKind, Case, Side};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum GeometryDimension {
    TwoD,
    #[default]
    ExtrudedThreeD,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct GeometryModel {
    /// A 3D extrusion is currently a workbench/visualization model. It does
    /// not claim that the 2D solver has become a 3D numerical solver.
    pub dimension: GeometryDimension,
    pub extrusion_depth: f64,
    /// Parametric solid primitives authored in the Geometry workbench.
    /// They are project data now and will become mesh inputs when the solver
    /// gains arbitrary 3D geometry support.
    pub parts: Vec<GeometryPart>,
}

impl Default for GeometryModel {
    fn default() -> Self {
        Self {
            dimension: GeometryDimension::ExtrudedThreeD,
            extrusion_depth: 0.25,
            parts: Vec::new(),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "kebab-case")]
pub enum GeometryPartKind {
    Box {
        length: f64,
        width: f64,
        height: f64,
    },
    Cylinder {
        radius: f64,
        height: f64,
        #[serde(default = "default_cylinder_segments")]
        segments: usize,
    },
}

impl GeometryPartKind {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Box { .. } => "Box",
            Self::Cylinder { .. } => "Cylinder",
        }
    }

    fn validate(&self) -> Result<(), String> {
        match self {
            Self::Box {
                length,
                width,
                height,
            } if positive(*length) && positive(*width) && positive(*height) => Ok(()),
            Self::Cylinder {
                radius,
                height,
                segments,
            } if positive(*radius) && positive(*height) && *segments >= 8 => Ok(()),
            Self::Box { .. } => Err("box dimensions must be finite and positive".to_string()),
            Self::Cylinder { .. } => Err(
                "cylinder radius/height must be finite and positive and segments must be at least 8"
                    .to_string(),
            ),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GeometryPart {
    pub name: String,
    #[serde(flatten)]
    pub kind: GeometryPartKind,
    /// Position of the primitive centre in model units.
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl GeometryPart {
    pub fn summary(&self) -> String {
        format!(
            "{} · {} · ({:.3}, {:.3}, {:.3})",
            self.name,
            self.kind.label(),
            self.x,
            self.y,
            self.z
        )
    }

    fn validate(&self) -> Result<(), String> {
        if self.name.trim().is_empty() {
            return Err("geometry part name cannot be empty".to_string());
        }
        if !self.x.is_finite() || !self.y.is_finite() || !self.z.is_finite() {
            return Err(format!(
                "geometry part {} has a non-finite position",
                self.name
            ));
        }
        self.kind.validate()
    }
}

fn default_cylinder_segments() -> usize {
    32
}

fn positive(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum MeshTopology {
    #[default]
    Structured,
    Unstructured,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct MeshSettings {
    pub topology: MeshTopology,
    /// Number of layers used by the 3D extrusion preview and retained for a
    /// future 3D mesh generator. The active 2D solver uses nx and ny.
    pub cells_z: usize,
    pub growth_rate: f64,
    pub boundary_layers: usize,
}

impl Default for MeshSettings {
    fn default() -> Self {
        Self {
            topology: MeshTopology::Structured,
            cells_z: 8,
            growth_rate: 1.0,
            boundary_layers: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum BoundaryFace {
    Left,
    Right,
    Bottom,
    Top,
    Front,
    Back,
}

impl BoundaryFace {
    pub const PLANAR: [Self; 4] = [Self::Left, Self::Right, Self::Bottom, Self::Top];

    pub fn side(self) -> Option<Side> {
        match self {
            Self::Left => Some(Side::Left),
            Self::Right => Some(Side::Right),
            Self::Bottom => Some(Side::Bottom),
            Self::Top => Some(Side::Top),
            Self::Front | Self::Back => None,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Left => "Left / inlet",
            Self::Right => "Right / outlet",
            Self::Bottom => "Bottom",
            Self::Top => "Top",
            Self::Front => "Front",
            Self::Back => "Back",
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "kebab-case")]
pub enum BoundaryConditionKind {
    /// Keep the physically meaningful profile supplied by the selected case.
    CaseDefault,
    Velocity {
        u: f64,
        v: f64,
        w: f64,
    },
    PressureOutlet {
        pressure: f64,
    },
    Wall {
        u: f64,
        v: f64,
        w: f64,
    },
    Symmetry,
}

impl BoundaryConditionKind {
    pub fn label(&self) -> &'static str {
        match self {
            Self::CaseDefault => "Case default",
            Self::Velocity { .. } => "Velocity",
            Self::PressureOutlet { .. } => "Pressure outlet",
            Self::Wall { .. } => "Wall",
            Self::Symmetry => "Symmetry",
        }
    }

    fn validate(&self) -> Result<(), String> {
        let finite = |value: f64| value.is_finite();
        match self {
            Self::CaseDefault | Self::Symmetry => Ok(()),
            Self::Velocity { u, v, w } | Self::Wall { u, v, w } => {
                if finite(*u) && finite(*v) && finite(*w) {
                    Ok(())
                } else {
                    Err("boundary velocity components must be finite".to_string())
                }
            }
            Self::PressureOutlet { pressure } if finite(*pressure) => Ok(()),
            Self::PressureOutlet { .. } => Err("outlet pressure must be finite".to_string()),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BoundaryCondition {
    pub name: String,
    pub face: BoundaryFace,
    #[serde(flatten)]
    pub kind: BoundaryConditionKind,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct PreprocessingModel {
    pub geometry: GeometryModel,
    pub mesh: MeshSettings,
    pub boundaries: Vec<BoundaryCondition>,
}

impl PreprocessingModel {
    pub fn validate(&self) -> Result<(), String> {
        if !self.geometry.extrusion_depth.is_finite() || self.geometry.extrusion_depth <= 0.0 {
            return Err("geometry extrusion_depth must be finite and positive".to_string());
        }
        if self.geometry.parts.len() > 128 {
            return Err("geometry contains more than 128 parts".to_string());
        }
        for (index, part) in self.geometry.parts.iter().enumerate() {
            part.validate()?;
            if self.geometry.parts[..index]
                .iter()
                .any(|existing| existing.name == part.name)
            {
                return Err(format!("geometry part name {} is not unique", part.name));
            }
        }
        if self.mesh.cells_z == 0 {
            return Err("mesh cells_z must be positive".to_string());
        }
        if !self.mesh.growth_rate.is_finite() || self.mesh.growth_rate < 1.0 {
            return Err("mesh growth_rate must be finite and at least 1".to_string());
        }
        if self.mesh.boundary_layers > self.mesh.cells_z {
            return Err("mesh boundary_layers cannot exceed cells_z".to_string());
        }
        for face in BoundaryFace::PLANAR {
            let count = self
                .boundaries
                .iter()
                .filter(|boundary| boundary.face == face)
                .count();
            if count != 1 {
                return Err(format!(
                    "expected exactly one named boundary for {}; found {count}",
                    face.label()
                ));
            }
        }
        for face in [BoundaryFace::Front, BoundaryFace::Back] {
            let count = self
                .boundaries
                .iter()
                .filter(|boundary| boundary.face == face)
                .count();
            if count > 1 {
                return Err(format!("duplicate named boundary for {}", face.label()));
            }
        }
        for boundary in &self.boundaries {
            if boundary.name.trim().is_empty() {
                return Err(format!(
                    "{} boundary name cannot be empty",
                    boundary.face.label()
                ));
            }
            boundary.kind.validate()?;
        }
        Ok(())
    }

    pub fn boundary(&self, face: BoundaryFace) -> Option<&BoundaryCondition> {
        self.boundaries
            .iter()
            .find(|boundary| boundary.face == face)
    }

    pub fn boundary_mut(&mut self, face: BoundaryFace) -> Option<&mut BoundaryCondition> {
        self.boundaries
            .iter_mut()
            .find(|boundary| boundary.face == face)
    }

    pub fn solver_overrides(&self) -> SolverBoundaryOverrides {
        let mut overrides = SolverBoundaryOverrides::default();
        for boundary in &self.boundaries {
            let Some(side) = boundary.face.side() else {
                continue;
            };
            let override_kind = match boundary.kind {
                BoundaryConditionKind::CaseDefault => None,
                BoundaryConditionKind::Velocity { u, v, .. }
                | BoundaryConditionKind::Wall { u, v, .. } => {
                    Some(SolverBoundaryOverride::Velocity { u, v })
                }
                BoundaryConditionKind::PressureOutlet { pressure } => {
                    Some(SolverBoundaryOverride::PressureOutlet { pressure })
                }
                BoundaryConditionKind::Symmetry => Some(SolverBoundaryOverride::Symmetry),
            };
            overrides.set(side, override_kind);
        }
        overrides
    }
}

#[derive(Clone, Copy, Debug)]
pub enum SolverBoundaryOverride {
    Velocity { u: f64, v: f64 },
    PressureOutlet { pressure: f64 },
    Symmetry,
}

#[derive(Clone, Debug, Default)]
pub struct SolverBoundaryOverrides {
    left: Option<SolverBoundaryOverride>,
    right: Option<SolverBoundaryOverride>,
    bottom: Option<SolverBoundaryOverride>,
    top: Option<SolverBoundaryOverride>,
}

impl SolverBoundaryOverrides {
    pub fn kind(&self, case: &Case, side: Side) -> BoundaryKind {
        match self.get(side) {
            Some(SolverBoundaryOverride::Velocity { .. }) => BoundaryKind::Velocity,
            Some(SolverBoundaryOverride::PressureOutlet { pressure }) => {
                BoundaryKind::PressureOutlet { pressure }
            }
            Some(SolverBoundaryOverride::Symmetry) => BoundaryKind::Symmetry,
            None => case.boundary_kind(side),
        }
    }

    pub fn velocity(&self, case: &Case, side: Side, x: f64, y: f64, time: f64) -> (f64, f64) {
        match self.get(side) {
            Some(SolverBoundaryOverride::Velocity { u, v }) => (u, v),
            Some(SolverBoundaryOverride::PressureOutlet { .. })
            | Some(SolverBoundaryOverride::Symmetry) => (0.0, 0.0),
            None => case.boundary_velocity(side, x, y, time),
        }
    }

    fn get(&self, side: Side) -> Option<SolverBoundaryOverride> {
        match side {
            Side::Left => self.left,
            Side::Right => self.right,
            Side::Bottom => self.bottom,
            Side::Top => self.top,
        }
    }

    fn set(&mut self, side: Side, value: Option<SolverBoundaryOverride>) {
        match side {
            Side::Left => self.left = value,
            Side::Right => self.right = value,
            Side::Bottom => self.bottom = value,
            Side::Top => self.top = value,
        }
    }
}
