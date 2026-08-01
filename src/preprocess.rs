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
    /// Source 2D profiles retained independently from their generated solids.
    pub sketches: Vec<GeometrySketch>,
    /// Parametric operations applied to sketches, in creation order.
    pub features: Vec<GeometryFeature>,
}

impl Default for GeometryModel {
    fn default() -> Self {
        Self {
            dimension: GeometryDimension::ExtrudedThreeD,
            extrusion_depth: 0.25,
            parts: Vec::new(),
            sketches: Vec::new(),
            features: Vec::new(),
        }
    }
}

/// Plane on which a parametric 2D sketch is authored.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum SketchPlane {
    #[default]
    Xy,
    Xz,
    Yz,
}

impl SketchPlane {
    pub fn label(self) -> &'static str {
        match self {
            Self::Xy => "XY",
            Self::Xz => "XZ",
            Self::Yz => "YZ",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "kebab-case")]
pub enum SketchProfileKind {
    Rectangle { width: f64, height: f64 },
    Circle { radius: f64 },
}

impl SketchProfileKind {
    fn validate(&self) -> Result<(), String> {
        match self {
            Self::Rectangle { width, height } if positive(*width) && positive(*height) => Ok(()),
            Self::Circle { radius } if positive(*radius) => Ok(()),
            Self::Rectangle { .. } => {
                Err("sketch rectangle width and height must be finite and positive".to_string())
            }
            Self::Circle { .. } => {
                Err("sketch circle radius must be finite and positive".to_string())
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GeometrySketch {
    pub name: String,
    pub plane: SketchPlane,
    #[serde(flatten)]
    pub profile: SketchProfileKind,
    /// Sketch origin in model units (mm in the workbench contract).
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// Editable 2D construction geometry; the profile remains available for
    /// feature materialisation and backwards-compatible project files.
    #[serde(default)]
    pub entities: Vec<SketchEntity>,
    #[serde(default)]
    pub dimensions: Vec<SketchDimension>,
    #[serde(default)]
    pub constraints: Vec<SketchConstraint>,
    #[serde(default)]
    pub selected_axis: SketchAxis,
    #[serde(default)]
    pub selected_entity: Option<u64>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum SketchAxis {
    #[default]
    Horizontal,
    Vertical,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum SketchConstraintKind {
    Horizontal,
    Vertical,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SketchConstraint {
    pub entity: u64,
    pub kind: SketchConstraintKind,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "kebab-case")]
pub enum SketchEntityKind {
    Line {
        x1: f64,
        y1: f64,
        x2: f64,
        y2: f64,
    },
    Circle {
        center_x: f64,
        center_y: f64,
        radius: f64,
    },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SketchEntity {
    pub id: u64,
    #[serde(flatten)]
    pub kind: SketchEntityKind,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum SketchDimensionKind {
    Distance,
    Radius,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SketchDimension {
    pub name: String,
    pub kind: SketchDimensionKind,
    /// The geometry controlled by this driving value. Measurement-only
    /// dimensions leave this empty for backwards-compatible project files.
    #[serde(default)]
    pub entity: Option<u64>,
    pub value: f64,
    pub x1: f64,
    pub y1: f64,
    pub x2: f64,
    pub y2: f64,
}

impl GeometrySketch {
    pub fn from_profile(
        name: String,
        plane: SketchPlane,
        profile: SketchProfileKind,
        x: f64,
        y: f64,
        z: f64,
    ) -> Self {
        let mut sketch = Self {
            name,
            plane,
            profile,
            x,
            y,
            z,
            entities: Vec::new(),
            dimensions: Vec::new(),
            constraints: Vec::new(),
            selected_axis: SketchAxis::Horizontal,
            selected_entity: None,
        };
        sketch.seed_profile_entities();
        sketch
    }

    pub fn add_rectangle(
        &mut self,
        center_x: f64,
        center_y: f64,
        width: f64,
        height: f64,
    ) -> Result<(), String> {
        if !positive(width) || !positive(height) || !center_x.is_finite() || !center_y.is_finite() {
            return Err(
                "rectangle coordinates and dimensions must be finite, with positive dimensions"
                    .to_string(),
            );
        }
        let hx = width * 0.5;
        let hy = height * 0.5;
        for (x1, y1, x2, y2) in [
            (center_x - hx, center_y - hy, center_x + hx, center_y - hy),
            (center_x + hx, center_y - hy, center_x + hx, center_y + hy),
            (center_x + hx, center_y + hy, center_x - hx, center_y + hy),
            (center_x - hx, center_y + hy, center_x - hx, center_y - hy),
        ] {
            self.add_line(x1, y1, x2, y2)?;
        }
        Ok(())
    }

    pub fn add_circle(&mut self, center_x: f64, center_y: f64, radius: f64) -> Result<(), String> {
        if !positive(radius) || !center_x.is_finite() || !center_y.is_finite() {
            return Err(
                "circle coordinates and radius must be finite, with a positive radius".to_string(),
            );
        }
        self.entities.push(SketchEntity {
            id: self.next_entity_id(),
            kind: SketchEntityKind::Circle {
                center_x,
                center_y,
                radius,
            },
        });
        Ok(())
    }

    pub fn add_line(&mut self, x1: f64, y1: f64, x2: f64, y2: f64) -> Result<(), String> {
        if [x1, y1, x2, y2].iter().any(|value| !value.is_finite())
            || (x1 - x2).hypot(y1 - y2) <= f64::EPSILON
        {
            return Err("line endpoints must be finite and distinct".to_string());
        }
        self.entities.push(SketchEntity {
            id: self.next_entity_id(),
            kind: SketchEntityKind::Line { x1, y1, x2, y2 },
        });
        Ok(())
    }

    pub fn add_distance_dimension(
        &mut self,
        x1: f64,
        y1: f64,
        x2: f64,
        y2: f64,
    ) -> Result<(), String> {
        if [x1, y1, x2, y2].iter().any(|value| !value.is_finite()) {
            return Err("dimension points must be finite".to_string());
        }
        let value = (x2 - x1).hypot(y2 - y1);
        if value <= f64::EPSILON {
            return Err("dimension points must be distinct".to_string());
        }
        self.dimensions.push(SketchDimension {
            name: format!("D{}", self.dimensions.len() + 1),
            kind: SketchDimensionKind::Distance,
            entity: None,
            value,
            x1,
            y1,
            x2,
            y2,
        });
        Ok(())
    }

    pub fn select_entity_near(&mut self, x: f64, y: f64, tolerance: f64) -> Option<u64> {
        let selected = self
            .entities
            .iter()
            .filter_map(|entity| {
                let distance = match &entity.kind {
                    SketchEntityKind::Line { x1, y1, x2, y2 } => {
                        point_line_distance(x, y, *x1, *y1, *x2, *y2)
                    }
                    SketchEntityKind::Circle {
                        center_x,
                        center_y,
                        radius,
                    } => ((x - *center_x).hypot(y - *center_y) - *radius).abs(),
                };
                (distance <= tolerance).then_some((entity.id, distance))
            })
            .min_by(|(_, left), (_, right)| left.total_cmp(right))
            .map(|(id, _)| id);
        self.selected_entity = selected;
        selected
    }

    /// Constrains the selected line to the requested axis and records the
    /// relationship in project data. The line's first endpoint is kept fixed.
    pub fn apply_selected_axis_constraint(&mut self, axis: SketchAxis) -> Result<(), String> {
        let Some(id) = self.selected_entity else {
            return Err("select a line before applying an axis constraint".to_string());
        };
        let Some(entity) = self.entities.iter_mut().find(|entity| entity.id == id) else {
            return Err("selected sketch entity no longer exists".to_string());
        };
        let SketchEntityKind::Line { x1, y1, x2, y2 } = &mut entity.kind else {
            return Err("axis constraints apply only to lines".to_string());
        };
        match axis {
            SketchAxis::Horizontal if (*x2 - *x1).abs() <= f64::EPSILON => {
                return Err(
                    "cannot make a vertical line horizontal without collapsing it".to_string(),
                )
            }
            SketchAxis::Vertical if (*y2 - *y1).abs() <= f64::EPSILON => {
                return Err(
                    "cannot make a horizontal line vertical without collapsing it".to_string(),
                )
            }
            SketchAxis::Horizontal => *y2 = *y1,
            SketchAxis::Vertical => *x2 = *x1,
        }
        self.constraints
            .retain(|constraint| constraint.entity != id);
        self.constraints.push(SketchConstraint {
            entity: id,
            kind: match axis {
                SketchAxis::Horizontal => SketchConstraintKind::Horizontal,
                SketchAxis::Vertical => SketchConstraintKind::Vertical,
            },
        });
        self.selected_axis = axis;
        Ok(())
    }

    /// Applies a driving length/radius to the selected entity. Lines preserve
    /// their start point and direction; circles preserve their centre.
    pub fn set_selected_dimension(&mut self, value: f64) -> Result<(), String> {
        if !positive(value) {
            return Err("driving dimension must be finite and positive".to_string());
        }
        let Some(id) = self.selected_entity else {
            return Err("select a line or circle before applying a driving dimension".to_string());
        };
        let Some(entity) = self.entities.iter_mut().find(|entity| entity.id == id) else {
            return Err("selected sketch entity no longer exists".to_string());
        };
        match &mut entity.kind {
            SketchEntityKind::Line { x1, y1, x2, y2 } => {
                let dx = *x2 - *x1;
                let dy = *y2 - *y1;
                let length = dx.hypot(dy);
                if length <= f64::EPSILON {
                    return Err("cannot dimension a zero-length line".to_string());
                }
                *x2 = *x1 + dx / length * value;
                *y2 = *y1 + dy / length * value;
            }
            SketchEntityKind::Circle { radius, .. } => *radius = value,
        }
        if let Some(dimension) = self
            .dimensions
            .iter_mut()
            .find(|dimension| dimension.entity == Some(id))
        {
            dimension.value = value;
            dimension.x2 = value;
        } else {
            self.dimensions.push(SketchDimension {
                name: format!("D{}", self.dimensions.len() + 1),
                kind: SketchDimensionKind::Distance,
                entity: Some(id),
                value,
                x1: 0.0,
                y1: 0.0,
                x2: value,
                y2: 0.0,
            });
        }
        Ok(())
    }

    /// Trims the closest line endpoint to its closest line-line intersection.
    /// It is deterministic and only succeeds where actual construction
    /// geometry supplies an intersection to trim to.
    pub fn trim_line_near(&mut self, x: f64, y: f64) -> Result<(), String> {
        if !x.is_finite() || !y.is_finite() {
            return Err("trim point must be finite".to_string());
        }
        let Some(target_index) = self.closest_line_index(x, y) else {
            return Err("trim requires a line near the cursor".to_string());
        };
        let (x1, y1, x2, y2) = match &self.entities[target_index].kind {
            SketchEntityKind::Line { x1, y1, x2, y2 } => (*x1, *y1, *x2, *y2),
            SketchEntityKind::Circle { .. } => {
                unreachable!("closest_line_index only returns lines")
            }
        };
        if point_line_distance(x, y, x1, y1, x2, y2) > 0.15 {
            return Err("trim cursor must be close to the line to trim".to_string());
        }
        let mut closest: Option<(f64, f64, f64)> = None;
        for (index, entity) in self.entities.iter().enumerate() {
            if index == target_index {
                continue;
            }
            let SketchEntityKind::Line {
                x1: ox1,
                y1: oy1,
                x2: ox2,
                y2: oy2,
            } = &entity.kind
            else {
                continue;
            };
            let Some((ix, iy, t)) =
                line_intersection_with_t((x1, y1), (x2, y2), (*ox1, *oy1), (*ox2, *oy2))
            else {
                continue;
            };
            let distance = (ix - x).hypot(iy - y);
            if closest.is_none_or(|(_, _, current)| distance < current) {
                closest = Some((ix, iy, distance));
            }
            let _ = t;
        }
        let Some((ix, iy, _)) = closest else {
            return Err("trim requires an intersecting construction line".to_string());
        };
        if (x - x1).hypot(y - y1) <= (x - x2).hypot(y - y2) {
            self.entities[target_index].kind = SketchEntityKind::Line {
                x1: ix,
                y1: iy,
                x2,
                y2,
            };
        } else {
            self.entities[target_index].kind = SketchEntityKind::Line {
                x1,
                y1,
                x2: ix,
                y2: iy,
            };
        }
        self.selected_entity = Some(self.entities[target_index].id);
        Ok(())
    }

    fn seed_profile_entities(&mut self) {
        match &self.profile {
            SketchProfileKind::Rectangle { width, height } => {
                let _ = self.add_rectangle(0.0, 0.0, *width, *height);
            }
            SketchProfileKind::Circle { radius } => {
                let _ = self.add_circle(0.0, 0.0, *radius);
            }
        }
    }

    fn next_entity_id(&self) -> u64 {
        self.entities
            .iter()
            .map(|entity| entity.id)
            .max()
            .unwrap_or(0)
            + 1
    }

    fn closest_line_index(&self, x: f64, y: f64) -> Option<usize> {
        self.entities
            .iter()
            .enumerate()
            .filter_map(|(index, entity)| match &entity.kind {
                SketchEntityKind::Line { x1, y1, x2, y2 } => {
                    Some((index, point_line_distance(x, y, *x1, *y1, *x2, *y2)))
                }
                SketchEntityKind::Circle { .. } => None,
            })
            .min_by(|(_, left), (_, right)| left.total_cmp(right))
            .map(|(index, _)| index)
    }

    /// Resolves editable construction geometry to one profile supported by the
    /// current feature materializer. This makes the feature reflect the drawn
    /// sketch rather than silently using its initial profile settings.
    fn materialization_profile(&self) -> Result<(SketchProfileKind, f64, f64), String> {
        if self.entities.is_empty() {
            return Ok((self.profile.clone(), 0.0, 0.0));
        }

        if let [SketchEntity {
            kind:
                SketchEntityKind::Circle {
                    center_x,
                    center_y,
                    radius,
                },
            ..
        }] = self.entities.as_slice()
        {
            return Ok((
                SketchProfileKind::Circle { radius: *radius },
                *center_x,
                *center_y,
            ));
        }

        let lines = self
            .entities
            .iter()
            .map(|entity| match entity.kind {
                SketchEntityKind::Line { x1, y1, x2, y2 } => Ok((x1, y1, x2, y2)),
                SketchEntityKind::Circle { .. } => Err(()),
            })
            .collect::<Result<Vec<_>, _>>();
        let Ok(lines) = lines else {
            return Err("a feature sketch must contain exactly one circle or a closed axis-aligned rectangle".to_string());
        };
        if lines.len() != 4 {
            return Err("a feature sketch must contain exactly one circle or a closed axis-aligned rectangle".to_string());
        }

        let (min_x, max_x, min_y, max_y) = lines.iter().fold(
            (
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
            ),
            |(min_x, max_x, min_y, max_y), (x1, y1, x2, y2)| {
                (
                    min_x.min(*x1).min(*x2),
                    max_x.max(*x1).max(*x2),
                    min_y.min(*y1).min(*y2),
                    max_y.max(*y1).max(*y2),
                )
            },
        );
        let expected_edges = [
            (min_x, min_y, max_x, min_y),
            (max_x, min_y, max_x, max_y),
            (max_x, max_y, min_x, max_y),
            (min_x, max_y, min_x, min_y),
        ];
        let mut matched = [false; 4];
        for line in lines {
            let Some(index) = expected_edges.iter().enumerate().find_map(|(index, edge)| {
                (!matched[index] && same_line_undirected(line, *edge)).then_some(index)
            }) else {
                return Err("a feature sketch must contain exactly one circle or a closed axis-aligned rectangle".to_string());
            };
            matched[index] = true;
        }

        Ok((
            SketchProfileKind::Rectangle {
                width: max_x - min_x,
                height: max_y - min_y,
            },
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
        ))
    }

    fn validate(&self) -> Result<(), String> {
        if self.name.trim().is_empty() {
            return Err("sketch name cannot be empty".to_string());
        }
        if !self.x.is_finite() || !self.y.is_finite() || !self.z.is_finite() {
            return Err(format!("sketch {} has a non-finite origin", self.name));
        }
        self.profile.validate()?;
        for entity in &self.entities {
            match entity.kind {
                SketchEntityKind::Line { x1, y1, x2, y2 }
                    if [x1, y1, x2, y2].iter().all(|value| value.is_finite())
                        && (x1 - x2).hypot(y1 - y2) > f64::EPSILON => {}
                SketchEntityKind::Circle {
                    center_x,
                    center_y,
                    radius,
                } if center_x.is_finite() && center_y.is_finite() && positive(radius) => {}
                SketchEntityKind::Line { .. } => {
                    return Err(format!("sketch {} contains an invalid line", self.name));
                }
                SketchEntityKind::Circle { .. } => {
                    return Err(format!("sketch {} contains an invalid circle", self.name));
                }
            }
        }
        for constraint in &self.constraints {
            let Some(entity) = self
                .entities
                .iter()
                .find(|entity| entity.id == constraint.entity)
            else {
                return Err(format!(
                    "sketch {} contains a constraint for a missing entity",
                    self.name
                ));
            };
            let valid = match (&constraint.kind, &entity.kind) {
                (SketchConstraintKind::Horizontal, SketchEntityKind::Line { y1, y2, .. }) => {
                    (*y1 - *y2).abs() <= f64::EPSILON
                }
                (SketchConstraintKind::Vertical, SketchEntityKind::Line { x1, x2, .. }) => {
                    (*x1 - *x2).abs() <= f64::EPSILON
                }
                _ => false,
            };
            if !valid {
                return Err(format!(
                    "sketch {} contains an unsatisfied axis constraint",
                    self.name
                ));
            }
        }
        for dimension in &self.dimensions {
            if dimension.name.trim().is_empty()
                || !positive(dimension.value)
                || [dimension.x1, dimension.y1, dimension.x2, dimension.y2]
                    .iter()
                    .any(|value| !value.is_finite())
            {
                return Err(format!(
                    "sketch {} contains an invalid dimension",
                    self.name
                ));
            }
        }
        Ok(())
    }
}

fn same_line_undirected(
    (x1, y1, x2, y2): (f64, f64, f64, f64),
    (expected_x1, expected_y1, expected_x2, expected_y2): (f64, f64, f64, f64),
) -> bool {
    const TOLERANCE: f64 = 1.0e-9;
    let same_point = |x: f64, y: f64, expected_x: f64, expected_y: f64| {
        (x - expected_x).abs() <= TOLERANCE && (y - expected_y).abs() <= TOLERANCE
    };
    (same_point(x1, y1, expected_x1, expected_y1) && same_point(x2, y2, expected_x2, expected_y2))
        || (same_point(x1, y1, expected_x2, expected_y2)
            && same_point(x2, y2, expected_x1, expected_y1))
}

fn point_line_distance(x: f64, y: f64, x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    let dx = x2 - x1;
    let dy = y2 - y1;
    let length_sq = dx * dx + dy * dy;
    let t = if length_sq <= f64::EPSILON {
        0.0
    } else {
        ((x - x1) * dx + (y - y1) * dy) / length_sq
    }
    .clamp(0.0, 1.0);
    (x - (x1 + t * dx)).hypot(y - (y1 + t * dy))
}

fn line_intersection_with_t(
    (x1, y1): (f64, f64),
    (x2, y2): (f64, f64),
    (x3, y3): (f64, f64),
    (x4, y4): (f64, f64),
) -> Option<(f64, f64, f64)> {
    let denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if denominator.abs() <= f64::EPSILON {
        return None;
    }
    let t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator;
    let u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / denominator;
    if !(0.0..=1.0).contains(&t) || !(0.0..=1.0).contains(&u) {
        return None;
    }
    Some((x1 + t * (x2 - x1), y1 + t * (y2 - y1), t))
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "kebab-case")]
pub enum GeometryFeatureKind {
    Extrude {
        depth: f64,
    },
    /// The current portable preview supports complete, 360 degree revolutions.
    Revolve {
        axis_offset: f64,
        angle_degrees: f64,
    },
}

impl GeometryFeatureKind {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Extrude { .. } => "Extrude",
            Self::Revolve { .. } => "Revolve",
        }
    }

    fn validate(&self) -> Result<(), String> {
        match self {
            Self::Extrude { depth } if positive(*depth) => Ok(()),
            Self::Extrude { .. } => Err("extrude depth must be finite and positive".to_string()),
            Self::Revolve {
                axis_offset,
                angle_degrees,
            } if positive(*axis_offset) && (*angle_degrees - 360.0).abs() < f64::EPSILON => Ok(()),
            Self::Revolve { .. } => Err(
                "revolve axis offset must be positive and the current preview supports a complete 360 degree revolution"
                    .to_string(),
            ),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GeometryFeature {
    pub name: String,
    pub sketch: String,
    #[serde(flatten)]
    pub kind: GeometryFeatureKind,
    /// Name of the generated solid in `GeometryModel.parts`.
    pub output_part: String,
}

impl GeometryModel {
    /// Adds a retained 2D sketch, a feature definition, and its deterministic
    /// preview solid together so saved projects retain the CAD design intent.
    pub fn add_sketch_feature(
        &mut self,
        mut sketch: GeometrySketch,
        feature_name: String,
        kind: GeometryFeatureKind,
    ) -> Result<String, String> {
        let (profile, offset_x, offset_y) = sketch.materialization_profile()?;
        sketch.profile = profile;
        sketch.x += offset_x;
        sketch.y += offset_y;
        sketch.validate()?;
        kind.validate()?;
        if feature_name.trim().is_empty() {
            return Err("feature name cannot be empty".to_string());
        }
        if self.sketches.iter().any(|item| item.name == sketch.name) {
            return Err(format!("sketch name {} is not unique", sketch.name));
        }
        if self.features.iter().any(|item| item.name == feature_name) {
            return Err(format!("feature name {feature_name} is not unique"));
        }
        if self.parts.len() >= 128 {
            return Err("geometry contains more than 128 parts".to_string());
        }
        let output_part = format!("{feature_name} solid");
        if self.parts.iter().any(|part| part.name == output_part) {
            return Err(format!("geometry part name {output_part} is not unique"));
        }
        let part = materialize_feature(&sketch, &output_part, &kind)?;
        self.sketches.push(sketch.clone());
        self.features.push(GeometryFeature {
            name: feature_name,
            sketch: sketch.name,
            kind,
            output_part: output_part.clone(),
        });
        self.parts.push(part);
        Ok(output_part)
    }
}

fn materialize_feature(
    sketch: &GeometrySketch,
    output_part: &str,
    feature: &GeometryFeatureKind,
) -> Result<GeometryPart, String> {
    let kind = match (&sketch.profile, feature) {
        (SketchProfileKind::Rectangle { width, height }, GeometryFeatureKind::Extrude { depth }) => {
            GeometryPartKind::Box {
                length: *width,
                width: *height,
                height: *depth,
            }
        }
        (SketchProfileKind::Circle { radius }, GeometryFeatureKind::Extrude { depth }) => {
            GeometryPartKind::Cylinder {
                radius: *radius,
                height: *depth,
                segments: 32,
            }
        }
        (
            SketchProfileKind::Circle { radius },
            GeometryFeatureKind::Revolve { axis_offset, .. },
        ) => GeometryPartKind::Torus {
            major_radius: *axis_offset,
            minor_radius: *radius,
            segments: 32,
        },
        (SketchProfileKind::Rectangle { .. }, GeometryFeatureKind::Revolve { .. }) => {
            return Err(
                "revolve currently requires a circular 2D profile; arbitrary profile revolution needs the native B-Rep kernel"
                    .to_string(),
            )
        }
    };
    Ok(GeometryPart {
        name: output_part.to_string(),
        kind,
        x: sketch.x,
        y: sketch.y,
        z: sketch.z,
    })
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
    Cone {
        radius: f64,
        height: f64,
        #[serde(default = "default_cylinder_segments")]
        segments: usize,
    },
    Sphere {
        radius: f64,
        #[serde(default = "default_sphere_segments")]
        segments: usize,
    },
    Torus {
        major_radius: f64,
        minor_radius: f64,
        #[serde(default = "default_cylinder_segments")]
        segments: usize,
    },
}

impl GeometryPartKind {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Box { .. } => "Box",
            Self::Cylinder { .. } => "Cylinder",
            Self::Cone { .. } => "Cone",
            Self::Sphere { .. } => "Sphere",
            Self::Torus { .. } => "Torus",
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
            Self::Cone {
                radius,
                height,
                segments,
            } if positive(*radius) && positive(*height) && *segments >= 8 => Ok(()),
            Self::Cone { .. } => Err(
                "cone radius/height must be finite and positive and segments must be at least 8"
                    .to_string(),
            ),
            Self::Sphere { radius, segments } if positive(*radius) && *segments >= 8 => Ok(()),
            Self::Sphere { .. } => Err(
                "sphere radius must be finite and positive and segments must be at least 8".to_string(),
            ),
            Self::Torus {
                major_radius,
                minor_radius,
                segments,
            } if positive(*major_radius)
                && positive(*minor_radius)
                && *major_radius > *minor_radius
                && *segments >= 8 => Ok(()),
            Self::Torus { .. } => Err(
                "torus radii must be finite and positive, major radius must exceed minor radius, and segments must be at least 8"
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

fn default_sphere_segments() -> usize {
    24
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
        for (index, sketch) in self.geometry.sketches.iter().enumerate() {
            sketch.validate()?;
            if self.geometry.sketches[..index]
                .iter()
                .any(|existing| existing.name == sketch.name)
            {
                return Err(format!("sketch name {} is not unique", sketch.name));
            }
        }
        for (index, feature) in self.geometry.features.iter().enumerate() {
            if feature.name.trim().is_empty() {
                return Err("feature name cannot be empty".to_string());
            }
            feature.kind.validate()?;
            if !self
                .geometry
                .sketches
                .iter()
                .any(|sketch| sketch.name == feature.sketch)
            {
                return Err(format!(
                    "feature {} references a missing sketch",
                    feature.name
                ));
            }
            if self.geometry.features[..index]
                .iter()
                .any(|existing| existing.name == feature.name)
            {
                return Err(format!("feature name {} is not unique", feature.name));
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
