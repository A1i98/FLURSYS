//! Editable sketch drafts and their geometric constraints.
use serde::{Deserialize, Serialize};
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
    pub fn validate(&self) -> Result<(), String> {
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
    /// Stable identity used by features.
    pub id: u64,
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
            id: 0,
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

    pub fn validate(&self) -> Result<(), String> {
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

fn positive(value: f64) -> bool {
    value.is_finite() && value > 0.0
}
