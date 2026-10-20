//! Analytic constraints for canonical circle/axis-aligned rectangle sketches.
//! These profiles have three/four independent scalar degrees of freedom.
use super::*;

impl GeometryTopology {
    /// Resolve a proposed edit against saved position and dimensional constraints.
    /// This is a reduced analytic solver, not an arbitrary curve constraint solver.
    pub fn cad_resolve_sketch_parameters(
        &self,
        id: CadSketchId,
        x: f64,
        y: f64,
        profile: CadSketchProfile,
    ) -> Result<(f64, f64, CadSketchProfile), GeometryError> {
        self.cad_resolve_sketch_parameters_using(id, x, y, profile, None)
    }
    pub fn cad_resolve_sketch_parameters_using(
        &self,
        id: CadSketchId,
        x: f64,
        y: f64,
        profile: CadSketchProfile,
        constraints: Option<&CadSketchConstraints>,
    ) -> Result<(f64, f64, CadSketchProfile), GeometryError> {
        let sketch = self.sketch(id).ok_or_else(|| error("Sketch not found"))?;
        if !x.is_finite() || !y.is_finite() {
            return Err(error("Sketch coordinates must be finite"));
        }
        let valid = match (&sketch.profile, &profile) {
            (Some(CadSketchProfile::Circle { .. }), CadSketchProfile::Circle { radius }) => {
                radius.is_finite() && *radius > GEOMETRY_EPSILON
            }
            (
                Some(CadSketchProfile::Rectangle { .. }),
                CadSketchProfile::Rectangle { width, height },
            ) => {
                width.is_finite()
                    && height.is_finite()
                    && *width > GEOMETRY_EPSILON
                    && *height > GEOMETRY_EPSILON
            }
            _ => false,
        };
        if !valid {
            return Err(error(
                "Keep the sketch type and use positive finite dimensions",
            ));
        }
        let c = constraints.unwrap_or(&sketch.constraints);
        let same = |a: f64, b: f64| (a - b).abs() <= 1e-9 * a.abs().max(b.abs()).max(1.0);
        let merge = |a: Option<f64>, b: Option<f64>| -> Result<Option<f64>, GeometryError> {
            if let (Some(a), Some(b)) = (a, b) {
                if !same(a, b) {
                    return Err(error("Conflicting driving constraints"));
                }
            }
            Ok(a.or(b))
        };
        for value in [c.x, c.y, c.size_x, c.size_y].into_iter().flatten() {
            if !value.is_finite() {
                return Err(error("Constraint values must be finite"));
            }
        }
        if [c.size_x, c.size_y]
            .into_iter()
            .flatten()
            .any(|v| v <= GEOMETRY_EPSILON)
        {
            return Err(error("Driving dimensions must be positive"));
        }
        let fx = merge(sketch.locked_position.map(|p| p[0]), c.x)?;
        let fy = merge(sketch.locked_position.map(|p| p[1]), c.y)?;
        let (old_w, old_h) = match sketch.locked_profile.as_ref() {
            Some(CadSketchProfile::Rectangle { width, height }) => (Some(*width), Some(*height)),
            Some(CadSketchProfile::Circle { radius }) => (Some(*radius), None),
            None => (None, None),
        };
        let fw = merge(old_w, c.size_x)?;
        let fh = merge(old_h, c.size_y)?;
        let (profile, cx, cy) = match profile {
            CadSketchProfile::Circle { radius } => {
                if c.equal_sides || c.size_y.is_some() {
                    return Err(error(
                        "Equal sides and height constraints apply to rectangles",
                    ));
                }
                (
                    CadSketchProfile::Circle {
                        radius: fw.unwrap_or(radius),
                    },
                    0.0,
                    0.0,
                )
            }
            CadSketchProfile::Rectangle { width, height } => {
                let mut w = fw.unwrap_or(width);
                let mut h = fh.unwrap_or(height);
                if c.equal_sides {
                    if let (Some(a), Some(b)) = (fw, fh) {
                        if !same(a, b) {
                            return Err(error(
                                "Equal sides conflicts with unequal width and height constraints",
                            ));
                        }
                    }
                    let side = fw.or(fh).unwrap_or(w);
                    w = side;
                    h = side;
                }
                (
                    CadSketchProfile::Rectangle {
                        width: w,
                        height: h,
                    },
                    -w / 2.0,
                    -h / 2.0,
                )
            }
        };
        if c.centered && (fx.is_some_and(|v| !same(v, cx)) || fy.is_some_and(|v| !same(v, cy))) {
            return Err(error(
                "Center at origin conflicts with the driving X/Y position",
            ));
        }
        Ok((
            if c.centered { cx } else { fx.unwrap_or(x) },
            if c.centered { cy } else { fy.unwrap_or(y) },
            profile,
        ))
    }

    pub fn cad_edit_sketch_constrained(
        &mut self,
        id: CadSketchId,
        x: f64,
        y: f64,
        profile: CadSketchProfile,
        offset: f64,
        constraints: CadSketchConstraints,
    ) -> Result<Vec<BodyId>, GeometryError> {
        let mut next = self.clone();
        next.sketches
            .get_mut(&id)
            .ok_or_else(|| error("Sketch not found"))?
            .constraints = constraints;
        let rebuilt = next.cad_edit_sketch_with_offset(id, x, y, profile, offset)?;
        *self = next;
        Ok(rebuilt)
    }

    pub(crate) fn cad_validate_sketch_constraints(
        &self,
        id: CadSketchId,
    ) -> Result<(), GeometryError> {
        let sketch = self.sketch(id).ok_or_else(|| error("Sketch not found"))?;
        if sketch.constraints == CadSketchConstraints::default() {
            return Ok(());
        }
        let (x, y, profile) = self.cad_sketch_parameters(id)?;
        let (rx, ry, resolved) = self.cad_resolve_sketch_parameters(id, x, y, profile.clone())?;
        let same = |a: f64, b: f64| (a - b).abs() <= 1e-8 * a.abs().max(b.abs()).max(1.0);
        let matches = match (profile, resolved) {
            (CadSketchProfile::Circle { radius: a }, CadSketchProfile::Circle { radius: b }) => {
                same(a, b)
            }
            (
                CadSketchProfile::Rectangle {
                    width: a,
                    height: b,
                },
                CadSketchProfile::Rectangle {
                    width: c,
                    height: d,
                },
            ) => same(a, c) && same(b, d),
            _ => false,
        };
        if !same(x, rx) || !same(y, ry) || !matches {
            return Err(error(
                "Sketch geometry does not satisfy its saved driving constraints",
            ));
        }
        Ok(())
    }

    /// Capture or release the current local position and dimensions. Publication
    /// is atomic and uses the same document Undo/Redo path as other CAD commands.
    pub fn cad_set_sketch_locks(
        &mut self,
        id: CadSketchId,
        position: bool,
        size: bool,
    ) -> Result<(), GeometryError> {
        let (x, y, profile) = self.cad_sketch_parameters(id)?;
        let sketch = self.sketches.get_mut(&id).unwrap();
        sketch.locked_position = position.then_some([x, y]);
        sketch.locked_profile = size.then_some(profile);
        self.bump_revision();
        Ok(())
    }

    pub fn cad_clear_sketch_constraints(&mut self, id: CadSketchId) -> Result<(), GeometryError> {
        let sketch = self
            .sketches
            .get_mut(&id)
            .ok_or_else(|| error("Sketch not found"))?;
        sketch.locked_position = None;
        sketch.locked_profile = None;
        sketch.constraints = CadSketchConstraints::default();
        self.bump_revision();
        Ok(())
    }

    pub fn cad_sketch_degrees_of_freedom(&self, id: CadSketchId) -> Option<usize> {
        self.cad_sketch_degrees_of_freedom_using(id, None)
    }
    pub fn cad_sketch_degrees_of_freedom_using(
        &self,
        id: CadSketchId,
        constraints: Option<&CadSketchConstraints>,
    ) -> Option<usize> {
        let s = self.sketch(id)?;
        if constraints.is_none() {
            if s.drawing.is_some(){return s.drawing_dof;}
            if let Some(owner)=s.region_owner {if let Some(root)=self.sketch(owner){if root.drawing.is_some(){return root.drawing_dof;}}}
        }
        let c = constraints.unwrap_or(&s.constraints);
        let position = if c.centered {
            0
        } else {
            usize::from(s.locked_position.is_none() && c.x.is_none())
                + usize::from(s.locked_position.is_none() && c.y.is_none())
        };
        let size = if s.locked_profile.is_some() {
            0
        } else {
            match s.profile.as_ref()? {
                CadSketchProfile::Circle { .. } => usize::from(c.size_x.is_none()),
                CadSketchProfile::Rectangle { .. } if c.equal_sides => {
                    usize::from(c.size_x.is_none() && c.size_y.is_none())
                }
                CadSketchProfile::Rectangle { .. } => {
                    usize::from(c.size_x.is_none()) + usize::from(c.size_y.is_none())
                }
            }
        };
        Some(position + size)
    }
}
