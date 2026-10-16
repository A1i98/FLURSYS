//! Exact clearance checks for circular subtraction from planar XY profiles.
use super::*;

fn invalid(message: &str) -> GeometryError {
    GeometryError::InvalidPrimitive {
        message: message.into(),
    }
}

impl GeometryTopology {
    pub(super) fn validate_circle_cut(
        &self,
        face: FaceId,
        center: Vec3,
        radius: f64,
    ) -> Result<(), GeometryError> {
        if self.cad_owner(face).is_some()
            || self.bodies.values().any(|body| body.faces.contains(&face))
            || self
                .extrude_features
                .values()
                .any(|f| f.source_face == face)
            || self.pocket_features.values().any(|f| f.source_face == face)
            || self.cad_bodies.values().any(|b| {
                b.dependencies
                    .as_ref()
                    .is_some_and(|d| d.profiles.contains(&face))
            })
        {
            return Err(invalid("This profile already drives a solid. Undo the extrusion or create a new independent fluid sketch before cutting it"));
        }
        let GeometryFaceRepresentation::Planar {
            outer_loop,
            inner_loops,
        } = &self.require_face(face)?.representation
        else {
            return Err(invalid("Circular subtraction requires a planar fluid face"));
        };
        for (index, boundary) in std::iter::once(outer_loop)
            .chain(inner_loops.iter())
            .enumerate()
        {
            let mut points = Vec::new();
            let mut circle: Option<(Vec3, f64)> = None;
            for e in boundary {
                match self.require_edge(e.edge)?.geometry {
                    EdgeGeometry::Line { start, end } => {
                        let a = self.vertex_position(if e.reversed { end } else { start })?;
                        let b = self.vertex_position(if e.reversed { start } else { end })?;
                        if (a.z - center.z).abs() > 1e-9 || (b.z - center.z).abs() > 1e-9 {
                            return Err(invalid(
                                "Circular subtraction requires coplanar XY geometry",
                            ));
                        }
                        let d = b - a;
                        let t = ((center - a).dot(d) / d.dot(d)).clamp(0.0, 1.0);
                        if (center - (a + d * t)).norm() <= radius + 1e-9 {
                            return Err(invalid(
                                "The circle touches or crosses an existing boundary",
                            ));
                        }
                        points.push(a);
                    }
                    EdgeGeometry::CircularArc {
                        start, center: c, ..
                    } => {
                        let c = self.vertex_position(c)?;
                        let r = (self.vertex_position(start)? - c).norm();
                        if (c.z - center.z).abs() > 1e-9 {
                            return Err(invalid(
                                "Circular subtraction requires coplanar XY geometry",
                            ));
                        }
                        if circle.is_some_and(|(previous, r0)| {
                            (previous - c).norm() > 1e-9 || (r0 - r).abs() > 1e-9
                        }) {
                            return Err(invalid("Mixed curved boundaries require a general planar Boolean operation"));
                        }
                        circle = Some((c, r));
                    }
                    _ => {
                        return Err(invalid(
                            "Use an independent planar profile for circular subtraction",
                        ))
                    }
                }
            }
            if let Some((c, r)) = circle {
                if !points.is_empty() {
                    return Err(invalid(
                        "Mixed line/arc boundaries require a general planar Boolean operation",
                    ));
                }
                let distance = (center - c).norm();
                if (index == 0 && distance + radius >= r - 1e-9)
                    || (index > 0 && distance <= radius + r + 1e-9)
                {
                    return Err(invalid("The circle must be strictly inside the outer boundary and clear of every existing hole"));
                }
            } else {
                let mut inside = false;
                for i in 0..points.len() {
                    let a = points[i];
                    let b = points[(i + 1) % points.len()];
                    if (a.y > center.y) != (b.y > center.y)
                        && center.x < (b.x - a.x) * (center.y - a.y) / (b.y - a.y) + a.x
                    {
                        inside = !inside;
                    }
                }
                if (index == 0 && !inside) || (index > 0 && inside) {
                    return Err(invalid(
                        "The circle center must lie in the fluid region, outside existing holes",
                    ));
                }
            }
        }
        Ok(())
    }
}
