//! Complete locally vendored parametric sketch core and FLURSYS command adapter.
//! Upstream provenance and licences: vendor/cad-workbench/HOST-INTEGRATION.json.
pub use cad_geom as geom;
pub use cad_sketch as core;
mod corners;
mod freeform;
mod modify;
mod support;
mod tools;
pub use tools::{constraint, draw, edit, segment_curve, ToolParameters};

pub fn validate(sk: &core::Sketch) -> Result<(), String> {
    if sk.points.len() > core::MAX_POINTS
        || sk.curves.len() > core::MAX_CURVES
        || sk.constraints.len() > core::MAX_CONSTRAINTS
    {
        return Err("Sketch exceeds the supported entity budget".into());
    }
    if sk.points.iter().any(|p| {
        !p.pos.is_finite() || p.pos.x.abs() >= core::MAX_COORD || p.pos.y.abs() >= core::MAX_COORD
    }) {
        return Err("Sketch contains invalid coordinates".into());
    }
    let mut ids = std::collections::BTreeSet::new();
    for p in &sk.points {
        if !ids.insert(p.id.clone()) {
            return Err("Duplicate sketch entity ID".into());
        }
    }
    for c in &sk.curves {
        if !ids.insert(c.id.clone()) || c.kind.point_ids().iter().any(|i| *i >= sk.points.len()) {
            return Err("Invalid sketch curve references".into());
        }
        match &c.kind {
            core::CurveKind::Circle { r, .. } | core::CurveKind::Ellipse { r, .. }
                if !r.is_finite() || *r <= 1e-9 =>
            {
                return Err("Invalid curve radius".into())
            }
            core::CurveKind::Conic { rho, .. }
                if !rho.is_finite() || *rho <= 0.0 || *rho >= 1.0 =>
            {
                return Err("Invalid conic ratio".into())
            }
            core::CurveKind::Spline { pts, degree, .. }
                if pts.len() < 2
                    || pts.len() > core::MAX_SPLINE_POINTS
                    || !(1..=9).contains(degree) =>
            {
                return Err("Invalid spline definition".into())
            }
            _ => {}
        }
    }
    for c in &sk.constraints {
        sk.validate(&c.kind).map_err(|e| e.to_string())?;
    }
    sk.validate_patterns().map_err(|e| e.to_string())?;
    sk.validate_periodic_splines().map_err(|e| e.to_string())?;
    sk.validate_freeform_offsets().map_err(|e| e.to_string())?;
    sk.validate_offset_topologies().map_err(|e| e.to_string())?;
    Ok(())
}

/// Publish only a consistent edit. A failed constraint or operation preserves
/// coordinates, IDs, constraints, links, patterns and dimensions together.
pub fn transaction(
    sk: &mut core::Sketch,
    f: impl FnOnce(&mut core::Sketch) -> Result<(), String>,
) -> Result<core::SolveReport, String> {
    let mut next = sk.clone();
    let old = next.driving_values();
    f(&mut next)?;
    validate(&next)?;
    let report = core::solve_after_value_edit(&mut next, &old);
    if !report.ok() {
        return Err(format!(
            "Constraint solve failed: residual {:.3e}; conflicting: {}",
            report.max_residual,
            report.failing.join(", ")
        ));
    }
    *sk = next;
    Ok(report)
}
