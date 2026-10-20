//! Thin drawing commands over the imported sketch model; geometry is retained
//! as analytic curves and constraints rather than UI-only polylines.
use super::{
    core::*,
    geom::{Seg2, Vec2},
};
use std::collections::{BTreeMap, BTreeSet};
#[derive(Clone, Debug)]
pub struct ToolParameters {
    pub value: f64,
    pub secondary: f64,
    pub count: usize,
    pub text: String,
}
impl Default for ToolParameters {
    fn default() -> Self {
        Self {
            value: 1.0,
            secondary: 30.0,
            count: 6,
            text: "Text".into(),
        }
    }
}
fn err(e: impl std::fmt::Display) -> String {
    e.to_string()
}
fn point(sk: &mut Sketch, p: Vec2) -> Result<usize, String> {
    if let Some(i) = sk.points.iter().position(|q| q.pos.dist(p) < 1e-7) {
        return Ok(i);
    }
    sk.add_point(p, None).map_err(err)
}
fn line(sk: &mut Sketch, a: Vec2, b: Vec2) -> Result<usize, String> {
    let a = point(sk, a)?;
    let b = point(sk, b)?;
    sk.add_line_pts(a, b, None).map_err(err)
}
fn circ3(a: Vec2, b: Vec2, c: Vec2) -> Result<Vec2, String> {
    let u = b - a;
    let v = c - a;
    let d = 2.0 * u.cross(v);
    if d.abs() < 1e-12 {
        return Err("Choose three distinct, non-collinear points".into());
    }
    Ok(a + Vec2::new(
        v.y * u.len2() - u.y * v.len2(),
        u.x * v.len2() - v.x * u.len2(),
    ) / d)
}
fn arc(sk: &mut Sketch, c: Vec2, a: Vec2, b: Vec2) -> Result<usize, String> {
    let ci = point(sk, c)?;
    let ai = point(sk, a)?;
    let bi = point(sk, b)?;
    sk.add_arc(c, a, b, [Some(ci), Some(ai), Some(bi)], None)
        .map_err(err)
}
pub fn segment_curve(sk: &mut Sketch, s: Seg2) -> Result<usize, String> {
    match s {
        Seg2::Line { a, b } => line(sk, a, b),
        Seg2::Arc { center, sweep, .. } => {
            if sweep >= 0.0 {
                arc(sk, center, s.start(), s.end())
            } else {
                let i = arc(sk, center, s.end(), s.start())?;
                sk.curves[i].reversed = true;
                Ok(i)
            }
        }
        Seg2::Cubic { p0, p1, p2, p3 } => {
            let pts = [p0, p1, p2, p3]
                .into_iter()
                .map(|p| point(sk, p))
                .collect::<Result<Vec<_>, _>>()?;
            sk.add_curve(
                CurveKind::Spline {
                    pts,
                    control: true,
                    degree: 3,
                    periodic: false,
                },
                None,
            )
            .map_err(err)
        }
        Seg2::Bezier { n, p } => {
            let pts = p
                .iter()
                .take(n as usize + 1)
                .map(|p| point(sk, *p))
                .collect::<Result<Vec<_>, _>>()?;
            sk.add_curve(
                CurveKind::Spline {
                    pts,
                    control: true,
                    degree: n,
                    periodic: false,
                },
                None,
            )
            .map_err(err)
        }
        Seg2::Conic { a, apex, b, w } => {
            let a = point(sk, a)?;
            let b = point(sk, b)?;
            let apex = point(sk, apex)?;
            sk.add_curve(
                CurveKind::Conic {
                    a,
                    b,
                    apex,
                    rho: w / (1.0 + w),
                },
                None,
            )
            .map_err(err)
        }
    }
}
/// Tool IDs match the self-contained sketch workspace, never old tool IDs.
pub fn draw(sk: &mut Sketch, tool: i32, pts: &[Vec2], p: &ToolParameters) -> Result<(), String> {
    if pts.iter().any(|p| !p.is_finite()) {
        return Err("Non-finite sketch position".into());
    }
    let need = match tool {
        0 => 0,
        5 | 7 | 8 | 9 | 10 | 15 => 3,
        11 | 12 => 2,
        16 | 17 => 1,
        _ => 2,
    };
    if pts.len() < need {
        return Err(format!("This tool requires {need} points"));
    }
    if !p.value.is_finite() || !p.secondary.is_finite() {
        return Err("Tool values must be finite".into());
    }
    match tool {
        1 | 2 | 18 => {
            for pair in pts.windows(2) {
                let i = line(sk, pair[0], pair[1])?;
                if tool == 18 {
                    sk.curves[i].construction = true;
                    sk.curves[i].centerline = true;
                }
            }
        }
        3 | 4 => {
            let (a, b) = if tool == 4 {
                (pts[0] * 2.0 - pts[1], pts[1])
            } else {
                (pts[0], pts[1])
            };
            let corners = [a, Vec2::new(b.x, a.y), b, Vec2::new(a.x, b.y)];
            for i in 0..4 {
                let l = line(sk, corners[i], corners[(i + 1) % 4])?;
                sk.add_constraint(
                    if i % 2 == 0 {
                        ConstraintKind::Horizontal { l }
                    } else {
                        ConstraintKind::Vertical { l }
                    },
                    None,
                )
                .map_err(err)?;
            }
        }
        5 => {
            let a = pts[0];
            let b = pts[1];
            let u = b - a;
            if u.len() < 1e-9 {
                return Err("Rectangle base has zero length".into());
            }
            let v = u.perp() * (u.cross(pts[2] - a) / u.len2());
            let corners = [a, b, b + v, a + v];
            let mut ls = Vec::new();
            for i in 0..4 {
                ls.push(line(sk, corners[i], corners[(i + 1) % 4])?);
            }
            sk.add_constraint(ConstraintKind::Parallel { a: ls[0], b: ls[2] }, None)
                .map_err(err)?;
            sk.add_constraint(ConstraintKind::Parallel { a: ls[1], b: ls[3] }, None)
                .map_err(err)?;
            sk.add_constraint(ConstraintKind::Perpendicular { a: ls[0], b: ls[1] }, None)
                .map_err(err)?;
        }
        6 => {
            let c = point(sk, pts[0])?;
            sk.add_circle(pts[0], pts[0].dist(pts[1]), Some(c), None)
                .map_err(err)?;
        }
        7 => {
            let c = circ3(pts[0], pts[1], pts[2])?;
            sk.add_circle(c, c.dist(pts[0]), None, None).map_err(err)?;
        }
        8 => {
            let c = pts[0];
            let r = c.dist(pts[1]);
            let v = pts[2] - c;
            if v.len() < 1e-9 {
                return Err("Arc endpoint coincides with center".into());
            }
            arc(sk, c, pts[1], c + v * (r / v.len()))?;
        }
        9 => {
            let c = circ3(pts[0], pts[1], pts[2])?;
            let aa = (pts[0] - c).angle();
            let sweep = ((pts[2] - c).angle() - aa).rem_euclid(std::f64::consts::TAU);
            let middle = ((pts[1] - c).angle() - aa).rem_euclid(std::f64::consts::TAU);
            if middle < sweep {
                arc(sk, c, pts[0], pts[2])?;
            } else {
                let i = arc(sk, c, pts[2], pts[0])?;
                sk.curves[i].reversed = true;
            }
        }
        10 => {
            let c = point(sk, pts[0])?;
            let m = point(sk, pts[1])?;
            let u = pts[1] - pts[0];
            let r = u.cross(pts[2] - pts[0]).abs() / u.len().max(1e-12);
            sk.add_curve(CurveKind::Ellipse { c, m, r }, None)
                .map_err(err)?;
        }
        11 | 12 => {
            let ids = pts
                .iter()
                .map(|p| point(sk, *p))
                .collect::<Result<Vec<_>, _>>()?;
            sk.add_curve(
                CurveKind::Spline {
                    degree: if tool == 12 {
                        3.min((ids.len() - 1) as u8)
                    } else {
                        3
                    },
                    pts: ids,
                    control: tool == 12,
                    periodic: false,
                },
                None,
            )
            .map_err(err)?;
        }
        13 => {
            let n = p.count.clamp(3, 128);
            let c = pts[0];
            let v = pts[1] - c;
            if v.len() < 1e-9 {
                return Err("Polygon radius must be positive".into());
            }
            let center = point(sk, c)?;
            let circle = sk.add_circle(c, v.len(), Some(center), None).map_err(err)?;
            sk.curves[circle].construction = true;
            let mut sides = Vec::new();
            for i in 0..n {
                let l = line(
                    sk,
                    c + v.rotate(i as f64 * std::f64::consts::TAU / n as f64),
                    c + v.rotate((i + 1) as f64 * std::f64::consts::TAU / n as f64),
                )?;
                sides.push(l);
                let a = sk.curves[l].kind.ends().unwrap().0;
                sk.add_constraint(ConstraintKind::PointOnCurve { p: a, c: circle }, None)
                    .map_err(err)?;
            }
            for &l in sides.iter().skip(1) {
                sk.add_constraint(ConstraintKind::Equal { a: sides[0], b: l }, None)
                    .map_err(err)?;
            }
        }
        14 => {
            let a = pts[0];
            let b = pts[1];
            let u = b - a;
            if u.len() < 1e-9 || p.value <= 0.0 {
                return Err("Slot requires distinct centers and positive width".into());
            }
            let n = u.perp() * (p.value * 0.5 / u.len());
            let l1 = line(sk, a + n, b + n)?;
            let c1 = arc(sk, b, b - n, b + n)?;
            let l2 = line(sk, b - n, a - n)?;
            let c2 = arc(sk, a, a + n, a - n)?;
            for l in [l1, l2] {
                for c in [c1, c2] {
                    sk.add_constraint(ConstraintKind::Tangent { a: l, b: c }, None)
                        .map_err(err)?;
                }
            }
            sk.add_constraint(ConstraintKind::Equal { a: c1, b: c2 }, None)
                .map_err(err)?;
            sk.add_constraint(
                ConstraintKind::Radius {
                    c: c1,
                    value: p.value * 0.5,
                },
                None,
            )
            .map_err(err)?;
        }
        15 => {
            let a = point(sk, pts[0])?;
            let b = point(sk, pts[1])?;
            let apex = point(sk, pts[2])?;
            sk.add_curve(
                CurveKind::Conic {
                    a,
                    b,
                    apex,
                    rho: p.value,
                },
                None,
            )
            .map_err(err)?;
        }
        16 => {
            point(sk, pts[0])?;
        }
        17 => {
            let geometry =
                text_geometry(&p.text, pts[0], p.value, p.secondary.to_radians()).map_err(err)?;
            let source = LinkSource::Text {
                text: p.text.clone(),
                at: pts[0],
                height: p.value,
                angle: p.secondary.to_radians(),
                font: None,
                style: TextStyle::default(),
                exprs: TextExprs::default(),
            };
            sk.add_link(LinkKind::Text, source, &geometry)
                .map_err(err)?;
        }
        _ => return Err("Unknown drawing tool".into()),
    }
    Ok(())
}

pub fn constraint(
    sk: &mut Sketch,
    kind: i32,
    cs: &[usize],
    ps: &[usize],
    value: f64,
) -> Result<(), String> {
    let c = |i| {
        cs.get(i)
            .copied()
            .ok_or_else(|| "Select the required curves".to_string())
    };
    let p = |i| {
        ps.get(i)
            .copied()
            .ok_or_else(|| "Select the required points".to_string())
    };
    let k = match kind {
        0 => ConstraintKind::Horizontal { l: c(0)? },
        1 => ConstraintKind::Vertical { l: c(0)? },
        2 => ConstraintKind::Coincident { p: p(0)?, q: p(1)? },
        3 => ConstraintKind::Parallel { a: c(0)?, b: c(1)? },
        4 => ConstraintKind::Perpendicular { a: c(0)?, b: c(1)? },
        5 => ConstraintKind::Tangent { a: c(0)?, b: c(1)? },
        6 => ConstraintKind::Equal { a: c(0)?, b: c(1)? },
        7 => ConstraintKind::Concentric { a: c(0)?, b: c(1)? },
        8 => ConstraintKind::Collinear { a: c(0)?, b: c(1)? },
        9 => ConstraintKind::Midpoint { p: p(0)?, l: c(0)? },
        10 => ConstraintKind::PointOnCurve { p: p(0)?, c: c(0)? },
        11 => ConstraintKind::Symmetric {
            p: p(0)?,
            q: p(1)?,
            l: c(0)?,
        },
        12 => ConstraintKind::Fix { p: p(0)? },
        13 => ConstraintKind::Length { l: c(0)?, value },
        14 => ConstraintKind::Radius { c: c(0)?, value },
        15 => ConstraintKind::Diameter { c: c(0)?, value },
        16 => ConstraintKind::Distance {
            p: p(0)?,
            q: p(1)?,
            value,
        },
        17 => ConstraintKind::DistanceX {
            p: p(0)?,
            q: p(1)?,
            value,
        },
        18 => ConstraintKind::DistanceY {
            p: p(0)?,
            q: p(1)?,
            value,
        },
        19 => ConstraintKind::Angle {
            a: c(0)?,
            b: c(1)?,
            value: value.to_radians(),
            flip: false,
        },
        20 => ConstraintKind::LineAngle {
            l: c(0)?,
            value: value.to_radians(),
        },
        21 => ConstraintKind::PointLineDistance {
            p: p(0)?,
            l: c(0)?,
            value,
        },
        22 => ConstraintKind::ArcSweep {
            c: c(0)?,
            value: value.to_radians(),
        },
        23 => ConstraintKind::HorizontalPoints { p: p(0)?, q: p(1)? },
        24 => ConstraintKind::VerticalPoints { p: p(0)?, q: p(1)? },
        25 => ConstraintKind::Smooth { a: c(0)?, b: c(1)? },
        26 => ConstraintKind::ArcLength { c: c(0)?, value },
        27 => ConstraintKind::LinearDiameter {
            p: p(0)?,
            l: c(0)?,
            value,
        },
        28 => ConstraintKind::SlotOverallLength {
            a: c(0)?,
            b: c(1)?,
            value,
        },
        _ => return Err("Unknown constraint".into()),
    };
    sk.add_constraint(k, None).map_err(err)?;
    Ok(())
}

pub fn edit(
    sk: &mut Sketch,
    action: i32,
    selected: &[usize],
    points: &[usize],
    at: Vec2,
    p: &ToolParameters,
) -> Result<(), String> {
    if selected.iter().any(|i| *i >= sk.curves.len())
        || points.iter().any(|i| *i >= sk.points.len())
    {
        return Err("Stale sketch selection".into());
    }
    match action {
        30 => {
            sk.remove_curves(selected);
            sk.remove_points(points);
        }
        31 | 32 => {
            for &i in selected {
                if action == 31 {
                    sk.curves[i].construction = !sk.curves[i].construction;
                } else {
                    sk.curves[i].centerline = !sk.curves[i].centerline;
                    sk.curves[i].construction = sk.curves[i].centerline;
                }
            }
        }
        33 => {
            let mut ids = points.to_vec();
            for &i in selected {
                ids.extend(sk.curves[i].kind.point_ids());
            }
            ids.sort();
            ids.dedup();
            for i in ids {
                sk.points[i].fixed = !sk.points[i].fixed;
            }
        }
        34..=38 => copy_pattern(sk, action, selected, at, p)?,
        39..=41 => {
            let mut ids = BTreeSet::new();
            ids.extend(points.iter().copied());
            for &c in selected {
                ids.extend(sk.curves[c].kind.point_ids());
            }
            for i in ids {
                let v = sk.points[i].pos;
                sk.points[i].pos = match action {
                    39 => v + Vec2::new(p.value, p.secondary),
                    40 => at + (v - at).rotate(p.value.to_radians()),
                    _ => at + (v - at) * p.value,
                };
            }
            if action == 41 {
                if p.value <= 0.0 {
                    return Err("Scale must be positive".into());
                }
                for &i in selected {
                    match &mut sk.curves[i].kind {
                        CurveKind::Circle { r, .. } | CurveKind::Ellipse { r, .. } => *r *= p.value,
                        _ => {}
                    }
                }
            }
        }
        42 | 43 => trim_split(sk, selected, at, action == 42)?,
        44 => extend(sk, selected, at)?,
        45 | 46 => corner(sk, selected, p.value, action == 45)?,
        47 | 48 => {
            if selected.is_empty() {
                return Err("Select spline curves".into());
            }
            for &i in selected {
                match &mut sk.curves[i].kind {
                    CurveKind::Spline {
                        periodic,
                        degree,
                        pts,
                        ..
                    } => {
                        if action == 47 {
                            *periodic = !*periodic;
                        } else {
                            if p.count < 1 || p.count > 7 || p.count >= pts.len() {
                                return Err(
                                    "Spline degree must be 1–7 and below its control point count"
                                        .into(),
                                );
                            }
                            *degree = p.count as u8;
                        }
                    }
                    _ => return Err("Select spline curves".into()),
                }
            }
        }
        49 => {
            for &i in selected {
                sk.curves[i].reversed = !sk.curves[i].reversed;
            }
        }
        50 => {
            for &i in selected {
                if !matches!(sk.curves[i].kind, CurveKind::Line { .. }) {
                    return Err("Only straight lines can be infinite".into());
                }
                sk.curves[i].infinite = !sk.curves[i].infinite;
            }
        }

        _ => return Err("Unknown edit command".into()),
    }
    Ok(())
}
fn copy_pattern(
    sk: &mut Sketch,
    action: i32,
    selected: &[usize],
    at: Vec2,
    p: &ToolParameters,
) -> Result<(), String> {
    if selected.is_empty() {
        return Err("Select curves to transform".into());
    }
    if action == 36 {
        let sources: Vec<_> = selected
            .iter()
            .map(|i| (sk.curves[*i].id.clone(), true))
            .collect();
        if sk.convex_offset_supported(&sources).map_err(err)? {
            sk.add_convex_offset(sources, p.value).map_err(err)?;
            return Ok(());
        }
        if selected.len() == 1 && sk.curves[selected[0]].kind.is_freeform() {
            sk.add_freeform_offset(selected[0], true, p.value)
                .map_err(err)?;
            return Ok(());
        }
        return Err(
            "Select an ordered closed convex boundary or one freeform curve for offset".into(),
        );
    }
    let count = if action >= 37 {
        p.count.clamp(2, 64)
    } else {
        2
    };
    let source: Vec<_> = selected.iter().map(|i| sk.curves[*i].clone()).collect();
    let axis = if action == 34 || action == 35 {
        let a = point(sk, Vec2::ZERO)?;
        let b = sk
            .add_point(if action == 34 { Vec2::X } else { Vec2::Y }, None)
            .map_err(err)?;
        sk.points[a].fixed = true;
        sk.points[b].fixed = true;
        let l = sk.add_line_pts(a, b, None).map_err(err)?;
        sk.curves[l].construction = true;
        Some(l)
    } else {
        None
    };
    for k in 1..count {
        let mut map = BTreeMap::new();
        for c in &source {
            for i in c.kind.point_ids() {
                if map.contains_key(&i) {
                    continue;
                }
                let old = sk.points[i].pos;
                let v = match action {
                    34 => Vec2::new(old.x, -old.y),
                    35 => Vec2::new(-old.x, old.y),
                    37 => old + Vec2::new(p.value, p.secondary) * (k as f64),
                    _ => at + (old - at).rotate(p.value.to_radians() * k as f64),
                };
                let j = sk.add_point(v, None).map_err(err)?;
                map.insert(i, j);
                if let Some(l) = axis {
                    sk.add_constraint(ConstraintKind::Symmetric { p: i, q: j, l }, None)
                        .map_err(err)?;
                }

                if action >= 37 {
                    let center = if action == 38 {
                        Some(point(sk, at)?)
                    } else {
                        None
                    };
                    sk.add_constraint(
                        ConstraintKind::PatternPoint {
                            p: i,
                            q: j,
                            center,
                            angle: if action == 38 {
                                p.value.to_radians() * k as f64
                            } else {
                                0.0
                            },
                            offset: if action == 37 {
                                Vec2::new(p.value, p.secondary) * (k as f64)
                            } else {
                                Vec2::ZERO
                            },
                            directions: [None, None],
                        },
                        None,
                    )
                    .map_err(err)?;
                }
            }
        }
        for c in &source {
            let mut kind = c.kind.clone();
            kind.map_points(&|i| map[&i]);
            if action == 34 || action == 35 {
                if let CurveKind::Arc { a, b, .. } = &mut kind {
                    std::mem::swap(a, b);
                }
            }
            let i = sk.add_curve(kind, None).map_err(err)?;
            sk.curves[i].construction = c.construction;
            sk.curves[i].centerline = c.centerline;
            if (action >= 37 || axis.is_some())
                && matches!(c.kind, CurveKind::Circle { .. } | CurveKind::Ellipse { .. })
            {
                let a = sk.curve_index(&c.id).ok_or("Missing pattern seed")?;
                sk.add_constraint(ConstraintKind::PatternRadius { a, b: i }, None)
                    .map_err(err)?;
            }
        }
    }
    Ok(())
}
fn trim_split(sk: &mut Sketch, ids: &[usize], at: Vec2, trim: bool) -> Result<(), String> {
    let &ci = ids.first().ok_or("Select a curve")?;
    if trim {
        super::modify::trim(sk, ci, at)
    } else {
        super::modify::split(sk, ci, at)
    }
    .map_err(err)
}
fn extend(sk: &mut Sketch, ids: &[usize], at: Vec2) -> Result<(), String> {
    let &ci = ids.first().ok_or("Select a line or arc")?;
    super::modify::extend(sk, ci, at).map_err(err)
}
fn corner(sk: &mut Sketch, ids: &[usize], value: f64, fillet: bool) -> Result<(), String> {
    super::corners::apply(sk, ids, value, fillet)
}

trait Rotate {
    fn rotate(self, a: f64) -> Self;
}
impl Rotate for Vec2 {
    fn rotate(self, a: f64) -> Self {
        Self::new(
            self.x * a.cos() - self.y * a.sin(),
            self.x * a.sin() + self.y * a.cos(),
        )
    }
}
