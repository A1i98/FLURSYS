//! Sketch modification adapter; see THIRD_PARTY_NOTICES.txt for attribution.
use serde_json::{json, Value};
use cad_geom::Vec2;
use cad_sketch::{intersections, ConstraintKind, Curve, CurveKind, Shape, Sketch};
use std::f64::consts::TAU;
type Result<T> = std::result::Result<T, cad_sketch::SketchError>;
fn bad(cmd: &str, message: impl std::fmt::Display) -> cad_sketch::SketchError {
    cad_sketch::SketchError::Invalid(format!("{cmd}: {message}"))
}
fn add_c(sk: &mut Sketch, kind: ConstraintKind) -> Result<String> {
    sk.add_constraint(kind, None)
}
fn str_<'a>(p: &'a Value, key: &str) -> Option<&'a str> {
    p.get(key)?.as_str()
}
const TOL: f64 = 1e-7;
fn on_bounded(sh: &Shape, p: Vec2) -> Option<f64> {
    if sh.dist(p) > TOL * 10.0 {
        return None;
    }
    let t = sh.param(p);
    let (t0, t1) = sh.range();
    let slack = match *sh {
        Shape::Line { a, b } => TOL * 10.0 / a.dist(b).max(1e-12),
        Shape::Round { r, .. } => TOL * 10.0 / r.max(1e-12),
    };
    if sh.is_full() || (t >= t0 - slack && t <= t1 + slack) {
        Some(t)
    } else {
        None
    }
}

fn uses(c: &Curve, p: usize) -> bool {
    c.kind.uses(p)
}

// ---------------------------------------------------------------------------------------------
// Corners: fillet and chamfer

/// A corner drawn as two curves whose ends are separate points held together by a Coincident
/// constraint (a line drawn to where an arc starts, #1208) becomes one shared point, so it can be
/// rounded or cut like a corner drawn through one point. Only a partner held by nothing but that
/// constraint is welded; anything else is left as it is.
fn weld_corner(sk: &mut Sketch, p: &Value) {
    let ends = |sk: &Sketch, c: usize| match sk.curves.get(c).map(|c| &c.kind) {
        Some(CurveKind::Line { a, b }) | Some(CurveKind::Arc { a, b, .. }) => Some([*a, *b]),
        _ => None,
    };
    let q = if let Some(r) = str_(p, "point") {
        sk.resolve_point(r)
    } else {
        let curve = |k: &str| {
            str_(p, k)
                .and_then(|i| sk.curve_index(i))
                .and_then(|c| ends(sk, c))
        };
        let (Some(ea), Some(eb)) = (curve("a"), curve("b")) else {
            return;
        };
        if ea.iter().any(|x| eb.contains(x)) {
            return;
        }
        // The end of `a` held coincident with an end of `b`.
        ea.into_iter().find(|x| {
            sk.constraints
                .iter()
                .any(|k| matches!(k.kind, ConstraintKind::Coincident { p, q } if (p == *x && eb.contains(&q)) || (q == *x && eb.contains(&p))))
        })
    };
    let Some(mut q) = q else { return };
    // Each pass welds one partner; a pass removes a constraint, so this ends.
    for _ in 0..sk.constraints.len() {
        let partner = sk.constraints.iter().enumerate().find_map(|(i, k)| {
            let o = match k.kind {
                ConstraintKind::Coincident { p, q: r } if p == q && r != q => r,
                ConstraintKind::Coincident { p, q: r } if r == q && p != q => p,
                _ => return None,
            };
            let held_elsewhere = sk
                .constraints
                .iter()
                .enumerate()
                .any(|(j, k)| j != i && k.kind.refers_to_point(o));
            let linked = (0..sk.curves.len())
                .any(|c| sk.is_linked_curve(c) && sk.curves.get(c).is_some_and(|x| x.kind.uses(o)));
            (o != 0 && !held_elsewhere && !linked).then_some((i, o))
        });
        let Some((i, o)) = partner else { return };
        for c in sk.curves.iter_mut() {
            c.kind.map_points(&|x| if x == o { q } else { x });
        }
        sk.constraints.remove(i);
        sk.remove_points(&[o]);
        if o < q {
            q -= 1;
        }
    }
}

/// Fillets and chamfers refuse a corner on an infinite line (#1359): it has no end there.
fn refuse_infinite_corner(sk: &Sketch, p: &Value, cmd: &str) -> Result<()> {
    let infinite = |c: Option<usize>| c.and_then(|c| sk.curves.get(c)).is_some_and(|c| c.infinite);
    let at_corner = match str_(p, "point").and_then(|r| sk.resolve_point(r)) {
        Some(q) => sk.curves.iter().any(|c| c.infinite && c.kind.uses(q)),
        None => ["a", "b"]
            .iter()
            .any(|k| infinite(str_(p, k).and_then(|i| sk.curve_index(i)))),
    };
    if at_corner {
        Err(bad(
            cmd,
            "an infinite line has no corner to round or cut: make it a normal line first",
        ))
    } else {
        Ok(())
    }
}

/// A corner of two lines: (line a, line b, corner point, far end of a, far end of b).
fn corner(sk: &Sketch, p: &Value, cmd: &str) -> Result<(usize, usize, usize, usize, usize)> {
    let ends = |l: usize| match sk.curves.get(l).map(|c| &c.kind) {
        Some(CurveKind::Line { a, b }) => Some((*a, *b)),
        _ => None,
    };
    let (la, lb) = if let Some(r) = str_(p, "point") {
        let q = sk
            .resolve_point(r)
            .ok_or_else(|| bad(cmd, format!("unknown point `{r}`")))?;
        let lines: Vec<usize> = (0..sk.curves.len())
            .filter(|i| ends(*i).is_some_and(|(a, b)| a == q || b == q) && !sk.is_linked_curve(*i))
            .collect();
        // A construction line ending there too (a diagonal that centres something) is not one of
        // the corner's sides: it keeps its end on the virtual sharp corner (#1026).
        let sides: Vec<usize> = if lines.len() == 2 {
            lines
        } else {
            lines
                .into_iter()
                .filter(|i| sk.curves.get(*i).is_some_and(|c| !c.construction))
                .collect()
        };
        match sides[..] {
            [a, b] => (a, b),
            _ => {
                return Err(bad(
                    cmd,
                    "the corner must join exactly two lines (or give the two lines as `a` and `b`)",
                ))
            }
        }
    } else {
        let a = str_(p, "a")
            .and_then(|i| sk.curve_index(i))
            .ok_or_else(|| bad(cmd, "`a` must be a line"))?;
        let b = str_(p, "b")
            .and_then(|i| sk.curve_index(i))
            .ok_or_else(|| bad(cmd, "`b` must be a line"))?;
        (a, b)
    };
    let (a0, a1) = ends(la).ok_or_else(|| bad(cmd, "fillets and chamfers work on two lines"))?;
    let (b0, b1) = ends(lb).ok_or_else(|| bad(cmd, "fillets and chamfers work on two lines"))?;
    let shared = [a0, a1]
        .into_iter()
        .find(|x| *x == b0 || *x == b1)
        .ok_or_else(|| bad(cmd, "the lines must share a corner point"))?;
    let fa = if a0 == shared { a1 } else { a0 };
    let fb = if b0 == shared { b1 } else { b0 };
    Ok((la, lb, shared, fa, fb))
}

/// Replace the corner end of line `l` with point `np`.
fn reattach(sk: &mut Sketch, l: usize, corner: usize, np: usize) {
    if let Some(Curve {
        kind: CurveKind::Line { a, b },
        ..
    }) = sk.curves.get_mut(l)
    {
        if *a == corner {
            *a = np;
        } else if *b == corner {
            *b = np;
        }
    }
}

/// Equal lengths and midpoints describe the original side, not its shortened tangent
/// segment. Keep that side as construction geometry sharing the original corner points.
/// Later corner cuts retain those points through `keep_corner`, so this also works when
/// the other end is rounded in a separate command.
fn keep_side_relations(sk: &mut Sketch, line: usize) -> Result<Vec<String>> {
    let affected = |k: &ConstraintKind| {
        matches!(*k, ConstraintKind::Equal { a, b } if a == line || b == line)
            || matches!(*k, ConstraintKind::Midpoint { l, .. } if l == line)
    };
    if !sk.constraints.iter().any(|k| affected(&k.kind)) {
        return Ok(Vec::new());
    }
    let Some(CurveKind::Line { a, b }) = sk.curves.get(line).map(|c| c.kind.clone()) else {
        return Ok(Vec::new());
    };
    let support = sk.add_line_pts(a, b, None)?;
    if let Some(c) = sk.curves.get_mut(support) {
        c.construction = true;
    }
    let mut moved = Vec::new();
    for k in &mut sk.constraints {
        if !affected(&k.kind) {
            continue;
        }
        match &mut k.kind {
            ConstraintKind::Equal { a, b } => {
                if *a == line {
                    *a = support;
                }
                if *b == line {
                    *b = support;
                }
            }
            ConstraintKind::Midpoint { l, .. } => *l = support,
            _ => {}
        }
        moved.push(k.id.clone());
    }
    Ok(moved)
}

/// What became of a rounded or cut corner's point: kept as a virtual sharp corner (its id),
/// and the constraints and dimensions that now measure to it.
#[derive(Default)]
struct CornerKept {
    virtual_corner: Option<String>,
    moved: Vec<String>,
}

/// Keep corner point `c` of curves `ca`, `cb` (whose corner ends were just moved to `na`, `nb`)
/// when anything measures to it: a constraint or dimension on the point, the length of either
/// line, or it is the origin or still ends another curve. It stays as a virtual sharp corner,
/// held on both curves' extensions (their intersection), and the line lengths become distances
/// from the far end to it, so the part keeps its size. Otherwise None: the
/// caller removes the point.
fn keep_corner(
    sk: &mut Sketch,
    c: usize,
    [(ca, na), (cb, nb)]: [(usize, usize); 2],
) -> Result<Option<CornerKept>> {
    let used = sk.curves.iter().any(|x| uses(x, c));
    let mut moved: Vec<String> = sk
        .constraints
        .iter()
        .filter(|k| k.kind.refers_to_point(c))
        .map(|k| k.id.clone())
        .collect();
    for k in sk.constraints.iter_mut() {
        let ConstraintKind::Length { l, value } = k.kind else {
            continue;
        };
        let new_end = if l == ca {
            na
        } else if l == cb {
            nb
        } else {
            continue;
        };
        let Some(CurveKind::Line { a, b }) = sk.curves.get(l).map(|x| &x.kind) else {
            continue;
        };
        // Same direction as the line, so the dimension keeps its side and text place.
        k.kind = if *a == new_end {
            ConstraintKind::Distance { p: c, q: *b, value }
        } else {
            ConstraintKind::Distance { p: *a, q: c, value }
        };
        moved.push(k.id.clone());
    }
    if moved.is_empty() && !used && c != 0 {
        return Ok(None);
    }
    for cu in [ca, cb] {
        let k = ConstraintKind::PointOnCurve { p: c, c: cu };
        if !sk.constraints.iter().any(|x| x.kind == k) {
            add_c(sk, k)?;
        }
    }
    // A point of its own now: `l1.end` names the line's new end, not the virtual corner.
    let virtual_corner = if used || c == 0 {
        None
    } else {
        let id = sk.fresh("corner")?;
        if let Some(pt) = sk.points.get_mut(c) {
            pt.id = id.clone();
        }
        Some(id)
    };
    Ok(Some(CornerKept {
        virtual_corner,
        moved,
    }))
}

/// Points at distances `da` along a and `db` along b from the corner; lines shortened to them.
fn cut_corner(
    sk: &mut Sketch,
    la: usize,
    lb: usize,
    c: usize,
    fa: usize,
    fb: usize,
    da: f64,
    db: f64,
    cmd: &str,
) -> Result<(usize, usize, Vec2, CornerKept)> {
    let (pc, pa, pb) = (
        sk.point(c).unwrap_or_default(),
        sk.point(fa).unwrap_or_default(),
        sk.point(fb).unwrap_or_default(),
    );
    if !(da > 1e-9 && db > 1e-9 && da < pc.dist(pa) - 1e-9 && db < pc.dist(pb) - 1e-9) {
        return Err(bad(cmd, "too large for these lines"));
    }
    let u = (pa - pc)
        .normalized()
        .ok_or_else(|| bad(cmd, "degenerate line"))?;
    let v = (pb - pc)
        .normalized()
        .ok_or_else(|| bad(cmd, "degenerate line"))?;
    let ta = sk.add_point(pc + u * da, None)?;
    let tb = sk.add_point(pc + v * db, None)?;
    let mut moved = keep_side_relations(sk, la)?;
    moved.extend(keep_side_relations(sk, lb)?);
    reattach(sk, la, c, ta);
    reattach(sk, lb, c, tb);
    let mut kept = keep_corner(sk, c, [(la, ta), (lb, tb)])?;
    if let Some(k) = &mut kept {
        k.moved.extend(moved);
    }
    let removed = kept.is_none();
    if removed {
        sk.remove_points(&[c]);
    }
    // Indices shift when the corner point was removed.
    let fix = |i: usize| if removed && i > c { i - 1 } else { i };
    Ok((fix(ta), fix(tb), pc, kept.unwrap_or_default()))
}

fn fillet_corner(
    sk: &mut Sketch,
    p: &Value,
    r: f64,
    cmd: &str,
) -> Result<(usize, Vec<String>, CornerKept)> {
    refuse_infinite_corner(sk, p, cmd)?;
    weld_corner(sk, p);
    let (la, lb, c, fa, fb) = match corner(sk, p, cmd) {
        Ok(x) => x,
        // Not two lines: a corner with an arc.
        Err(e) => return round_corner(sk, p, r, cmd)?.ok_or(e),
    };
    let (pc, pa, pb) = (
        sk.point(c).unwrap_or_default(),
        sk.point(fa).unwrap_or_default(),
        sk.point(fb).unwrap_or_default(),
    );
    let u = (pa - pc)
        .normalized()
        .ok_or_else(|| bad(cmd, "degenerate line"))?;
    let v = (pb - pc)
        .normalized()
        .ok_or_else(|| bad(cmd, "degenerate line"))?;
    let half = 0.5 * u.cross(v).atan2(u.dot(v)).abs();
    if !(1e-6..=std::f64::consts::FRAC_PI_2 - 1e-9).contains(&half) {
        return Err(bad(cmd, "the lines are parallel"));
    }
    let d = r / half.tan();
    if d >= pa.dist(pc) - 1e-9 || d >= pb.dist(pc) - 1e-9 {
        return Err(bad(
            cmd,
            format!("the radius is too large for that corner (it needs {d:.3} mm of each line)"),
        ));
    }
    let bis = (u + v)
        .normalized()
        .ok_or_else(|| bad(cmd, "the lines are parallel"))?;
    let center = pc + bis * (r / half.sin());
    let (ta, tb, _, kept) = cut_corner(sk, la, lb, c, fa, fb, d, d, cmd)?;
    let (qa, qb) = (
        sk.point(ta).unwrap_or_default(),
        sk.point(tb).unwrap_or_default(),
    );
    let ccw = (qa - center).cross(qb - center) > 0.0;
    let arc = if ccw {
        sk.add_arc(center, qa, qb, [None, Some(ta), Some(tb)], None)?
    } else {
        sk.add_arc(center, qb, qa, [None, Some(tb), Some(ta)], None)?
    };
    let cons = vec![
        add_c(sk, ConstraintKind::Tangent { a: la, b: arc })?,
        add_c(sk, ConstraintKind::Tangent { a: lb, b: arc })?,
    ];
    Ok((arc, cons, kept))
}

/// Fillet a corner where a line meets an arc, or two arcs meet: the arc of radius `r` touching
/// both, nearest the corner; both curves are cut back to it. None when the corner is not of
/// that kind.
fn round_corner(
    sk: &mut Sketch,
    p: &Value,
    r: f64,
    cmd: &str,
) -> Result<Option<(usize, Vec<String>, CornerKept)>> {
    let ends = |c: usize| match sk.curves.get(c).map(|c| &c.kind) {
        Some(CurveKind::Line { a, b }) | Some(CurveKind::Arc { a, b, .. }) => Some((*a, *b)),
        _ => None,
    };
    let (ca, cb, q) = if let Some(rf) = str_(p, "point") {
        let Some(q) = sk.resolve_point(rf) else {
            return Ok(None);
        };
        let cs: Vec<usize> = (0..sk.curves.len())
            .filter(|i| ends(*i).is_some_and(|(a, b)| a == q || b == q) && !sk.is_linked_curve(*i))
            .collect();
        match cs[..] {
            [a, b] => (a, b, q),
            _ => return Ok(None),
        }
    } else {
        let (Some(a), Some(b)) = (
            str_(p, "a").and_then(|i| sk.curve_index(i)),
            str_(p, "b").and_then(|i| sk.curve_index(i)),
        ) else {
            return Ok(None);
        };
        let (Some((a0, a1)), Some((b0, b1))) = (ends(a), ends(b)) else {
            return Ok(None);
        };
        let Some(q) = [a0, a1].into_iter().find(|x| *x == b0 || *x == b1) else {
            return Ok(None);
        };
        (a, b, q)
    };
    let (Some(sa), Some(sb), Some(pc)) = (sk.shape(ca), sk.shape(cb), sk.point(q)) else {
        return Ok(None);
    };
    let offsets = |sh: &Shape| -> Vec<Shape> {
        match *sh {
            Shape::Line { a, b } => match (b - a).normalized() {
                Some(d) => [1.0, -1.0]
                    .iter()
                    .map(|s| Shape::Line {
                        a: a + d.perp() * (r * s),
                        b: b + d.perp() * (r * s),
                    })
                    .collect(),
                None => Vec::new(),
            },
            Shape::Round { c, r: rr, .. } => {
                let mut v = vec![Shape::Round {
                    c,
                    r: rr + r,
                    start: 0.0,
                    sweep: TAU,
                }];
                if rr - r > 1e-9 {
                    v.push(Shape::Round {
                        c,
                        r: rr - r,
                        start: 0.0,
                        sweep: TAU,
                    });
                }
                v
            }
        }
    };
    let mut best: Option<(f64, Vec2, Vec2, Vec2)> = None;
    for oa in offsets(&sa) {
        for ob in offsets(&sb) {
            for cen in intersections(&oa, &ob) {
                let (ta, tb) = (sa.project(cen), sb.project(cen));
                if on_bounded(&sa, ta).is_none()
                    || on_bounded(&sb, tb).is_none()
                    || ta.dist(pc) < 1e-9
                    || tb.dist(pc) < 1e-9
                {
                    continue;
                }
                let score = ta.dist(pc) + tb.dist(pc);
                if best.is_none_or(|b| score < b.0) {
                    best = Some((score, cen, ta, tb));
                }
            }
        }
    }
    let (_, cen, ta, tb) = best.ok_or_else(|| bad(cmd, "that radius does not fit the corner"))?;
    let (na, nb) = (sk.add_point(ta, None)?, sk.add_point(tb, None)?);
    for (c, n) in [(ca, na), (cb, nb)] {
        if let Some(cu) = sk.curves.get_mut(c) {
            cu.kind.map_points(&|i| if i == q { n } else { i });
        }
    }
    let ccw = (ta - cen).cross(tb - cen) > 0.0;
    let arc = if ccw {
        sk.add_arc(cen, ta, tb, [None, Some(na), Some(nb)], None)?
    } else {
        sk.add_arc(cen, tb, ta, [None, Some(nb), Some(na)], None)?
    };
    let cons = vec![
        add_c(sk, ConstraintKind::Tangent { a: ca, b: arc })?,
        add_c(sk, ConstraintKind::Tangent { a: cb, b: arc })?,
    ];
    match keep_corner(sk, q, [(ca, na), (cb, nb)])? {
        Some(kept) => Ok(Some((arc, cons, kept))),
        None => {
            let arc_id = sk.curves.get(arc).map(|c| c.id.clone()).unwrap_or_default();
            sk.remove_points(&[q]);
            let arc = sk.curve_index(&arc_id).unwrap_or(arc);
            Ok(Some((arc, cons, CornerKept::default())))
        }
    }
}

pub(super) fn apply(
    sk: &mut Sketch,
    ids: &[usize],
    value: f64,
    fillet: bool,
) -> std::result::Result<(), String> {
    if ids.len() != 2 || !value.is_finite() || value <= 0.0 {
        return Err("Select two curves and a positive radius/distance".into());
    }
    let p = json!({"a":sk.curves[ids[0]].id,"b":sk.curves[ids[1]].id});
    let result = (|| -> Result<()> {
        if fillet {
            let (arc, _, kept) = fillet_corner(sk, &p, value, "fillet")?;
            let _virtual_corner = kept.virtual_corner;
            add_c(sk, ConstraintKind::Radius { c: arc, value })?;
        } else {
            refuse_infinite_corner(sk, &p, "chamfer")?;
            weld_corner(sk, &p);
            let (la, lb, c, fa, fb) = corner(sk, &p, "chamfer")?;
            let (ta, tb, _, _) = cut_corner(sk, la, lb, c, fa, fb, value, value, "chamfer")?;
            sk.add_line_pts(ta, tb, None)?;
        }
        Ok(())
    })();
    result.map_err(|e| e.to_string())
}
