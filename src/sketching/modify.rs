//! Sketch modification adapter; see THIRD_PARTY_NOTICES.txt for attribution.
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
const TOL: f64 = 1e-7;
use super::support;
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

/// Points where other curves meet curve `ci` (crossings and end points touching it), with the
/// curve that makes each one and the parameter on `ci`.
fn cut_points(sk: &Sketch, ci: usize) -> Vec<(f64, Vec2, usize)> {
    let Some(sh) = sk.shape(ci) else {
        return Vec::new();
    };
    let mut out: Vec<(f64, Vec2, usize)> = Vec::new();
    for (oi, _) in sk.curves.iter().enumerate() {
        if oi == ci {
            continue;
        }
        let Some(other) = sk.shape(oi) else { continue };
        let mut pts = intersections(&sh, &other);
        // Overlapping or touching ends that the analytic intersection misses (parallel lines).
        if let Shape::Line { a, b } = other {
            pts.extend([a, b]);
        }
        if matches!(other,Shape::Round{sweep,..} if sweep<TAU-1e-12) {
            let Shape::Round { sweep, .. } = other else {
                unreachable!()
            };
            pts.extend([other.point(0.0), other.point(sweep)]);
        }
        // An infinite line (#1359) cuts wherever its line crosses, beyond its two points too.
        let other_infinite = sk.curves.get(oi).is_some_and(|c| c.infinite);
        for q in pts {
            if other_infinite || on_bounded(&other, q).is_some() {
                if let Some(t) = on_bounded(&sh, q)
                    .filter(|_| !out.iter().any(|(_, x, _)| x.dist(q) < TOL * 10.0))
                {
                    out.push((t, q, oi));
                }
            }
        }
    }
    for (q, oi) in super::freeform::analytic_cut_points(sk, ci) {
        if let Some(t) =
            on_bounded(&sh, q).filter(|_| !out.iter().any(|(_, x, _)| x.dist(q) < TOL * 10.0))
        {
            out.push((t, q, oi));
        }
    }
    out
}

/// Constrain a new end point onto the curve that cut there.
fn pin(sk: &mut Sketch, p: usize, cutter: usize) -> Result<()> {
    let at = sk.point(p).unwrap_or_default();
    // An end point of the cutter there: coincident with it; otherwise on the curve.
    let ends: Vec<usize> = match sk.curves.get(cutter).map(|c| c.kind.clone()) {
        Some(CurveKind::Line { a, b }) | Some(CurveKind::Arc { a, b, .. }) => vec![a, b],
        _ => Vec::new(),
    };
    if let Some(e) = ends
        .into_iter()
        .find(|e| sk.point(*e).is_some_and(|q| q.dist(at) < TOL * 10.0))
    {
        if e != p {
            add_c(sk, ConstraintKind::Coincident { p, q: e })?;
        }
        return Ok(());
    }
    add_c(sk, ConstraintKind::PointOnCurve { p, c: cutter })?;
    Ok(())
}

/// Split curve `ci` at interior parameters `ts` (sorted). Returns the pieces in parameter
/// order (the first keeps the original index) and the new points.
fn split_at(sk: &mut Sketch, ci: usize, ts: &[f64]) -> Result<(Vec<usize>, Vec<usize>)> {
    let sh = sk.shape(ci).ok_or_else(|| bad("split", "curve"))?;
    let kind = sk
        .curves
        .get(ci)
        .map(|c| c.kind.clone())
        .ok_or_else(|| bad("split", "curve"))?;
    let construction = sk.curves.get(ci).is_some_and(|c| c.construction);
    let mut pts = Vec::new();
    for t in ts {
        pts.push(sk.add_point(sh.point(*t), None)?);
    }
    let mut pieces = vec![ci];
    let mut push = |sk: &mut Sketch, kind: CurveKind, prefix: &str| -> Result<()> {
        let id = sk.fresh(prefix)?;
        sk.curves.push(Curve {
            id,
            kind,
            construction,
            reversed: false,
            link: None,
            centerline: false,
            fixed: false,
            infinite: false,
        });
        pieces.push(sk.curves.len() - 1);
        Ok(())
    };
    match kind {
        CurveKind::Line { a, b } => {
            let mut chain = vec![a];
            chain.extend(&pts);
            chain.push(b);
            if let Some(c) = sk.curves.get_mut(ci) {
                c.kind = CurveKind::Line {
                    a,
                    b: chain.get(1).copied().unwrap_or(b),
                };
            }
            for w in chain.windows(2).skip(1) {
                push(sk, CurveKind::Line { a: w[0], b: w[1] }, "l")?;
            }
        }
        CurveKind::Arc { c, a, b } => {
            let mut chain = vec![a];
            chain.extend(&pts);
            chain.push(b);
            if let Some(cu) = sk.curves.get_mut(ci) {
                cu.kind = CurveKind::Arc {
                    c,
                    a,
                    b: chain.get(1).copied().unwrap_or(b),
                };
            }
            for w in chain.windows(2).skip(1) {
                push(
                    sk,
                    CurveKind::Arc {
                        c,
                        a: w[0],
                        b: w[1],
                    },
                    "a",
                )?;
            }
        }
        CurveKind::Circle { c, .. } => {
            if pts.len() < 2 {
                return Err(bad("split", "a circle needs two points to split at"));
            }
            let mut chain = pts.clone();
            if let Some(f) = pts.first() {
                chain.push(*f);
            }
            if let Some(cu) = sk.curves.get_mut(ci) {
                cu.kind = CurveKind::Arc {
                    c,
                    a: chain[0],
                    b: chain[1],
                };
                cu.reversed = false;
            }
            for w in chain.windows(2).skip(1) {
                push(
                    sk,
                    CurveKind::Arc {
                        c,
                        a: w[0],
                        b: w[1],
                    },
                    "a",
                )?;
            }
        }
        _ => {
            return Err(bad(
                "split",
                "not supported yet for ellipses, splines and conics",
            ))
        }
    }
    Ok((pieces, pts))
}

/// Keep the pieces of a split tied together: lines collinear, arcs on one circle.
fn tie(sk: &mut Sketch, pieces: &[usize]) -> Result<()> {
    for w in pieces.windows(2) {
        let (a, b) = (w[0], w[1]);
        let line = matches!(
            sk.curves.get(a).map(|c| &c.kind),
            Some(CurveKind::Line { .. })
        );
        add_c(
            sk,
            if line {
                ConstraintKind::Collinear { a, b }
            } else {
                ConstraintKind::Equal { a, b }
            },
        )?;
    }
    Ok(())
}

/// Dimensions and relations that measure the whole of curve `ci` (a line's length, an equal
/// length, its midpoint, an arc's length or sweep) would stay on the piece that keeps its id
/// after a trim and stretch that piece back to the old size, moving the rest of the sketch. A
/// length becomes the distance between the curve's old ends, as a fillet keeps it to its virtual
/// corner (it goes with an end the trim removes); the others go.
fn release_extent(sk: &mut Sketch, ci: usize) {
    let ends = match sk.curves.get(ci).map(|c| &c.kind) {
        Some(CurveKind::Line { a, b }) => Some((*a, *b)),
        _ => None,
    };
    sk.constraints.retain_mut(|k| match k.kind {
        ConstraintKind::Length { l, value } if l == ci => match ends {
            Some((p, q)) => {
                k.kind = ConstraintKind::Distance { p, q, value };
                true
            }
            None => false,
        },
        ConstraintKind::Equal { a, b } if ends.is_some() && (a == ci || b == ci) => false,
        ConstraintKind::Midpoint { l, .. } if l == ci => false,
        ConstraintKind::ArcLength { c, .. } | ConstraintKind::ArcSweep { c, .. } if c == ci => {
            false
        }
        _ => true,
    });
}

/// The cut parameters bracketing `tq` on curve `ci`: (before, after) with their cutter curves.
fn bracket(sk: &Sketch, ci: usize, tq: f64) -> (Vec<(f64, Vec2, usize)>, bool) {
    let Some(sh) = sk.shape(ci) else {
        return (Vec::new(), false);
    };
    let (t0, t1) = sh.range();
    let full = sh.is_full();
    let span = t1 - t0;
    let mut cuts: Vec<(f64, Vec2, usize)> = cut_points(sk, ci)
        .into_iter()
        .filter(|(t, _, _)| full || (*t > t0 + 1e-9 * span && *t < t1 - 1e-9 * span))
        .collect();
    cuts.sort_by(|a, b| a.0.total_cmp(&b.0));
    if full {
        if cuts.len() < 2 {
            return (Vec::new(), true);
        }
        // Cuts around tq on the circle (wrapping).
        let after = cuts.iter().position(|c| c.0 > tq).unwrap_or(0);
        let before = (after + cuts.len() - 1) % cuts.len();
        let mut two = vec![cuts[before], cuts[after]];
        two.sort_by(|a, b| a.0.total_cmp(&b.0));
        return (two, true);
    }
    let before = cuts.iter().rev().find(|c| c.0 < tq).copied();
    let after = cuts.iter().find(|c| c.0 > tq).copied();
    (before.into_iter().chain(after).collect(), false)
}

pub(super) fn trim(sk: &mut Sketch, ci: usize, at: Vec2) -> Result<()> {
    let cmd = "trim";
    if sk.curves[ci].infinite {
        return Err(bad(cmd, "Infinite lines must be made bounded first"));
    }
    if sk.curves[ci].kind.is_freeform() {
        super::freeform::trim_or_break(sk, ci, at, true, cmd)?;
        return Ok(());
    }
    let sh = sk.shape(ci).ok_or_else(|| bad(cmd, "curve"))?;
    let tq = sh.param(sh.project(at));
    let (cuts, full) = bracket(sk, ci, tq);
    if cuts.is_empty() {
        // Nothing crosses it: the whole curve goes.
        sk.remove_curves(&[ci]);
        return Ok(());
    }
    release_extent(sk, ci);
    let ts: Vec<f64> = cuts.iter().map(|c| c.0).collect();
    let (pieces, pts) = split_at(sk, ci, &ts)?;
    // The piece containing tq.
    let k = if full {
        let (a, b) = (ts[0], ts[1]);
        if tq > a && tq < b {
            0
        } else {
            1
        }
    } else {
        ts.iter().filter(|t| **t < tq).count()
    };
    for (pi, c) in pts.iter().zip(&cuts) {
        pin(sk, *pi, c.2)?;
    }
    let gone = pieces.get(k).copied().ok_or_else(|| bad(cmd, "piece"))?;
    let keep: Vec<usize> = pieces.iter().copied().filter(|x| *x != gone).collect();
    // The original curve keeps its id (and constraints): swap geometry into it if needed.
    let gone = match keep.first() {
        Some(&other) if gone == ci => {
            let ka = sk.curves.get(ci).map(|c| c.kind.clone());
            let kb = sk.curves.get(other).map(|c| c.kind.clone());
            if let (Some(ka), Some(kb)) = (ka, kb) {
                if let Some(c) = sk.curves.get_mut(ci) {
                    c.kind = kb;
                }
                if let Some(c) = sk.curves.get_mut(other) {
                    c.kind = ka;
                }
            }
            other
        }
        _ => gone,
    };
    let keep: Vec<usize> = pieces.iter().copied().filter(|x| *x != gone).collect();
    tie(sk, &keep)?;
    sk.remove_curves(&[gone]);
    Ok(())
}
pub(super) fn split(sk: &mut Sketch, ci: usize, at: Vec2) -> Result<()> {
    if sk.curves[ci].kind.is_freeform() {
        super::freeform::break_at_points(sk, ci, &[at], "split")?;
        return Ok(());
    }
    let sh = sk
        .shape(ci)
        .ok_or_else(|| bad("split", "Select a supported curve"))?;
    let ts = if sh.is_full() {
        let (cuts, _) = bracket(sk, ci, sh.param(sh.project(at)));
        cuts.iter().map(|c| c.0).collect::<Vec<_>>()
    } else {
        vec![sh.param(sh.project(at))]
    };
    let (a, b) = sh.range();
    if ts.is_empty() || (!sh.is_full() && ts.iter().any(|t| *t <= a + 1e-9 || *t >= b - 1e-9)) {
        return Err(bad(
            "split",
            "Choose an interior point; a circle requires two crossings",
        ));
    }
    release_extent(sk, ci);
    let (pieces, _) = split_at(sk, ci, &ts)?;
    tie(sk, &pieces)?;
    Ok(())
}
pub(super) fn extend(sk: &mut Sketch, ci: usize, at: Vec2) -> Result<()> {
    let cmd = "extend";
    if !cad_sketch::solve(sk).ok() {
        return Err(bad(cmd, "Solve the current sketch first"));
    }
    let sh = sk.shape(ci).ok_or_else(|| bad(cmd, "curve"))?;
    let (pa, pb) = match sk.curves.get(ci).map(|c| c.kind.clone()) {
        Some(CurveKind::Line { a, b }) | Some(CurveKind::Arc { a, b, .. }) => (a, b),
        _ => return Err(bad(cmd, "only lines and arcs can be extended")),
    };
    let (qa, qb) = (
        sk.point(pa).unwrap_or_default(),
        sk.point(pb).unwrap_or_default(),
    );
    let at_b = at.dist(qb) <= at.dist(qa);
    let end = if at_b { pb } else { pa };
    let shared = sk
        .curves
        .iter()
        .enumerate()
        .filter(|(i, c)| *i != ci && c.kind.uses(end))
        .count();
    if shared > 0 || sk.points.get(end).is_some_and(|q| q.fixed) {
        return Err(bad(cmd, "that end is connected to other geometry"));
    }
    // Candidate hits on the unbounded curve beyond the chosen end.
    let unbounded = match sh {
        Shape::Line { .. } => sh,
        Shape::Round { c, r, start, .. } => Shape::Round {
            c,
            r,
            start,
            sweep: TAU,
        },
    };
    let (_, t1) = sh.range();
    let mut best: Option<(f64, Vec2, usize)> = None;
    let mut ambiguous = false;
    let mut budget = 1_000_000usize;
    let mut segment_count = 0usize;
    for (oi, _) in sk.curves.iter().enumerate() {
        if oi == ci {
            continue;
        }
        let mut pieces = sk.exact_segs(oi).unwrap_or_else(|| sk.segs(oi));
        // An infinite line (#1359) is a boundary along its whole line.
        if sk.curves.get(oi).is_some_and(|c| c.infinite) {
            pieces = pieces
                .iter()
                .map(|s| support::endless(s, sh.point(sh.param(at))))
                .collect();
        }
        segment_count += pieces.len();
        if segment_count > 16_384 {
            return Err(bad(cmd, "too many exact curve pieces to extend safely"));
        }
        for piece in &pieces {
            for q in support::hits(&unbounded, piece, &mut budget)? {
                let t = sh.param(q);
                let gap = match sh {
                    Shape::Line { a, b } => {
                        let l = a.dist(b);
                        if at_b {
                            (t - 1.0) * l
                        } else {
                            -t * l
                        }
                    }
                    Shape::Round { r, .. } => {
                        // A hit already inside the arc cannot lengthen either end. In
                        // particular, extending the start must not turn a quarter into an eighth.
                        if t <= t1 + TOL / r.max(1e-12) || t >= TAU - TOL / r.max(1e-12) {
                            continue;
                        }
                        if at_b {
                            (t - t1) * r
                        } else {
                            (TAU - t) * r
                        }
                    }
                };
                if gap > TOL * 10.0 {
                    if best.is_none_or(|b| gap < b.0 - TOL) {
                        best = Some((gap, q, oi));
                        ambiguous = false;
                    } else if best.is_some_and(|b| {
                        (gap - b.0).abs() <= TOL && (b.2 != oi || b.1.dist(q) > TOL)
                    }) {
                        ambiguous = true;
                    }
                }
            }
        }
    }
    if ambiguous {
        return Err(bad(
            cmd,
            "multiple curves stop the extension at the same distance",
        ));
    }
    let (_, q, oi) = best.ok_or_else(|| bad(cmd, "nothing to extend to"))?;
    if let Some(pt) = sk.points.get_mut(end) {
        pt.pos = q;
    }
    pin(sk, end, oi)?;
    let before = sk.points.iter().map(|p| p.pos).collect::<Vec<_>>();
    let hold = cad_sketch::Hold {
        points: (0..sk.points.len()).filter(|i| *i != end).collect(),
        radii: (0..sk.curves.len()).collect(),
    };
    if !cad_sketch::solve_with(sk, &hold, None, cad_sketch::Analysis::Full, None).ok()
        || sk.point(end).is_none_or(|point| point.dist(q) > TOL)
        || sk
            .points
            .iter()
            .enumerate()
            .any(|(i, point)| i != end && point.pos.dist(before[i]) > TOL)
    {
        return Err(bad(
            cmd,
            "the extension conflicts with constraints or cannot preserve its original support",
        ));
    }
    Ok(())
}
