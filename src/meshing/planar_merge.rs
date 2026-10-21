//! Recover planar CAD-face loops without changing curved tessellated faces.
use crate::Vec3;
use std::collections::{BTreeMap, BTreeSet};
/// Returns boundary loops in the face's original winding. Degenerate,
/// disconnected, inconsistent or non-manifold triangulations are not merged.
pub(super) fn loops(
    triangles: &[[usize; 3]],
    points: &[Vec3],
    tolerance: f64,
) -> Option<Vec<Vec<i64>>> {
    let first = triangles.first()?;
    let p = points[first[0] - 1];
    let normal = (points[first[1] - 1] - p).cross(points[first[2] - 1] - p);
    let magnitude = normal.norm();
    if !magnitude.is_finite() || magnitude <= tolerance * tolerance {
        return None;
    }
    let n = normal * (1.0 / magnitude);
    let mut edge_uses = BTreeMap::<(usize, usize), Vec<(usize, usize)>>::new();
    for t in triangles {
        let a = points[t[0] - 1];
        let b = points[t[1] - 1];
        let c = points[t[2] - 1];
        if (b - a).cross(c - a).dot(n) <= tolerance * tolerance {
            return None;
        }
        if t.iter()
            .any(|v| (points[*v - 1] - p).dot(n).abs() > tolerance)
        {
            return None;
        }
        for i in 0..3 {
            let a = t[i];
            let b = t[(i + 1) % 3];
            edge_uses
                .entry((a.min(b), a.max(b)))
                .or_default()
                .push((a, b));
        }
    }
    let mut next = BTreeMap::new();
    let mut incoming = BTreeSet::new();
    for uses in edge_uses.values() {
        match uses.as_slice() {
            [(a, b)] => {
                if next.insert(*a, *b).is_some() || !incoming.insert(*b) {
                    return None;
                }
            }
            [(a, b), (c, d)] if a == d && b == c => {}
            _ => return None,
        }
    }
    if next.keys().copied().collect::<BTreeSet<_>>() != incoming {
        return None;
    }
    let mut result = vec![];
    while let Some((&start, _)) = next.first_key_value() {
        let mut vertex = start;
        let mut path = vec![];
        loop {
            let end = next.remove(&vertex)?;
            path.push(vertex as i64);
            vertex = end;
            if vertex == start {
                break;
            }
        }
        if path.len() < 3 {
            return None;
        }
        result.push(path);
    }
    // The largest area loop is outer; hole winding is kept from the triangles.
    let area = |path: &Vec<i64>| {
        (0..path.len())
            .map(|i| {
                ((points[path[i] as usize - 1] - p)
                    .cross(points[path[(i + 1) % path.len()] as usize - 1] - p))
                .dot(n)
            })
            .sum::<f64>()
    };
    let outer = (0..result.len())
        .max_by(|a, b| area(&result[*a]).abs().total_cmp(&area(&result[*b]).abs()))?;
    result.swap(0, outer);
    if area(&result[0]) <= 0.0 || result.iter().skip(1).any(|l| area(l) >= 0.0) {
        return None;
    }
    // Area conservation also rejects disconnected caps masquerading as holes.
    let boundary_area = result.iter().map(|l| area(l) * 0.5).sum::<f64>();
    let triangle_area = triangles
        .iter()
        .map(|t| {
            0.5 * (points[t[1] - 1] - points[t[0] - 1])
                .cross(points[t[2] - 1] - points[t[0] - 1])
                .dot(n)
        })
        .sum::<f64>();
    if !boundary_area.is_finite()
        || !triangle_area.is_finite()
        || (boundary_area - triangle_area).abs() > triangle_area.abs() * 1e-8
    {
        return None;
    }
    Some(result)
}
