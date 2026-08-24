//! Ray picking over canonical CAD topology.
//!
//! This module deliberately derives hits from stable geometry faces and bodies,
//! never from renderer or mesh triangles.

use super::{
    BodyId, EdgeGeometry, FaceId, GeometryBodyRepresentation, GeometryFaceRepresentation,
    GeometryTopology,
};
use crate::Vec3;

const PICK_EPSILON: f64 = 1.0e-9;

/// A finite, normalized world-space ray for CAD picking.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ray3 {
    pub origin: Vec3,
    pub direction: Vec3,
}

impl Ray3 {
    pub fn new(origin: Vec3, direction: Vec3) -> Option<Self> {
        (is_finite(origin) && is_finite(direction))
            .then(|| direction.normalized())
            .flatten()
            .map(|direction| Self { origin, direction })
    }
}

/// A nearest visible canonical CAD face hit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FaceRayHit {
    pub face: FaceId,
    /// The body owning the visible face, when the face belongs to a body.
    pub body: Option<BodyId>,
    pub distance: f64,
    pub position: Vec3,
}

impl GeometryTopology {
    /// Returns the nearest intersection with a canonical planar or extrusion
    /// face. The returned ID is a stable `FaceId`, not a renderer triangle ID.
    pub fn pick_face(&self, ray: Ray3) -> Option<FaceRayHit> {
        let mut candidates = Vec::new();

        for face in self.faces.values() {
            if let GeometryFaceRepresentation::Planar {
                outer_loop,
                inner_loops,
            } = &face.representation
            {
                let Some(outer) = self.loop_points(outer_loop) else {
                    continue;
                };
                let Some(holes) = inner_loops
                    .iter()
                    .map(|loop_edges| self.loop_points(loop_edges))
                    .collect::<Option<Vec<_>>>()
                else {
                    continue;
                };
                candidates.push(FaceCandidate {
                    face: face.id,
                    outer,
                    holes,
                });
            }
        }

        for body in self.bodies.values() {
            let GeometryBodyRepresentation::Extrude {
                source_face,
                distance,
                top_face,
                side_faces,
            } = &body.representation
            else {
                continue;
            };
            let GeometryFaceRepresentation::Planar {
                outer_loop,
                inner_loops,
            } = &self.face(*source_face)?.representation
            else {
                continue;
            };
            let outer = self.loop_points(outer_loop)?;
            let holes = inner_loops
                .iter()
                .map(|loop_edges| self.loop_points(loop_edges))
                .collect::<Option<Vec<_>>>()?;
            let normal = polygon_normal(&outer)?;
            let displacement = normal * *distance;
            candidates.push(FaceCandidate {
                face: *top_face,
                outer: translated(&outer, displacement),
                holes: holes
                    .iter()
                    .map(|hole| translated(hole, displacement))
                    .collect(),
            });

            let mut boundaries = Vec::with_capacity(1 + holes.len());
            boundaries.push(outer);
            boundaries.extend(holes);
            if side_faces.len() != boundaries.iter().map(Vec::len).sum::<usize>() {
                continue;
            }
            for (face, (start, end)) in side_faces.iter().copied().zip(
                boundaries
                    .iter()
                    .flat_map(|loop_points| loop_edges(loop_points)),
            ) {
                candidates.push(FaceCandidate {
                    face,
                    outer: vec![start, end, end + displacement, start + displacement],
                    holes: Vec::new(),
                });
            }
        }

        candidates
            .into_iter()
            .filter_map(|candidate| {
                intersect_face(ray, &candidate)
                    .map(|(distance, position)| (candidate.face, distance, position))
            })
            .min_by(
                |(left_face, left_distance, _), (right_face, right_distance, _)| {
                    left_distance
                        .total_cmp(right_distance)
                        .then_with(|| left_face.cmp(right_face))
                },
            )
            .map(|(face, distance, position)| FaceRayHit {
                face,
                body: self
                    .bodies
                    .values()
                    .find(|body| body.faces.contains(&face))
                    .map(|body| body.id),
                distance,
                position,
            })
    }

    /// Selects the body owning the nearest visible CAD face.
    pub fn pick_body(&self, ray: Ray3) -> Option<BodyId> {
        self.pick_face(ray)?.body
    }

    fn loop_points(&self, loop_edges: &[super::OrientedEdge]) -> Option<Vec<Vec3>> {
        loop_edges
            .iter()
            .map(|oriented| {
                let edge = self.edge(oriented.edge)?;
                let EdgeGeometry::Line { start, end } = edge.geometry else {
                    return None;
                };
                let vertex = if oriented.reversed { end } else { start };
                Some(self.vertex(vertex)?.position)
            })
            .collect()
    }
}

struct FaceCandidate {
    face: FaceId,
    outer: Vec<Vec3>,
    holes: Vec<Vec<Vec3>>,
}

fn intersect_face(ray: Ray3, candidate: &FaceCandidate) -> Option<(f64, Vec3)> {
    let normal = polygon_normal(&candidate.outer)?;
    let denominator = normal.dot(ray.direction);
    if denominator.abs() <= PICK_EPSILON {
        return None;
    }
    let distance = normal.dot(candidate.outer[0] - ray.origin) / denominator;
    if distance <= PICK_EPSILON {
        return None;
    }
    let position = ray.origin + ray.direction * distance;
    point_in_face(position, &candidate.outer, &candidate.holes, normal)
        .then_some((distance, position))
}

fn polygon_normal(points: &[Vec3]) -> Option<Vec3> {
    if points.len() < 3 {
        return None;
    }
    let origin = points[0];
    for index in 1..points.len() - 1 {
        if let Some(normal) = (points[index] - origin)
            .cross(points[index + 1] - origin)
            .normalized()
        {
            return Some(normal);
        }
    }
    None
}

fn point_in_face(point: Vec3, outer: &[Vec3], holes: &[Vec<Vec3>], normal: Vec3) -> bool {
    let axis = if normal.x.abs() < 0.9 {
        normal.cross(Vec3::new(1.0, 0.0, 0.0)).normalized()
    } else {
        normal.cross(Vec3::new(0.0, 1.0, 0.0)).normalized()
    };
    let Some(u) = axis else { return false };
    let v = normal.cross(u);
    let project = |position: Vec3| (position.dot(u), position.dot(v));
    point_in_polygon(
        project(point),
        &outer.iter().copied().map(project).collect::<Vec<_>>(),
    ) && !holes.iter().any(|hole| {
        point_in_polygon(
            project(point),
            &hole.iter().copied().map(project).collect::<Vec<_>>(),
        )
    })
}

fn point_in_polygon(point: (f64, f64), polygon: &[(f64, f64)]) -> bool {
    let mut inside = false;
    for index in 0..polygon.len() {
        let start = polygon[index];
        let end = polygon[(index + 1) % polygon.len()];
        if point_on_segment(point, start, end) {
            return true;
        }
        if (start.1 > point.1) != (end.1 > point.1) {
            let crossing_x = (end.0 - start.0) * (point.1 - start.1) / (end.1 - start.1) + start.0;
            if point.0 < crossing_x {
                inside = !inside;
            }
        }
    }
    inside
}

fn point_on_segment(point: (f64, f64), start: (f64, f64), end: (f64, f64)) -> bool {
    let cross = (point.0 - start.0) * (end.1 - start.1) - (point.1 - start.1) * (end.0 - start.0);
    cross.abs() <= PICK_EPSILON
        && point.0 >= start.0.min(end.0) - PICK_EPSILON
        && point.0 <= start.0.max(end.0) + PICK_EPSILON
        && point.1 >= start.1.min(end.1) - PICK_EPSILON
        && point.1 <= start.1.max(end.1) + PICK_EPSILON
}

fn loop_edges(points: &[Vec3]) -> impl Iterator<Item = (Vec3, Vec3)> + '_ {
    points
        .iter()
        .copied()
        .zip(points.iter().copied().cycle().skip(1))
        .take(points.len())
}

fn translated(points: &[Vec3], displacement: Vec3) -> Vec<Vec3> {
    points.iter().map(|point| *point + displacement).collect()
}

fn is_finite(vector: Vec3) -> bool {
    vector.x.is_finite() && vector.y.is_finite() && vector.z.is_finite()
}
