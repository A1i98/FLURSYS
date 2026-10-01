//! Stateful, headless 2D geometry editor used by the Slint workbench.
//!
//! Rust owns tool state and all coordinate/picking rules.  The UI only sends
//! pointer actions and renders the resulting stable-topology selection.

use super::GeometrySelectionTarget;
use crate::{
    EdgeGeometry, EdgeId, GeometryError, GeometryFaceRepresentation, GeometryTopology,
    OrientedEdge, Vec3, VertexId,
};

const MIN_GEOMETRY: f64 = 1.0e-9;
const HISTORY_LIMIT: usize = 64;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeometryTool {
    Select,
    Line,
    Rectangle,
    Circle,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViewTransform {
    pub center_x: f64,
    pub center_y: f64,
    pub pixels_per_unit: f64,
    pub viewport_width: f64,
    pub viewport_height: f64,
}

impl Default for ViewTransform {
    fn default() -> Self {
        Self {
            center_x: 0.0,
            center_y: 0.0,
            pixels_per_unit: 100.0,
            viewport_width: 1.0,
            viewport_height: 1.0,
        }
    }
}

impl ViewTransform {
    pub const MIN_ZOOM: f64 = 1.0e-3;
    pub const MAX_ZOOM: f64 = 1.0e6;

    pub fn set_viewport(&mut self, width: f64, height: f64) {
        self.viewport_width = if width.max(1.0).is_finite() {
            width.max(1.0)
        } else {
            1.0
        };
        self.viewport_height = if height.max(1.0).is_finite() {
            height.max(1.0)
        } else {
            1.0
        };
    }

    pub fn world_to_screen(&self, point: (f64, f64)) -> (f64, f64) {
        (
            (point.0 - self.center_x) * self.pixels_per_unit + self.viewport_width * 0.5,
            self.viewport_height * 0.5 - (point.1 - self.center_y) * self.pixels_per_unit,
        )
    }

    pub fn screen_to_world(&self, point: (f64, f64)) -> (f64, f64) {
        (
            (point.0 - self.viewport_width * 0.5) / self.pixels_per_unit + self.center_x,
            (self.viewport_height * 0.5 - point.1) / self.pixels_per_unit + self.center_y,
        )
    }

    pub fn pan_pixels(&mut self, dx: f64, dy: f64) {
        self.center_x -= dx / self.pixels_per_unit;
        self.center_y += dy / self.pixels_per_unit;
    }

    pub fn zoom_at(&mut self, screen: (f64, f64), wheel_delta: f64) {
        let before = self.screen_to_world(screen);
        self.pixels_per_unit = (self.pixels_per_unit * (1.0015_f64).powf(-wheel_delta))
            .clamp(Self::MIN_ZOOM, Self::MAX_ZOOM);
        let after = self.screen_to_world(screen);
        self.center_x += before.0 - after.0;
        self.center_y += before.1 - after.1;
    }

    pub fn fit(&mut self, bounds: Option<(f64, f64, f64, f64)>) {
        let Some((min_x, min_y, max_x, max_y)) = bounds else {
            self.center_x = 0.0;
            self.center_y = 0.0;
            self.pixels_per_unit = 100.0;
            return;
        };
        self.center_x = (min_x + max_x) * 0.5;
        self.center_y = (min_y + max_y) * 0.5;
        let width = (max_x - min_x).abs().max(1.0e-6);
        let height = (max_y - min_y).abs().max(1.0e-6);
        self.pixels_per_unit = (self.viewport_width * 0.9 / width)
            .min(self.viewport_height * 0.9 / height)
            .clamp(Self::MIN_ZOOM, Self::MAX_ZOOM);
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PreviewPrimitive {
    Line((f64, f64), (f64, f64)),
    Rectangle((f64, f64), (f64, f64)),
    Circle((f64, f64), f64),
}

#[derive(Clone, Debug)]
pub struct GeometryEditorState {
    pub active_tool: GeometryTool,
    pub hover_target: Option<GeometrySelectionTarget>,
    pub selection: Vec<GeometrySelectionTarget>,
    pub transform: ViewTransform,
    pub snap_enabled: bool,
    pub grid_enabled: bool,
    pub prefer_faces: bool,
    start: Option<(f64, f64)>,
    cursor: Option<(f64, f64)>,
    undo: Vec<GeometryTopology>,
    redo: Vec<GeometryTopology>,
}

impl Default for GeometryEditorState {
    fn default() -> Self {
        Self::new()
    }
}
impl GeometryEditorState {
    pub fn new() -> Self {
        Self {
            active_tool: GeometryTool::Select,
            hover_target: None,
            selection: Vec::new(),
            transform: ViewTransform::default(),
            snap_enabled: true,
            grid_enabled: true,
            prefer_faces: true,
            start: None,
            cursor: None,
            undo: Vec::new(),
            redo: Vec::new(),
        }
    }
    pub fn preview(&self) -> Option<PreviewPrimitive> {
        let (start, cursor) = (self.start?, self.cursor?);
        match self.active_tool {
            GeometryTool::Line => Some(PreviewPrimitive::Line(start, cursor)),
            GeometryTool::Rectangle => Some(PreviewPrimitive::Rectangle(start, cursor)),
            GeometryTool::Circle => Some(PreviewPrimitive::Circle(start, distance(start, cursor))),
            GeometryTool::Select => None,
        }
    }
    pub fn has_start(&self) -> bool {
        self.start.is_some()
    }
    pub fn cursor_world(&self) -> Option<(f64, f64)> {
        self.cursor
    }
    pub fn grid_spacing(&self) -> f64 {
        adaptive_grid(self.transform.pixels_per_unit)
    }
    pub fn set_tool(&mut self, tool: GeometryTool) {
        self.active_tool = tool;
        self.start = None;
        self.cursor = None;
        self.hover_target = None;
    }
    pub fn cancel(&mut self) {
        self.set_tool(GeometryTool::Select);
    }
    pub fn cursor_moved(&mut self, topology: &GeometryTopology, screen: (f64, f64)) {
        let world = self.snap(topology, self.transform.screen_to_world(screen));
        self.cursor = Some(world);
        self.hover_target = (self.active_tool == GeometryTool::Select)
            .then(|| self.pick(topology, screen, 8.0))
            .flatten();
    }
    pub fn click(
        &mut self,
        topology: &mut GeometryTopology,
        screen: (f64, f64),
        additive: bool,
    ) -> Result<bool, GeometryError> {
        let world = self.snap(topology, self.transform.screen_to_world(screen));
        self.cursor = Some(world);
        if self.active_tool == GeometryTool::Select {
            let hit = self.pick(topology, screen, 8.0);
            if !additive {
                self.selection.clear();
            }
            if let Some(hit) = hit {
                if let Some(index) = self.selection.iter().position(|target| *target == hit) {
                    if additive {
                        self.selection.remove(index);
                    } else {
                        self.selection.push(hit);
                    }
                } else {
                    self.selection.push(hit);
                }
            }
            return Ok(false);
        }
        if let Some(start) = self.start {
            let before = topology.clone();
            let result = match self.active_tool {
                GeometryTool::Line => add_line_at(topology, start, world),
                GeometryTool::Rectangle => add_rectangle_at(topology, start, world),
                GeometryTool::Circle => add_circle(topology, start, world),
                GeometryTool::Select => unreachable!(),
            };
            if let Err(error) = result {
                *topology = before;
                return Err(error);
            }
            self.selection = topology
                .faces()
                .find(|face| before.face(face.id).is_none())
                .map(|face| vec![GeometrySelectionTarget::Face(face.id)])
                .unwrap_or_default();
            self.start = None;
            self.push_undo(before);
            self.redo.clear();
            Ok(true)
        } else {
            self.start = Some(world);
            Ok(false)
        }
    }
    pub fn undo(&mut self, topology: &mut GeometryTopology) -> bool {
        self.start = None;
        let Some(before) = self.undo.pop() else {
            return false;
        };
        self.redo.push(topology.clone());
        *topology = before;
        self.selection
            .retain(|target| target_exists(topology, *target));
        true
    }
    pub fn redo(&mut self, topology: &mut GeometryTopology) -> bool {
        self.start = None;
        let Some(after) = self.redo.pop() else {
            return false;
        };
        self.undo.push(topology.clone());
        *topology = after;
        self.selection
            .retain(|target| target_exists(topology, *target));
        true
    }
    pub fn snapshot_before_delete(&mut self, topology: &GeometryTopology) {
        self.push_undo(topology.clone());
        self.redo.clear();
    }

    /// Removes a just-recorded delete snapshot when the validated delete
    /// command could not remove any selected entity.
    pub fn discard_last_undo_snapshot(&mut self) {
        self.undo.pop();
    }
    pub fn pick(
        &self,
        topology: &GeometryTopology,
        screen: (f64, f64),
        tolerance_pixels: f64,
    ) -> Option<GeometrySelectionTarget> {
        let point = self.transform.screen_to_world(screen);
        let tolerance = tolerance_pixels / self.transform.pixels_per_unit;
        let face_hit = || {
            topology
                .faces()
                .filter(|face| {
                    face_on_xy(topology, &face.representation)
                        && face_contains(topology, &face.representation, point)
                })
                .last()
                .map(|face| GeometrySelectionTarget::Face(face.id))
        };
        if self.prefer_faces {
            if let Some(face) = face_hit() {
                return Some(face);
            }
        }
        topology
            .vertices()
            .filter(|vertex| vertex.position.z.abs() <= MIN_GEOMETRY)
            .filter_map(|vertex| {
                let d = distance((vertex.position.x, vertex.position.y), point);
                (d <= tolerance).then_some((d, GeometrySelectionTarget::Vertex(vertex.id)))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, target)| target)
            .or_else(|| {
                topology
                    .edges()
                    .filter(|edge| edge_on_xy(topology, &edge.geometry))
                    .filter_map(|edge| {
                        edge_distance(topology, edge.geometry.clone(), point)
                            .filter(|d| *d <= tolerance)
                            .map(|d| (d, GeometrySelectionTarget::Edge(edge.id)))
                    })
                    .min_by(|a, b| a.0.total_cmp(&b.0))
                    .map(|(_, target)| target)
            })
            .or_else(face_hit)
    }
    pub fn fit_view(&mut self, topology: &GeometryTopology) {
        let mut points = topology
            .vertices()
            .filter(|v| v.position.z.abs() <= MIN_GEOMETRY)
            .map(|v| (v.position.x, v.position.y));
        let Some((x, y)) = points.next() else {
            self.transform.fit(None);
            return;
        };
        let (mut minx, mut miny, mut maxx, mut maxy) = (x, y, x, y);
        for (x, y) in points {
            minx = minx.min(x);
            miny = miny.min(y);
            maxx = maxx.max(x);
            maxy = maxy.max(y);
        }
        self.transform.fit(Some((minx, miny, maxx, maxy)));
    }
    fn snap(&self, topology: &GeometryTopology, point: (f64, f64)) -> (f64, f64) {
        if !self.snap_enabled {
            return point;
        }
        let tolerance = 10.0 / self.transform.pixels_per_unit;
        if let Some(vertex) = topology
            .vertices()
            .filter(|v| v.position.z.abs() <= MIN_GEOMETRY)
            .min_by(|a, b| {
                distance((a.position.x, a.position.y), point)
                    .total_cmp(&distance((b.position.x, b.position.y), point))
            })
        {
            let p = (vertex.position.x, vertex.position.y);
            if distance(p, point) <= tolerance {
                return p;
            }
        }
        let grid = adaptive_grid(self.transform.pixels_per_unit);
        (
            (point.0 / grid).round() * grid,
            (point.1 / grid).round() * grid,
        )
    }
    fn push_undo(&mut self, snapshot: GeometryTopology) {
        if self.undo.len() == HISTORY_LIMIT {
            self.undo.remove(0);
        }
        self.undo.push(snapshot);
    }
}

fn target_exists(topology: &GeometryTopology, target: GeometrySelectionTarget) -> bool {
    match target {
        GeometrySelectionTarget::Vertex(id) => topology.vertex(id).is_some(),
        GeometrySelectionTarget::Edge(id) => topology.edge(id).is_some(),
        GeometrySelectionTarget::Face(id) => topology.face(id).is_some(),
        GeometrySelectionTarget::Body(id) => topology.body(id).is_some(),
    }
}
fn distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}
fn adaptive_grid(ppu: f64) -> f64 {
    let target = 60.0 / ppu;
    let power = 10_f64.powf(target.max(MIN_GEOMETRY).log10().floor());
    for scale in [1.0, 2.0, 5.0, 10.0] {
        if power * scale >= target {
            return power * scale;
        }
    }
    power * 10.0
}
fn add_vertex_if_needed(
    topology: &mut GeometryTopology,
    point: (f64, f64),
) -> Result<VertexId, GeometryError> {
    if let Some(vertex) = topology
        .vertices()
        .find(|v| distance((v.position.x, v.position.y), point) <= MIN_GEOMETRY)
    {
        return Ok(vertex.id);
    }
    topology.add_vertex(Vec3::new(point.0, point.1, 0.0))
}
fn add_line_at(
    topology: &mut GeometryTopology,
    start: (f64, f64),
    end: (f64, f64),
) -> Result<(), GeometryError> {
    if distance(start, end) <= MIN_GEOMETRY {
        return Err(GeometryError::DegenerateEdge);
    }
    let a = add_vertex_if_needed(topology, start)?;
    let b = add_vertex_if_needed(topology, end)?;
    let edge = topology.add_line(a, b)?;
    close_line_loop_if_possible(topology, edge);
    Ok(())
}

/// A sequence of free sketch lines becomes a profile once it closes a simple
/// cycle. Edges already used by another face and branches are never absorbed.
fn close_line_loop_if_possible(topology: &mut GeometryTopology, new_edge: EdgeId) {
    let used: Vec<_> = topology
        .faces()
        .filter_map(|face| {
            let GeometryFaceRepresentation::Planar {
                outer_loop,
                inner_loops,
            } = &face.representation
            else {
                return None;
            };
            Some(
                outer_loop
                    .iter()
                    .chain(inner_loops.iter().flatten())
                    .map(|entry| entry.edge),
            )
        })
        .flatten()
        .collect();
    let edges: Vec<_> = topology
        .edges()
        .filter_map(|edge| {
            if used.contains(&edge.id) {
                return None;
            }
            let EdgeGeometry::Line { start, end } = edge.geometry else {
                return None;
            };
            let (a, b) = (topology.vertex(start)?, topology.vertex(end)?);
            (a.position.z.abs() <= MIN_GEOMETRY && b.position.z.abs() <= MIN_GEOMETRY)
                .then_some((edge.id, start, end))
        })
        .collect();
    let Some(&(_, start, end)) = edges.iter().find(|(id, _, _)| *id == new_edge) else {
        return;
    };
    let mut loop_edges = vec![OrientedEdge {
        edge: new_edge,
        reversed: false,
    }];
    let mut vertices = vec![start, end];
    let mut current = end;
    while current != start {
        if loop_edges.len() > edges.len() {
            return;
        }
        let choices: Vec<_> = edges
            .iter()
            .filter(|(id, a, b)| {
                !loop_edges.iter().any(|entry| entry.edge == *id)
                    && (*a == current || *b == current)
            })
            .collect();
        let [&(id, a, b)] = choices.as_slice() else {
            return;
        };
        let next = if a == current { b } else { a };
        if vertices.contains(&next) && next != start {
            return;
        }
        loop_edges.push(OrientedEdge {
            edge: id,
            reversed: b == current,
        });
        current = next;
        vertices.push(current);
    }
    if loop_edges.len() < 3
        || edges
            .iter()
            .filter(|(_, a, b)| *a == start || *b == start)
            .count()
            != 2
    {
        return;
    }
    let Some(polygon) = vertices[..vertices.len() - 1]
        .iter()
        .map(|id| vertex_point(topology, *id))
        .collect::<Option<Vec<_>>>()
    else {
        return;
    };
    if !simple_polygon(&polygon) {
        return;
    }
    let _ = topology.add_planar_face(loop_edges, Vec::new());
}

fn simple_polygon(points: &[(f64, f64)]) -> bool {
    let count = points.len();
    let area = (0..count)
        .map(|i| {
            let (a, b) = (points[i], points[(i + 1) % count]);
            a.0 * b.1 - b.0 * a.1
        })
        .sum::<f64>()
        .abs();
    if area <= MIN_GEOMETRY * MIN_GEOMETRY {
        return false;
    }
    for i in 0..count {
        for j in i + 1..count {
            if j == i + 1 || (i == 0 && j == count - 1) {
                continue;
            }
            let (a, b) = (points[i], points[(i + 1) % count]);
            let (c, d) = (points[j], points[(j + 1) % count]);
            let orient = |p: (f64, f64), q: (f64, f64), r: (f64, f64)| {
                (q.0 - p.0) * (r.1 - p.1) - (q.1 - p.1) * (r.0 - p.0)
            };
            let (u, v, w, x) = (
                orient(a, b, c),
                orient(a, b, d),
                orient(c, d, a),
                orient(c, d, b),
            );
            let opposite = |a: f64, b: f64| {
                (a > MIN_GEOMETRY && b < -MIN_GEOMETRY) || (a < -MIN_GEOMETRY && b > MIN_GEOMETRY)
            };
            let touching = (u.abs() <= MIN_GEOMETRY && segment_distance(c, a, b) <= MIN_GEOMETRY)
                || (v.abs() <= MIN_GEOMETRY && segment_distance(d, a, b) <= MIN_GEOMETRY)
                || (w.abs() <= MIN_GEOMETRY && segment_distance(a, c, d) <= MIN_GEOMETRY)
                || (x.abs() <= MIN_GEOMETRY && segment_distance(b, c, d) <= MIN_GEOMETRY);
            if (opposite(u, v) && opposite(w, x)) || touching {
                return false;
            }
        }
    }
    true
}
fn add_rectangle_at(
    topology: &mut GeometryTopology,
    a: (f64, f64),
    b: (f64, f64),
) -> Result<(), GeometryError> {
    if (a.0 - b.0).abs() <= MIN_GEOMETRY || (a.1 - b.1).abs() <= MIN_GEOMETRY {
        return Err(GeometryError::InvalidPrimitive {
            message: "rectangle width and height must be positive".into(),
        });
    }
    let p = [(a.0, a.1), (b.0, a.1), (b.0, b.1), (a.0, b.1)];
    let v = [
        add_vertex_if_needed(topology, p[0])?,
        add_vertex_if_needed(topology, p[1])?,
        add_vertex_if_needed(topology, p[2])?,
        add_vertex_if_needed(topology, p[3])?,
    ];
    let e = [
        topology.add_line(v[0], v[1])?,
        topology.add_line(v[1], v[2])?,
        topology.add_line(v[2], v[3])?,
        topology.add_line(v[3], v[0])?,
    ];
    topology.add_planar_face(
        e.into_iter()
            .map(|edge| OrientedEdge {
                edge,
                reversed: false,
            })
            .collect(),
        Vec::new(),
    )?;
    Ok(())
}
fn add_circle(
    topology: &mut GeometryTopology,
    center: (f64, f64),
    point: (f64, f64),
) -> Result<(), GeometryError> {
    let radius = distance(center, point);
    if radius <= MIN_GEOMETRY {
        return Err(GeometryError::InvalidPrimitive {
            message: "circle radius must be positive".into(),
        });
    }
    let containing_face = topology
        .faces()
        .find(|face| {
            matches!(
                face.representation,
                GeometryFaceRepresentation::Planar { .. }
            ) && face_contains(topology, &face.representation, center)
        })
        .map(|face| (face.id, face.representation.clone()));
    let Some((face, representation)) = containing_face else {
        let center_id = topology.add_vertex(Vec3::new(center.0, center.1, 0.0))?;
        let vertices = [
            (center.0 + radius, center.1),
            (center.0, center.1 + radius),
            (center.0 - radius, center.1),
            (center.0, center.1 - radius),
        ]
        .map(|point| topology.add_vertex(Vec3::new(point.0, point.1, 0.0)))
        .into_iter()
        .collect::<Result<Vec<_>, _>>()?;
        let mut boundary = Vec::new();
        for index in 0..4 {
            boundary.push(OrientedEdge {
                edge: topology.add_circular_arc(
                    vertices[index],
                    center_id,
                    vertices[(index + 1) % 4],
                )?,
                reversed: false,
            });
        }
        topology.add_planar_face(boundary, Vec::new())?;
        return Ok(());
    };
    // A circular hole must remain wholly inside the selected planar domain.
    // Sampling is sufficient for this phase's deterministic arcs and avoids
    // claiming a general CAD curve-intersection kernel.
    let entirely_inside = (0..32).all(|index| {
        let angle = std::f64::consts::TAU * f64::from(index) / 32.0;
        face_contains(
            topology,
            &representation,
            (
                center.0 + radius * angle.cos(),
                center.1 + radius * angle.sin(),
            ),
        )
    });
    if !entirely_inside {
        return Err(GeometryError::InvalidPrimitive {
            message: "circle must remain wholly inside the planar face".into(),
        });
    }
    topology.add_circle_hole(face, Vec3::new(center.0, center.1, 0.0), radius)?;
    Ok(())
}
fn edge_distance(
    topology: &GeometryTopology,
    geometry: EdgeGeometry,
    point: (f64, f64),
) -> Option<f64> {
    match geometry {
        EdgeGeometry::Line { start, end } => Some(segment_distance(
            point,
            vertex_point(topology, start)?,
            vertex_point(topology, end)?,
        )),
        EdgeGeometry::CircularArc { start, center, end } => {
            let a = vertex_point(topology, start)?;
            let c = vertex_point(topology, center)?;
            let b = vertex_point(topology, end)?;
            let r = distance(a, c);
            let angle = angle_on_arc(
                (point.0 - c.0, point.1 - c.1),
                (a.0 - c.0, a.1 - c.1),
                (b.0 - c.0, b.1 - c.1),
            );
            angle.then_some((distance(point, c) - r).abs())
        }
    }
}
fn vertex_point(t: &GeometryTopology, id: VertexId) -> Option<(f64, f64)> {
    t.vertex(id).map(|v| (v.position.x, v.position.y))
}
fn segment_distance(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let dx = b.0 - a.0;
    let dy = b.1 - a.1;
    let q = if dx == 0.0 && dy == 0.0 {
        0.0
    } else {
        ((p.0 - a.0) * dx + (p.1 - a.1) * dy) / (dx * dx + dy * dy)
    }
    .clamp(0.0, 1.0);
    distance(p, (a.0 + q * dx, a.1 + q * dy))
}
fn angle_on_arc(p: (f64, f64), start: (f64, f64), end: (f64, f64)) -> bool {
    let tau = std::f64::consts::TAU;
    let a = p.1.atan2(p.0).rem_euclid(tau);
    let s = start.1.atan2(start.0).rem_euclid(tau);
    let e = end.1.atan2(end.0).rem_euclid(tau);
    (e - s).rem_euclid(tau) >= (a - s).rem_euclid(tau)
}
fn face_contains(
    topology: &GeometryTopology,
    representation: &GeometryFaceRepresentation,
    point: (f64, f64),
) -> bool {
    let GeometryFaceRepresentation::Planar {
        outer_loop,
        inner_loops,
    } = representation
    else {
        return false;
    };
    point_in_loop(topology, outer_loop, point)
        && !inner_loops
            .iter()
            .any(|loop_edges| point_in_loop(topology, loop_edges, point))
}

fn face_on_xy(topology: &GeometryTopology, representation: &GeometryFaceRepresentation) -> bool {
    let GeometryFaceRepresentation::Planar { outer_loop, .. } = representation else {
        return false;
    };
    !outer_loop.is_empty()
        && outer_loop.iter().all(|entry| {
            let Some(edge) = topology.edge(entry.edge) else {
                return false;
            };
            let (start, end) = match edge.geometry {
                EdgeGeometry::Line { start, end }
                | EdgeGeometry::CircularArc { start, end, .. } => (start, end),
            };
            [start, end].into_iter().all(|id| {
                topology
                    .vertex(id)
                    .is_some_and(|v| v.position.z.abs() <= MIN_GEOMETRY)
            })
        })
}

fn edge_on_xy(topology: &GeometryTopology, geometry: &EdgeGeometry) -> bool {
    let vertices = match *geometry {
        EdgeGeometry::Line { start, end } => vec![start, end],
        EdgeGeometry::CircularArc { start, center, end } => vec![start, center, end],
    };
    vertices.into_iter().all(|id| {
        topology
            .vertex(id)
            .is_some_and(|vertex| vertex.position.z.abs() <= MIN_GEOMETRY)
    })
}
fn point_in_loop(topology: &GeometryTopology, edges: &[OrientedEdge], point: (f64, f64)) -> bool {
    let polygon: Vec<_> = edges
        .iter()
        .flat_map(|oriented| {
            let Some(edge) = topology.edge(oriented.edge) else {
                return Vec::new();
            };
            match edge.geometry {
                EdgeGeometry::Line { start, end } => {
                    vertex_point(topology, if oriented.reversed { end } else { start })
                        .into_iter()
                        .collect()
                }
                EdgeGeometry::CircularArc { start, center, end } => {
                    let (Some(a), Some(c), Some(b)) = (
                        vertex_point(topology, start),
                        vertex_point(topology, center),
                        vertex_point(topology, end),
                    ) else {
                        return Vec::new();
                    };
                    let mut points = Vec::with_capacity(16);
                    for i in 0..16 {
                        let t = i as f64 / 16.0;
                        let sa = (a.1 - c.1).atan2(a.0 - c.0);
                        let delta =
                            ((b.1 - c.1).atan2(b.0 - c.0) - sa).rem_euclid(std::f64::consts::TAU);
                        let angle = if oriented.reversed {
                            sa + delta * (1.0 - t)
                        } else {
                            sa + delta * t
                        };
                        points.push((
                            c.0 + distance(a, c) * angle.cos(),
                            c.1 + distance(a, c) * angle.sin(),
                        ));
                    }
                    points
                }
            }
        })
        .collect();
    if polygon.len() < 3 {
        return false;
    };
    let mut inside = false;
    for i in 0..polygon.len() {
        let a = polygon[i];
        let b = polygon[(i + 1) % polygon.len()];
        if (a.1 > point.1) != (b.1 > point.1)
            && point.0 < (b.0 - a.0) * (point.1 - a.1) / (b.1 - a.1) + a.0
        {
            inside = !inside
        }
    }
    inside
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn transform_round_trip_zoom_and_pan_are_stable() {
        let mut v = ViewTransform::default();
        v.set_viewport(800.0, 600.0);
        let p = (1.25, -0.75);
        let s = v.world_to_screen(p);
        assert!((v.screen_to_world(s).0 - p.0).abs() < 1e-12);
        v.zoom_at(s, 120.0);
        let after = v.screen_to_world(s);
        assert!((after.0 - p.0).abs() < 1e-9);
        v.pan_pixels(20.0, -10.0);
        assert_ne!(v.center_x, 0.0)
    }
    #[test]
    fn drawing_picking_and_history_use_stable_ids() {
        let mut t = GeometryTopology::new();
        let mut e = GeometryEditorState::new();
        e.transform.set_viewport(1000.0, 800.0);
        e.set_tool(GeometryTool::Rectangle);
        assert!(!e.click(&mut t, (400.0, 500.0), false).unwrap());
        assert!(e.click(&mut t, (600.0, 300.0), false).unwrap());
        assert_eq!(t.faces().count(), 1);
        assert_eq!(
            e.selection,
            vec![GeometrySelectionTarget::Face(t.faces().next().unwrap().id)]
        );
        e.set_tool(GeometryTool::Select);
        let hit = e.pick(&t, (500.0, 400.0), 8.0);
        assert!(matches!(hit, Some(GeometrySelectionTarget::Face(_))));
        e.set_tool(GeometryTool::Line);
        e.click(&mut t, (400.0, 500.0), false).unwrap();
        assert!(e.click(&mut t, (300.0, 600.0), false).unwrap());
        let edges = t.edges().count();
        assert!(e.undo(&mut t));
        assert_eq!(t.edges().count(), edges - 1);
        assert!(e.redo(&mut t));
        assert_eq!(t.edges().count(), edges)
    }

    #[test]
    fn previews_and_rejected_commits_do_not_mutate_topology() {
        let mut topology = GeometryTopology::new();
        let mut editor = GeometryEditorState::new();
        editor.transform.set_viewport(1000.0, 800.0);
        editor.set_tool(GeometryTool::Line);
        editor.click(&mut topology, (500.0, 400.0), false).unwrap();
        let revision = topology.revision();
        editor.cursor_moved(&topology, (500.0, 400.0));
        assert!(editor.preview().is_some());
        assert!(editor.click(&mut topology, (500.0, 400.0), false).is_err());
        assert_eq!(topology.revision(), revision);
        editor.cancel();
        assert_eq!(topology.revision(), revision);
    }

    #[test]
    fn vertex_edge_face_priority_and_circle_hole_exclusion() {
        let mut topology = GeometryTopology::new();
        let mut editor = GeometryEditorState::new();
        editor.prefer_faces = false;
        editor.transform.set_viewport(1000.0, 800.0);
        editor.set_tool(GeometryTool::Rectangle);
        editor.click(&mut topology, (400.0, 500.0), false).unwrap();
        editor.click(&mut topology, (600.0, 300.0), false).unwrap();
        editor.set_tool(GeometryTool::Circle);
        editor.snap_enabled = false;
        editor.click(&mut topology, (500.0, 400.0), false).unwrap();
        editor.click(&mut topology, (540.0, 400.0), false).unwrap();
        editor.set_tool(GeometryTool::Select);
        assert!(matches!(
            editor.pick(&topology, (400.0, 500.0), 8.0),
            Some(GeometrySelectionTarget::Vertex(_))
        ));
        assert!(matches!(
            editor.pick(&topology, (450.0, 500.0), 8.0),
            Some(GeometrySelectionTarget::Edge(_))
        ));
        assert_eq!(editor.pick(&topology, (520.0, 400.0), 4.0), None);
        assert!(matches!(
            editor.pick(&topology, (570.0, 400.0), 4.0),
            Some(GeometrySelectionTarget::Face(_))
        ));
    }

    #[test]
    fn closed_profile_is_face_selectable_near_its_border_with_explicit_edge_mode() {
        let mut topology = GeometryTopology::new();
        let mut editor = GeometryEditorState::new();
        editor.transform.set_viewport(1000.0, 800.0);
        editor.set_tool(GeometryTool::Rectangle);
        editor.click(&mut topology, (400.0, 500.0), false).unwrap();
        editor.click(&mut topology, (600.0, 300.0), false).unwrap();
        let face = topology.faces().next().unwrap().id;
        editor.set_tool(GeometryTool::Select);
        assert_eq!(
            editor.pick(&topology, (403.0, 400.0), 8.0),
            Some(GeometrySelectionTarget::Face(face))
        );
        editor.prefer_faces = false;
        assert!(matches!(
            editor.pick(&topology, (403.0, 400.0), 8.0),
            Some(GeometrySelectionTarget::Edge(_))
        ));
        editor.prefer_faces = true;
        editor.click(&mut topology, (500.0, 400.0), false).unwrap();
        assert_eq!(editor.selection, vec![GeometrySelectionTarget::Face(face)]);
    }

    #[test]
    fn four_connected_lines_form_a_selectable_extrudable_face() {
        let mut topology = GeometryTopology::new();
        let mut editor = GeometryEditorState::new();
        editor.transform.set_viewport(1000.0, 800.0);
        editor.set_tool(GeometryTool::Line);
        let corners = [
            (400.0, 500.0),
            (600.0, 500.0),
            (600.0, 300.0),
            (400.0, 300.0),
        ];
        for i in 0..4 {
            editor.click(&mut topology, corners[i], false).unwrap();
            editor
                .click(&mut topology, corners[(i + 1) % 4], false)
                .unwrap();
            assert_eq!(topology.faces().count(), usize::from(i == 3));
        }
        let face = topology.faces().next().unwrap().id;
        assert_eq!(editor.selection, vec![GeometrySelectionTarget::Face(face)]);
        assert_eq!(
            editor.pick(&topology, (500.0, 400.0), 8.0),
            Some(GeometrySelectionTarget::Face(face))
        );
        assert!(editor.undo(&mut topology));
        assert_eq!(topology.faces().count(), 0);
        assert!(editor.redo(&mut topology));
        assert_eq!(topology.faces().count(), 1);
    }

    #[test]
    fn open_lines_and_self_intersecting_loops_do_not_create_faces() {
        let mut topology = GeometryTopology::new();
        let mut editor = GeometryEditorState::new();
        editor.transform.set_viewport(1000.0, 800.0);
        editor.set_tool(GeometryTool::Line);
        let points = [
            (400.0, 500.0),
            (600.0, 300.0),
            (400.0, 300.0),
            (600.0, 500.0),
        ];
        for i in 0..3 {
            editor.click(&mut topology, points[i], false).unwrap();
            editor.click(&mut topology, points[i + 1], false).unwrap();
        }
        assert_eq!(topology.faces().count(), 0);
        editor.click(&mut topology, points[3], false).unwrap();
        editor.click(&mut topology, points[0], false).unwrap();
        assert_eq!(topology.faces().count(), 0);
    }

    #[test]
    fn concave_closed_line_loop_creates_a_face() {
        let mut topology = GeometryTopology::new();
        let mut editor = GeometryEditorState::new();
        editor.transform.set_viewport(1000.0, 800.0);
        editor.set_tool(GeometryTool::Line);
        let points = [
            (400.0, 500.0),
            (600.0, 500.0),
            (600.0, 400.0),
            (500.0, 400.0),
            (500.0, 300.0),
            (400.0, 300.0),
        ];
        for i in 0..points.len() {
            editor.click(&mut topology, points[i], false).unwrap();
            editor
                .click(&mut topology, points[(i + 1) % points.len()], false)
                .unwrap();
        }
        assert_eq!(topology.faces().count(), 1);
    }

    #[test]
    fn selection_replaces_or_toggles_without_changing_geometry() {
        let mut topology = GeometryTopology::new();
        let mut editor = GeometryEditorState::new();
        editor.transform.set_viewport(1000.0, 800.0);
        editor.set_tool(GeometryTool::Rectangle);
        editor.click(&mut topology, (400.0, 500.0), false).unwrap();
        editor.click(&mut topology, (600.0, 300.0), false).unwrap();
        editor.set_tool(GeometryTool::Select);
        let revision = topology.revision();
        editor.click(&mut topology, (450.0, 500.0), false).unwrap();
        assert_eq!(editor.selection.len(), 1);
        editor.click(&mut topology, (600.0, 450.0), true).unwrap();
        assert_eq!(editor.selection.len(), 2);
        editor.click(&mut topology, (450.0, 500.0), true).unwrap();
        assert_eq!(editor.selection.len(), 1);
        editor.click(&mut topology, (500.0, 400.0), false).unwrap();
        assert_eq!(editor.selection.len(), 1);
        assert_eq!(topology.revision(), revision);
    }

    #[test]
    fn circle_crossing_a_planar_boundary_is_rejected_without_revision_change() {
        let mut topology = GeometryTopology::new();
        let mut editor = GeometryEditorState::new();
        editor.transform.set_viewport(1000.0, 800.0);
        editor.set_tool(GeometryTool::Rectangle);
        editor.click(&mut topology, (400.0, 500.0), false).unwrap();
        editor.click(&mut topology, (600.0, 300.0), false).unwrap();
        editor.set_tool(GeometryTool::Circle);
        editor.snap_enabled = false;
        editor.click(&mut topology, (500.0, 400.0), false).unwrap();
        let revision = topology.revision();
        assert!(editor.click(&mut topology, (610.0, 400.0), false).is_err());
        assert_eq!(topology.revision(), revision);
    }

    #[test]
    fn invalid_rectangle_keeps_first_corner_and_does_not_leave_orphan_vertices() {
        let mut topology = GeometryTopology::new();
        let mut editor = GeometryEditorState::new();
        editor.transform.set_viewport(1000.0, 800.0);
        editor.set_tool(GeometryTool::Rectangle);
        editor.click(&mut topology, (400.0, 500.0), false).unwrap();
        let revision = topology.revision();
        assert!(editor.click(&mut topology, (400.0, 300.0), false).is_err());
        assert!(editor.has_start());
        assert_eq!(topology.revision(), revision);
        assert_eq!(topology.vertices().count(), 0);
        assert!(editor.click(&mut topology, (600.0, 300.0), false).unwrap());
        assert_eq!(topology.faces().count(), 1);
    }

    #[test]
    fn circle_creates_a_selectable_face_without_an_existing_rectangle() {
        let mut topology = GeometryTopology::new();
        let mut editor = GeometryEditorState::new();
        editor.transform.set_viewport(1000.0, 800.0);
        editor.snap_enabled = false;
        editor.set_tool(GeometryTool::Circle);
        editor.click(&mut topology, (500.0, 400.0), false).unwrap();
        assert!(editor.click(&mut topology, (550.0, 400.0), false).unwrap());
        assert_eq!(topology.faces().count(), 1);
        assert!(matches!(
            editor.pick(&topology, (525.0, 400.0), 4.0),
            Some(GeometrySelectionTarget::Face(_))
        ));
        assert_eq!(editor.pick(&topology, (545.0, 355.0), 4.0), None);
        assert!(editor.undo(&mut topology));
        assert_eq!(topology.vertices().count(), 0);
    }

    #[test]
    fn displayed_grid_and_snapping_share_spacing_at_different_zooms() {
        let mut editor = GeometryEditorState::new();
        editor.transform.set_viewport(1000.0, 800.0);
        editor.set_tool(GeometryTool::Line);
        for zoom in [0.1, 100.0, 10000.0] {
            editor.transform.pixels_per_unit = zoom;
            let grid = editor.grid_spacing();
            editor.cursor_moved(&GeometryTopology::new(), (538.0, 431.0));
            let (x, y) = editor.cursor_world().unwrap();
            assert!((x / grid - (x / grid).round()).abs() < 1e-9);
            assert!((y / grid - (y / grid).round()).abs() < 1e-9);
        }
    }

    #[test]
    fn xy_sketch_picker_does_not_select_vertices_or_edges_above_the_sketch_plane() {
        let mut topology = GeometryTopology::new();
        let rectangle = topology.add_rectangle(2.0, 1.0).unwrap();
        let extrusion = topology.extrude_planar_face(rectangle.face, 0.5).unwrap();
        let mut editor = GeometryEditorState::new();
        editor.transform.set_viewport(1000.0, 800.0);
        editor.prefer_faces = false;
        let p = editor.transform.world_to_screen((0.0, 0.0));
        assert_eq!(
            editor.pick(&topology, p, 8.0),
            Some(GeometrySelectionTarget::Vertex(rectangle.vertices[0]))
        );
        let top = topology.edge(extrusion.top_edges[0]).unwrap();
        let EdgeGeometry::Line { start, .. } = top.geometry else {
            panic!("expected edge")
        };
        assert_ne!(start, rectangle.vertices[0]);
        let p = editor.transform.world_to_screen((1.0, 0.0));
        assert_eq!(
            editor.pick(&topology, p, 8.0),
            Some(GeometrySelectionTarget::Edge(rectangle.bottom))
        );
    }
}
