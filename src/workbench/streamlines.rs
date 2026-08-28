//! Headless 2D streamline integration over immutable run result data.
//!
//! The evaluation policy is explicit: at each query point the owning cell is
//! blended linearly toward its nearest face-neighbour along the two cell-centre
//! segment. This gives a continuous local interpolation across ordinary shared
//! faces while retaining only the immutable cell-centred velocities. Integration
//! uses RK4 on the normalized direction field, so the spatial step is
//! independent of velocity magnitude. This module deliberately owns no GUI
//! state.

use super::{ResultDataError, ResultDataset};
use crate::{MeshDimension, Vec3};
use std::collections::HashMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StreamlineDirection {
    Forward,
    Backward,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StreamlineOptions {
    pub step_size: f64,
    pub max_steps: usize,
    pub max_length: f64,
    pub stagnation_speed: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StreamlinePath {
    pub points: Vec<Vec3>,
}

#[derive(Clone, Debug)]
pub struct StreamlineField {
    polygons: Vec<Vec<(f64, f64)>>,
    velocity: Vec<Vec3>,
    cell_centers: Vec<Vec3>,
    neighbours: Vec<Vec<usize>>,
    bounds: (f64, f64, f64, f64),
    bins_x: usize,
    bins_y: usize,
    cell_bins: HashMap<(usize, usize), Vec<usize>>,
}

impl StreamlineField {
    pub fn from_dataset(dataset: &ResultDataset) -> Result<Self, ResultDataError> {
        let mesh = dataset.mesh();
        if mesh.dimension() != MeshDimension::TwoD {
            return Err(ResultDataError::Unsupported(
                "streamlines currently require a 2D result dataset".into(),
            ));
        }
        let polygons: Vec<Vec<(f64, f64)>> = mesh
            .cells()
            .iter()
            .map(|cell| {
                let mut polygon = Vec::new();
                for &face in &cell.faces {
                    for &vertex in &mesh.faces()[face].vertices {
                        let point = mesh.points()[vertex].position;
                        if !polygon.contains(&(point.x, point.y)) {
                            polygon.push((point.x, point.y));
                        }
                    }
                }
                polygon.sort_by(|left, right| {
                    (left.1 - cell.center.y)
                        .atan2(left.0 - cell.center.x)
                        .total_cmp(&(right.1 - cell.center.y).atan2(right.0 - cell.center.x))
                });
                polygon
            })
            .collect();
        let bounds = polygons.iter().flatten().fold(
            (
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ),
            |(min_x, min_y, max_x, max_y), &(x, y)| {
                (min_x.min(x), min_y.min(y), max_x.max(x), max_y.max(y))
            },
        );
        if !(bounds.0.is_finite()
            && bounds.1.is_finite()
            && bounds.2 > bounds.0
            && bounds.3 > bounds.1)
        {
            return Err(ResultDataError::Invalid(
                "streamline field requires finite non-degenerate 2D bounds".into(),
            ));
        }
        let cell_count = polygons.len().max(1);
        let aspect = (bounds.2 - bounds.0) / (bounds.3 - bounds.1);
        let bins_x = ((cell_count as f64 * aspect).sqrt().ceil() as usize).max(1);
        let bins_y = ((cell_count as f64 / aspect).sqrt().ceil() as usize).max(1);
        let mut cell_bins = HashMap::<(usize, usize), Vec<usize>>::new();
        for (cell, polygon) in polygons.iter().enumerate() {
            let cell_bounds = polygon.iter().fold(
                (
                    f64::INFINITY,
                    f64::INFINITY,
                    f64::NEG_INFINITY,
                    f64::NEG_INFINITY,
                ),
                |(min_x, min_y, max_x, max_y), &(x, y)| {
                    (min_x.min(x), min_y.min(y), max_x.max(x), max_y.max(y))
                },
            );
            let (start_x, start_y) =
                bin_for((cell_bounds.0, cell_bounds.1), bounds, bins_x, bins_y);
            let (end_x, end_y) = bin_for((cell_bounds.2, cell_bounds.3), bounds, bins_x, bins_y);
            for y in start_y..=end_y {
                for x in start_x..=end_x {
                    cell_bins.entry((x, y)).or_default().push(cell);
                }
            }
        }
        let cell_centers = mesh.cells().iter().map(|cell| cell.center).collect();
        let neighbours = mesh
            .cells()
            .iter()
            .enumerate()
            .map(|(cell_index, cell)| {
                let mut neighbours = Vec::new();
                for &face_index in &cell.faces {
                    let face = &mesh.faces()[face_index];
                    if let Some(neighbour) = face.neighbour {
                        let neighbour = if face.owner == cell_index {
                            neighbour
                        } else {
                            face.owner
                        };
                        if !neighbours.contains(&neighbour) {
                            neighbours.push(neighbour);
                        }
                    }
                }
                neighbours
            })
            .collect();
        Ok(Self {
            polygons,
            velocity: dataset.velocity().to_vec(),
            cell_centers,
            neighbours,
            bounds,
            bins_x,
            bins_y,
            cell_bins,
        })
    }

    pub fn locate_cell(&self, point: Vec3) -> Option<usize> {
        if !(point.x.is_finite() && point.y.is_finite()) {
            return None;
        }
        let bin = bin_for((point.x, point.y), self.bounds, self.bins_x, self.bins_y);
        self.cell_bins.get(&bin)?.iter().copied().find(|&cell| {
            self.polygons
                .get(cell)
                .is_some_and(|polygon| point_in_polygon((point.x, point.y), polygon))
        })
    }

    pub fn integrate(
        &self,
        seed: Vec3,
        direction: StreamlineDirection,
        options: StreamlineOptions,
    ) -> Result<StreamlinePath, ResultDataError> {
        if !(options.step_size.is_finite()
            && options.step_size > 0.0
            && options.max_steps > 0
            && options.max_length.is_finite()
            && options.max_length > 0.0
            && options.stagnation_speed.is_finite()
            && options.stagnation_speed >= 0.0)
        {
            return Err(ResultDataError::Invalid(
                "invalid streamline options".into(),
            ));
        }
        if self.locate_cell(seed).is_none() {
            return Err(ResultDataError::Invalid(
                "streamline seed lies outside the 2D fluid domain".into(),
            ));
        }
        let sign = match direction {
            StreamlineDirection::Forward => 1.0,
            StreamlineDirection::Backward => -1.0,
        };
        let mut points = vec![seed];
        let mut length = 0.0;
        let mut current = seed;
        for _ in 0..options.max_steps {
            let Some(next) =
                self.rk4_step(current, options.step_size * sign, options.stagnation_speed)
            else {
                break;
            };
            let segment = (next - current).norm();
            if !segment.is_finite() || length + segment > options.max_length {
                break;
            }
            points.push(next);
            length += segment;
            current = next;
        }
        Ok(StreamlinePath { points })
    }

    /// Integrates evenly distributed point seeds along a physical line segment.
    /// The line and all generated paths are evaluated against this result
    /// dataset's immutable mesh and velocity values.
    pub fn integrate_rake(
        &self,
        start: Vec3,
        end: Vec3,
        seed_count: usize,
        direction: StreamlineDirection,
        options: StreamlineOptions,
    ) -> Result<Vec<StreamlinePath>, ResultDataError> {
        if seed_count < 2 {
            return Err(ResultDataError::Invalid(
                "streamline rake requires at least two seeds".into(),
            ));
        }
        (0..seed_count)
            .map(|index| {
                let t = index as f64 / (seed_count - 1) as f64;
                self.integrate(start + (end - start) * t, direction, options)
            })
            .collect()
    }

    /// Evaluates the documented local continuous reconstruction of the
    /// cell-centred velocity field at a point inside the fluid domain.
    pub fn velocity_at(&self, point: Vec3) -> Option<Vec3> {
        let cell = self.locate_cell(point)?;
        let base = *self.velocity.get(cell)?;
        let center = *self.cell_centers.get(cell)?;
        let Some(neighbour) = self
            .neighbours
            .get(cell)?
            .iter()
            .copied()
            .min_by(|&left, &right| {
                let left_distance = (point - self.cell_centers[left]).norm_squared();
                let right_distance = (point - self.cell_centers[right]).norm_squared();
                left_distance.total_cmp(&right_distance)
            })
        else {
            return Some(base);
        };
        let neighbour_center = *self.cell_centers.get(neighbour)?;
        let offset = neighbour_center - center;
        let distance_squared = offset.norm_squared();
        if !distance_squared.is_finite() || distance_squared <= f64::EPSILON {
            return Some(base);
        }
        let fraction = ((point - center).x * offset.x
            + (point - center).y * offset.y
            + (point - center).z * offset.z)
            / distance_squared;
        let neighbour_velocity = *self.velocity.get(neighbour)?;
        Some(base + (neighbour_velocity - base) * fraction.clamp(0.0, 1.0))
    }

    fn rk4_step(&self, point: Vec3, step: f64, stagnation_speed: f64) -> Option<Vec3> {
        let k1 = self.direction_at(point, stagnation_speed)?;
        let k2 = self.direction_at(point + k1 * (step * 0.5), stagnation_speed)?;
        let k3 = self.direction_at(point + k2 * (step * 0.5), stagnation_speed)?;
        let k4 = self.direction_at(point + k3 * step, stagnation_speed)?;
        let next = point + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (step / 6.0);
        self.locate_cell(next).map(|_| next)
    }

    fn direction_at(&self, point: Vec3, stagnation_speed: f64) -> Option<Vec3> {
        let velocity = self.velocity_at(point)?;
        let speed = velocity.norm();
        (speed.is_finite() && speed > stagnation_speed).then(|| velocity / speed)
    }
}

fn bin_for(
    point: (f64, f64),
    (min_x, min_y, max_x, max_y): (f64, f64, f64, f64),
    bins_x: usize,
    bins_y: usize,
) -> (usize, usize) {
    let normalized_x = ((point.0 - min_x) / (max_x - min_x)).clamp(0.0, 1.0);
    let normalized_y = ((point.1 - min_y) / (max_y - min_y)).clamp(0.0, 1.0);
    (
        (normalized_x * bins_x as f64).floor() as usize % bins_x,
        (normalized_y * bins_y as f64).floor() as usize % bins_y,
    )
}

fn point_in_polygon(point: (f64, f64), polygon: &[(f64, f64)]) -> bool {
    if polygon.len() < 3 {
        return false;
    }
    let mut inside = false;
    for index in 0..polygon.len() {
        let a = polygon[index];
        let b = polygon[(index + 1) % polygon.len()];
        let on_edge = ((b.0 - a.0) * (point.1 - a.1) - (b.1 - a.1) * (point.0 - a.0)).abs()
            <= 1.0e-12
            && point.0 >= a.0.min(b.0) - 1.0e-12
            && point.0 <= a.0.max(b.0) + 1.0e-12
            && point.1 >= a.1.min(b.1) - 1.0e-12
            && point.1 <= a.1.max(b.1) + 1.0e-12;
        if on_edge {
            return true;
        }
        if (a.1 > point.1) != (b.1 > point.1)
            && point.0 < (b.0 - a.0) * (point.1 - a.1) / (b.1 - a.1) + a.0
        {
            inside = !inside;
        }
    }
    inside
}
