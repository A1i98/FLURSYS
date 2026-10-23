//! Piecewise-constant cell-field initialization across fixed meshes.
//! This is not conservative remapping or a transient restart.
use super::results::ResultDataset;
use crate::{CellField, IncompressibleInitialConditions, UnstructuredMesh, Vec3};
/// Own only the fields needed by initialization, excluding derived display caches.
pub struct SourceFields {
    mesh: UnstructuredMesh,
    pressure: Vec<f64>,
    velocity: Vec<Vec3>,
}
impl SourceFields {
    pub fn copy_from_result(source: &ResultDataset) -> Self {
        Self {
            mesh: source.mesh().clone(),
            pressure: source.pressure().to_vec(),
            velocity: source.velocity().to_vec(),
        }
    }
    pub fn from_result(source: ResultDataset) -> Self {
        let (mesh, pressure, velocity) = source.into_primary_fields();
        Self {
            mesh,
            pressure,
            velocity,
        }
    }
}
#[derive(Clone, Copy)]
struct Bounds {
    low: Vec3,
    high: Vec3,
}
impl Bounds {
    fn contains(self, p: Vec3, t: f64) -> bool {
        p.x >= self.low.x - t
            && p.x <= self.high.x + t
            && p.y >= self.low.y - t
            && p.y <= self.high.y + t
            && p.z >= self.low.z - t
            && p.z <= self.high.z + t
    }
    fn merge(self, b: Self) -> Self {
        Self {
            low: Vec3::new(
                self.low.x.min(b.low.x),
                self.low.y.min(b.low.y),
                self.low.z.min(b.low.z),
            ),
            high: Vec3::new(
                self.high.x.max(b.high.x),
                self.high.y.max(b.high.y),
                self.high.z.max(b.high.z),
            ),
        }
    }
}
enum Node {
    Leaf(Bounds, Vec<usize>),
    Branch(Bounds, Box<Node>, Box<Node>),
}
fn component(v: Vec3, a: usize) -> f64 {
    match a {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}
impl Node {
    fn build(indices: &mut [usize], bounds: &[Bounds], mesh: &UnstructuredMesh) -> Self {
        let b = indices
            .iter()
            .skip(1)
            .fold(bounds[indices[0]], |b, i| b.merge(bounds[*i]));
        if indices.len() <= 12 {
            return Self::Leaf(b, indices.to_vec());
        }
        let d = b.high - b.low;
        let axis = if d.x >= d.y && d.x >= d.z {
            0
        } else if d.y >= d.z {
            1
        } else {
            2
        };
        let mid = indices.len() / 2;
        indices.select_nth_unstable_by(mid, |a, b| {
            component(mesh.cells()[*a].center, axis)
                .total_cmp(&component(mesh.cells()[*b].center, axis))
                .then(a.cmp(b))
        });
        let (a, c) = indices.split_at_mut(mid);
        Self::Branch(
            b,
            Box::new(Self::build(a, bounds, mesh)),
            Box::new(Self::build(c, bounds, mesh)),
        )
    }
    fn find(&self, p: Vec3, t: f64, mesh: &UnstructuredMesh) -> Option<usize> {
        let b = match self {
            Self::Leaf(b, _) | Self::Branch(b, _, _) => b,
        };
        if !b.contains(p, t) {
            return None;
        }
        match self {
            Self::Leaf(_, ids) => ids
                .iter()
                .copied()
                .filter(|i| {
                    mesh.cells()[*i].faces.iter().all(|fi| {
                        let f = &mesh.faces()[*fi];
                        let sign = if f.owner == *i { 1.0 } else { -1.0 };
                        sign * (p - f.center).dot(f.normal) <= t
                    })
                })
                .min(),
            Self::Branch(_, a, b) => match (a.find(p, t, mesh), b.find(p, t, mesh)) {
                (Some(a), Some(b)) => Some(a.min(b)),
                (a, b) => a.or(b),
            },
        }
    }
}
pub struct MappedFields {
    pub description: String,
    pub initial: IncompressibleInitialConditions,
    pub source_cells: usize,
    pub target_cells: usize,
    pub max_centre_distance: f64,
}
pub fn map_initial_fields(
    source: &SourceFields,
    target: &UnstructuredMesh,
) -> Result<MappedFields, String> {
    map_initial_fields_with_cancel(source, target, || false)
}
pub fn map_initial_fields_with_cancel(
    source: &SourceFields,
    target: &UnstructuredMesh,
    cancelled: impl Fn() -> bool,
) -> Result<MappedFields, String> {
    let mesh = &source.mesh;
    if mesh.dimension() != target.dimension() {
        return Err("Source and target mesh dimensions must match".into());
    }
    if mesh.cell_count() == 0 || target.cell_count() == 0 {
        return Err("Both meshes must contain cells".into());
    }
    let mut bounds = Vec::with_capacity(mesh.cell_count());
    for cell in mesh.cells() {
        if cancelled() {
            return Err("Field mapping cancelled".into());
        }
        let mut b = Bounds {
            low: Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
            high: Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
        };
        for fi in &cell.faces {
            for vi in &mesh.faces()[*fi].vertices {
                let p = mesh.points()[*vi].position;
                b = b.merge(Bounds { low: p, high: p });
            }
        }
        bounds.push(b);
    }
    let extent = bounds.iter().skip(1).fold(bounds[0], |a, b| a.merge(*b));
    let tolerance = (extent.high - extent.low).norm().max(1e-12) * 1e-10;
    for (id, cell) in mesh.cells().iter().enumerate() {
        if cancelled() {
            return Err("Field mapping cancelled".into());
        }
        for fi in &cell.faces {
            let face = &mesh.faces()[*fi];
            let sign = if face.owner == id { 1.0 } else { -1.0 };
            if cell
                .faces
                .iter()
                .flat_map(|i| mesh.faces()[*i].vertices.iter())
                .any(|i| {
                    sign * (mesh.points()[*i].position - face.center).dot(face.normal) > tolerance
                })
            {
                return Err(format!("Source cell {id} is not convex/planar within mapping tolerance. This initializer supports convex cells only."));
            }
        }
    }
    let mut indices = (0..mesh.cell_count()).collect::<Vec<_>>();
    let tree = Node::build(&mut indices, &bounds, mesh);
    let mut pressure = Vec::with_capacity(target.cell_count());
    let mut velocity = Vec::with_capacity(target.cell_count());
    let mut missing = Vec::new();
    let mut max_distance = 0.0_f64;
    for (i, c) in target.cells().iter().enumerate() {
        if cancelled() {
            return Err("Field mapping cancelled".into());
        }
        if let Some(owner) = tree.find(c.center, tolerance, mesh) {
            pressure.push(source.pressure[owner]);
            velocity.push(source.velocity[owner]);
            max_distance = max_distance.max((c.center - mesh.cells()[owner].center).norm());
        } else {
            if missing.len() < 12 {
                missing.push(i);
            }
            // No partial result is ever returned or applied.
            pressure.push(f64::NAN);
            velocity.push(Vec3::ZERO);
        }
    }
    if !missing.is_empty() {
        return Err(format!("Mapping rejected: target centres outside source cells (first IDs: {missing:?}). Match geometry and coordinates; extrapolation is disabled."));
    }
    Ok(MappedFields {
        description:format!("Source-cell mapping: {} source → {} target cells; maximum centre distance {max_distance:.6e} m",mesh.cell_count(),target.cell_count()),
        initial: IncompressibleInitialConditions::fields(
            CellField::from_values(target, velocity).map_err(|e| format!("{e:?}"))?,
            CellField::from_values(target, pressure).map_err(|e| format!("{e:?}"))?,
        ),
        source_cells: mesh.cell_count(),
        target_cells: target.cell_count(),
        max_centre_distance: max_distance,
    })
}

/// SetFields-style box initialization. Cell centres determine membership.
pub fn patch_box(
    mesh: &UnstructuredMesh,
    initial: IncompressibleInitialConditions,
    low: Vec3,
    high: Vec3,
    value: Vec3,
    p: f64,
    cancelled: impl Fn() -> bool,
) -> Result<MappedFields, String> {
    let values = [
        low.x, low.y, low.z, high.x, high.y, high.z, value.x, value.y, value.z, p,
    ];
    if values.iter().any(|v| !v.is_finite())
        || low.x >= high.x
        || low.y >= high.y
        || low.z > high.z
        || (mesh.dimension() == crate::MeshDimension::ThreeD && low.z >= high.z)
    {
        return Err(
            "Require finite box bounds and values; min < max in X/Y and in Z for 3D.".into(),
        );
    }
    let (mut velocity, mut pressure) = match initial {
        IncompressibleInitialConditions::Uniform { velocity, pressure } => (
            vec![velocity; mesh.cell_count()],
            vec![pressure; mesh.cell_count()],
        ),
        IncompressibleInitialConditions::Fields { velocity, pressure }
        | IncompressibleInitialConditions::Restart {
            velocity, pressure, ..
        } => {
            velocity.ensure_mesh(mesh).map_err(|e| format!("{e:?}"))?;
            pressure.ensure_mesh(mesh).map_err(|e| format!("{e:?}"))?;
            (velocity.values().to_vec(), pressure.values().to_vec())
        }
    };
    let bounds = Bounds { low, high };
    let mut count = 0;
    for (i, c) in mesh.cells().iter().enumerate() {
        if cancelled() {
            return Err("Field patch cancelled".into());
        }
        if bounds.contains(c.center, 0.0) {
            velocity[i] = value;
            pressure[i] = p;
            count += 1;
        }
    }
    if count == 0 {
        return Err(
            "No target cell centres lie in the initialization box; fields unchanged.".into(),
        );
    }
    Ok(MappedFields {
        initial: IncompressibleInitialConditions::fields(
            CellField::from_values(mesh, velocity).map_err(|e| format!("{e:?}"))?,
            CellField::from_values(mesh, pressure).map_err(|e| format!("{e:?}"))?,
        ),
        source_cells: mesh.cell_count(),
        target_cells: mesh.cell_count(),
        max_centre_distance: 0.0,
        description: format!(
            "Box patch applied to {count} cell centres; fields outside the box retained"
        ),
    })
}
