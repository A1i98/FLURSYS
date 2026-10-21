//! Nearest distance to wall segments/triangles; a BVH avoids cell × wall scans.
use super::*;
#[derive(Clone, Copy)]
struct Primitive([Vec3; 3]);
struct Node {
    min: [f64; 3],
    max: [f64; 3],
    items: Vec<Primitive>,
    children: Option<Box<[Node; 2]>>,
}
fn xyz(p: Vec3) -> [f64; 3] {
    [p.x, p.y, p.z]
}
impl Node {
    fn build(mut items: Vec<Primitive>) -> Self {
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        for p in &items {
            for v in p.0 {
                for (a, x) in xyz(v).into_iter().enumerate() {
                    min[a] = min[a].min(x);
                    max[a] = max[a].max(x);
                }
            }
        }
        let mut node = Self {
            min,
            max,
            items: Vec::new(),
            children: None,
        };
        if items.len() <= 8 {
            node.items = items;
        } else {
            let axis = (0..3)
                .max_by(|a, b| (max[*a] - min[*a]).total_cmp(&(max[*b] - min[*b])))
                .unwrap();
            items.sort_unstable_by(|a, b| {
                a.0.iter()
                    .map(|p| xyz(*p)[axis])
                    .sum::<f64>()
                    .total_cmp(&b.0.iter().map(|p| xyz(*p)[axis]).sum::<f64>())
            });
            let right = items.split_off(items.len() / 2);
            node.children = Some(Box::new([Self::build(items), Self::build(right)]));
        }
        node
    }
    fn bound(&self, p: Vec3) -> f64 {
        xyz(p)
            .into_iter()
            .enumerate()
            .map(|(i, x)| {
                let d = (self.min[i] - x).max(0.0).max(x - self.max[i]);
                d * d
            })
            .sum()
    }
    fn nearest(&self, p: Vec3, best: &mut f64) {
        if self.bound(p) >= *best {
            return;
        }
        if let Some(c) = &self.children {
            let (a, b) = if c[0].bound(p) < c[1].bound(p) {
                (0, 1)
            } else {
                (1, 0)
            };
            c[a].nearest(p, best);
            c[b].nearest(p, best);
        } else {
            for primitive in &self.items {
                *best = best.min(distance(p, primitive.0));
            }
        }
    }
}
fn segment(p: Vec3, a: Vec3, b: Vec3) -> f64 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.norm_squared().max(1e-300)).clamp(0.0, 1.0);
    (p - a - ab * t).norm_squared()
}
fn distance(p: Vec3, [a, b, c]: [Vec3; 3]) -> f64 {
    let n = (b - a).cross(c - a);
    let n2 = n.norm_squared();
    if n2 > 0.0 {
        let q = p - n * ((p - a).dot(n) / n2);
        if (b - a).cross(q - a).dot(n) >= 0.0
            && (c - b).cross(q - b).dot(n) >= 0.0
            && (a - c).cross(q - c).dot(n) >= 0.0
        {
            return (p - q).norm_squared();
        }
    }
    segment(p, a, b).min(segment(p, b, c)).min(segment(p, c, a))
}
pub(super) fn distances(mesh: &UnstructuredMesh, walls: &[bool]) -> Result<Vec<f64>, SimpleError> {
    let mut primitives = Vec::new();
    for (f, wall) in mesh.faces().iter().zip(walls) {
        if *wall {
            let a = mesh.points()[f.vertices[0]].position;
            if f.vertices.len() == 2 {
                primitives.push(Primitive([a, mesh.points()[f.vertices[1]].position, a]));
            } else {
                for edge in f.vertices[1..].windows(2) {
                    primitives.push(Primitive([
                        a,
                        mesh.points()[edge[0]].position,
                        mesh.points()[edge[1]].position,
                    ]));
                }
            }
        }
    }
    if primitives.is_empty() {
        return Ok(vec![f64::INFINITY; mesh.cell_count()]);
    }
    let tree = Node::build(primitives);
    let distances = mesh
        .cells()
        .iter()
        .map(|c| {
            let mut d = f64::INFINITY;
            tree.nearest(c.center, &mut d);
            d.sqrt()
        })
        .collect::<Vec<_>>();
    if distances.iter().any(|d| !d.is_finite() || *d <= 0.0) {
        return Err(SimpleError::PhysicalBoundary(
            "Invalid wall distance for a turbulence cell".into(),
        ));
    }
    Ok(distances)
}
