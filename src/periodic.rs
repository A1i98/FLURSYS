//! Strict one-to-one translational matching for conformal cyclic patches.
//! No rotational transform, AMI interpolation or pressure operator is implied.
use crate::{MeshId, UnstructuredMesh, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

fn default_tolerance() -> f64 {
    1e-8
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TranslationalCyclicPair {
    pub first: String,
    pub second: String,
    /// Maps first-patch coordinates to second-patch coordinates [m].
    pub translation: Vec3,
    #[serde(default = "default_tolerance")]
    pub tolerance: f64,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CyclicLink {
    pub peer_face: usize,
    /// Local-to-peer translation; subtract it to image a peer cell locally.
    pub translation: Vec3,
}
#[derive(Clone, Debug, PartialEq)]
pub struct CyclicMap {
    mesh_id: MeshId,
    links: Vec<Option<CyclicLink>>,
    pair_count: usize,
}
impl CyclicMap {
    pub fn link(&self, face: usize) -> Option<CyclicLink> {
        self.links.get(face).copied().flatten()
    }
    pub fn pair_count(&self) -> usize {
        self.pair_count
    }
    pub fn mesh_id(&self) -> MeshId {
        self.mesh_id
    }
}
fn finite(v: Vec3) -> bool {
    [v.x, v.y, v.z].iter().all(|v| v.is_finite())
}
fn bucket(p: Vec3, origin: Vec3, tol: f64) -> Result<(i64, i64, i64), String> {
    let d = (p - origin) / tol;
    if !finite(d) || [d.x, d.y, d.z].iter().any(|v| v.abs() > 1e15) {
        return Err("Cyclic matching tolerance is too small for the coordinate span".into());
    }
    Ok((d.x.floor() as i64, d.y.floor() as i64, d.z.floor() as i64))
}
fn vertices_match(mesh: &UnstructuredMesh, a: usize, b: usize, t: Vec3, tol: f64) -> bool {
    let fa = &mesh.faces()[a];
    let fb = &mesh.faces()[b];
    if fa.vertices.len() != fb.vertices.len() {
        return false;
    }
    let mut used = vec![false; fb.vertices.len()];
    for &i in &fa.vertices {
        let point = mesh.points()[i].position + t;
        let candidates = fb
            .vertices
            .iter()
            .enumerate()
            .filter(|(j, v)| !used[*j] && (mesh.points()[**v].position - point).norm() <= tol)
            .map(|(j, _)| j)
            .collect::<Vec<_>>();
        let [j] = candidates.as_slice() else {
            return false;
        };
        let j = *j;
        used[j] = true;
    }
    true
}
pub fn resolve(
    mesh: &UnstructuredMesh,
    pairs: &[TranslationalCyclicPair],
) -> Result<CyclicMap, String> {
    if pairs.len() > 32 {
        return Err("At most 32 cyclic patch pairs are supported".into());
    }
    let mut result = CyclicMap {
        mesh_id: mesh.id(),
        links: vec![None; mesh.face_count()],
        pair_count: 0,
    };
    let mut names = BTreeSet::new();
    for pair in pairs {
        if pair.first == pair.second
            || !names.insert(pair.first.clone())
            || !names.insert(pair.second.clone())
        {
            return Err("Cyclic patches must be distinct and belong to exactly one pair".into());
        }
        if !finite(pair.translation)
            || !pair.translation.norm().is_finite()
            || pair.translation.norm() == 0.0
            || !pair.tolerance.is_finite()
            || pair.tolerance <= 0.0
            || pair.tolerance >= pair.translation.norm() * 0.01
        {
            return Err("Cyclic translation must be finite/nonzero and 0 < absolute tolerance < 1% of translation length".into());
        }
        let patch = |name: &str| {
            mesh.boundary_patches()
                .iter()
                .find(|p| p.name == name)
                .ok_or_else(|| format!("Unknown cyclic patch {name}"))
        };
        let a = patch(&pair.first)?;
        let b = patch(&pair.second)?;
        if a.face_indices.is_empty() || a.face_indices.len() != b.face_indices.len() {
            return Err("Conformal cyclic patches require equal nonzero face counts; AMI is not implemented".into());
        }
        let origin = mesh.faces()[b.face_indices[0]].center;
        let mut bins: BTreeMap<(i64, i64, i64), Vec<usize>> = BTreeMap::new();
        for &i in &b.face_indices {
            if mesh.faces()[i].neighbour.is_some() {
                return Err("Cyclic patch contains an internal face".into());
            }
            bins.entry(bucket(mesh.faces()[i].center, origin, pair.tolerance)?)
                .or_default()
                .push(i);
        }
        for &i in &a.face_indices {
            let f = &mesh.faces()[i];
            if f.neighbour.is_some() || result.links[i].is_some() {
                return Err("Overlapping or internal cyclic face".into());
            }
            let target = f.center + pair.translation;
            let (x, y, z) = bucket(target, origin, pair.tolerance)?;
            let mut matches = Vec::new();
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        if let Some(candidates) = bins.get(&(x + dx, y + dy, z + dz)) {
                            for &j in candidates {
                                let g = &mesh.faces()[j];
                                if (g.center - target).norm() <= pair.tolerance
                                    && vertices_match(mesh, i, j, pair.translation, pair.tolerance)
                                {
                                    matches.push(j);
                                }
                            }
                        }
                    }
                }
            }
            if matches.len() != 1 {
                return Err(format!(
                    "Cyclic face {i} has {} matching peers; need exactly one",
                    matches.len()
                ));
            }
            let j = matches[0];
            let g = &mesh.faces()[j];
            if result.links[j].is_some() {
                return Err("Cyclic matching is not one-to-one".into());
            }
            if (f.area - g.area).abs() > 1e-8 * f.area.max(g.area)
                || (f.normal + g.normal).norm() > 1e-8
            {
                return Err("Cyclic faces require equal areas and opposite outward normals".into());
            }
            let d = mesh.cells()[g.owner].center - pair.translation - mesh.cells()[f.owner].center;
            if !finite(d) || d.dot(f.normal) <= 0.0 {
                return Err("Invalid cyclic image-cell normal distance".into());
            }
            result.links[i] = Some(CyclicLink {
                peer_face: j,
                translation: pair.translation,
            });
            result.links[j] = Some(CyclicLink {
                peer_face: i,
                translation: -pair.translation,
            });
            result.pair_count += 1;
        }
    }
    Ok(result)
}
