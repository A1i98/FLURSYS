//! Post-processing gradients from archived cell values. No wall data or face
//! fluxes are present in the artifact: these are spatial LSQ reconstructions,
//! not the solver's conservative continuity residual or wall-vorticity values.
use crate::{MeshDimension, UnstructuredMesh, Vec3};
pub(super) fn velocity_derivatives(
    mesh: &UnstructuredMesh,
    u: &[Vec3],
) -> (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>) {
    let mut neighbors = vec![Vec::new(); mesh.cell_count()];
    for face in mesh.faces() {
        if let Some(n) = face.neighbour {
            neighbors[face.owner].push(n);
            neighbors[n].push(face.owner);
        }
    }
    let mut curl = Vec::with_capacity(u.len());
    let mut divergence = Vec::with_capacity(u.len());
    let mut q = Vec::with_capacity(u.len());
    let mut flow_type = Vec::with_capacity(u.len());
    let mut strain_rate = Vec::with_capacity(u.len());
    for (i, adjacent) in neighbors.iter().enumerate() {
        let mut stencil = adjacent.clone();
        for &j in adjacent {
            stencil.extend(neighbors[j].iter().copied().filter(|k| *k != i));
        }
        stencil.sort_unstable();
        stencil.dedup();
        let mut a = [[0.0; 3]; 3];
        let mut rhs = [[0.0; 3]; 3];
        for j in stencil {
            let d = mesh.cells()[j].center - mesh.cells()[i].center;
            let w = 1.0 / d.norm_squared().max(1e-30);
            let d = [d.x, d.y, d.z];
            let du = u[j] - u[i];
            let du = [du.x, du.y, du.z];
            for r in 0..3 {
                for c in 0..3 {
                    a[r][c] += w * d[r] * d[c];
                    rhs[r][c] += w * d[r] * du[c];
                }
            }
        }
        if mesh.dimension() == MeshDimension::TwoD {
            a[2][2] = 1.0;
        }
        // Small Tikhonov term gives the minimum-norm estimate on deficient stencils.
        let regularization = (a[0][0] + a[1][1] + a[2][2]).max(1.0) * 1e-12;
        for (k, row) in a.iter_mut().enumerate() {
            row[k] += regularization;
        }
        for k in 0..3 {
            let pivot = (k..3)
                .max_by(|&l, &r| a[l][k].abs().total_cmp(&a[r][k].abs()))
                .unwrap();
            a.swap(k, pivot);
            rhs.swap(k, pivot);
            let d = a[k][k];
            for j in 0..3 {
                a[k][j] /= d;
                rhs[k][j] /= d;
            }
            for r in 0..3 {
                if r != k {
                    let factor = a[r][k];
                    for c in 0..3 {
                        a[r][c] -= factor * a[k][c];
                        rhs[r][c] -= factor * rhs[k][c];
                    }
                }
            }
        }
        let rotation = Vec3::new(
            rhs[1][2] - rhs[2][1],
            rhs[2][0] - rhs[0][2],
            rhs[0][1] - rhs[1][0],
        );
        curl.push(rotation.norm());
        let trace = rhs[0][0] + rhs[1][1] + rhs[2][2];
        divergence.push(trace);
        // Q = 0.5 * ((tr grad U)^2 - tr((grad U)^2)).
        let trace_square: f64 = (0..3)
            .flat_map(|r| (0..3).map(move |c| rhs[r][c] * rhs[c][r]))
            .sum();
        q.push(0.5 * (trace * trace - trace_square));
        let mut strain_squared = 0.0;
        let mut dev_norm_squared = 0.0;
        let mut dev_square_trace = 0.0;
        for r in 0..3 {
            for c in 0..3 {
                let sym = 0.5 * (rhs[r][c] + rhs[c][r]);
                strain_squared += sym * sym;
                let diagonal = if r == c { trace / 3.0 } else { 0.0 };
                let a = rhs[r][c] - diagonal;
                let b = rhs[c][r] - diagonal;
                dev_norm_squared += a * a;
                dev_square_trace += a * b;
            }
        }
        strain_rate.push((2.0 * strain_squared).sqrt());
        // Flow type: rotation +1, strain -1, shear 0.
        flow_type.push(if dev_norm_squared > 1e-150 {
            (-dev_square_trace / dev_norm_squared).clamp(-1.0, 1.0)
        } else {
            0.0
        });
    }
    (curl, divergence, q, flow_type, strain_rate)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn linear_rotation_and_expansion_reconstruct_curl_and_divergence() {
        let points = (0..4)
            .flat_map(|y| (0..4).map(move |x| crate::Point::new(x as f64, y as f64, 0.0)))
            .collect();
        let cells = (0..3)
            .flat_map(|y| {
                (0..3).map(move |x| {
                    let i = y * 4 + x;
                    crate::CellDefinition::polygon(vec![i, i + 1, i + 5, i + 4])
                })
            })
            .collect();
        let mesh = UnstructuredMesh::from_cells(MeshDimension::TwoD, points, cells).unwrap();
        let velocity = mesh
            .cells()
            .iter()
            .map(|c| {
                Vec3::new(
                    c.center.x - 2.0 * c.center.y,
                    2.0 * c.center.x + c.center.y,
                    0.0,
                )
            })
            .collect::<Vec<_>>();
        let (curl, div, _, _, _) = velocity_derivatives(&mesh, &velocity);
        for value in curl {
            assert!((value - 4.0).abs() < 1e-9);
        }
        for value in div {
            assert!((value - 2.0).abs() < 1e-9);
        }
    }
}
