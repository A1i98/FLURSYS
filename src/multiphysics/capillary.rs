use crate::{CellField, FaceField, ResolvedScalarBoundaryConditions, UnstructuredMesh, Vec3};

pub(super) fn surface_force(
    mesh: &UnstructuredMesh,
    alpha: &[f64],
    sigma: f64,
    contact_angle_degrees: Option<f64>,
    boundary: Option<&ResolvedScalarBoundaryConditions>,
) -> Result<CellField<Vec3>, String> {
    if alpha.len() != mesh.cell_count()
        || alpha
            .iter()
            .any(|a| !a.is_finite() || !(0.0..=1.0).contains(a))
        || !sigma.is_finite()
        || sigma < 0.0
        || contact_angle_degrees.is_some_and(|a| !a.is_finite() || !(0.0..=180.0).contains(&a))
    {
        return Err("Invalid surface tension, contact angle, or phase fraction".into());
    }
    let mut force = CellField::filled(mesh, Vec3::ZERO);
    if sigma == 0.0 {
        return Ok(force);
    }
    let mut alpha_face = FaceField::filled(mesh, 0.0);
    for (i, face) in mesh.faces().iter().enumerate() {
        let neighbour = face
            .neighbour
            .map(|n| (n, mesh.cells()[n].center))
            .or_else(|| boundary.and_then(|bc| bc.cyclic_neighbour(mesh, i)));
        alpha_face[i] = if let Some((n, center)) = neighbour {
            let a = (face.center - mesh.cells()[face.owner].center).norm();
            let b = (face.center - center).norm();
            (b * alpha[face.owner] + a * alpha[n]) / (a + b)
        } else {
            alpha[face.owner]
        };
    }
    let grad = crate::gauss_gradient_from_faces(mesh, &alpha_face).map_err(|e| format!("{e:?}"))?;
    let mut curvature = vec![0.0; mesh.cell_count()];
    for (i, face) in mesh.faces().iter().enumerate() {
        let neighbour = face
            .neighbour
            .map(|n| (n, mesh.cells()[n].center))
            .or_else(|| boundary.and_then(|bc| bc.cyclic_neighbour(mesh, i)));
        let g = if let Some((n, center)) = neighbour {
            let a = (face.center - mesh.cells()[face.owner].center).norm();
            let b = (face.center - center).norm();
            (grad[face.owner] * b + grad[n] * a) / (a + b)
        } else {
            grad[face.owner]
        };
        let mut normal = g.normalized().unwrap_or(Vec3::ZERO);
        if neighbour.is_none() && normal != Vec3::ZERO {
            if let Some(theta) = contact_angle_degrees {
                let wall_normal = face.normal;
                let tangent = normal - wall_normal * normal.dot(wall_normal);
                let angle = theta.to_radians();
                if let Some(tangent) = tangent.normalized() {
                    normal = tangent * angle.sin() + wall_normal * angle.cos();
                } else if angle.sin().abs() < 1e-12 {
                    normal = wall_normal * angle.cos();
                } else if alpha[face.owner] > 1e-12 && alpha[face.owner] < 1.0 - 1e-12 {
                    return Err("Contact angle is undefined where the interface normal is parallel to the wall normal".into());
                }
            }
        }
        let flux = normal.dot(face.area_vector);
        curvature[face.owner] -= flux / mesh.cells()[face.owner].volume;
        if let Some(n) = face.neighbour {
            curvature[n] += flux / mesh.cells()[n].volume;
        }
    }
    for i in 0..mesh.cell_count() {
        force[i] = grad[i] * (sigma * curvature[i]);
    }
    Ok(force)
}

pub(super) fn multiphase_surface_force(
    mesh: &UnstructuredMesh,
    fractions: &[Vec<f64>],
    interfaces: &[super::multiphase::Interface],
    boundary: Option<&ResolvedScalarBoundaryConditions>,
) -> Result<CellField<Vec3>, String> {
    let mut gradients = Vec::with_capacity(fractions.len());
    for alpha in fractions {
        if alpha.len() != mesh.cell_count() {
            return Err("Invalid multiphase field".into());
        }
        let mut values = FaceField::filled(mesh, 0.0);
        for (i, face) in mesh.faces().iter().enumerate() {
            let neighbour = face
                .neighbour
                .map(|n| (n, mesh.cells()[n].center))
                .or_else(|| boundary.and_then(|b| b.cyclic_neighbour(mesh, i)));
            values[i] = if let Some((n, center)) = neighbour {
                let a = (face.center - mesh.cells()[face.owner].center).norm();
                let b = (face.center - center).norm();
                (alpha[face.owner] * b + alpha[n] * a) / (a + b)
            } else {
                alpha[face.owner]
            };
        }
        gradients
            .push(crate::gauss_gradient_from_faces(mesh, &values).map_err(|e| format!("{e:?}"))?);
    }
    let mut total = CellField::filled(mesh, Vec3::ZERO);
    for interface in interfaces {
        if interface.surface_tension == 0.0 {
            continue;
        }
        let first = interface.first;
        let second = interface.second;
        if first >= fractions.len() || second >= fractions.len() || first == second {
            return Err("Invalid interface phases".into());
        }
        let direction: Vec<_> = (0..mesh.cell_count())
            .map(|i| {
                gradients[first][i] * fractions[second][i]
                    - gradients[second][i] * fractions[first][i]
            })
            .collect();
        let mut angles = vec![None; mesh.face_count()];
        for patch in mesh.boundary_patches() {
            if let Some(&angle) = interface.contact_angles.get(&patch.name) {
                for &face in &patch.face_indices {
                    angles[face] = Some(angle);
                }
            }
        }
        let mut curvature = vec![0.0; mesh.cell_count()];
        for (i, face) in mesh.faces().iter().enumerate() {
            let neighbour = face
                .neighbour
                .map(|n| (n, mesh.cells()[n].center))
                .or_else(|| boundary.and_then(|b| b.cyclic_neighbour(mesh, i)));
            let g = if let Some((n, center)) = neighbour {
                let a = (face.center - mesh.cells()[face.owner].center).norm();
                let b = (face.center - center).norm();
                (direction[face.owner] * b + direction[n] * a) / (a + b)
            } else {
                direction[face.owner]
            };
            let mut normal = g.normalized().unwrap_or(Vec3::ZERO);
            if neighbour.is_none() && normal != Vec3::ZERO {
                if let Some(theta) = angles[i] {
                    let tangent = normal - face.normal * normal.dot(face.normal);
                    let angle = theta.to_radians();
                    if let Some(tangent) = tangent.normalized() {
                        normal = tangent * angle.sin() + face.normal * angle.cos();
                    } else if angle.sin().abs() < 1e-12 {
                        normal = face.normal * angle.cos();
                    } else if fractions[first][face.owner] > 1e-12
                        && fractions[second][face.owner] > 1e-12
                    {
                        return Err(format!("Contact angle is undefined at face {i}, center {:?}, phase fractions {:.6e}/{:.6e}, gradient {:?}", face.center, fractions[first][face.owner], fractions[second][face.owner], g));
                    }
                }
            }
            let flux = normal.dot(face.area_vector);
            curvature[face.owner] -= flux / mesh.cells()[face.owner].volume;
            if let Some(n) = face.neighbour {
                curvature[n] += flux / mesh.cells()[n].volume;
            }
        }
        for i in 0..mesh.cell_count() {
            total[i] += direction[i] * (interface.surface_tension * curvature[i]);
        }
    }
    Ok(total)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{CellDefinition, MeshDimension, Point};

    fn grid(n: usize) -> UnstructuredMesh {
        let index = |x: usize, y: usize| x + (n + 1) * y;
        let points = (0..=n)
            .flat_map(|y| {
                (0..=n).map(move |x| Point::new(x as f64 / n as f64, y as f64 / n as f64, 0.0))
            })
            .collect();
        let cells = (0..n)
            .flat_map(|y| {
                (0..n).map(move |x| {
                    CellDefinition::polygon(vec![
                        index(x, y),
                        index(x + 1, y),
                        index(x + 1, y + 1),
                        index(x, y + 1),
                    ])
                })
            })
            .collect();
        UnstructuredMesh::from_cells(MeshDimension::TwoD, points, cells).unwrap()
    }

    #[test]
    fn pairwise_force_matches_two_phase_limit_and_vanishes_when_pair_absent() {
        let mesh = grid(24);
        let first: Vec<_> = mesh
            .cells()
            .iter()
            .map(|cell| {
                let r = (cell.center - Vec3::new(0.5, 0.5, 0.0)).norm();
                (0.5 * (1.0 - ((r - 0.25) / 0.06).tanh())).clamp(0.0, 1.0)
            })
            .collect();
        let second: Vec<_> = first.iter().map(|a| 1.0 - a).collect();
        let absent = vec![0.0; mesh.cell_count()];
        let interface = super::super::multiphase::Interface {
            first: 0,
            second: 1,
            surface_tension: 0.072,
            contact_angles: Default::default(),
        };
        let binary = surface_force(&mesh, &first, 0.072, None, None).unwrap();
        let pair = multiphase_surface_force(
            &mesh,
            &[first.clone(), second, absent.clone()],
            &[interface.clone()],
            None,
        )
        .unwrap();
        assert!(binary
            .values()
            .iter()
            .zip(pair.values())
            .all(|(a, b)| (*a - *b).norm() < 1e-10));
        let absent_force =
            multiphase_surface_force(&mesh, &[first, absent.clone(), absent], &[interface], None)
                .unwrap();
        assert!(absent_force.values().iter().all(|f| f.norm() < 1e-12));
    }

    #[test]
    fn contact_angle_ignores_degenerate_normal_in_pure_wall_cells() {
        use crate::{BoundaryPatch, BoundaryType};
        let mesh = grid(4);
        let wall = mesh
            .faces()
            .iter()
            .enumerate()
            .filter_map(|(i, f)| (f.neighbour.is_none() && f.center.x.abs() < 1e-12).then_some(i))
            .collect();
        let mesh = mesh
            .with_boundary_patches(vec![BoundaryPatch {
                name: "wall".into(),
                face_indices: wall,
                boundary_type: BoundaryType::Wall,
            }])
            .unwrap();
        let first: Vec<_> = mesh
            .cells()
            .iter()
            .map(|c| if c.center.x < 0.25 { 1.0 } else { 0.0 })
            .collect();
        let second: Vec<_> = first.iter().map(|a| 1.0 - a).collect();
        let interface = super::super::multiphase::Interface {
            first: 0,
            second: 1,
            surface_tension: 0.1,
            contact_angles: [("wall".into(), 30.0)].into_iter().collect(),
        };
        assert!(multiphase_surface_force(&mesh, &[first, second], &[interface], None).is_ok());
    }

    #[test]
    fn contact_angle_can_be_assigned_to_one_wall_of_a_phase_pair() {
        use crate::{BoundaryPatch, BoundaryType};
        let mesh = grid(12);
        let wall: Vec<_> = mesh
            .faces()
            .iter()
            .enumerate()
            .filter_map(|(i, f)| (f.neighbour.is_none() && f.center.y.abs() < 1e-12).then_some(i))
            .collect();
        let mesh = mesh
            .with_boundary_patches(vec![BoundaryPatch {
                name: "floor".into(),
                face_indices: wall,
                boundary_type: BoundaryType::Wall,
            }])
            .unwrap();
        let first: Vec<_> = mesh
            .cells()
            .iter()
            .map(|c| (0.5 + (c.center.x + c.center.y - 0.6) * 2.0).clamp(0.0, 1.0))
            .collect();
        let second: Vec<_> = first.iter().map(|a| 1.0 - a).collect();
        let mut interface = super::super::multiphase::Interface {
            first: 0,
            second: 1,
            surface_tension: 0.072,
            contact_angles: Default::default(),
        };
        let original = multiphase_surface_force(
            &mesh,
            &[first.clone(), second.clone()],
            &[interface.clone()],
            None,
        )
        .unwrap();
        interface.contact_angles.insert("floor".into(), 45.0);
        let wetting =
            multiphase_surface_force(&mesh, &[first, second], &[interface], None).unwrap();
        assert!(original
            .values()
            .iter()
            .zip(wetting.values())
            .any(|(a, b)| (*a - *b).norm() > 1e-5));
    }

    #[test]
    fn uniform_phase_has_no_capillary_force_and_invalid_parameters_fail() {
        let mesh = grid(4);
        let alpha = vec![0.5; mesh.cell_count()];
        let force = surface_force(&mesh, &alpha, 0.072, Some(45.0), None).unwrap();
        assert!(force.values().iter().all(|f| f.norm() < 1e-12));
        assert!(surface_force(&mesh, &alpha, -1.0, None, None).is_err());
        assert!(surface_force(&mesh, &alpha, 1.0, Some(181.0), None).is_err());
    }

    #[test]
    fn wall_angle_changes_the_interfacial_force() {
        let mesh = grid(12);
        let alpha: Vec<_> = mesh
            .cells()
            .iter()
            .map(|cell| (0.5 + (cell.center.x + cell.center.y - 0.6) * 2.0).clamp(0.0, 1.0))
            .collect();
        let first = surface_force(&mesh, &alpha, 0.072, Some(45.0), None).unwrap();
        let second = surface_force(&mesh, &alpha, 0.072, Some(135.0), None).unwrap();
        assert!(first
            .values()
            .iter()
            .zip(second.values())
            .any(|(a, b)| (*a - *b).norm() > 1e-5));
    }

    #[test]
    fn curved_interface_force_is_inward_and_scales_with_surface_tension() {
        let mesh = grid(40);
        let alpha: Vec<_> = mesh
            .cells()
            .iter()
            .map(|cell| {
                let r = (cell.center - Vec3::new(0.5, 0.5, 0.0)).norm();
                (0.5 * (1.0 - ((r - 0.25) / 0.04).tanh())).clamp(0.0, 1.0)
            })
            .collect();
        let a = surface_force(&mesh, &alpha, 0.1, None, None).unwrap();
        let b = surface_force(&mesh, &alpha, 0.2, None, None).unwrap();
        let cell = mesh
            .cells()
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                let target = Vec3::new(0.75, 0.5, 0.0);
                (a.center - target)
                    .norm()
                    .total_cmp(&(b.center - target).norm())
            })
            .unwrap()
            .0;
        assert!(a[cell].x < 0.0, "expected inward force: {:?}", a[cell]);
        assert!((b[cell] - a[cell] * 2.0).norm() < 1e-10);
    }
}
