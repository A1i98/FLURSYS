//! Newtonian deviatoric stress and Fourier heat conduction, conservative face fluxes.
use super::*;
fn values(p: Primitive, gas: IdealGas) -> [f64; 4] {
    [p.velocity.x, p.velocity.y, p.velocity.z, gas.temperature(p)]
}
fn boundary_face(
    owner: Primitive,
    other: Primitive,
    bc: Boundary,
    n: Vec3,
    gas: IdealGas,
) -> [f64; 4] {
    match bc {
        Boundary::NoSlipWall { temperature } => {
            [0.0, 0.0, 0.0, temperature.unwrap_or(gas.temperature(owner))]
        }
        Boundary::SlipWall => {
            let v = owner.velocity - n * owner.velocity.dot(n);
            [v.x, v.y, v.z, gas.temperature(owner)]
        }
        Boundary::Transmissive | Boundary::PressureOutlet { .. } => values(owner, gas),
        _ => {
            let a = values(owner, gas);
            let b = values(other, gas);
            std::array::from_fn(|i| 0.5 * (a[i] + b[i]))
        }
    }
}
pub(super) fn gradients(
    mesh: &UnstructuredMesh,
    p: &[Primitive],
    ghost: &[Option<Primitive>],
    bc: &[Option<Boundary>],
    cyclic: &crate::periodic::CyclicMap,
    gas: IdealGas,
) -> Vec<[Vec3; 4]> {
    let mut g = vec![[Vec3::ZERO; 4]; mesh.cell_count()];
    for (i, f) in mesh.faces().iter().enumerate() {
        let link = cyclic.link(i);
        if link.is_some_and(|link| i > link.peer_face) {
            continue;
        }
        let neighbour = f
            .neighbour
            .or_else(|| link.map(|link| mesh.faces()[link.peer_face].owner));
        let n = f.area_vector / f.area;
        let a = values(p[f.owner], gas);
        let value = if let Some(j) = neighbour {
            let b = values(p[j], gas);
            std::array::from_fn(|k| 0.5 * (a[k] + b[k]))
        } else {
            boundary_face(p[f.owner], ghost[i].unwrap(), bc[i].unwrap(), n, gas)
        };
        for k in 0..4 {
            g[f.owner][k] += f.area_vector * (value[k] / mesh.cells()[f.owner].volume);
            if let Some(j) = neighbour {
                g[j][k] = g[j][k] - f.area_vector * (value[k] / mesh.cells()[j].volume);
            }
        }
    }
    g
}
pub(super) fn evaluate(
    mesh: &UnstructuredMesh,
    index: usize,
    p: &[Primitive],
    ghost: Option<Primitive>,
    bc: Option<Boundary>,
    periodic_neighbor: Option<(usize, Vec3)>,
    g: &[[Vec3; 4]],
    settings: &Settings,
) -> Result<Conservative, String> {
    let f = &mesh.faces()[index];
    let n = f.area_vector / f.area;
    let o = f.owner;
    let a = values(p[o], settings.gas);
    let neighbour = f.neighbour.or_else(|| periodic_neighbor.map(|p| p.0));
    let (d, b, mut gradient, face) = if let Some(j) = neighbour {
        let b = values(p[j], settings.gas);
        (
            periodic_neighbor
                .map(|(_, d)| d)
                .unwrap_or(mesh.cells()[j].center - mesh.cells()[o].center),
            b,
            std::array::from_fn(|k| (g[o][k] + g[j][k]) * 0.5),
            std::array::from_fn(|k| 0.5 * (a[k] + b[k])),
        )
    } else {
        let face = boundary_face(p[o], ghost.unwrap(), bc.unwrap(), n, settings.gas);
        (f.center - mesh.cells()[o].center, face, g[o], face)
    };
    let distance = d.dot(n);
    if !distance.is_finite() || distance <= 0.0 {
        return Err("Nonpositive viscous face-normal distance".into());
    }
    for k in 0..4 {
        gradient[k] += n * ((b[k] - a[k] - gradient[k].dot(d)) / distance);
    }
    let (mu, conductivity) = settings.viscosity.coefficients(face[3], settings.gas)?;
    let diagonal = gradient[0].x + gradient[1].y + gradient[2].z;
    let normal = [n.x, n.y, n.z];
    let component = |v: Vec3, i: usize| [v.x, v.y, v.z][i];
    let mut t = [0.0; 3];
    for a in 0..3 {
        for b in 0..3 {
            t[a] += mu
                * (component(gradient[a], b) + component(gradient[b], a)
                    - if a == b { 2.0 / 3.0 * diagonal } else { 0.0 })
                * normal[b];
        }
    }
    let mut traction = Vec3::new(t[0], t[1], t[2]);
    if matches!(bc, Some(Boundary::SlipWall)) {
        traction = n * traction.dot(n);
    }
    let heat = if matches!(
        bc,
        Some(
            Boundary::SlipWall
                | Boundary::NoSlipWall { temperature: None }
                | Boundary::Transmissive
                | Boundary::PressureOutlet { .. }
        )
    ) {
        0.0
    } else {
        conductivity * gradient[3].dot(n)
    };
    let u = Vec3::new(face[0], face[1], face[2]);
    let energy = traction.dot(u) + heat;
    let q = Conservative([0.0, traction.x, traction.y, traction.z, energy]);
    if q.0.iter().any(|v| !v.is_finite()) {
        return Err("Non-finite viscous or heat flux".into());
    }
    Ok(q)
}
