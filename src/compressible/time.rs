//! SSP-RK2 with acoustic/diffusive step restriction and rejected-step rollback.
use super::*;
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Report {
    pub time: f64,
    pub accepted_steps: usize,
    #[serde(default)]
    pub cyclic_face_pairs: usize,
    pub rejected_steps: usize,
    pub hllc_fallback_evaluations: usize,
    pub reached_end_time: bool,
    pub last_dt: f64,
    pub min_density: f64,
    pub min_pressure: f64,
    pub max_mach: f64,
    pub initial_integrals: [f64; 5],
    pub final_integrals: [f64; 5],
    pub boundary_outflow_integrals: [f64; 5],
    pub conservation_defect: [f64; 5],
}
#[derive(Clone, Debug)]
pub struct Solution {
    pub mesh_id: crate::MeshId,
    pub fields: Vec<Primitive>,
    pub report: Report,
}
fn integral(mesh: &UnstructuredMesh, q: &[Conservative]) -> Conservative {
    mesh.cells()
        .iter()
        .zip(q)
        .fold(Conservative([0.0; 5]), |sum, (c, q)| {
            sum.add(q.scale(c.volume))
        })
}
struct Evaluation {
    rhs: Vec<Conservative>,
    boundary: Conservative,
    dt: f64,
    fallbacks: usize,
}
fn evaluate(
    mesh: &UnstructuredMesh,
    q: &[Conservative],
    bc: &[Option<Boundary>],
    cyclic: &crate::periodic::CyclicMap,
    s: &Settings,
) -> Result<Evaluation, String> {
    let p = q
        .iter()
        .map(|q| q.primitive(s.gas))
        .collect::<Result<Vec<_>, _>>()?;
    for state in &p {
        let a = s.gas.sound_speed(*state);
        let t = s.gas.temperature(*state);
        if !a.is_finite() || a <= 0.0 || !t.is_finite() || t <= 0.0 {
            return Err("Unphysical sound speed or absolute temperature".into());
        }
    }
    let mut ghost = vec![None; mesh.face_count()];
    for (i, f) in mesh.faces().iter().enumerate() {
        if f.neighbour.is_none() {
            let g = if let Some(link) = cyclic.link(i) {
                p[mesh.faces()[link.peer_face].owner]
            } else {
                bc[i].ok_or("Missing exterior boundary condition")?.ghost(
                    p[f.owner],
                    f.area_vector / f.area,
                    s.gas,
                )?
            };
            g.validate()?;
            ghost[i] = Some(g);
        }
    }
    let gradients = if matches!(s.viscosity, Viscosity::Inviscid) {
        None
    } else {
        Some(viscous::gradients(mesh, &p, &ghost, bc, cyclic, s.gas))
    };
    let mut rhs = vec![Conservative([0.0; 5]); mesh.cell_count()];
    let mut rates = vec![0.0; mesh.cell_count()];
    let mut boundary = Conservative([0.0; 5]);
    let mut fallbacks = 0;
    for (i, f) in mesh.faces().iter().enumerate() {
        let link = cyclic.link(i);
        if link.is_some_and(|link| i > link.peer_face) {
            continue;
        }
        let periodic_neighbor = link.map(|link| {
            (
                mesh.faces()[link.peer_face].owner,
                mesh.cells()[mesh.faces()[link.peer_face].owner].center
                    - link.translation
                    - mesh.cells()[f.owner].center,
            )
        });
        let neighbour = f.neighbour.or_else(|| periodic_neighbor.map(|p| p.0));
        let o = f.owner;
        let l = p[o];
        let r = f
            .neighbour
            .map(|j| p[j])
            .unwrap_or_else(|| ghost[i].unwrap());
        let n = f.area_vector / f.area;
        let (mut value, fallback) = flux::evaluate(l, r, s.gas, n, s.flux);
        fallbacks += usize::from(fallback);
        let speed = (l.velocity.dot(n).abs() + s.gas.sound_speed(l))
            .max(r.velocity.dot(n).abs() + s.gas.sound_speed(r));
        let mut rate = f.area * speed;
        if let Some(g) = &gradients {
            value = value.sub(viscous::evaluate(
                mesh,
                i,
                &p,
                ghost[i],
                bc[i],
                periodic_neighbor,
                g,
                s,
            )?);
            let distance = if let Some((_, d)) = periodic_neighbor {
                d.dot(n)
            } else if let Some(j) = f.neighbour {
                (mesh.cells()[j].center - mesh.cells()[o].center).dot(n)
            } else {
                (f.center - mesh.cells()[o].center).dot(n)
            };
            if distance <= 0.0 {
                return Err("Nonpositive diffusion distance".into());
            }
            let (mu_l, k_l) = s.viscosity.coefficients(s.gas.temperature(l), s.gas)?;
            let (mu_r, k_r) = s.viscosity.coefficients(s.gas.temperature(r), s.gas)?;
            let rho = l.density.min(r.density);
            let diffusion =
                (4.0 / 3.0 * mu_l.max(mu_r) / rho).max(k_l.max(k_r) / (rho * s.gas.cv()));
            rate += 2.0 * f.area * diffusion / distance;
        }
        let integrated = value.scale(f.area);
        rhs[o] = rhs[o].sub(integrated.scale(1.0 / mesh.cells()[o].volume));
        rates[o] += rate;
        if let Some(j) = neighbour {
            rhs[j] = rhs[j].add(integrated.scale(1.0 / mesh.cells()[j].volume));
            rates[j] += rate;
        } else {
            boundary = boundary.add(integrated);
        }
    }
    let mut dt = s.max_time_step;
    for (c, rate) in mesh.cells().iter().zip(rates) {
        if !rate.is_finite() || rate <= 0.0 {
            return Err("Non-finite acoustic/diffusive spectral rate".into());
        }
        dt = dt.min(s.courant * c.volume / rate);
    }
    if rhs.iter().any(|q| q.0.iter().any(|v| !v.is_finite())) {
        return Err("Non-finite conservative residual".into());
    }
    Ok(Evaluation {
        rhs,
        boundary,
        dt,
        fallbacks,
    })
}
pub fn solve(mesh: &UnstructuredMesh, s: &Settings) -> Result<Solution, String> {
    let (bc, cyclic) = s.validate(mesh)?;
    let mut q = Vec::with_capacity(mesh.cell_count());
    for c in mesh.cells() {
        let mut p = s.initial;
        for r in &s.regions {
            if c.center.x >= r.minimum.x
                && c.center.x <= r.maximum.x
                && c.center.y >= r.minimum.y
                && c.center.y <= r.maximum.y
                && c.center.z >= r.minimum.z
                && c.center.z <= r.maximum.z
            {
                p = r.state;
            }
        }
        q.push(p.conservative(s.gas));
    }
    let initial = integral(mesh, &q);
    let mut time = 0.0;
    let mut steps = 0;
    let mut rejected = 0;
    let mut fallbacks = 0;
    let mut last_dt = 0.0;
    let mut boundary = Conservative([0.0; 5]);
    while time < s.end_time && steps < s.max_steps {
        let first = evaluate(mesh, &q, &bc, &cyclic, s)?;
        let remaining = s.end_time - time;
        let mut dt = first.dt.min(remaining);
        let mut accepted = None;
        for _ in 0..=20 {
            if !dt.is_finite()
                || dt <= 0.0
                || time + dt == time
                || dt < s.min_time_step.min(remaining)
            {
                return Err(format!(
                    "Compressible dt below allowed minimum at t={time:.8e}; state was not advanced"
                ));
            }
            let stage = q
                .iter()
                .zip(&first.rhs)
                .map(|(a, r)| a.add(r.scale(dt)))
                .collect::<Vec<_>>();
            // Reject unphysical candidates without clipping density or energy.
            if stage.iter().all(|v| v.primitive(s.gas).is_ok()) {
                let second = evaluate(mesh, &stage, &bc, &cyclic, s)?;
                if dt <= second.dt * (1.0 + 1e-10) {
                    let next = q
                        .iter()
                        .zip(stage.iter().zip(&second.rhs))
                        .map(|(a, (b, r))| a.scale(0.5).add(b.add(r.scale(dt)).scale(0.5)))
                        .collect::<Vec<_>>();
                    if next.iter().all(|v| v.primitive(s.gas).is_ok()) {
                        accepted = Some((
                            next,
                            first.boundary.add(second.boundary).scale(0.5 * dt),
                            first.fallbacks + second.fallbacks,
                        ));
                        break;
                    }
                }
            }
            rejected += 1;
            dt *= 0.5;
        }
        let (next, outflow, fallback) = accepted.ok_or_else(|| {
            format!("Compressible step rejected after 20 retries at t={time:.8e}")
        })?;
        q = next;
        boundary = boundary.add(outflow);
        fallbacks += fallback;
        time += dt;
        steps += 1;
        last_dt = dt;
    }
    let final_q = integral(mesh, &q);
    let defect = final_q.sub(initial).add(boundary);
    let fields = q
        .iter()
        .map(|q| q.primitive(s.gas))
        .collect::<Result<Vec<_>, _>>()?;
    let report = Report {
        time,
        accepted_steps: steps,
        cyclic_face_pairs: cyclic.pair_count(),
        rejected_steps: rejected,
        hllc_fallback_evaluations: fallbacks,
        reached_end_time: time >= s.end_time,
        last_dt,
        min_density: fields
            .iter()
            .map(|p| p.density)
            .fold(f64::INFINITY, f64::min),
        min_pressure: fields
            .iter()
            .map(|p| p.pressure)
            .fold(f64::INFINITY, f64::min),
        max_mach: fields
            .iter()
            .map(|p| p.velocity.norm() / s.gas.sound_speed(*p))
            .fold(0.0, f64::max),
        initial_integrals: initial.0,
        final_integrals: final_q.0,
        boundary_outflow_integrals: boundary.0,
        conservation_defect: defect.0,
    };
    if report
        .initial_integrals
        .iter()
        .chain(report.final_integrals.iter())
        .chain(report.conservation_defect.iter())
        .any(|v| !v.is_finite())
        || !report.max_mach.is_finite()
    {
        return Err("Non-finite conservation integral or Mach number".into());
    }
    Ok(Solution {
        mesh_id: mesh.id(),
        fields,
        report,
    })
}
