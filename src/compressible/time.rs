//! SSP-RK2 with acoustic/diffusive step restriction and rejected-step rollback.
use super::*;
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Report {
    #[serde(default)]
    pub coordinates: crate::multiphysics::metrics::Coordinates,
    #[serde(default)]
    pub source_integrals: [f64; 5],
    #[serde(default)]
    pub performance: crate::compute::PerformanceReport,
    #[serde(default)]
    pub stopped: bool,
    #[serde(default)]
    pub residuals: [f64; 4],
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
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Snapshot {
    pub gas: IdealGas,
    pub fields: Vec<Primitive>,
    pub report: Report,
}
impl Snapshot {
    pub fn validate(&self, mesh: &UnstructuredMesh, gas: IdealGas) -> Result<(), String> {
        if self.gas != gas
            || self.fields.len() != mesh.cell_count()
            || !self.report.time.is_finite()
            || self.report.time < 0.
        {
            return Err(
                "Restart gas, field count or physical time does not match this case".into(),
            );
        }
        for p in &self.fields {
            p.validate()?;
        }
        if self
            .report
            .initial_integrals
            .iter()
            .chain(self.report.boundary_outflow_integrals.iter())
            .chain(self.report.source_integrals.iter())
            .chain(self.report.residuals.iter())
            .any(|v| !v.is_finite())
        {
            return Err("Invalid compressible restart conservation state".into());
        }
        Ok(())
    }
    pub fn dataset(
        &self,
        name: impl Into<String>,
        mesh: &UnstructuredMesh,
    ) -> Result<crate::workbench::ResultDataset, String> {
        self.validate(mesh, self.gas)?;
        crate::workbench::ResultDataset::from_compressible(name, mesh, &self.fields, self.gas)
            .map_err(|e| e.to_string())
    }
}
impl Solution {
    pub fn snapshot(&self, gas: IdealGas) -> Snapshot {
        Snapshot {
            gas,
            fields: self.fields.clone(),
            report: self.report.clone(),
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepAction {
    Continue,
    Stop,
}

fn integral(mesh: &UnstructuredMesh, q: &[Conservative], s: &Settings) -> Conservative {
    let measures = s.coordinates.measures(mesh).expect("validated coordinates");
    mesh.cells()
        .iter()
        .zip(q)
        .zip(measures.volumes)
        .fold(Conservative([0.0; 5]), |sum, ((_, q), volume)| {
            sum.add(q.scale(volume))
        })
}
struct Evaluation {
    rhs: Vec<Conservative>,
    boundary: Conservative,
    source: Conservative,
    dt: f64,
    fallbacks: usize,
}
fn evaluate(
    mesh: &UnstructuredMesh,
    q: &[Conservative],
    bc: &[Option<Boundary>],
    cyclic: &crate::periodic::CyclicMap,
    s: &Settings,
    programs: &boundaries::Programs,
    time: f64,
) -> Result<Evaluation, String> {
    let measures = s.coordinates.measures(mesh)?;
    let p = q
        .iter()
        .map(|q| q.primitive(s.gas))
        .collect::<Result<Vec<_>, _>>()?;
    for state in &p {
        if s.coordinates != crate::multiphysics::metrics::Coordinates::Cartesian
            && state.velocity.z.abs() > 1e-12
        {
            return Err("Axisymmetric compressible flow requires zero swirl velocity".into());
        }
        let a = s.gas.sound_speed(*state);
        let t = s.gas.temperature(*state);
        if !a.is_finite() || a <= 0.0 || !t.is_finite() || t <= 0.0 {
            return Err("Unphysical sound speed or absolute temperature".into());
        }
    }
    let mut evaluated_bc = bc.to_vec();
    for (i, program) in programs.iter().enumerate() {
        if let Some(program) = program {
            evaluated_bc[i] = Some(
                bc[i]
                    .ok_or("Scripted face has no physical condition")?
                    .scripted(program, mesh.faces()[i].center, time, s.gas)?,
            );
        }
    }
    let bc = evaluated_bc.as_slice();
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
            if s.coordinates != crate::multiphysics::metrics::Coordinates::Cartesian
                && g.velocity.z.abs() > 1e-12
            {
                return Err("Axisymmetric boundaries must have zero swirl velocity".into());
            }
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
    let convective = if let Some(runtime) = crate::compute::current() {
        runtime.conservative_fluxes(
            mesh.face_count(),
            mesh.faces().iter().enumerate().map(|(i, f)| flux::Input {
                left: p[f.owner],
                right: f
                    .neighbour
                    .map(|j| p[j])
                    .unwrap_or_else(|| ghost[i].unwrap()),
                normal: f.area_vector / f.area,
            }),
            s.gas,
            s.flux,
        )?
    } else {
        None
    };
    let mut contributions = vec![None; mesh.face_count()];
    crate::compute::try_for_each(&mut contributions, |i, output| {
        let f = &mesh.faces()[i];
        let link = cyclic.link(i);
        if link.is_some_and(|link| i > link.peer_face) {
            return Ok(());
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
        let (mut value, fallback) = convective
            .as_ref()
            .map(|v| v[i])
            .unwrap_or_else(|| flux::evaluate(l, r, s.gas, n, s.flux));

        let speed = (l.velocity.dot(n).abs() + s.gas.sound_speed(l))
            .max(r.velocity.dot(n).abs() + s.gas.sound_speed(r));
        let mut rate = measures.areas[i] * speed;
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
        let integrated = value.scale(measures.areas[i]);
        *output = Some((o, neighbour, integrated, rate, fallback));
        Ok::<_, String>(())
    })?;
    for (o, neighbour, integrated, rate, fallback) in contributions.into_iter().flatten() {
        fallbacks += usize::from(fallback);
        rhs[o] = rhs[o].sub(integrated.scale(1.0 / measures.volumes[o]));
        rates[o] += rate;
        if let Some(j) = neighbour {
            rhs[j] = rhs[j].add(integrated.scale(1.0 / measures.volumes[j]));
            rates[j] += rate;
        } else {
            boundary = boundary.add(integrated);
        }
    }
    let mut source = Conservative([0.; 5]);
    for (i, cell) in mesh.cells().iter().enumerate() {
        let state = p[i];
        let (mu, _) = s.viscosity.coefficients(s.gas.temperature(state), s.gas)?;
        let (sink, model_force) =
            s.volume_models
                .cell_coefficients(cell.center, state.velocity, mu / state.density)?;
        let acceleration = s.acceleration + model_force - state.velocity * sink;
        let force = acceleration * state.density;
        if s.coordinates != crate::multiphysics::metrics::Coordinates::Cartesian
            && force.z.abs() > 1e-12
        {
            return Err("Axisymmetric flow does not support azimuthal body forces".into());
        }
        let local = Conservative([
            0.,
            force.x,
            force.y,
            force.z,
            force.dot(state.velocity) + s.heat_source,
        ]);
        let mut geometric = Conservative([0.; 5]);
        let pressure_force = std::f64::consts::TAU * state.pressure * cell.volume;
        match s.coordinates {
            crate::multiphysics::metrics::Coordinates::Cartesian => {}
            crate::multiphysics::metrics::Coordinates::AxisymmetricX { .. } => {
                geometric.0[2] = pressure_force
            }
            crate::multiphysics::metrics::Coordinates::AxisymmetricY { .. } => {
                geometric.0[1] = pressure_force
            }
        }
        rhs[i] = rhs[i]
            .add(local)
            .add(geometric.scale(1.0 / measures.volumes[i]));
        source = source.add(local.scale(measures.volumes[i])).add(geometric);
        rates[i] += measures.volumes[i]
            * (sink
                + 2. * s
                    .volume_models
                    .rotating_frame
                    .map_or(0., |r| r.angular_velocity.norm()));
    }
    let mut dt = s.max_time_step;
    for (volume, rate) in measures.volumes.iter().zip(rates) {
        if !rate.is_finite() || rate <= 0.0 {
            return Err("Non-finite acoustic/diffusive spectral rate".into());
        }
        dt = dt.min(s.courant * volume / rate);
    }
    if rhs.iter().any(|q| q.0.iter().any(|v| !v.is_finite())) {
        return Err("Non-finite conservative residual".into());
    }
    Ok(Evaluation {
        rhs,
        boundary,
        source,
        dt,
        fallbacks,
    })
}
pub fn solve(mesh: &UnstructuredMesh, s: &Settings) -> Result<Solution, String> {
    solve_controlled(mesh, s, |_, _| Ok(()))
}
pub fn solve_controlled(
    mesh: &UnstructuredMesh,
    s: &Settings,
    mut control: impl FnMut(usize, f64) -> Result<(), String> + Send,
) -> Result<Solution, String> {
    solve_resuming(mesh, s, None, |snapshot| {
        control(snapshot.report.accepted_steps, snapshot.report.time)?;
        Ok(StepAction::Continue)
    })
}
fn initial_fields(mesh: &UnstructuredMesh, s: &Settings) -> Vec<Conservative> {
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
    q
}
pub fn initialize(mesh: &UnstructuredMesh, s: &Settings) -> Result<Snapshot, String> {
    let (_, cyclic) = s.validate(mesh)?;
    let q = initial_fields(mesh, s);
    let initial = integral(mesh, &q, s);
    Ok(solution(
        mesh,
        s,
        &q,
        0.,
        0,
        0,
        0,
        0.,
        initial,
        Conservative([0.; 5]),
        Conservative([0.; 5]),
        [0.; 4],
        false,
        cyclic.pair_count(),
    )?
    .snapshot(s.gas))
}
pub fn solve_resuming(
    mesh: &UnstructuredMesh,
    s: &Settings,
    restart: Option<&Snapshot>,
    mut observe: impl FnMut(std::sync::Arc<Snapshot>) -> Result<StepAction, String> + Send,
) -> Result<Solution, String> {
    let execution = crate::compute::Execution::controlled_kernel(s.compute.clone(), None, true)?;
    let threads = crate::compute::worker_count_for(&s.compute, s.threads, mesh.cell_count());
    let pool = crate::compute::worker_pool(execution.clone(), threads)?;
    let offset = restart.map_or(0, |state| state.report.accepted_steps);
    let mut result = pool.install(|| {
        let _scope = crate::compute::Scope::enter(execution.clone());
        solve_time_loop(mesh, s, restart, &mut observe)
    });
    if let Ok(solution) = &mut result {
        solution.report.performance = execution.report(
            threads,
            mesh.cell_count(),
            solution.report.accepted_steps.saturating_sub(offset),
        );
    }
    result
}
fn solve_time_loop(
    mesh: &UnstructuredMesh,
    s: &Settings,
    restart: Option<&Snapshot>,
    mut observe: impl FnMut(std::sync::Arc<Snapshot>) -> Result<StepAction, String>,
) -> Result<Solution, String> {
    let (bc, cyclic) = s.validate(mesh)?;
    let programs = boundaries::compile(mesh, s)?;
    let mut q = initial_fields(mesh, s);
    if let Some(state) = restart {
        state.validate(mesh, s.gas)?;
        if state.report.coordinates != s.coordinates {
            return Err("Restart coordinate system does not match the current flow model".into());
        }
        q = state.fields.iter().map(|p| p.conservative(s.gas)).collect();
    }
    let initial = restart
        .map(|p| Conservative(p.report.initial_integrals))
        .unwrap_or_else(|| integral(mesh, &q, s));
    let mut time = restart.map_or(0., |p| p.report.time);
    let mut steps = restart.map_or(0, |p| p.report.accepted_steps);
    let target = steps.saturating_add(s.max_steps);
    let mut rejected = restart.map_or(0, |p| p.report.rejected_steps);
    let mut fallbacks = restart.map_or(0, |p| p.report.hllc_fallback_evaluations);
    let mut last_dt = restart.map_or(0., |p| p.report.last_dt);
    let mut boundary = restart
        .map(|p| Conservative(p.report.boundary_outflow_integrals))
        .unwrap_or(Conservative([0.; 5]));
    let mut sources = restart.map_or(Conservative([0.; 5]), |p| {
        Conservative(p.report.source_integrals)
    });
    let mut residuals = restart.map_or([0.; 4], |p| p.report.residuals);
    let mut stopped = false;
    let initial_solution = solution(
        mesh,
        s,
        &q,
        time,
        steps,
        rejected,
        fallbacks,
        last_dt,
        initial,
        boundary,
        sources,
        residuals,
        false,
        cyclic.pair_count(),
    )?;
    if observe(std::sync::Arc::new(initial_solution.snapshot(s.gas)))? == StepAction::Stop {
        return Ok(Solution {
            report: Report {
                stopped: true,
                ..initial_solution.report
            },
            ..initial_solution
        });
    }
    while time < s.end_time && steps < target {
        let first = evaluate(mesh, &q, &bc, &cyclic, s, &programs, time)?;
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
                let second = evaluate(mesh, &stage, &bc, &cyclic, s, &programs, time + dt)?;
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
                            first.source.add(second.source).scale(0.5 * dt),
                        ));
                        break;
                    }
                }
            }
            rejected += 1;
            dt *= 0.5;
        }
        let (next, outflow, fallback, injection) = accepted.ok_or_else(|| {
            format!("Compressible step rejected after 20 retries at t={time:.8e}")
        })?;
        let mut sums = [0.; 4];
        for (old, new) in q.iter().zip(&next) {
            let delta = new.sub(*old).scale(1. / dt);
            sums[0] += delta.0[0].powi(2);
            sums[1] += delta.0[1..4].iter().map(|v| v * v).sum::<f64>() / 3.;
            sums[2] += delta.0[4].powi(2);
            sums[3] += old
                .0
                .iter()
                .zip(new.0)
                .map(|(a, b)| ((b - a) / a.abs().max(1.)).powi(2))
                .sum::<f64>()
                / 5.;
        }
        residuals = sums.map(|v| (v / mesh.cell_count() as f64).sqrt());
        q = next;
        boundary = boundary.add(outflow);
        sources = sources.add(injection);
        fallbacks += fallback;
        time += dt;
        steps += 1;
        last_dt = dt;
        let state = solution(
            mesh,
            s,
            &q,
            time,
            steps,
            rejected,
            fallbacks,
            last_dt,
            initial,
            boundary,
            sources,
            residuals,
            false,
            cyclic.pair_count(),
        )?;
        if observe(std::sync::Arc::new(Snapshot {
            gas: s.gas,
            fields: state.fields,
            report: state.report,
        }))? == StepAction::Stop
        {
            stopped = true;
            break;
        }
    }
    solution(
        mesh,
        s,
        &q,
        time,
        steps,
        rejected,
        fallbacks,
        last_dt,
        initial,
        boundary,
        sources,
        residuals,
        stopped,
        cyclic.pair_count(),
    )
}
fn solution(
    mesh: &UnstructuredMesh,
    s: &Settings,
    q: &[Conservative],
    time: f64,
    steps: usize,
    rejected: usize,
    fallbacks: usize,
    last_dt: f64,
    initial: Conservative,
    boundary: Conservative,
    sources: Conservative,
    residuals: [f64; 4],
    stopped: bool,
    cyclic_face_pairs: usize,
) -> Result<Solution, String> {
    let final_q = integral(mesh, &q, s);
    let defect = final_q.sub(initial).add(boundary).sub(sources);
    let fields = q
        .iter()
        .map(|q| q.primitive(s.gas))
        .collect::<Result<Vec<_>, _>>()?;
    let report = Report {
        coordinates: s.coordinates,
        source_integrals: sources.0,
        performance: Default::default(),
        stopped,
        residuals,
        time,
        accepted_steps: steps,
        cyclic_face_pairs,
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

#[cfg(test)]
mod axisymmetric_tests {
    use super::*;
    use crate::multiphysics::metrics::Coordinates;
    use crate::{BoundaryPatch, BoundaryType, CellDefinition, MeshDimension, Point};

    fn mesh() -> UnstructuredMesh {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0., 0., 0.),
                Point::new(1., 0., 0.),
                Point::new(2., 0., 0.),
                Point::new(0., 1., 0.),
                Point::new(1., 1., 0.),
                Point::new(2., 1., 0.),
            ],
            vec![
                CellDefinition::polygon(vec![0, 1, 4, 3]),
                CellDefinition::polygon(vec![1, 2, 5, 4]),
            ],
        )
        .unwrap();
        let faces = mesh
            .faces()
            .iter()
            .enumerate()
            .filter_map(|(i, f)| f.neighbour.is_none().then_some(i))
            .collect();
        mesh.with_boundary_patches(vec![BoundaryPatch {
            name: "wall".into(),
            face_indices: faces,
            boundary_type: BoundaryType::Wall,
        }])
        .unwrap()
    }

    #[test]
    fn uniform_static_pressure_is_balanced_on_both_axes() {
        let mesh = mesh();
        for coordinates in [
            Coordinates::AxisymmetricX { radial_origin: 0. },
            Coordinates::AxisymmetricY { radial_origin: 0. },
        ] {
            let mut s = Settings::default();
            s.coordinates = coordinates;
            s.initial.pressure = 1.;
            s.boundaries.insert("wall".into(), Boundary::SlipWall);
            let bc = mesh
                .faces()
                .iter()
                .map(|f| f.neighbour.is_none().then_some(Boundary::SlipWall))
                .collect::<Vec<_>>();
            let cyclic = crate::periodic::resolve(&mesh, &[]).unwrap();
            let programs = boundaries::compile(&mesh, &s).unwrap();
            let q = initial_fields(&mesh, &s);
            let evaluation = evaluate(&mesh, &q, &bc, &cyclic, &s, &programs, 0.).unwrap();
            for residual in evaluation.rhs {
                assert!(
                    residual.0.iter().all(|v| v.abs() < 1e-10),
                    "{coordinates:?}: {residual:?}"
                );
            }
            let mass = integral(&mesh, &q, &s).0[0];
            let expected_volume = match coordinates {
                Coordinates::AxisymmetricX { .. } => 2. * std::f64::consts::PI,
                _ => 4. * std::f64::consts::PI,
            };
            assert!((mass - s.initial.density * expected_volume).abs() < 1e-10);
        }
    }

    #[test]
    fn axisymmetric_restart_preserves_weighted_conservation_and_checks_coordinates() {
        let mesh = mesh();
        let mut s = Settings::default();
        s.coordinates = Coordinates::AxisymmetricX { radial_origin: 0. };
        s.initial.pressure = 1.;
        s.end_time = 0.01;
        s.max_time_step = 0.001;
        s.max_steps = 2;
        s.boundaries.insert("wall".into(), Boundary::SlipWall);
        let first = solve(&mesh, &s).unwrap();
        let snapshot = first.snapshot(s.gas);
        let continued =
            solve_resuming(&mesh, &s, Some(&snapshot), |_| Ok(StepAction::Continue)).unwrap();
        assert_eq!(
            continued.report.accepted_steps,
            first.report.accepted_steps + 2
        );
        assert!(continued
            .report
            .conservation_defect
            .iter()
            .all(|v| v.abs() < 1e-10));
        s.coordinates = Coordinates::Cartesian;
        assert!(
            solve_resuming(&mesh, &s, Some(&snapshot), |_| Ok(StepAction::Continue))
                .unwrap_err()
                .contains("coordinate")
        );
    }
}
