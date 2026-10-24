//! Dedicated scalar physics editor and archive viewer. No CFD field is fabricated.
use super::*;
use flursys::multiphysics::{
    self as physics, Options, Properties, Region, State as Fields, checkpoint::Checkpoint,
    energy::Boundary, metrics::Coordinates,
};
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, Ordering},
};
#[derive(Default)]
struct Controls {
    stop: AtomicBool,
    paused: AtomicBool,
}
#[derive(Default)]
struct Mailbox {
    latest: Mutex<Option<(Arc<Fields>, String)>>,
    finished: Mutex<Option<Result<(PathBuf, Option<Checkpoint>), String>>>,
}
struct Local {
    project_id: String,
    options: Options,
    mesh: Option<Arc<flursys::UnstructuredMesh>>,
    fields: Option<Arc<Fields>>,
    restart: Option<Checkpoint>,
    folder: Option<PathBuf>,
    frames: Vec<PathBuf>,
    controls: Arc<Controls>,
    mail: Arc<Mailbox>,
    view: viewport::View3d,
    display_field: std::cell::Cell<i32>,
    archive_ranges: Option<[[f64; 2]; 4]>,
    archive_options: Option<Options>,
    anchor: Option<(f32, f32)>,
    timer: Timer,
}
impl Default for Local {
    fn default() -> Self {
        Self {
            project_id: String::new(),
            options: Default::default(),
            mesh: None,
            fields: None,
            restart: None,
            folder: None,
            frames: Vec::new(),
            controls: Default::default(),
            mail: Default::default(),
            view: Default::default(),
            display_field: std::cell::Cell::new(-1),
            archive_ranges: None,
            archive_options: None,
            anchor: None,
            timer: Timer::default(),
        }
    }
}
fn n(s: &SharedString, label: &str) -> Result<f64, String> {
    let n = s
        .trim()
        .parse::<f64>()
        .map_err(|_| format!("Invalid {label}"))?;
    if !n.is_finite() {
        return Err(format!("{label} must be finite"));
    }
    Ok(n)
}
fn rows<T: Clone + 'static>(model: ModelRc<T>) -> Vec<T> {
    (0..model.row_count())
        .filter_map(|i| model.row_data(i))
        .collect()
}
fn model<T: Clone + 'static>(values: Vec<T>) -> ModelRc<T> {
    ModelRc::from(Rc::new(VecModel::from(values)))
}
fn text(v: f64) -> SharedString {
    v.to_string().into()
}
fn push(ui: &MainWindow, options: &Options, mesh: &flursys::UnstructuredMesh) {
    let (coordinates, origin) = match options.coordinates {
        Coordinates::Cartesian => (0, 0.0),
        Coordinates::AxisymmetricX { radial_origin } => (1, radial_origin),
        Coordinates::AxisymmetricY { radial_origin } => (2, radial_origin),
    };
    ui.set_transport_draft(TransportDraft {
        coordinates,
        origin: text(origin),
        dt: text(options.dt),
        steps: options.steps.min(i32::MAX as usize) as i32,
        every: options.write_every.min(i32::MAX as usize) as i32,
        alpha: text(options.initial_phase_fraction),
        temperature: text(options.initial_temperature),
        rho1: text(options.first.density),
        cp1: text(options.first.specific_heat),
        k1: text(options.first.conductivity),
        rho2: text(options.second.density),
        cp2: text(options.second.specific_heat),
        k2: text(options.second.conductivity),
        courant: text(options.phase.max_courant),
        subcycles: options.phase.max_subcycles.min(i32::MAX as usize) as i32,
    });
    let mut boundaries = Vec::new();
    for patch in mesh.boundary_patches() {
        let (kind, value, ambient) = match options
            .energy
            .boundaries
            .get(&patch.name)
            .copied()
            .unwrap_or(Boundary::Adiabatic)
        {
            Boundary::Adiabatic => (0, 0.0, 293.15),
            Boundary::FixedTemperature { temperature } => (1, temperature, 0.0),
            Boundary::InletOutlet {
                incoming_temperature,
            } => (2, incoming_temperature, 0.0),
            Boundary::HeatFlux { inward_flux } => (3, inward_flux, 0.0),
            Boundary::Convection {
                coefficient,
                ambient_temperature,
            } => (4, coefficient, ambient_temperature),
            Boundary::Radiation {
                emissivity,
                ambient_temperature,
            } => (5, emissivity, ambient_temperature),
        };
        boundaries.push(TransportBoundaryDraft {
            name: patch.name.clone().into(),
            kind,
            value: text(value),
            ambient: text(ambient),
            incoming: text(
                options
                    .phase
                    .incoming
                    .get(&patch.name)
                    .copied()
                    .unwrap_or(options.initial_phase_fraction),
            ),
        });
    }
    ui.set_transport_boundaries(model(boundaries));
    ui.set_transport_regions(model(
        options
            .regions
            .iter()
            .map(|r| {
                let p = r.solid.unwrap_or(Properties {
                    density: 8000.0,
                    specific_heat: 450.0,
                    conductivity: 80.0,
                });
                TransportRegionDraft {
                    solid: r.solid.is_some(),
                    xmin: text(r.minimum.x),
                    ymin: text(r.minimum.y),
                    zmin: text(r.minimum.z),
                    xmax: text(r.maximum.x),
                    ymax: text(r.maximum.y),
                    zmax: text(r.maximum.z),
                    alpha: text(r.phase_fraction),
                    temperature: text(r.temperature),
                    rho: text(p.density),
                    cp: text(p.specific_heat),
                    k: text(p.conductivity),
                    source: text(r.source_w_m3),
                }
            })
            .collect(),
    ));
    ui.set_transport_contacts(model(
        options
            .energy
            .contact_resistance
            .iter()
            .map(|(f, r)| TransportContactDraft {
                face: f.to_string().into(),
                resistance: text(*r),
            })
            .collect(),
    ));
    ui.set_transport_ready(true);
    let three = mesh.dimension() == MeshDimension::ThreeD;
    ui.set_transport_three_d(three);
    ui.set_transport_view_axis(if three { 3 } else { 2 });
    ui.set_transport_orientation(render::render_orientation_overlay(
        if three { 0.65 } else { 0.0 },
        if three {
            0.48
        } else {
            std::f32::consts::FRAC_PI_2
        },
    ));
}
fn pull(ui: &MainWindow, base: &Options) -> Result<Options, String> {
    let d = ui.get_transport_draft();
    let mut o = base.clone();
    o.catalog = None;
    o.coordinates = match d.coordinates {
        0 => Coordinates::Cartesian,
        1 => Coordinates::AxisymmetricX {
            radial_origin: n(&d.origin, "radial origin")?,
        },
        2 => Coordinates::AxisymmetricY {
            radial_origin: n(&d.origin, "radial origin")?,
        },
        _ => return Err("Invalid coordinate system".into()),
    };
    o.dt = n(&d.dt, "time step")?;
    o.steps = d.steps.max(1) as usize;
    o.write_every = d.every.max(1) as usize;
    o.initial_phase_fraction = n(&d.alpha, "phase fraction")?;
    o.initial_temperature = n(&d.temperature, "temperature")?;
    o.first = Properties {
        density: n(&d.rho1, "phase 1 density")?,
        specific_heat: n(&d.cp1, "phase 1 Cp")?,
        conductivity: n(&d.k1, "phase 1 conductivity")?,
    };
    o.second = Properties {
        density: n(&d.rho2, "phase 2 density")?,
        specific_heat: n(&d.cp2, "phase 2 Cp")?,
        conductivity: n(&d.k2, "phase 2 conductivity")?,
    };
    o.phase.max_courant = n(&d.courant, "phase Courant limit")?;
    o.phase.max_subcycles = d.subcycles.max(1) as usize;
    o.energy.boundaries.clear();
    o.phase.incoming.clear();
    for b in rows(ui.get_transport_boundaries()) {
        let value = if b.kind == 0 {
            0.0
        } else {
            n(&b.value, "thermal boundary value")?
        };
        let boundary = match b.kind {
            0 => Boundary::Adiabatic,
            1 => Boundary::FixedTemperature { temperature: value },
            2 => Boundary::InletOutlet {
                incoming_temperature: value,
            },
            3 => Boundary::HeatFlux { inward_flux: value },
            4 => Boundary::Convection {
                coefficient: value,
                ambient_temperature: n(&b.ambient, "ambient temperature")?,
            },
            5 => Boundary::Radiation {
                emissivity: value,
                ambient_temperature: n(&b.ambient, "enclosure temperature")?,
            },
            _ => return Err("Invalid thermal condition".into()),
        };
        o.energy.boundaries.insert(b.name.to_string(), boundary);
        o.phase.incoming.insert(
            b.name.to_string(),
            n(&b.incoming, "incoming phase fraction")?,
        );
    }
    o.regions = rows(ui.get_transport_regions())
        .iter()
        .map(|r| {
            Ok(Region {
                minimum: Vec3::new(
                    n(&r.xmin, "min X")?,
                    n(&r.ymin, "min Y")?,
                    n(&r.zmin, "min Z")?,
                ),
                maximum: Vec3::new(
                    n(&r.xmax, "max X")?,
                    n(&r.ymax, "max Y")?,
                    n(&r.zmax, "max Z")?,
                ),
                phase_fraction: n(&r.alpha, "region phase fraction")?,
                temperature: n(&r.temperature, "region temperature")?,
                solid: if r.solid {
                    Some(Properties {
                        density: n(&r.rho, "solid density")?,
                        specific_heat: n(&r.cp, "solid Cp")?,
                        conductivity: n(&r.k, "solid conductivity")?,
                    })
                } else {
                    None
                },
                source_w_m3: n(&r.source, "heat source")?,
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    o.energy.contact_resistance.clear();
    for c in rows(ui.get_transport_contacts()) {
        let face = c
            .face
            .trim()
            .parse::<usize>()
            .map_err(|_| "Internal contact face must be a nonnegative integer")?;
        if o.energy
            .contact_resistance
            .insert(face, n(&c.resistance, "contact resistance")?)
            .is_some()
        {
            return Err("Duplicate thermal contact face".into());
        }
    }
    if o.dt <= 0.0
        || !(0.0..=1.0).contains(&o.initial_phase_fraction)
        || o.initial_temperature <= 0.0
        || o.phase.max_courant <= 0.0
        || o.phase.max_courant > 1.0
    {
        return Err("Require dt/T > 0, phase fraction in [0,1] and Courant limit in (0,1]".into());
    }
    for p in [o.first, o.second] {
        if p.density <= 0.0 || p.specific_heat <= 0.0 || p.conductivity <= 0.0 {
            return Err("Material rho, Cp and k must be positive".into());
        }
    }
    Ok(o)
}
fn frames(folder: &Path) -> Vec<PathBuf> {
    std::fs::read(folder.join("frames.json"))
        .ok()
        .and_then(|b| serde_json::from_slice::<Vec<serde_json::Value>>(&b).ok())
        .unwrap_or_default()
        .into_iter()
        .filter_map(|v| {
            v.get("state")
                .and_then(|v| v.as_str())
                .filter(|s| {
                    Path::new(s).components().count() == 1
                        && s.starts_with("step-")
                        && s.ends_with(".json")
                })
                .map(|s| folder.join(s))
        })
        .collect()
}
fn render(ui: &MainWindow, local: &Local) {
    let (Some(mesh), Some(fields)) = (&local.mesh, &local.fields) else {
        return;
    };
    let values = match ui.get_transport_field() {
        1 => fields.phase_fraction.clone(),
        2 => fields.density.clone(),
        3 => fields
            .solid
            .iter()
            .map(|v| if *v { 1.0 } else { 0.0 })
            .collect(),
        _ => fields.temperature.clone(),
    };
    if values.len() != mesh.cell_count() {
        return;
    }
    let field = ui.get_transport_field().clamp(0, 3) as usize;
    if local.display_field.get() != field as i32 {
        local.display_field.set(field as i32);
        ui.set_transport_range_min("".into());
        ui.set_transport_range_max("".into());
    }
    let current = [
        values.iter().copied().fold(f64::INFINITY, f64::min),
        values.iter().copied().fold(f64::NEG_INFINITY, f64::max),
    ];
    let [mut lo, mut hi] = if field == 1 || field == 3 {
        [0.0, 1.0]
    } else {
        local.archive_ranges.map_or(current, |r| r[field])
    };
    let a = ui.get_transport_range_min();
    let b = ui.get_transport_range_max();
    if !a.trim().is_empty() || !b.trim().is_empty() {
        match (n(&a, "colour minimum"), n(&b, "colour maximum")) {
            (Ok(a), Ok(b)) if b > a => {
                lo = a;
                hi = b;
            }
            _ => {
                ui.set_transport_status(
                    "Enter finite colour limits with minimum < maximum, or clear both.".into(),
                );
                return;
            }
        }
    }

    let mut view = local.view;
    if mesh.dimension() == MeshDimension::TwoD {
        view.yaw = 0.0;
        view.pitch = std::f32::consts::FRAC_PI_2;
    }
    let mut scene = scene_view::SceneView::spatial(mesh.points().iter().map(|p| p.position), view);
    let color = |v: f64| {
        render::scalar_color(
            if hi > lo {
                ((v - lo) / (hi - lo)) as f32
            } else {
                0.5
            },
            1,
        )
    };
    if mesh.dimension() == MeshDimension::TwoD {
        for (i, c) in mesh.cells().iter().enumerate() {
            let mut vertices = c
                .faces
                .iter()
                .flat_map(|&f| mesh.faces()[f].vertices.iter().copied())
                .collect::<std::collections::BTreeSet<_>>()
                .into_iter()
                .map(|i| mesh.points()[i].position)
                .collect::<Vec<_>>();
            vertices.sort_by(|a, b| {
                (a.y - c.center.y)
                    .atan2(a.x - c.center.x)
                    .total_cmp(&(b.y - c.center.y).atan2(b.x - c.center.x))
            });
            scene.polygon(&vertices, &[], color(values[i]));
        }
        if ui.get_transport_edges() {
            for f in mesh.faces() {
                scene.line(
                    &f.vertices
                        .iter()
                        .map(|&i| mesh.points()[i].position)
                        .collect::<Vec<_>>(),
                    [22, 30, 40, 255],
                    0.65,
                    false,
                );
            }
        }
    } else {
        let (a, b) = bounds(mesh);
        let mode = ui.get_transport_cut();
        let fraction = f64::from(ui.get_transport_cut_position().clamp(0.0, 100.0)) / 100.0;
        let coordinate = |p: Vec3| match mode {
            1 => p.z,
            2 => p.y,
            _ => p.x,
        };
        let plane = coordinate(a) + (coordinate(b) - coordinate(a)) * fraction;
        let keep = mesh
            .cells()
            .iter()
            .map(|c| {
                mode == 0
                    || if ui.get_transport_cut_reverse() {
                        coordinate(c.center) >= plane
                    } else {
                        coordinate(c.center) <= plane
                    }
            })
            .collect::<Vec<_>>();
        for f in mesh.faces() {
            let owner = keep[f.owner];
            let neighbour = f.neighbour.is_some_and(|n| keep[n]);
            if owner == neighbour {
                continue;
            }
            let visible = if owner { f.owner } else { f.neighbour.unwrap() };
            let mut vertices = f
                .vertices
                .iter()
                .map(|&i| mesh.points()[i].position)
                .collect::<Vec<_>>();
            if !owner {
                vertices.reverse();
            }
            scene.polygon(&vertices, &[], color(values[visible]));
            if ui.get_transport_edges() {
                let mut line = vertices.clone();
                if let Some(first) = vertices.first() {
                    line.push(*first);
                }
                scene.line(&line, [22, 30, 40, 255], 0.65, false);
            }
        }
    }
    ui.set_transport_orientation(render::render_orientation_overlay(view.yaw, view.pitch));
    ui.set_transport_image(scene.image(true, false));
    ui.set_transport_progress(
        format!(
            "Step {} · time {:.6e} s · displayed range [{:.6e}, {:.6e}]",
            fields.step, fields.time, lo, hi
        )
        .into(),
    );
}
fn load_frame(ui: &MainWindow, local: &mut Local, index: usize) -> Result<(), String> {
    let path = local
        .frames
        .get(index)
        .ok_or("No recorded frame at this index")?;
    let checkpoint = Checkpoint::load(path)?;
    let mesh = local.mesh.as_ref().ok_or("No archived mesh")?;
    checkpoint.validate(
        mesh,
        local.archive_options.as_ref().unwrap_or(&local.options),
    )?;
    local.fields = Some(Arc::new(checkpoint.state));
    ui.set_transport_frame(index as i32);
    render(ui, local);
    Ok(())
}
fn status(ui: &MainWindow, result: Result<(), String>) {
    if let Err(e) = result {
        ui.set_transport_status(e.into());
    }
}
pub(super) fn bind(ui: &MainWindow, app: &Rc<RefCell<AppState>>) {
    let local = Rc::new(RefCell::new(Local::default()));
    let weak = ui.as_weak();
    let shared = app.clone();
    let draft = local.clone();
    ui.on_open_transport(move || {
        let Some(ui) = weak.upgrade() else { return };
        if !ui.get_transport_running() {
            let s = shared.borrow();
            let Some(generated) = s.workbench.mesh() else {
                let mut l = draft.borrow_mut();
                l.mesh = None;
                l.fields = None;
                l.restart = None;
                l.frames.clear();
                ui.set_transport_ready(false);
                ui.set_transport_has_restart(false);
                ui.set_transport_frames(0);
                ui.set_transport_image(render::render_empty_image());
                ui.set_transport_status(
                    "Generate/restore a mesh, or open a scalar archive from Results.".into(),
                );
                ui.set_transport_open(true);
                return;
            };
            let mut l = draft.borrow_mut();
            let mut o = s.workbench.scalar_transport().cloned().unwrap_or_else(|| {
                let mut o = Options::default();
                o.first = Properties {
                    density: s.workbench.material().density,
                    specific_heat: s.workbench.solver_options().thermal.specific_heat,
                    conductivity: s.workbench.solver_options().thermal.conductivity,
                };
                o
            });
            for p in generated.mesh.boundary_patches() {
                o.energy
                    .boundaries
                    .entry(p.name.clone())
                    .or_insert(Boundary::Adiabatic);
            }
            let changed_mesh = l.mesh.as_ref().is_some_and(|m| {
                physics::mesh_signature(m) != physics::mesh_signature(&generated.mesh)
            });
            if l.project_id != s.workbench_project.project_id || changed_mesh {
                l.restart = None;
                l.fields = None;
                l.archive_ranges = None;
                l.frames.clear();
                l.folder = None;
                ui.set_transport_has_restart(false);
                ui.set_transport_frames(0);
                ui.set_transport_image(render::render_empty_image());
            }
            l.project_id = s.workbench_project.project_id.clone();
            l.mesh = Some(Arc::new(generated.mesh.clone()));
            l.options = o.clone();
            push(&ui, &o, &generated.mesh);
        }
        ui.set_transport_open(true);
    });
    let weak = ui.as_weak();
    let draft = local.clone();
    let shared = app.clone();
    ui.on_save_transport(move || {
        let Some(ui) = weak.upgrade() else { return };
        status(
            &ui,
            (|| {
                let mut l = draft.borrow_mut();
                let o = pull(&ui, &l.options)?;
                if l.project_id!=shared.borrow().workbench_project.project_id{return Err("Archive settings are independent of the current project; export settings and reopen its editor before applying".into());}
                if let Some(m) = &l.mesh {
                    o.coordinates.measures(m)?;
                }
                let mut s = shared.borrow_mut();
                s.workbench.set_scalar_transport(o.clone());
                s.mark_workbench_dirty();
                l.options = o;
                ui.set_transport_status(
                    "Scalar setup saved in the project draft. Save the project to persist it."
                        .into(),
                );
                Ok(())
            })(),
        );
    });
    let weak = ui.as_weak();
    let draft = local.clone();
    ui.on_add_transport_region(move || {
        let Some(ui) = weak.upgrade() else { return };
        let mut data = rows(ui.get_transport_regions());
        let mut row = TransportRegionDraft {
            solid: true,
            alpha: "0".into(),
            temperature: "293.15".into(),
            rho: "8000".into(),
            cp: "450".into(),
            k: "80".into(),
            source: "0".into(),
            ..Default::default()
        };
        if let Some(mesh) = &draft.borrow().mesh {
            let (a, b) = bounds(mesh);
            row.xmin = text(a.x);
            row.ymin = text(a.y);
            row.zmin = text(a.z);
            row.xmax = text(b.x);
            row.ymax = text(b.y);
            row.zmax = text(b.z);
        }
        data.push(row);
        ui.set_transport_regions(model(data));
    });
    let weak = ui.as_weak();
    ui.on_remove_transport_region(move |i| {
        if let Some(ui) = weak.upgrade() {
            let mut data = rows(ui.get_transport_regions());
            if i >= 0 && (i as usize) < data.len() {
                data.remove(i as usize);
            }
            ui.set_transport_regions(model(data));
        }
    });
    let weak = ui.as_weak();
    ui.on_add_transport_contact(move || {
        if let Some(ui) = weak.upgrade() {
            let mut data = rows(ui.get_transport_contacts());
            data.push(TransportContactDraft {
                face: "0".into(),
                resistance: "0".into(),
            });
            ui.set_transport_contacts(model(data));
        }
    });
    let weak = ui.as_weak();
    ui.on_remove_transport_contact(move |i| {
        if let Some(ui) = weak.upgrade() {
            let mut data = rows(ui.get_transport_contacts());
            if i >= 0 && (i as usize) < data.len() {
                data.remove(i as usize);
            }
            ui.set_transport_contacts(model(data));
        }
    });
    let weak = ui.as_weak();
    let draft = local.clone();
    ui.on_import_transport(move||{let Some(ui)=weak.upgrade()else{return};let Some(path)=rfd::FileDialog::new().add_filter("Transport settings",&["json"]).pick_file()else{return};status(&ui,(||{let o:Options=serde_json::from_slice(&std::fs::read(path).map_err(|e|e.to_string())?).map_err(|e|e.to_string())?;let (o,_)=o.resolve_catalog()?;let mut l=draft.borrow_mut();let mesh=l.mesh.as_ref().ok_or("Load a mesh first")?;push(&ui,&o,mesh);l.options=o;l.restart=None;ui.set_transport_has_restart(false);ui.set_transport_status("Settings loaded; values are explicit and frozen. Save setup to keep them in the project.".into());Ok(())})());});
    let weak = ui.as_weak();
    let draft = local.clone();
    ui.on_export_transport(move || {
        let Some(ui) = weak.upgrade() else { return };
        let Some(path) = rfd::FileDialog::new()
            .set_file_name("transport.json")
            .save_file()
        else {
            return;
        };
        status(
            &ui,
            (|| {
                let o = pull(&ui, &draft.borrow().options)?;
                std::fs::write(
                    path,
                    serde_json::to_vec_pretty(&o).map_err(|e| e.to_string())?,
                )
                .map_err(|e| e.to_string())?;
                ui.set_transport_status("Transport settings exported.".into());
                Ok(())
            })(),
        );
    });
    let weak = ui.as_weak();
    let draft = local.clone();
    let shared = app.clone();
    ui.on_use_transport_flow(move||{let Some(ui)=weak.upgrade()else{return};status(&ui,(||{if ui.get_transport_draft().coordinates!=0{return Err("A Cartesian CFD flux cannot be silently converted to axisymmetric flux".into());}let s=shared.borrow();let mesh=&s.workbench.mesh().ok_or("No current mesh")?.mesh;let solution=s.workbench.solution().ok_or("No completed CFD solution on the current mesh")?;solution.face_flux.ensure_mesh(mesh).map_err(|e|format!("{e:?}"))?;let mut l=draft.borrow_mut();l.options.face_flux=Some(solution.face_flux.values().to_vec());l.options.mesh_signature=Some(physics::mesh_signature(mesh));ui.set_transport_status("Completed CFD face flux copied as a fixed transport input. Flow will not respond to temperature or phase changes.".into());Ok(())})());});
    let weak = ui.as_weak();
    let draft = local.clone();
    ui.on_reset_transport(move || {
        if let Some(ui) = weak.upgrade() {
            let mut l = draft.borrow_mut();
            l.restart = None;
            l.fields = None;
            l.frames.clear();
            l.archive_options = None;
            l.archive_ranges = None;
            ui.set_transport_playing(false);
            ui.set_transport_frames(0);
            ui.set_transport_frame(0);
            ui.set_transport_has_restart(false);
            ui.set_transport_image(render::render_empty_image());
            ui.set_transport_status(
                "Retained state cleared. Next Run starts from the configured initial fields."
                    .into(),
            );
        }
    });
    let weak = ui.as_weak();
    let draft = local.clone();
    ui.on_load_transport_restart(move || {
        let Some(ui) = weak.upgrade() else { return };
        let Some(path) = rfd::FileDialog::new()
            .add_filter("Scalar checkpoint", &["json"])
            .pick_file()
        else {
            return;
        };
        status(
            &ui,
            (|| {
                let c = Checkpoint::load(&path)?;
                let mut l = draft.borrow_mut();
                let o = pull(&ui, &l.options)?;
                c.validate(l.mesh.as_ref().ok_or("No mesh")?, &o)?;
                l.options = o;
                l.fields = Some(Arc::new(c.state.clone()));
                l.restart = Some(c);
                ui.set_transport_has_restart(true);
                render(&ui, &l);
                ui.set_transport_status(
                    "Restart loaded; Run continues the physical time and step count.".into(),
                );
                Ok(())
            })(),
        );
    });
    let weak = ui.as_weak();
    let draft = local.clone();
    let shared = app.clone();
    ui.on_run_transport(move||{let Some(ui)=weak.upgrade()else{return};if ui.get_transport_running(){return;}ui.set_transport_playing(false);
  status(&ui,(||{
   let mut l=draft.borrow_mut();let o=pull(&ui,&l.options)?;let mesh=l.mesh.clone().ok_or("No mesh available")?;o.coordinates.measures(&mesh)?;
   if let Some(c)=&l.restart{c.validate(&mesh,&o)?;}
   let same_project=l.project_id==shared.borrow().workbench_project.project_id;
   let parent=if same_project{shared.borrow().workspace_path.clone().map(|p|p.join("transport"))}else{None};
   let parent=match parent{Some(p)=>p,None=>rfd::FileDialog::new().set_title("Choose parent folder for a new scalar run").pick_folder().ok_or("No output directory chosen")?};
   let nonce=std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_err(|e|e.to_string())?.as_nanos();let folder=parent.join(format!("run-{nonce}"));
   if same_project {let mut app=shared.borrow_mut();app.workbench.set_scalar_transport(o.clone());app.mark_workbench_dirty();}
   l.options=o.clone();l.archive_ranges=None;l.archive_options=None;l.controls=Default::default();l.mail=Default::default();l.frames.clear();ui.set_transport_frames(0);ui.set_transport_output(folder.display().to_string().into());ui.set_transport_running(true);ui.set_transport_paused(false);ui.set_transport_playing(false);ui.set_transport_status("Running independent phase/energy transport…".into());
   let controls=l.controls.clone();let mail=l.mail.clone();let restart=l.restart.clone();let weak=ui.as_weak();
   std::thread::spawn(move||{
    let job=std::panic::catch_unwind(std::panic::AssertUnwindSafe(||execute_job(&mesh,&o,&folder,&parent,restart.as_ref(),&controls,&mail,&weak)))
        .unwrap_or_else(|_|Err("Scalar worker panicked; recorded checkpoints remain in the run directory".into()));
    if let Ok(mut result)=mail.finished.lock(){*result=Some(job);}
    let _=weak.upgrade_in_event_loop(|ui|ui.invoke_update_transport_job());
   });Ok(())
  })());
 });
    let weak = ui.as_weak();
    let draft = local.clone();
    ui.on_stop_transport(move || {
        if let Some(ui) = weak.upgrade() {
            let l = draft.borrow();
            l.controls.stop.store(true, Ordering::Relaxed);
            l.controls.paused.store(false, Ordering::Relaxed);
            ui.set_transport_paused(false);
            ui.set_transport_status("Stop requested; retaining the last accepted state.".into());
        }
    });
    let weak = ui.as_weak();
    let draft = local.clone();
    ui.on_pause_transport(move || {
        if let Some(ui) = weak.upgrade() {
            draft
                .borrow()
                .controls
                .paused
                .store(true, Ordering::Relaxed);
            ui.set_transport_paused(true);
            ui.set_transport_status("Pause requested; fields retained in worker.".into());
        }
    });
    let weak = ui.as_weak();
    let draft = local.clone();
    ui.on_resume_transport(move || {
        if let Some(ui) = weak.upgrade() {
            draft
                .borrow()
                .controls
                .paused
                .store(false, Ordering::Relaxed);
            ui.set_transport_paused(false);
            ui.set_transport_status("Resuming the same scalar run.".into());
        }
    });
    let weak = ui.as_weak();
    let draft = local.clone();
    ui.on_update_transport_job(move||{let Some(ui)=weak.upgrade()else{return};let mut l=draft.borrow_mut();let latest=l.mail.latest.lock().ok().and_then(|mut v|v.take());if let Some((fields,message))=latest{l.fields=Some(fields);ui.set_transport_status(message.into());if ui.get_transport_open(){render(&ui,&l);}}
  let finished=l.mail.finished.lock().ok().and_then(|mut v|v.take());if let Some(result)=finished{ui.set_transport_running(false);ui.set_transport_paused(false);match result{Ok((folder,checkpoint))=>{l.archive_options=Some(l.options.clone());l.archive_ranges=archive_ranges(&folder);l.frames=frames(&folder);l.folder=Some(folder.clone());l.restart=checkpoint;if let Some(c)=&l.restart{l.fields=Some(Arc::new(c.state.clone()));}ui.set_transport_has_restart(l.restart.is_some());ui.set_transport_frames(l.frames.len() as i32);ui.set_transport_frame(l.frames.len().saturating_sub(1) as i32);let report=std::fs::read(folder.join("report.json")).ok().and_then(|v|serde_json::from_slice::<serde_json::Value>(&v).ok());let message=report.as_ref().and_then(|v|v.get("message")).and_then(|v|v.as_str()).unwrap_or("Run ended");let outcome=report.as_ref().and_then(|v|v.get("status")).and_then(|v|v.as_str()).unwrap_or("unknown");ui.set_transport_status(format!("{outcome}: {message}. Archive: {}. Continue adds steps; reset starts again.",folder.display()).into());render(&ui,&l);},Err(e)=>ui.set_transport_status(format!("Scalar run failed: {e}").into())}}
 });
    let weak = ui.as_weak();
    let draft = local.clone();
    ui.on_open_transport_archive(move||{let Some(ui)=weak.upgrade()else{return};if ui.get_transport_running(){return;}let Some(folder)=rfd::FileDialog::new().set_title("Open scalar transport archive").pick_folder()else{return};status(&ui,(||{let mesh=Arc::new(physics::archive::load_mesh(&folder)?);let o:Options=serde_json::from_slice(&std::fs::read(folder.join("resolved-transport.json")).map_err(|e|e.to_string())?).map_err(|e|e.to_string())?;let c=Checkpoint::load(&folder.join("checkpoint.json"))?;c.validate(&mesh,&o)?;let mut l=draft.borrow_mut();l.project_id.clear();l.mesh=Some(mesh.clone());l.options=o.clone();l.fields=Some(Arc::new(c.state.clone()));l.restart=Some(c);l.archive_options=Some(l.options.clone());l.archive_ranges=archive_ranges(&folder);l.frames=frames(&folder);l.folder=Some(folder.clone());push(&ui,&o,&mesh);ui.set_transport_output(folder.display().to_string().into());ui.set_transport_frames(l.frames.len() as i32);ui.set_transport_frame(l.frames.len().saturating_sub(1) as i32);ui.set_transport_has_restart(true);ui.set_transport_status("Archived scalar fields loaded with their own mesh. Current CFD project was not replaced.".into());render(&ui,&l);Ok(())})());});
    let weak = ui.as_weak();
    let draft = local.clone();
    ui.on_refresh_transport(move || {
        if let Some(ui) = weak.upgrade() {
            render(&ui, &draft.borrow());
        }
    });
    let weak = ui.as_weak();
    let draft = local.clone();
    ui.on_select_transport_frame(move |index| {
        let Some(ui) = weak.upgrade() else { return };
        status(
            &ui,
            load_frame(&ui, &mut draft.borrow_mut(), index.max(0) as usize),
        );
    });
    let weak = ui.as_weak();
    let draft = local.clone();
    ui.on_orbit_transport(move |dx, dy, w, h| {
        if let Some(ui) = weak.upgrade() {
            if !ui.get_transport_three_d() {
                return;
            }
            let mut l = draft.borrow_mut();
            let current = (l.view.yaw, l.view.pitch);
            let (yaw, pitch) = *l.anchor.get_or_insert(current);
            l.view.orbit_from(yaw, pitch, dx - w, dy - h);
            ui.set_transport_view_axis(7);
            render(&ui, &l);
        }
    });
    let weak = ui.as_weak();
    let draft = local.clone();
    ui.on_zoom_transport(move |delta| {
        if let Some(ui) = weak.upgrade() {
            let mut l = draft.borrow_mut();
            l.view.zoom_by(delta);
            render(&ui, &l);
        }
    });
    let weak = ui.as_weak();
    let draft = local.clone();
    ui.on_home_transport(move || {
        if let Some(ui) = weak.upgrade() {
            let mut l = draft.borrow_mut();
            l.view = Default::default();
            ui.set_transport_view_axis(if ui.get_transport_three_d() { 3 } else { 2 });
            l.anchor = None;

            render(&ui, &l);
        }
    });
    let weak = ui.as_weak();
    let draft = local.clone();
    ui.on_end_transport_orbit(move || {
        if weak.upgrade().is_some() {
            let mut l = draft.borrow_mut();
            l.anchor = None;
        }
    });
    let weak = ui.as_weak();
    let draft = local.clone();
    ui.on_pan_transport(move |dx, dy, w, h| {
        if let Some(ui) = weak.upgrade() {
            let mut l = draft.borrow_mut();
            l.view.pan.0 += f64::from(dx / w.max(1.0)) * f64::from(PREVIEW_WIDTH);
            l.view.pan.1 += f64::from(dy / h.max(1.0)) * f64::from(PREVIEW_HEIGHT);
            render(&ui, &l);
        }
    });
    let weak = ui.as_weak();
    let draft = local.clone();
    ui.on_axis_transport(move |axis| {
        if let Some(ui) = weak.upgrade() {
            let mut l = draft.borrow_mut();
            let axis = if ui.get_transport_three_d() {
                axis.clamp(0, 6)
            } else {
                2
            };
            ui.set_transport_view_axis(axis);
            let (yaw, pitch) = geometry_view_angles(axis);
            l.view.yaw = yaw;
            l.view.pitch = pitch;
            l.anchor = None;
            render(&ui, &l);
        }
    });
    let weak = ui.as_weak();
    let draft = local.clone();
    ui.on_pick_transport_orientation(move |x, y| {
        if let Some(ui) = weak.upgrade() {
            let l = draft.borrow();
            let (yaw, pitch) = if ui.get_transport_three_d() {
                (l.view.yaw, l.view.pitch)
            } else {
                (0.0, std::f32::consts::FRAC_PI_2)
            };
            let axis = render::pick_orientation_overlay(yaw, pitch, x, y);
            drop(l);
            if let Some(axis) = axis {
                ui.invoke_axis_transport(axis);
            }
        }
    });
    let weak = ui.as_weak();
    let draft = Rc::downgrade(&local);
    local
        .borrow()
        .timer
        .start(TimerMode::Repeated, Duration::from_millis(150), move || {
            let (Some(ui), Some(local)) = (weak.upgrade(), draft.upgrade()) else {
                return;
            };
            if !ui.get_transport_open() || !ui.get_transport_playing() || ui.get_transport_running()
            {
                return;
            }
            let mut l = local.borrow_mut();
            if l.frames.is_empty() {
                return;
            }
            let next = (ui.get_transport_frame().max(0) as usize + 1) % l.frames.len();
            status(&ui, load_frame(&ui, &mut l, next));
        });
}
fn bounds(mesh: &flursys::UnstructuredMesh) -> (Vec3, Vec3) {
    let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in mesh.points() {
        lo.x = lo.x.min(p.position.x);
        lo.y = lo.y.min(p.position.y);
        lo.z = lo.z.min(p.position.z);
        hi.x = hi.x.max(p.position.x);
        hi.y = hi.y.max(p.position.y);
        hi.z = hi.z.max(p.position.z);
    }
    (lo, hi)
}

fn archive_ranges(folder: &Path) -> Option<[[f64; 2]; 4]> {
    let frames: Vec<serde_json::Value> =
        serde_json::from_slice(&std::fs::read(folder.join("frames.json")).ok()?).ok()?;
    let mut ranges = [[f64::INFINITY, f64::NEG_INFINITY]; 4];
    for f in frames {
        for (i, r) in ranges.iter_mut().enumerate() {
            let a = f["ranges"][i][0].as_f64()?;
            let b = f["ranges"][i][1].as_f64()?;
            if !a.is_finite() || !b.is_finite() || b < a {
                return None;
            }
            r[0] = r[0].min(a);
            r[1] = r[1].max(b);
        }
    }
    ranges
        .iter()
        .flatten()
        .all(|v| v.is_finite())
        .then_some(ranges)
}

fn execute_job(
    mesh: &flursys::UnstructuredMesh,
    options: &Options,
    folder: &Path,
    parent: &Path,
    restart: Option<&Checkpoint>,
    controls: &Arc<Controls>,
    mail: &Mailbox,
    weak: &slint::Weak<MainWindow>,
) -> Result<(PathBuf, Option<Checkpoint>), String> {
    std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    let provenance = serde_json::json!({"source":"GUI explicit/frozen transport input","history":options.provenance,"pressure_momentum_feedback":false});
    let mut archive = physics::archive::Archive::create(folder, options, &provenance)?;
    let control = controls.clone();
    let callback: flursys::compute::LinearControl = Arc::new(move || {
        while control.paused.load(Ordering::Relaxed) && !control.stop.load(Ordering::Relaxed) {
            std::thread::sleep(Duration::from_millis(25));
        }
        !control.stop.load(Ordering::Relaxed)
    });
    let mut last_update = std::time::Instant::now();
    let solved = physics::run_controlled(
        mesh,
        options,
        flursys::LinearSolverOptions::default(),
        restart,
        Some(callback),
        |state, report| {
            archive.record(mesh, state, report)?;
            if state.step % options.write_every == 0
                || last_update.elapsed() > Duration::from_millis(200)
            {
                let message = format!(
                    "Accepted step {} · t={:.6e} s · phase balance {:.3e} · energy balance {:.3e} (per depth in planar mode)",
                    state.step,
                    state.time,
                    report.phase.conservation_error,
                    report.energy.balance_error
                );
                if let Ok(mut latest) = mail.latest.lock() {
                    *latest = Some((Arc::new(state.clone()), message));
                }
                let _ = weak.upgrade_in_event_loop(|ui| ui.invoke_update_transport_job());
                last_update = std::time::Instant::now();
            }
            Ok(())
        },
    );
    let (status, message) = match &solved {
        Ok(_) => (
            "completed",
            "Requested additional physical steps completed".to_owned(),
        ),
        Err(_) if controls.stop.load(Ordering::Relaxed) => (
            "stopped",
            "Stopped; last accepted state retained".to_owned(),
        ),
        Err(e) => ("failed", e.clone()),
    };
    archive.finish(mesh, status, &message)?;
    Ok((
        folder.into(),
        archive
            .last_checkpoint()
            .cloned()
            .or_else(|| restart.cloned()),
    ))
}
