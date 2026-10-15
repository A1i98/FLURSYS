//! CAD commands run on a document snapshot; the UI adopts a result only if the
//! source document is still current. No kernel work runs on the event loop.
use super::*;
use flursys::{
    BodyId, CadBoolean, CadFeatureId, CadPrimitive, CadSketchId, CadTransform, FaceId,
    GeometryTopology,
};
use std::sync::mpsc::{self, TryRecvError};
pub(super) struct CadPreview {
    source: GeometryTopology,
    workspace: Option<PathBuf>,
    project_id: String,
    pub(super) geometry: GeometryTopology,
    pub(super) selection: Vec<BodyId>,
    message: String,
}
pub(super) fn preview(state: &AppState) -> Option<&CadPreview> {
    state.cad_preview.as_ref().filter(|p| {
        state.workbench.geometry() == &p.source
            && state.workspace_path == p.workspace
            && state.workbench_project.project_id == p.project_id
    })
}
pub(super) struct CadJob {
    preview: bool,
    cancelled: bool,
    source: GeometryTopology,
    workspace: Option<PathBuf>,
    project_id: String,
    result: Receiver<Result<(GeometryTopology, Vec<BodyId>, bool, String), String>>,
}
struct Command {
    action: i32,
    through_all: bool,
    source: Option<BodyId>,
    target: Option<BodyId>,
    face: Option<FaceId>,
    axis: Vec3,
    second_face: Option<FaceId>,
    edit_sketch: Option<CadSketchId>,
    edit_feature: Option<CadFeatureId>,
    kind: i32,
    name: String,
    size: [f64; 3],
    position: Vec3,
    angle: f64,
    factor: f64,
    tolerance: f64,
    count: usize,
    path: Option<PathBuf>,
}
fn number(s: SharedString, label: &str) -> Result<f64, String> {
    let v = s
        .trim()
        .parse::<f64>()
        .map_err(|_| format!("{label} must be a number"))?;
    if !v.is_finite() {
        Err(format!("{label} must be finite"))
    } else {
        Ok(v)
    }
}
pub(super) fn bind(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
    super::sketch_edit::bind(ui, state);
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_cad_pick_orientation(move |x, y| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if let Some(axis) =
            super::render::pick_orientation_overlay(state.geometry_yaw, state.geometry_pitch, x, y)
        {
            let (yaw, pitch) = geometry_view_angles(axis);
            state.geometry_yaw = yaw;
            state.geometry_pitch = pitch;
            state.geometry_view_axis = axis;
            state.geometry_drag_anchor = None;
            refresh_ui(&ui, &state);
        }
    });

    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_cad_select_body(move |index| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        let id = state
            .workbench
            .geometry()
            .cad_bodies()
            .nth(index.max(0) as usize)
            .map(|(id, _)| id);
        if let Some(id) = id {
            select_tree_geometry_target(&mut state, GeometrySelectionTarget::Body(id), false);
            state.show_geometry_3d = true;
            state.show_mesh = false;
            ui.set_geometry_cad_view_3d(true);
            rebuild_tree_rows(&mut state);
            refresh_ui(&ui, &state);
        }
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_cad_command(move |action| {
        let Some(ui) = weak.upgrade() else { return; };
        let mut state = shared.borrow_mut();
        if action == -1 {
            if let Some(job) = state.cad_job.as_mut() {
                job.cancelled = true;
                ui.set_cad_feedback("Result will be discarded when the kernel operation finishes.".into());
            }
            return;
        }
        if action==36 {
            state.cad_preview=None;
            if super::sketch_edit::valid(&state).is_some() {state.show_geometry_3d=false;ui.set_geometry_cad_view_3d(false);}
            refresh_ui(&ui,&state);
            ui.set_cad_feedback("Preview discarded. Source model unchanged.".into());
            return;
        }
        if state.cad_job.is_some() { return; }
        if action==35 {
            if let Some(p)=state.cad_preview.take() {
                let (tx,rx)=mpsc::channel();
                let _=tx.send(Ok((p.geometry,p.selection,true,p.message)));
                state.cad_job=Some(CadJob {preview:false,cancelled:false,source:p.source,workspace:p.workspace,project_id:p.project_id,result:rx});
                poll(&ui,&mut state);
                refresh_ui(&ui,&state);
            }
            return;
        }
        if !require_project(&mut state) { refresh_ui(&ui, &state); return; }
        if matches!(action, 21 | 22 | 23 | 26 | 27 | 28 | 30 | 32 | 33 | 39 | 40) {
            let outcome = sketch_command(&ui, &mut state, action);
            ui.set_cad_feedback(outcome.unwrap_or_else(|e| e).into());
            rebuild_tree_rows(&mut state);
            refresh_ui(&ui, &state);
            return;
        }
        let is_preview=action>=100;
        let action=if is_preview {action-100} else {action};
        if is_preview && !matches!(action,0|1|2|24|29|31|34) {return;}
        state.cad_preview=None;
        let command = match read_command(&ui, &state, action) {
            Ok(Some(command)) => command,
            Ok(None) => return,
            Err(error) => { ui.set_cad_feedback(error.into()); return; }
        };
        let source = state.workbench.geometry().clone();
        let geometry = source.clone();
        let workspace = state.workspace_path.clone();
        let project_id = state.workbench_project.project_id.clone();
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let outcome = std::panic::catch_unwind(|| execute(geometry, command))
                .unwrap_or_else(|_| Err("The geometry kernel rejected this operation. The source document was preserved.".into()));
            let _ = tx.send(outcome);
        });
        state.cad_job = Some(CadJob { preview:is_preview, cancelled: false, source, workspace, project_id, result: rx });
        ui.set_cad_busy(true);
        ui.set_cad_feedback("Building geometry…".into());
    });
}
fn read_command(ui: &MainWindow, state: &AppState, action: i32) -> Result<Option<Command>, String> {
    let bodies = state
        .workbench
        .geometry()
        .cad_bodies()
        .map(|(id, _)| id)
        .collect::<Vec<_>>();
    let source = bodies.get(ui.get_cad_source().max(0) as usize).copied();
    let target = bodies.get(ui.get_cad_target().max(0) as usize).copied();
    let face = selected_extrusion_face(state).ok();
    let second_face = state
        .wb_selected_targets
        .iter()
        .filter_map(|t| {
            if let GeometrySelectionTarget::Face(id) = t {
                Some(*id)
            } else {
                None
            }
        })
        .nth(1);
    let mut c = Command {
        action,
        through_all: ui.get_cad_through_all(),
        source,
        target,
        face,
        second_face,
        edit_sketch: state
            .workbench
            .geometry()
            .sketches()
            .find(|s| s.id.get() == ui.get_cad_edit_sketch_id().max(0) as u64)
            .map(|s| s.id),
        edit_feature: state
            .workbench
            .geometry()
            .extrude_features()
            .nth(ui.get_cad_feature_index().max(0) as usize)
            .map(|f| f.id),
        axis: match ui.get_cad_axis() {
            0 => Vec3::new(1., 0., 0.),
            2 => Vec3::new(0., 0., 1.),
            _ => Vec3::new(0., 1., 0.),
        },
        kind: ui.get_cad_primitive_kind(),
        name: ui.get_cad_name().to_string(),
        size: [1.0; 3],
        position: Vec3::new(0.0, 0.0, 0.0),
        angle: 0.0,
        factor: 1.0,
        tolerance: 1e-3,
        count: ui.get_cad_count().max(0) as usize,
        path: None,
    };
    if action == 20 {
        c.face = state.wb_selected_targets.iter().find_map(|t| {
            if let GeometrySelectionTarget::Face(id) = t {
                Some(*id)
            } else {
                None
            }
        });
    }
    if matches!(action, 0 | 3 | 4 | 5 | 6 | 7 | 8) {
        c.position = Vec3::new(
            number(ui.get_cad_pos_x(), "X")?,
            number(ui.get_cad_pos_y(), "Y")?,
            number(ui.get_cad_pos_z(), "Z")?,
        );
    }
    if matches!(action, 37 | 38) {
        c.edit_sketch = state
            .workbench
            .geometry()
            .sketches()
            .nth(ui.get_cad_sketch_index().max(0) as usize)
            .map(|s| s.id);
    }
    if action == 2 {
        c.position = Vec3::new(
            number(ui.get_cad_revolve_x(), "Pivot X")?,
            number(ui.get_cad_revolve_y(), "Pivot Y")?,
            number(ui.get_cad_revolve_z(), "Pivot Z")?,
        );
    }
    if action == 29 {
        c.kind = ui.get_cad_sketch_kind();
        c.position = Vec3::new(
            number(ui.get_cad_pos_x(), "Local X")?,
            number(ui.get_cad_pos_y(), "Local Y")?,
            0.0,
        );
        c.size[0] = number(ui.get_cad_size_x(), "Radius / width")?;
        if c.kind == 1 {
            c.size[1] = number(ui.get_cad_size_y(), "Height")?;
        }
    }
    if action == 0 {
        c.size[0] = number(ui.get_cad_size_x(), "Length / radius")?;
        if c.kind != 2 {
            c.size[1] = number(ui.get_cad_size_y(), "Width / height")?;
        }
        if c.kind == 0 {
            c.size[2] = number(ui.get_cad_size_z(), "Height")?;
        }
    }
    if action == 34 && c.through_all {
        c.size[2] = 1.0;
    }
    if matches!(action, 1 | 24 | 31) || (action == 34 && !c.through_all) {
        c.size[2] = number(ui.get_cad_size_z(), "Extrusion distance")?;
    }
    if matches!(action, 2 | 4 | 8) {
        c.angle = number(ui.get_cad_angle(), "Angle")?;
    }
    if action == 8 && c.angle.abs() < 1e-8 {
        return Err("Circular pattern angular step must be nonzero".into());
    }
    if action == 5 {
        c.factor = number(ui.get_cad_factor(), "Scale")?;
    }
    if matches!(action, 9..=11 | 24 | 34) {
        c.tolerance = number(ui.get_cad_tolerance(), "Boolean tolerance")?;
    }
    if matches!(action, 15..=19) {
        c.path = if action == 18 || action == 19 {
            if action == 19 {
                rfd::FileDialog::new()
                    .add_filter("STEP solid", &["step", "stp"])
                    .pick_file()
            } else {
                rfd::FileDialog::new()
                    .add_filter("Truck B-rep", &["json"])
                    .pick_file()
            }
        } else {
            let (ext, desc) = match action {
                15 => ("step", "STEP CAD"),
                16 => ("stl", "STL mesh"),
                _ => ("json", "Truck B-rep"),
            };
            rfd::FileDialog::new()
                .add_filter(desc, &[ext])
                .set_file_name(format!("body.{ext}"))
                .save_file()
        };
        if c.path.is_none() {
            return Ok(None);
        }
    }
    Ok(Some(c))
}
fn execute(
    mut g: GeometryTopology,
    c: Command,
) -> Result<(GeometryTopology, Vec<BodyId>, bool, String), String> {
    let source = || {
        c.source
            .ok_or_else(|| "Select a CAD body from the model tree".to_string())
    };
    let result = (|| -> Result<Vec<BodyId>, String> {
        let one = match c.action {
            29 => {
                return g
                    .cad_edit_sketch(
                        c.edit_sketch.ok_or("Load a sketch for editing first")?,
                        c.position.x,
                        c.position.y,
                        if c.kind == 0 {
                            flursys::CadSketchProfile::Circle { radius: c.size[0] }
                        } else {
                            flursys::CadSketchProfile::Rectangle {
                                width: c.size[0],
                                height: c.size[1],
                            }
                        },
                    )
                    .map_err(|e| e.to_string())
            }
            31 => {
                return g
                    .cad_edit_pad(c.edit_feature.ok_or("Select a Pad feature")?, c.size[2])
                    .map_err(|e| e.to_string())
            }
            0 => g.cad_add_primitive(
                match c.kind {
                    0 => CadPrimitive::Box {
                        length: c.size[0],
                        width: c.size[1],
                        height: c.size[2],
                    },
                    1 => CadPrimitive::Cylinder {
                        radius: c.size[0],
                        height: c.size[1],
                    },
                    2 => CadPrimitive::Sphere { radius: c.size[0] },
                    3 => CadPrimitive::Cone {
                        radius: c.size[0],
                        height: c.size[1],
                    },
                    _ => CadPrimitive::Torus {
                        major_radius: c.size[0],
                        minor_radius: c.size[1],
                    },
                },
                c.position,
                &c.name,
            ),
            38 => {
                g.cad_delete_sketch(c.edit_sketch.ok_or("Select a sketch")?)
                    .map_err(|e| e.to_string())?;
                return Ok(Vec::new());
            }
            37 => {
                let id = c.edit_sketch.ok_or("Select a sketch")?;
                let face = g
                    .sketch(id)
                    .and_then(|s| s.profile_face)
                    .ok_or("Sketch has no profile")?;
                g.cad_set_sketch_visible(id, !g.is_face_visible(face))
                    .map_err(|e| e.to_string())?;
                return Ok(Vec::new());
            }
            34 => g.cad_pocket(
                c.face.ok_or("Select a sketch on the solid's planar face")?,
                c.size[2],
                c.through_all,
                c.tolerance,
            ),
            1 | 24 => g.cad_pad(
                c.face.ok_or("Select a closed planar sketch profile")?,
                c.size[2],
                c.action == 24,
                c.tolerance,
            ),
            2 => g.cad_revolve(
                c.face.ok_or("Select a closed planar sketch profile")?,
                c.position,
                c.axis,
                c.angle,
            ),
            3 | 6 => g.cad_transform(
                source()?,
                CadTransform::Translate(c.position),
                c.action == 6,
            ),
            4 => g.cad_transform(
                source()?,
                CadTransform::Rotate {
                    pivot: c.position,
                    axis: c.axis,
                    degrees: c.angle,
                },
                false,
            ),
            5 => g.cad_transform(
                source()?,
                CadTransform::Scale {
                    pivot: c.position,
                    factor: c.factor,
                },
                false,
            ),
            7 | 8 => {
                return g
                    .cad_pattern(
                        source()?,
                        c.count,
                        c.position,
                        if c.action == 8 { c.angle } else { 0.0 },
                    )
                    .map_err(|e| e.to_string())
            }
            9..=11 => g.cad_boolean(
                source()?,
                c.target.ok_or("Select a tool body")?,
                match c.action {
                    9 => CadBoolean::Union,
                    10 => CadBoolean::Difference,
                    _ => CadBoolean::Intersection,
                },
                c.tolerance,
            ),
            12 => {
                let id = source()?;
                let visible = !g.cad_body(id).ok_or("Select a CAD body")?.visible;
                g.cad_set_visible(id, visible).map(|_| id)
            }
            13 => {
                let id = source()?;
                g.cad_rename(id, &c.name).map(|_| id)
            }
            14 => {
                g.cad_delete_feature_body(source()?)
                    .map_err(|e| e.to_string())?;
                return Ok(Vec::new());
            }
            15..=17 => {
                let id = source()?;
                let data = match c.action {
                    15 => g.cad_export_step(id),
                    16 => g.cad_export_stl(id),
                    _ => g.cad_export_brep(id),
                }
                .map_err(|e| e.to_string())?;
                let path = c.path.as_ref().ok_or("Choose an export file")?;
                let mut f =
                    tempfile::NamedTempFile::new_in(path.parent().unwrap_or(Path::new(".")))
                        .map_err(|e| e.to_string())?;
                use std::io::Write;
                f.write_all(data.as_bytes()).map_err(|e| e.to_string())?;
                f.persist(path).map_err(|e| e.to_string())?;
                return Ok(vec![id]);
            }
            20 => g.cad_loft(
                c.face.ok_or("Select two planar profiles")?,
                c.second_face.ok_or("Select two planar profiles")?,
            ),
            18 | 19 => {
                if std::fs::metadata(c.path.as_ref().ok_or("Choose a B-rep file")?)
                    .map_err(|e| e.to_string())?
                    .len()
                    > 64 * 1024 * 1024
                {
                    return Err("B-rep file exceeds 64 MiB".into());
                }
                let data = std::fs::read_to_string(c.path.as_ref().ok_or("Choose a B-rep file")?)
                    .map_err(|e| e.to_string())?;
                if c.action == 19 {
                    g.cad_import_step(&data, &c.name)
                } else {
                    g.cad_import_brep(&data, &c.name)
                }
            }
            _ => return Err("Unknown CAD operation".into()),
        }
        .map_err(|e| e.to_string())?;
        Ok(vec![one])
    })()?;
    let changed = !matches!(c.action, 15..=17);
    g.validate().map_err(|e| e.to_string())?;
    let message = if matches!(c.action, 29 | 31) {
        format!(
            "Dimensions applied · {} Pad bodies rebuilt · dependent sketches updated",
            result.len()
        )
    } else if changed {
        format!(
            "Operation completed · {} CAD bodies",
            g.cad_bodies().count()
        )
    } else {
        format!("Exported {}", c.path.unwrap().display())
    };
    Ok((g, result, changed, message))
}
pub(super) fn poll(ui: &MainWindow, state: &mut AppState) -> bool {
    if state.sketch_edit.is_some() && super::sketch_edit::valid(state).is_none() {
        state.sketch_edit = None;
        ui.set_cad_sketch_editing(false);
        ui.set_cad_edit_sketch_id(0);
        ui.set_cad_feedback(
            "The source document changed. The uncommitted sketch draft was closed.".into(),
        );
        return true;
    }

    let Some(job) = state.cad_job.as_ref() else {
        return false;
    };
    let result = match job.result.try_recv() {
        Ok(v) => v,
        Err(TryRecvError::Empty) => return false,
        Err(_) => Err("CAD worker stopped unexpectedly".into()),
    };
    let job = state.cad_job.take().unwrap();
    ui.set_cad_busy(false);
    if job.cancelled {
        ui.set_cad_feedback("CAD result discarded; source document preserved.".into());
        return true;
    }
    match result {
        Ok((geometry, selection, changed, message)) => {
            if state.workbench.geometry() != &job.source
                || state.workspace_path != job.workspace
                || state.workbench_project.project_id != job.project_id
            {
                ui.set_cad_feedback(
                    "Document changed while CAD was running; result was discarded.".into(),
                );
                return true;
            }
            if job.preview {
                state.cad_preview = Some(CadPreview {
                    source: job.source,
                    workspace: job.workspace,
                    project_id: job.project_id,
                    geometry,
                    selection,
                    message,
                });
                state.show_geometry_3d = true;
                state.show_mesh = false;
                ui.set_geometry_cad_view_3d(true);
                ui.set_cad_feedback(
                    "Preview ready. Apply to commit, or Cancel to preserve the source model."
                        .into(),
                );
                return true;
            }
            if changed {
                state.geometry_editor.record_snapshot(&job.source);
                *state.workbench.geometry_mut() = geometry;
                state.workbench.geometry_changed();
                state.cad_measure_points.clear();
                ui.set_cad_measure_active(false);
                if state.workbench.geometry().cad_bodies().next().is_some()
                    && state.workbench.mesh_dimension() == MeshDimension::TwoD
                {
                    let (global, min, max) = state.workbench.mesh_sizes();
                    let order = state.workbench.element_order();
                    if let Err(error) = state.workbench.set_mesh_configuration(
                        MeshDimension::ThreeD,
                        global,
                        min,
                        max,
                        order,
                    ) {
                        state.log(error.to_string());
                    }
                    ui.set_mesh_dimension_index(1);
                }
                state.mark_workbench_dirty();
                state.sketch_edit = None;
                let edited_profile = state
                    .workbench
                    .geometry()
                    .sketches()
                    .find(|s| s.id.get() == ui.get_cad_edit_sketch_id().max(0) as u64)
                    .and_then(|s| s.profile_face);
                ui.set_cad_sketch_editing(false);
                ui.set_cad_editing_pad(false);
                ui.set_cad_edit_sketch_id(0);
                state.wb_selected_targets = selection
                    .into_iter()
                    .map(GeometrySelectionTarget::Body)
                    .collect();
                if state.wb_selected_targets.is_empty() {
                    if let Some(face) = edited_profile {
                        state
                            .wb_selected_targets
                            .push(GeometrySelectionTarget::Face(face));
                    }
                }
                state.geometry_editor.selection = state.wb_selected_targets.clone();
                state.selected_tree = state
                    .wb_selected_targets
                    .last()
                    .copied()
                    .and_then(tree_selection_for_target);
                state.show_geometry_3d = true;
                state.show_mesh = false;
                ui.set_geometry_cad_view_3d(true);
                rebuild_tree_rows(state);
            }
            state.log(&message);
            ui.set_cad_feedback(message.into());
        }
        Err(e) => {
            state.log(format!("CAD: {e}"));
            ui.set_cad_feedback(e.into());
        }
    }
    true
}
pub(super) fn refresh(ui: &MainWindow, state: &AppState) {
    super::sketch_edit::refresh(ui, state);
    ui.set_cad_preview_active(preview(state).is_some());
    ui.set_cad_preview_label(
        preview(state)
            .and_then(|p| p.selection.first().and_then(|id| p.geometry.cad_body(*id)))
            .map(|b| b.operation.clone())
            .unwrap_or_default()
            .into(),
    );
    ui.set_cad_orientation_image(super::render::render_orientation_overlay(
        state.geometry_yaw,
        state.geometry_pitch,
    ));
    ui.set_cad_can_loft(
        state
            .wb_selected_targets
            .iter()
            .filter(|t| matches!(t, GeometrySelectionTarget::Face(_)))
            .count()
            == 2,
    );
    let g = state.workbench.geometry();
    ui.set_cad_can_pocket(selected_extrusion_face(state).ok().is_some_and(|face| {
        g.sketches().any(|s| {
            s.profile_face == Some(face) && s.host_face.and_then(|host| g.cad_owner(host)).is_some()
        })
    }));
    ui.set_cad_features(ModelRc::new(VecModel::from(
        g.extrude_features()
            .map(|f| {
                SharedString::from(format!(
                    "Pad {} · Sketch {} · Body {}",
                    f.id.get(),
                    f.sketch.get(),
                    f.body.get()
                ))
            })
            .collect::<Vec<_>>(),
    )));
    let selected_sketch = state
        .selected_tree
        .filter(|t| t.kind == TREE_KIND_SKETCH)
        .and_then(|t| g.sketches().position(|s| s.id.get() == t.payload as u64))
        .or_else(|| {
            state.wb_selected_targets.iter().find_map(|t| {
                if let GeometrySelectionTarget::Face(face) = t {
                    g.sketches().position(|s| s.profile_face == Some(*face))
                } else {
                    None
                }
            })
        });
    if let Some(index) = selected_sketch {
        ui.set_cad_sketch_index(index as i32);
    }
    if let Some(selection) = state
        .selected_tree
        .filter(|t| t.kind == TREE_KIND_EXTRUDE_FEATURE)
    {
        if let Some(index) = g
            .extrude_features()
            .position(|f| f.id.get() == selection.payload as u64)
        {
            ui.set_cad_feature_index(index as i32);
        }
    }
    ui.set_cad_sketches(ModelRc::new(VecModel::from(
        g.sketches()
            .map(|s| {
                let profile = match &s.profile {
                    Some(flursys::CadSketchProfile::Circle { radius }) => {
                        format!("Circle · R {radius:.4}")
                    }
                    Some(flursys::CadSketchProfile::Rectangle { width, height }) => {
                        format!("Rectangle · {width:.4} × {height:.4}")
                    }
                    None => {
                        if s.profile_face.is_some() {
                            "Closed profile".into()
                        } else {
                            "Empty sketch".into()
                        }
                    }
                };
                let plane = match s.plane {
                    flursys::CadSketchPlane::Xy => "XY",
                    flursys::CadSketchPlane::Yz => "YZ",
                    flursys::CadSketchPlane::Zx => "ZX",
                    flursys::CadSketchPlane::Face => "Face",
                };
                format!(
                    "{} Sketch {} · {} · {}",
                    if s.profile_face.is_some_and(|f| g.is_face_visible(f)) {
                        "●"
                    } else {
                        "○"
                    },
                    s.id.get(),
                    profile,
                    plane
                )
                .into()
            })
            .collect::<Vec<SharedString>>(),
    )));
    ui.set_cad_history(
        g.extrude_features()
            .map(|f| {
                format!(
                    "Pad {} ← Sketch {} · depth {} → Body {}",
                    f.id.get(),
                    f.sketch.get(),
                    f.distance.map_or("?".into(), |v| v.to_string()),
                    f.body.get()
                )
            })
            .chain(g.pocket_features().map(|f| {
                format!(
                    "Pocket {} ← Sketch {} · {} → Body {}",
                    f.id.get(),
                    f.sketch.get(),
                    if f.through_all {
                        "Through all".into()
                    } else {
                        format!("Depth {}", f.depth)
                    },
                    f.body.get()
                )
            }))
            .collect::<Vec<_>>()
            .join("\n")
            .into(),
    );
    let bodies = g.cad_bodies().collect::<Vec<_>>();
    ui.set_cad_bodies(ModelRc::new(VecModel::from(
        bodies
            .iter()
            .map(|(id, b)| {
                SharedString::from(format!(
                    "{} {} · #{}",
                    if b.visible { "●" } else { "○" },
                    b.name,
                    id.get()
                ))
            })
            .collect::<Vec<_>>(),
    )));
    if let Some(selected) = state
        .wb_selected_targets
        .iter()
        .find_map(|target| match target {
            GeometrySelectionTarget::Body(id) if g.cad_body(*id).is_some() => Some(*id),
            GeometrySelectionTarget::Face(id) => g.cad_owner(*id),
            _ => None,
        })
    {
        if let Some(index) = bodies.iter().position(|(id, _)| *id == selected) {
            ui.set_cad_source(index as i32);
        }
    }
    let body = bodies.get(ui.get_cad_source().max(0) as usize);
    ui.set_cad_inspector(
        body.map(|(id, b)| {
            let triangles = b
                .face_meshes
                .iter()
                .map(|m| m.triangles.len())
                .sum::<usize>();
            let mut area = 0.0;
            let mut volume = 0.0;
            for m in b.face_meshes.iter() {
                for t in &m.triangles {
                    area += (t[1] - t[0]).cross(t[2] - t[0]).norm() * 0.5;
                    volume += t[0].dot(t[1].cross(t[2])) / 6.0;
                }
            }
            SharedString::from(format!(
                "{} · Body {}\n{} faces · {} display triangles\nArea ≈ {:.5} · Volume ≈ {:.5}\n{}",
                b.name,
                id.get(),
                b.face_meshes.len(),
                triangles,
                area,
                volume.abs(),
                b.operation
            ))
        })
        .unwrap_or_else(|| "Create a solid or select a closed sketch profile.".into()),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use slint::platform::{
        software_renderer::{MinimalSoftwareWindow, RepaintBufferType},
        Platform, WindowAdapter,
    };
    struct SoftwarePlatform(Rc<MinimalSoftwareWindow>);
    impl Platform for SoftwarePlatform {
        fn create_window_adapter(&self) -> Result<Rc<dyn WindowAdapter>, slint::PlatformError> {
            Ok(self.0.clone())
        }
        fn duration_since_start(&self) -> Duration {
            Duration::from_secs(1)
        }
    }
    #[test]
    fn cad_ui_builds_real_brep_and_restores_it_through_undo_redo() {
        let window = MinimalSoftwareWindow::new(RepaintBufferType::NewBuffer);
        slint::platform::set_platform(Box::new(SoftwarePlatform(window.clone()))).unwrap();
        let ui = MainWindow::new().unwrap();
        window.set_size(slint::PhysicalSize::new(1440, 1000));
        let state = Rc::new(RefCell::new(AppState::new()));
        super::super::bind_callbacks(&ui, &state);
        bind(&ui, &state);
        ui.set_cad_name("CAD verification".into());
        ui.set_cad_size_x("10".into());
        ui.set_cad_size_y("8".into());
        ui.set_cad_size_z("6".into());
        ui.invoke_cad_command(0);
        assert!(state.borrow().cad_job.is_some());
        let deadline = std::time::Instant::now() + Duration::from_secs(15);
        while state.borrow().cad_job.is_some() {
            poll(&ui, &mut state.borrow_mut());
            assert!(std::time::Instant::now() < deadline, "CAD worker timeout");
            std::thread::sleep(Duration::from_millis(1));
        }
        assert_eq!(state.borrow().workbench.geometry().cad_bodies().count(), 1);
        assert_eq!(
            state.borrow().workbench.mesh_dimension(),
            MeshDimension::ThreeD
        );
        super::super::refresh_ui(&ui, &state.borrow());
        ui.show().unwrap();
        // Real software-rendered snapshots also exercise the Slint layout bindings.
        for (tab, label) in [
            (1, "solid"),
            (2, "transform"),
            (3, "boolean"),
            (4, "exchange"),
        ] {
            ui.set_cad_workbench(tab);
            window.request_redraw();
            let mut pixels = vec![slint::Rgb8Pixel::default(); 1440 * 1000];
            window.draw_if_needed(|renderer| {
                renderer.render(&mut pixels, 1440);
            });
            let mut ppm = format!("P6\n1440 1000\n255\n").into_bytes();
            for p in pixels {
                ppm.extend([p.r, p.g, p.b]);
            }
            std::fs::write(format!("/tmp/flursys-cad-{label}.ppm"), ppm).unwrap();
        }
        // A concurrent document edit must never be overwritten by a stale worker.
        let id = state
            .borrow()
            .workbench
            .geometry()
            .cad_bodies()
            .next()
            .unwrap()
            .0;
        ui.invoke_cad_command(0);
        state
            .borrow_mut()
            .workbench
            .geometry_mut()
            .cad_rename(id, "Concurrent edit")
            .unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(15);
        while state.borrow().cad_job.is_some() {
            poll(&ui, &mut state.borrow_mut());
            assert!(std::time::Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        assert_eq!(state.borrow().workbench.geometry().cad_bodies().count(), 1);
        assert_eq!(
            state
                .borrow()
                .workbench
                .geometry()
                .cad_body(id)
                .unwrap()
                .name,
            "Concurrent edit"
        );
        ui.invoke_cad_command(0);
        ui.invoke_cad_command(-1);
        assert!(state.borrow().cad_job.as_ref().unwrap().cancelled);
        let deadline = std::time::Instant::now() + Duration::from_secs(15);
        while state.borrow().cad_job.is_some() {
            poll(&ui, &mut state.borrow_mut());
            assert!(std::time::Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        assert_eq!(state.borrow().workbench.geometry().cad_bodies().count(), 1);
        ui.invoke_geometry_undo();
        assert_eq!(state.borrow().workbench.geometry().cad_bodies().count(), 0);
        ui.invoke_geometry_redo();
        assert_eq!(state.borrow().workbench.geometry().cad_bodies().count(), 1);
        ui.hide().unwrap();
    }
}

fn sketch_command(ui: &MainWindow, state: &mut AppState, action: i32) -> Result<String, String> {
    if action == 39 {
        return super::sketch_edit::begin(ui, state);
    }
    if action == 40 {
        super::sketch_edit::cancel(ui, state);
        return Ok("Sketch edit cancelled. Source model unchanged.".into());
    }

    if action == 28 {
        state.sketch_edit = None;
        let id = state
            .workbench
            .geometry()
            .sketches()
            .nth(ui.get_cad_sketch_index().max(0) as usize)
            .map(|s| s.id)
            .ok_or("Select a sketch")?;
        let plane = state.workbench.geometry().sketch(id).unwrap().plane;
        ui.set_cad_sketch_plane(match plane {
            flursys::CadSketchPlane::Xy => 0,
            flursys::CadSketchPlane::Yz => 1,
            flursys::CadSketchPlane::Zx => 2,
            flursys::CadSketchPlane::Face => 3,
        });
        ui.set_cad_editing_pad(false);
        let (x, y, profile) = state
            .workbench
            .geometry()
            .cad_sketch_parameters(id)
            .map_err(|e| e.to_string())?;
        ui.set_cad_pos_x(x.to_string().into());
        ui.set_cad_pos_y(y.to_string().into());
        match profile {
            flursys::CadSketchProfile::Circle { radius } => {
                ui.set_cad_sketch_kind(0);
                ui.set_cad_size_x(radius.to_string().into());
            }
            flursys::CadSketchProfile::Rectangle { width, height } => {
                ui.set_cad_sketch_kind(1);
                ui.set_cad_size_x(width.to_string().into());
                ui.set_cad_size_y(height.to_string().into());
            }
        }
        ui.set_cad_edit_sketch_id(id.get() as i32);
        ui.set_cad_sketch_editing(true);
        return Ok(
            "Edit dimensions, then Apply & rebuild. Cancel leaves the document unchanged.".into(),
        );
    }
    if action == 30 {
        let depth = state
            .workbench
            .geometry()
            .extrude_features()
            .nth(ui.get_cad_feature_index().max(0) as usize)
            .and_then(|f| f.distance)
            .ok_or("Selected feature has no saved Pad depth")?;
        ui.set_cad_size_z(depth.to_string().into());
        ui.set_cad_editing_pad(true);
        return Ok("Pad depth loaded. Change it and apply to rebuild dependent Pads.".into());
    }

    if action == 23 && super::sketch_edit::valid(state).is_some() {
        return Err("Finish or cancel the sketch edit before measuring".into());
    }
    if matches!(action, 23 | 32 | 33) {
        return Ok(super::measure::command(ui, state, action));
    }
    if action == 27 {
        state.face_sketch_host = None;
        state.face_sketch_start = None;
        state.face_sketch_cursor = None;
        state.geometry_editor.cancel();
        state.show_geometry_3d = true;
        ui.set_geometry_cad_view_3d(true);
        return Ok("Sketch cancelled. Document preserved.".into());
    }

    if action == 26 {
        begin_face_sketch(state)?;
        state.geometry_editor.set_tool(GeometryTool::Circle);
        state.show_geometry_3d = false;
        ui.set_geometry_cad_view_3d(false);
        return Ok(
            "Circle sketch: click the center, then the radius point (or drag). Esc cancels.".into(),
        );
    }

    if action == 22 {
        let face = state
            .workbench
            .geometry()
            .sketches()
            .nth(ui.get_cad_sketch_index().max(0) as usize)
            .and_then(|s| s.profile_face)
            .ok_or("Select a completed sketch")?;
        select_tree_geometry_target(state, GeometrySelectionTarget::Face(face), false);
        return Ok("Sketch selected. Set Pad depth, then apply Pad.".into());
    }
    let mut next = state.workbench.geometry().clone();
    let sketch = if ui.get_cad_sketch_plane() == 3 {
        let face = state
            .wb_selected_targets
            .iter()
            .find_map(|s| {
                if let GeometrySelectionTarget::Face(f) = s {
                    Some(*f)
                } else {
                    None
                }
            })
            .ok_or("Select a planar solid face first")?;
        next.create_sketch_on_face(face)
    } else {
        next.create_sketch_on_plane(match ui.get_cad_sketch_plane() {
            1 => flursys::CadSketchPlane::Yz,
            2 => flursys::CadSketchPlane::Zx,
            _ => flursys::CadSketchPlane::Xy,
        })
    }
    .map_err(|e| e.to_string())?;
    let x = number(ui.get_cad_pos_x(), "Local X")?;
    let y = number(ui.get_cad_pos_y(), "Local Y")?;
    let width = number(ui.get_cad_size_x(), "Radius / width")?;
    let face = if ui.get_cad_sketch_kind() == 0 {
        next.materialize_sketch_circle_at(sketch.id, x, y, width)
    } else {
        next.materialize_sketch_rectangle_at(
            sketch.id,
            x,
            y,
            width,
            number(ui.get_cad_size_y(), "Height")?,
        )
    }
    .map_err(|e| e.to_string())?;
    state
        .geometry_editor
        .record_snapshot(state.workbench.geometry());
    *state.workbench.geometry_mut() = next;
    state.workbench.geometry_changed();
    state.mark_workbench_dirty();
    select_tree_geometry_target(state, GeometrySelectionTarget::Face(face), false);
    state.show_geometry_3d = true;
    state.show_mesh = false;
    ui.set_geometry_cad_view_3d(true);
    ui.set_cad_sketch_editing(false);
    Ok(format!(
        "Sketch {} finished. Set the depth and apply Pad.",
        sketch.id.get()
    ))
}

pub(super) fn measure_selection(state: &AppState) -> Result<String, String> {
    let g = state.workbench.geometry();
    let selected = &state.wb_selected_targets;
    if let [GeometrySelectionTarget::Face(face)] = selected.as_slice() {
        if let Some(profile) = g
            .sketches()
            .find(|s| s.profile_face == Some(*face))
            .and_then(|s| s.profile.as_ref())
        {
            return Ok(match profile {
                flursys::CadSketchProfile::Circle { radius } => format!("Radius {:.6} · Diameter {:.6} · Area {:.6} · Circumference {:.6} (model units)", radius, 2.0*radius, std::f64::consts::PI*radius*radius, std::f64::consts::TAU*radius),
                flursys::CadSketchProfile::Rectangle { width, height } => format!("Width {:.6} · Height {:.6} · Area {:.6} · Perimeter {:.6} (model units)", width, height, width*height, 2.0*(width+height)),
            });
        }
    }

    if let [GeometrySelectionTarget::Edge(edge)] = selected.as_slice() {
        let edge = g.edge(*edge).ok_or("Missing edge")?;
        return match edge.geometry {
            flursys::EdgeGeometry::Line { start, end } => Ok(format!(
                "Length {:.6} (model units)",
                (g.vertex(end).ok_or("Missing vertex")?.position
                    - g.vertex(start).ok_or("Missing vertex")?.position)
                    .norm()
            )),
            flursys::EdgeGeometry::CircularArc { start, center, end } => {
                let c = g.vertex(center).ok_or("Missing center")?.position;
                let a = g.vertex(start).ok_or("Missing vertex")?.position - c;
                let b = g.vertex(end).ok_or("Missing vertex")?.position - c;
                let radius = a.norm();
                let angle = (a.dot(b) / (radius * b.norm())).clamp(-1.0, 1.0).acos();
                Ok(format!(
                    "Radius {:.6} · Diameter {:.6} · Minor arc length {:.6} (model units)",
                    radius,
                    2.0 * radius,
                    radius * angle
                ))
            }
        };
    }
    if let [GeometrySelectionTarget::Face(face)] = selected.as_slice() {
        let area = g
            .renderable_faces()
            .iter()
            .filter(|f| f.face == *face)
            .map(|f| {
                let polygon_area = |v: &[Vec3]| {
                    if v.len() < 3 {
                        0.0
                    } else {
                        (1..v.len() - 1)
                            .map(|i| (v[i] - v[0]).cross(v[i + 1] - v[0]))
                            .fold(Vec3::new(0.0, 0.0, 0.0), |sum, v| sum + v)
                            .norm()
                            * 0.5
                    }
                };
                polygon_area(&f.vertices) - f.holes.iter().map(|v| polygon_area(v)).sum::<f64>()
            })
            .sum::<f64>();
        return Ok(format!("Face area ≈ {:.6} (model units²). Pick another planar face for distance / angle; use Point distance for surface points.",area));
    }
    if let [GeometrySelectionTarget::Vertex(a), GeometrySelectionTarget::Vertex(b)] =
        selected.as_slice()
    {
        let a = g.vertex(*a).ok_or("Missing vertex")?.position;
        let b = g.vertex(*b).ok_or("Missing vertex")?.position;
        let d = b - a;
        return Ok(format!(
            "Distance {:.6} · ΔX {:.6} · ΔY {:.6} · ΔZ {:.6} (model units)",
            d.norm(),
            d.x,
            d.y,
            d.z
        ));
    }
    if let [GeometrySelectionTarget::Face(a), GeometrySelectionTarget::Face(b)] =
        selected.as_slice()
    {
        let a = g.sketch_frame_for_face(*a).map_err(|e| e.to_string())?;
        let b = g.sketch_frame_for_face(*b).map_err(|e| e.to_string())?;
        let dot = a.normal.dot(b.normal).abs().clamp(0.0, 1.0);
        let angle = dot.acos().to_degrees();
        return Ok(if dot > 1.0 - 1e-10 {
            format!(
                "Parallel-plane distance {:.6} (model units)",
                (b.origin - a.origin).dot(a.normal).abs()
            )
        } else {
            format!("Plane angle {:.6}°", angle)
        });
    }
    Err("Pick a second vertex for distance, or two planar faces for distance / angle.".into())
}
