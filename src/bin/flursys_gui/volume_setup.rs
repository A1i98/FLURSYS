//! Atomic editor for fixed-mesh volumetric momentum models.
use super::*;
use flursys::volume_models::{CellRegion, PorousResistance, RotatingFrame, VolumeModels};
pub(super) fn bind(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_open_volume_models(move || {
        let Some(ui) = weak.upgrade() else { return };
        let state = shared.borrow();
        if state.solving || state.meshing {
            return;
        }
        let models = state.workbench.solver_options().volume_models;
        let r = models.rotating_frame.unwrap_or(RotatingFrame {
            angular_velocity: Vec3::new(0.0, 0.0, 1.0),
            origin: Vec3::ZERO,
        });
        let p = models.resistance.unwrap_or(PorousResistance {
            region: CellRegion::All,
            darcy: 1e6,
            forchheimer: 0.0,
        });
        let (a, b) = match p.region {
            CellRegion::All => (Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0)),
            CellRegion::Box { minimum, maximum } => (minimum, maximum),
        };
        let value = |v: f64| SharedString::from(v.to_string());
        ui.set_volume_model_draft(VolumeModelDraft {
            rotation_enabled: models.rotating_frame.is_some(),
            resistance_enabled: models.resistance.is_some(),
            bounded: matches!(p.region, CellRegion::Box { .. }),
            omega_x: value(r.angular_velocity.x),
            omega_y: value(r.angular_velocity.y),
            omega_z: value(r.angular_velocity.z),
            origin_x: value(r.origin.x),
            origin_y: value(r.origin.y),
            origin_z: value(r.origin.z),
            minimum_x: value(a.x),
            minimum_y: value(a.y),
            minimum_z: value(a.z),
            maximum_x: value(b.x),
            maximum_y: value(b.y),
            maximum_z: value(b.z),
            darcy: value(p.darcy),
            forchheimer: value(p.forchheimer),
        });
        ui.set_volume_model_feedback(
            "Disabled models do not change the equations. Box coordinates use metres.".into(),
        );
        ui.set_volume_models_open(true);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_apply_volume_models(move ||{
        let Some(ui)=weak.upgrade()else{return};let mut state=shared.borrow_mut();
        if state.solving||state.meshing{return}
        let result=(||->Result<(),String>{
            let d=ui.get_volume_model_draft();
            let num=|v:SharedString|v.trim().parse::<f64>().ok().filter(|v|v.is_finite()).ok_or_else(||"All enabled model values must be finite numbers".to_string());
            let vector=|x,y,z|->Result<Vec3,String>{Ok(Vec3::new(num(x)?,num(y)?,num(z)?))};
            let models=VolumeModels{
                rotating_frame:if d.rotation_enabled{Some(RotatingFrame{
                    angular_velocity:vector(d.omega_x,d.omega_y,d.omega_z)?,
                    origin:vector(d.origin_x,d.origin_y,d.origin_z)?,
                })}else{None},
                resistance:if d.resistance_enabled{Some(PorousResistance{
                    region:if d.bounded{CellRegion::Box{minimum:vector(d.minimum_x,d.minimum_y,d.minimum_z)?,maximum:vector(d.maximum_x,d.maximum_y,d.maximum_z)?}}else{CellRegion::All},
                    darcy:num(d.darcy)?,forchheimer:num(d.forchheimer)?,
                })}else{None},
            };
            models.validate()?;
            // A ready mesh/case permits full region/dimension checks before mutation.
            if let Ok(mut case)=state.workbench.prepare_case(){
                case.solver.volume_models=models;models.validate_case(&case)?;
            }
            state.workbench.set_volume_models(models).map_err(|e|e.to_string())?;
            state.mark_workbench_dirty();
            state.log(if models.rotating_frame.is_some(){"Volume models applied. All velocities and boundary velocities are relative to the global rotating frame."}else{"Volume momentum models applied; velocity reference is inertial."});
            ui.set_volume_models_open(false);Ok(())
        })();
        if let Err(e)=result{ui.set_volume_model_feedback(e.into());}
    });
}
