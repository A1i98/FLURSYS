use flursys::workbench::{
    self, ResultDataset, ResultFieldKind as Field, StreamlineDirection, StreamlineField,
    StreamlineOptions,
};
use flursys::{MeshDimension, Vec3};
use serde_json::{json, Value};
use std::{fs::OpenOptions, io::Write, path::Path};

fn field(s: &str) -> Result<Field, String> {
    Ok(match s {
        "pressure" => Field::Pressure,
        "speed" => Field::VelocityMagnitude,
        "velocity_x" => Field::VelocityX,
        "velocity_y" => Field::VelocityY,
        "velocity_z" => Field::VelocityZ,
        "density" => Field::Density,
        "absolute_pressure" => Field::AbsolutePressure,
        "temperature" => Field::Temperature,
        "mach" => Field::Mach,
        "vorticity" => Field::VorticityMagnitude,
        "divergence" => Field::VelocityDivergence,
        "q_criterion" => Field::QCriterion,
        "strain_rate" => Field::StrainRate,
        "turbulent_k" => Field::TurbulentK,
        "omega" => Field::SpecificDissipation,
        "turbulent_viscosity" => Field::TurbulentViscosity,
        _ => return Err(format!("Unknown field: {s}")),
    })
}
fn point(v: &Value) -> Result<Vec3, String> {
    let x = v
        .as_array()
        .filter(|x| x.len() == 3)
        .ok_or("Point must have three coordinates")?;
    let get = |i: usize| {
        x[i].as_f64()
            .filter(|v| v.is_finite())
            .ok_or("Point coordinates must be finite numbers")
    };
    Ok(Vec3::new(get(0)?, get(1)?, get(2)?))
}
fn dataset(dir: &Path, id: &str) -> Result<ResultDataset, String> {
    let p = flursys::load_workspace(dir).map_err(|e| e.to_string())?;
    workbench::load_workspace_result(dir, &p, id).map_err(|e| e.to_string())
}
fn sample(d: &ResultDataset, locator: &StreamlineField, p: Vec3, fields: &[Field]) -> Value {
    let Some(i) = locator.locate_cell(p) else {
        return json!({"point":p,"inside":false});
    };
    let mut vals = serde_json::Map::new();
    for f in fields {
        let value = d.scalar_values(*f).get(i).copied();
        vals.insert(f.label().into(), json!(value));
    }
    json!({"point":p,"inside":true,"cell":i,"values":vals,"velocity_m_s":d.velocity()[i],"sampling":"cell-centred, containing cell"})
}
fn save(path: &Path, data: &str) -> Result<(), String> {
    let mut f = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .map_err(|e| e.to_string())?;
    f.write_all(data.as_bytes()).map_err(|e| e.to_string())
}
pub fn run(name: &str, a: &Value, dir: &Path, output: Option<&Path>) -> Result<Value, String> {
    let id = a["run_id"].as_str().ok_or("Missing run_id")?;
    let d = dataset(dir, id)?;
    match name {
        "result_fields" => {
            let names = [
                "pressure",
                "speed",
                "velocity_x",
                "velocity_y",
                "velocity_z",
                "density",
                "absolute_pressure",
                "temperature",
                "mach",
                "vorticity",
                "divergence",
                "q_criterion",
                "strain_rate",
                "turbulent_k",
                "omega",
                "turbulent_viscosity",
            ];
            let fields:Vec<_>=names.iter().filter_map(|n|{let f=field(n).ok()?;let range=d.scalar_range(f).ok()?;Some(json!({"name":n,"label":f.label(),"min":range.0,"max":range.1,"volume_mean":d.volume_mean(f).ok()}))}).collect();
            Ok(json!({"run_id":id,"cells":d.mesh().cell_count(),"fields":fields}))
        }
        "result_probe" | "result_profile" => {
            let locator = StreamlineField::from_dataset(&d).map_err(|e| e.to_string())?;
            let names = a
                .get("fields")
                .and_then(Value::as_array)
                .ok_or("fields must be an array")?;
            if names.is_empty() || names.len() > 16 {
                return Err("Select 1 to 16 fields".into());
            }
            let fields = names
                .iter()
                .map(|v| field(v.as_str().ok_or("Field names must be strings")?))
                .collect::<Result<Vec<_>, _>>()?;
            for f in &fields {
                if d.scalar_values(*f).is_empty() {
                    return Err(format!("Field unavailable: {}", f.label()));
                }
            }
            let points = if name == "result_probe" {
                vec![point(&a["point"])?]
            } else {
                let p = point(&a["start"])?;
                let q = point(&a["end"])?;
                let n = a["count"]
                    .as_u64()
                    .filter(|n| *n >= 2 && *n <= 1000)
                    .ok_or("count must be 2..1000")? as usize;
                (0..n)
                    .map(|i| p + (q - p) * (i as f64 / (n - 1) as f64))
                    .collect()
            };
            Ok(
                json!({"run_id":id,"samples":points.into_iter().map(|p|sample(&d,&locator,p,&fields)).collect::<Vec<_>>() }),
            )
        }
        "result_streamline" => {
            let locator = StreamlineField::from_dataset(&d).map_err(|e| e.to_string())?;
            let seed = point(&a["seed"])?;
            let step = a["step_size"].as_f64().ok_or("Missing step_size")?;
            let length = a["max_length"].as_f64().ok_or("Missing max_length")?;
            let steps = a
                .get("max_steps")
                .and_then(Value::as_u64)
                .unwrap_or(1000)
                .min(10000) as usize;
            let direction = if a.get("direction").and_then(Value::as_str) == Some("backward") {
                StreamlineDirection::Backward
            } else {
                StreamlineDirection::Forward
            };
            let line = locator
                .integrate(
                    seed,
                    direction,
                    StreamlineOptions {
                        step_size: step,
                        max_length: length,
                        max_steps: steps,
                        stagnation_speed: 1e-12,
                    },
                )
                .map_err(|e| e.to_string())?;
            Ok(
                json!({"run_id":id,"points":line.points,"units":"m","interpretation":"2D streamline through a steady or saved velocity field; not a particle trajectory"}),
            )
        }
        "result_section_svg" => {
            use workbench::plane_monitor::{FlowPlaneMonitor, PlaneCell, PlaneDisplay};
            let plane: FlowPlaneMonitor =
                serde_json::from_value(a["plane"].clone()).map_err(|e| e.to_string())?;
            let style: PlaneDisplay =
                serde_json::from_value(a.get("display").cloned().unwrap_or(json!({})))
                    .map_err(|e| e.to_string())?;
            style.validate()?;
            let cells = if d.mesh().dimension() == MeshDimension::ThreeD {
                workbench::plane_monitor::intersect(d.mesh(), &plane)?
            } else {
                plane.validate()?;
                let mesh = d.mesh();
                let normal = plane.normal();
                if normal.x.abs() > 1e-7
                    || normal.y.abs() > 1e-7
                    || mesh
                        .points()
                        .iter()
                        .any(|p| (p.position - plane.origin).dot(normal).abs() > 1e-8)
                {
                    return Err(
                        "For a 2D dataset the display plane must coincide with its XY mesh".into(),
                    );
                }
                mesh.cells()
                    .iter()
                    .enumerate()
                    .map(|(i, c)| {
                        let mut ids = Vec::new();
                        for face in &c.faces {
                            for &v in &mesh.faces()[*face].vertices {
                                if !ids.contains(&v) {
                                    ids.push(v)
                                }
                            }
                        }
                        let mut poly: Vec<_> =
                            ids.into_iter().map(|v| mesh.points()[v].position).collect();
                        poly.sort_by(|a, b| {
                            (a.y - c.center.y)
                                .atan2(a.x - c.center.x)
                                .total_cmp(&(b.y - c.center.y).atan2(b.x - c.center.x))
                        });
                        PlaneCell {
                            cell: i,
                            point: c.center,
                            polygon: poly,
                        }
                    })
                    .collect()
            };
            if cells.is_empty() {
                return Err("Plane does not intersect the mesh".into());
            }
            let snapshot =
                workbench::plane_monitor::sample(&plane, &cells, 0, d.velocity(), d.pressure())?;
            let svg = workbench::plane_monitor::section_svg(&snapshot, &style);
            let path = output.ok_or("Missing output")?;
            save(path, &svg)?;
            Ok(json!({"output":path,"samples":cells.len(),"format":"svg","run_id":id}))
        }
        _ => Err("Unknown result tool".into()),
    }
}
pub fn history_plot(dir: &Path, id: &str, path: &Path) -> Result<Value, String> {
    let p = flursys::load_workspace(dir).map_err(|e| e.to_string())?;
    let run = p.runs.iter().find(|r| r.id == id).ok_or("Unknown run")?;
    let report = flursys::headless::report(dir, run)?;
    let histories = report["convergence_histories"]
        .as_object()
        .ok_or("Convergence history is unavailable")?;
    let mut series = Vec::new();
    for key in ["momentum", "velocity", "pressure_linear"] {
        if let Some(v) = histories.get(key).and_then(Value::as_array) {
            let points = v
                .iter()
                .enumerate()
                .map(|(i, y)| ((i + 1) as f64, y.as_f64()))
                .collect();
            series.push(workbench::series_plot::Series {
                name: key.into(),
                points,
            });
        }
    }
    if series.is_empty() {
        return Err("Convergence history is empty".into());
    }
    let svg = workbench::series_plot::svg(
        &series,
        "Convergence history (segment index)",
        "Saved sample index",
        "Monitor value",
    )?;
    save(path, &svg)?;
    Ok(
        json!({"output":path,"run_id":id,"note":"Sample index within the stored history; series use different definitions"}),
    )
}
