use crate::{IncompressibleInitialConditions, IncompressibleSolution, UnstructuredMesh};
use serde::{Deserialize, Serialize};
use std::{
    io::{BufRead, Read, Seek, SeekFrom, Write},
    path::Path,
};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Patch {
    pub name: String,
    pub volume_flux: f64,
    pub mass_flux: f64,
    pub pressure_pa: Option<f64>,
    pub temperature_k: Option<f64>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Sample {
    pub index: usize,
    pub time: Option<f64>,
    pub dt: Option<f64>,
    pub courant: Option<f64>,
    pub mass_imbalance: f64,
    pub relative_mass_imbalance: Option<f64>,
    pub temperature_mean: Option<f64>,
    #[serde(default)]
    pub energy_imbalance_w: Option<f64>,
    #[serde(default)]
    pub relative_energy_imbalance: Option<f64>,
    pub patches: Vec<Patch>,
}
struct Stamp {
    index: usize,
    time: Option<f64>,
    dt: Option<f64>,
    courant: Option<f64>,
}
impl Sample {
    pub fn from_initial(
        mesh: &UnstructuredMesh,
        density: f64,
        initial: &IncompressibleInitialConditions,
        index: usize,
        physical: bool,
    ) -> Option<Self> {
        let IncompressibleInitialConditions::Restart {
            pressure,
            face_flux,
            temperature,
            time,
            last_dt,
            ..
        } = initial
        else {
            return None;
        };
        Self::evaluate(
            mesh,
            density,
            pressure.values(),
            face_flux.values(),
            temperature.as_ref().map(|t| t.values()),
            Stamp {
                index,
                time: physical.then_some(*time),
                dt: if physical { *last_dt } else { None },
                courant: None,
            },
        )
    }
    pub fn from_solution(
        mesh: &UnstructuredMesh,
        density: f64,
        solution: &IncompressibleSolution,
        index: usize,
    ) -> Option<Self> {
        let mut sample = Self::evaluate(
            mesh,
            density,
            solution.pressure.values(),
            solution.face_flux.values(),
            solution.temperature.as_ref().map(|t| t.values()),
            Stamp {
                index,
                time: solution.physical_time,
                dt: solution.last_time_step,
                courant: solution.max_courant,
            },
        )?;
        if let Ok(report) = &solution.budgets {
            if let Some(energy) = &report.energy {
                sample.energy_imbalance_w = Some(energy.imbalance_w);
                sample.relative_energy_imbalance = energy.relative_imbalance;
            }
        }
        Some(sample)
    }
    fn evaluate(
        mesh: &UnstructuredMesh,
        density: f64,
        pressure: &[f64],
        flux: &[f64],
        temperature: Option<&[f64]>,
        stamp: Stamp,
    ) -> Option<Self> {
        let Stamp {
            index,
            time,
            dt,
            courant,
        } = stamp;
        if pressure.len() != mesh.cell_count()
            || flux.len() != mesh.faces().len()
            || temperature.is_some_and(|t| t.len() != mesh.cell_count())
        {
            return None;
        }
        let mut inflow = 0.0;
        let mut outflow = 0.0;
        let patches = mesh
            .boundary_patches()
            .iter()
            .map(|patch| {
                let mut measure = 0.0;
                let mut q = 0.0;
                let mut p = 0.0;
                let mut t = 0.0;
                for &i in &patch.face_indices {
                    let face = &mesh.faces()[i];
                    let f = flux[i];
                    if f < 0.0 {
                        inflow -= f;
                    } else {
                        outflow += f;
                    }
                    q += f;
                    measure += face.area;
                    p += face.area * pressure[face.owner] * density;
                    if let Some(values) = temperature {
                        t += face.area * values[face.owner];
                    }
                }
                Patch {
                    name: patch.name.clone(),
                    volume_flux: q,
                    mass_flux: q * density,
                    pressure_pa: (measure > 0.0).then_some(p / measure),
                    temperature_k: temperature.and_then(|_| (measure > 0.0).then_some(t / measure)),
                }
            })
            .collect();
        let volume: f64 = mesh.cells().iter().map(|c| c.volume).sum();
        let temperature_mean = temperature.and_then(|values| {
            (volume > 0.0).then(|| {
                mesh.cells()
                    .iter()
                    .zip(values)
                    .map(|(c, t)| c.volume * t)
                    .sum::<f64>()
                    / volume
            })
        });
        Some(Self {
            index,
            time,
            dt,
            courant,
            mass_imbalance: (outflow - inflow) * density,
            relative_mass_imbalance: (inflow.max(outflow) > 0.0)
                .then_some((outflow - inflow).abs() / inflow.max(outflow)),
            temperature_mean,
            energy_imbalance_w: None,
            relative_energy_imbalance: None,
            patches,
        })
    }
}
pub fn append(path: &Path, sample: &Sample) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    let mut file = std::fs::OpenOptions::new()
        .append(true)
        .read(true)
        .create(true)
        .open(path)
        .map_err(|e| e.to_string())?;
    let mut end = file.seek(SeekFrom::End(0)).map_err(|e| e.to_string())?;
    let mut byte = [0u8];
    while end > 0 {
        file.seek(SeekFrom::Start(end - 1))
            .map_err(|e| e.to_string())?;
        file.read_exact(&mut byte).map_err(|e| e.to_string())?;
        if byte[0] == b'\n' {
            break;
        }
        end -= 1;
    }
    file.set_len(end).map_err(|e| e.to_string())?;
    let mut bytes = serde_json::to_vec(sample).map_err(|e| e.to_string())?;
    bytes.push(b'\n');
    file.write_all(&bytes).map_err(|e| e.to_string())
}
pub fn load(path: &Path) -> Result<Vec<Sample>, String> {
    let file = std::fs::File::open(path).map_err(|e| e.to_string())?;
    let mut samples = Vec::new();
    let mut reader = std::io::BufReader::new(file);
    let mut line = Vec::new();
    loop {
        line.clear();
        if reader
            .read_until(b'\n', &mut line)
            .map_err(|e| e.to_string())?
            == 0
        {
            break;
        }
        if !line.ends_with(b"\n") {
            break;
        }
        if line.iter().all(u8::is_ascii_whitespace) {
            continue;
        }
        let sample: Sample = serde_json::from_slice(&line).map_err(|e| e.to_string())?;
        match samples.binary_search_by_key(&sample.index, |s: &Sample| s.index) {
            Ok(i) => samples[i] = sample,
            Err(i) => samples.insert(i, sample),
        }
    }
    Ok(samples)
}
