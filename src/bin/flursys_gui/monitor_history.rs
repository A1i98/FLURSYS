//! Live section monitor samples and background reconstruction from raw frame archives.
use super::*;
use flursys::workbench::{plane_integrals, plane_monitor::PlaneSnapshot};
#[derive(Default)]
pub(super) struct State {
    pub points: std::collections::BTreeMap<String, Vec<plane_integrals::Sample>>,
    pending:
        Option<std::sync::mpsc::Receiver<Result<(String, Vec<plane_integrals::Sample>), String>>>,
}
impl State {
    pub fn clear(&mut self) {
        self.points.clear();
        self.pending = None;
    }
    pub fn record(&mut self, snapshot: &PlaneSnapshot) -> Result<(), String> {
        let point = plane_integrals::sample(snapshot)?;
        let history = self.points.entry(snapshot.plane.id.clone()).or_default();
        match history.binary_search_by_key(&point.iteration, |p| p.iteration) {
            Ok(i) => history[i] = point,
            Err(i) => history.insert(i, point),
        }
        Ok(())
    }
    pub fn load(
        &mut self,
        paths: Vec<PathBuf>,
        plane: flursys::workbench::plane_monitor::FlowPlaneMonitor,
    ) {
        self.points.remove(&plane.id);
        let (tx, rx) = std::sync::mpsc::channel();
        self.pending = Some(rx);
        std::thread::spawn(move || {
            let result = (|| -> Result<_, String> {
                let mut points = Vec::with_capacity(paths.len());
                for path in paths {
                    let s: PlaneSnapshot =
                        serde_json::from_slice(&std::fs::read(path).map_err(|e| e.to_string())?)
                            .map_err(|e| {
                                format!("Monitor history requires raw JSON frames: {e}")
                            })?;
                    if s.plane.id != plane.id
                        || s.plane.origin != plane.origin
                        || s.plane.u != plane.u
                        || s.plane.v != plane.v
                    {
                        return Err("Monitor archive plane geometry mismatch".into());
                    }
                    points.push(plane_integrals::sample(&s)?);
                }
                plane_integrals::series(&points, &plane.name, 0)?;
                Ok((plane.id, points))
            })();
            let _ = tx.send(result);
        });
    }
    pub fn poll(&mut self) -> Option<Result<(), String>> {
        let rx = self.pending.take()?;
        match rx.try_recv() {
            Ok(Ok((id, points))) => {
                self.points.insert(id, points);
                Some(Ok(()))
            }
            Ok(Err(e)) => Some(Err(e)),
            Err(std::sync::mpsc::TryRecvError::Empty) => {
                self.pending = Some(rx);
                None
            }
            Err(_) => Some(Err("Monitor history worker disconnected".into())),
        }
    }
}
pub(super) fn chart(
    state: &AppState,
    index: usize,
    kind: usize,
) -> Result<(String, String), String> {
    let plane = state
        .workbench_project
        .plane_monitors
        .get(index)
        .ok_or("Choose a monitor plane in Display")?;
    let points = state
        .post
        .monitors
        .points
        .get(&plane.id)
        .ok_or("No history: enable recording before Run, or select a recorded plane")?;
    let (series, x, y) = plane_integrals::series(points, &plane.name, kind)?;
    let caption=format!("{} · {} samples · {} · {}. Flux uses cell velocity and section polygons, not conservative solver face flux.",plane.name,points.len(),x,y);
    Ok((
        flursys::workbench::series_plot::svg(&[series], "Section monitor", x, y)?,
        caption,
    ))
}
pub(super) fn csv(state: &AppState, index: usize) -> Result<String, String> {
    let plane = state
        .workbench_project
        .plane_monitors
        .get(index)
        .ok_or("Choose a monitor plane")?;
    let points = state
        .post
        .monitors
        .points
        .get(&plane.id)
        .ok_or("No recorded monitor samples")?;
    plane_integrals::series(points, &plane.name, 0)?;
    Ok(plane_integrals::csv(points))
}
