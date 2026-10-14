use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::OnceLock;
use std::time::Instant;

#[derive(Clone, Copy)]
pub(super) enum ProfileStage {
    Step,
    UAssembly,
    USolve,
    VAssembly,
    VSolve,
    PressureAssembly,
    PressureSolve,
    PressureCorrection,
    CellFields,
    Output,
    PressureRhs,
    PressurePreconditionerSetup,
    PressurePredictor,
}

const STAGE_COUNT: usize = 13;
static ENABLED: OnceLock<bool> = OnceLock::new();
static NANOS: [AtomicU64; STAGE_COUNT] = [
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
];

fn enabled() -> bool {
    *ENABLED.get_or_init(|| std::env::var_os("FLURSYS_PROFILE").is_some())
}

fn index(stage: ProfileStage) -> usize {
    match stage {
        ProfileStage::Step => 0,
        ProfileStage::UAssembly => 1,
        ProfileStage::USolve => 2,
        ProfileStage::VAssembly => 3,
        ProfileStage::VSolve => 4,
        ProfileStage::PressureAssembly => 5,
        ProfileStage::PressureSolve => 6,
        ProfileStage::PressureCorrection => 7,
        ProfileStage::CellFields => 8,
        ProfileStage::Output => 9,
        ProfileStage::PressureRhs => 10,
        ProfileStage::PressurePreconditionerSetup => 11,
        ProfileStage::PressurePredictor => 12,
    }
}

pub(super) struct ProfileScope {
    stage: usize,
    started: Option<Instant>,
}

impl ProfileScope {
    pub(super) fn new(stage: ProfileStage) -> Self {
        let active = enabled();
        Self {
            stage: index(stage),
            started: active.then(Instant::now),
        }
    }

    pub(super) fn report() {
        if !enabled() {
            return;
        }
        let labels = [
            "step_total",
            "u_assembly",
            "u_solve",
            "v_assembly",
            "v_solve",
            "pressure_assembly",
            "pressure_solve",
            "pressure_correction",
            "cell_fields",
            "output",
            "pressure_rhs",
            "pressure_setup",
            "pressure_predictor",
        ];
        let step_nanos = NANOS[0].load(Ordering::Relaxed).max(1) as f64;
        println!("Performance profile (FLURSYS_PROFILE enabled):");
        for (i, label) in labels.into_iter().enumerate() {
            let nanos = NANOS[i].load(Ordering::Relaxed);
            let percent = if i == 0 {
                100.0
            } else {
                100.0 * nanos as f64 / step_nanos
            };
            println!(
                "  {label:>20}: {:9.3} s ({percent:6.2}% of step time)",
                nanos as f64 * 1.0e-9
            );
        }
        let cg = crate::linear::cg_profile_snapshot();
        println!(
            "CG totals: {} solves, {} iterations, {} SpMV calls, {} preconditioner applications",
            cg.solves, cg.iterations, cg.spmv_calls, cg.preconditioner_calls
        );
        println!(
            "CG substage times (included in pressure_solve): SpMV {:.6} s; preconditioner {:.6} s",
            cg.spmv_seconds, cg.preconditioner_seconds
        );
    }
}

impl Drop for ProfileScope {
    fn drop(&mut self) {
        if let Some(started) = self.started {
            let nanos = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            NANOS[self.stage].fetch_add(nanos, Ordering::Relaxed);
        }
    }
}
