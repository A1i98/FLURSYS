//! Optional accumulated CG timings. Counters are committed once per solve.
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::OnceLock;
use std::time::Instant;

static ENABLED: OnceLock<bool> = OnceLock::new();
static TOTALS: [AtomicU64; 6] = [const { AtomicU64::new(0) }; 6];

#[derive(Clone, Copy, Debug, Default)]
pub struct CgProfileSnapshot {
    pub solves: u64,
    pub iterations: u64,
    pub spmv_calls: u64,
    pub preconditioner_calls: u64,
    pub spmv_seconds: f64,
    pub preconditioner_seconds: f64,
}

/// Process totals; timings are collected only with FLURSYS_PROFILE enabled.
pub fn cg_profile_snapshot() -> CgProfileSnapshot {
    let values = TOTALS.each_ref().map(|value| value.load(Ordering::Relaxed));
    CgProfileSnapshot {
        solves: values[0],
        iterations: values[1],
        spmv_calls: values[2],
        preconditioner_calls: values[3],
        spmv_seconds: values[4] as f64 * 1.0e-9,
        preconditioner_seconds: values[5] as f64 * 1.0e-9,
    }
}

pub(super) struct CgProfileSession {
    enabled: bool,
    values: [u64; 6],
}

impl CgProfileSession {
    pub(super) fn new() -> Self {
        Self {
            enabled: *ENABLED.get_or_init(|| std::env::var_os("FLURSYS_PROFILE").is_some()),
            values: [1, 0, 0, 0, 0, 0],
        }
    }

    pub(super) fn iteration(&mut self) {
        self.values[1] += 1;
    }

    pub(super) fn measure<T>(&mut self, preconditioner: bool, action: impl FnOnce() -> T) -> T {
        if !self.enabled {
            return action();
        }
        let started = Instant::now();
        let result = action();
        let nanos = started.elapsed().as_nanos().min(u64::MAX as u128) as u64;
        let index = usize::from(preconditioner);
        self.values[2 + index] += 1;
        self.values[4 + index] += nanos;
        result
    }
}

impl Drop for CgProfileSession {
    fn drop(&mut self) {
        if self.enabled {
            for (total, value) in TOTALS.iter().zip(self.values) {
                total.fetch_add(value, Ordering::Relaxed);
            }
        }
    }
}
