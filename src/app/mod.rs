//! Application services that coordinate long-running workbench operations.
//!
//! This layer is deliberately independent of Slint. Presentation code polls an
//! executor and projects its typed events into the UI.

pub mod jobs;

pub use jobs::{
    CancellationToken, JobLifecycle, JobProgress, WorkbenchJobEvent, WorkbenchJobExecutor,
};
