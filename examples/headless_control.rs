//! cargo run --no-default-features --example headless_control -- WORKSPACE
//! Enter pause, resume or stop on stdin while the solve runs.
use flursys::headless;
use std::io::BufRead;
fn main() -> Result<(), String> {
    let directory = std::path::PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("Usage: headless_control WORKSPACE")?,
    );
    let (mut project, mut session) = headless::load(&directory)?;
    if session.mesh().is_none() {
        headless::mesh(
            &directory,
            &mut project,
            &mut session,
            &flursys::GmshMesher::auto(),
        )?;
    }
    let control = headless::ExecutionControl::default();
    let input = control.clone();
    std::thread::spawn(move || {
        for line in std::io::stdin().lock().lines() {
            match line.as_deref() {
                Ok("pause") => input.pause(),
                Ok("resume") => input.resume(),
                Ok("stop") => {
                    input.stop();
                    break;
                }
                _ => eprintln!("Commands: pause, resume, stop"),
            }
        }
    });
    let outcome = headless::run_with_control(
        &directory,
        &mut project,
        &mut session,
        &Default::default(),
        &control,
        |event| {
            if let flursys::app::WorkbenchJobEvent::SolveProgress(progress) = event {
                eprintln!("{progress:?}")
            }
        },
    )?;
    println!(
        "{}",
        serde_json::to_string_pretty(&outcome).map_err(|e| e.to_string())?
    );
    if let Some(path) = outcome.restart_checkpoint {
        println!(
            "Resume with RunOptions.restart = {:?}",
            directory.join(path)
        );
    }
    if let Some(error) = outcome.failure {
        return Err(error);
    }
    Ok(())
}
