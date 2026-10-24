//! Headless frontend for canonical FLURSYS workspaces.
#[path = "flursys_cli/workspace.rs"]
mod workspace;
fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().is_some_and(|a| a == "--version" || a == "-V") {
        println!("FLURSYS {}", env!("CARGO_PKG_VERSION"));
        return;
    }
    if let Err(error) = workspace::run(&args) {
        if args.iter().any(|a| a == "--json") {
            eprintln!("{}", serde_json::json!({"error":error}));
        } else {
            eprintln!("Error: {error}");
        }
        std::process::exit(1);
    }
}
