fn main() {
    embed_cases().expect("cannot embed FLURSYS case files");
    for path in [
        "src/compute/linear.wgsl",
        "src/compute/amg.wgsl",
        "src/compute/flux.wgsl",
    ] {
        println!("cargo:rerun-if-changed={path}");
        let source = std::fs::read_to_string(path).expect("compute shader source");
        let module = naga::front::wgsl::parse_str(&source)
            .unwrap_or_else(|error| panic!("invalid {path}: {}", error.emit_to_string(&source)));
        naga::valid::Validator::new(
            naga::valid::ValidationFlags::all(),
            naga::valid::Capabilities::empty(),
        )
        .validate(&module)
        .unwrap_or_else(|error| panic!("invalid {path}: {error:?}"));
    }
    println!("cargo:rerun-if-changed=packaging/windows/flursys.ico");
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows") {
        let out =
            std::path::PathBuf::from(std::env::var_os("OUT_DIR").expect("OUT_DIR is required"));
        let icon = out.join("flursys.ico");
        std::fs::copy("packaging/windows/flursys.ico", &icon)
            .expect("cannot copy application icon");
        let resource = out.join("flursys.rc");
        std::fs::write(
            &resource,
            format!(
                "1 ICON \"{}\"\n",
                icon.display().to_string().replace('\\', "/")
            ),
        )
        .expect("cannot write application icon resource");
        embed_resource::compile_for(&resource, ["flursys-gui"], embed_resource::NONE)
            .manifest_required()
            .expect("cannot compile application icon resource");
    }
}

use std::{env, fs, path::PathBuf};
fn embed_cases() -> Result<(), Box<dyn std::error::Error>> {
    let root = PathBuf::from(env::var("CARGO_MANIFEST_DIR")?).join("examples/cases");
    println!("cargo:rerun-if-changed=examples/cases");
    let mut files = Vec::new();
    for entry in fs::read_dir(&root)? {
        let entry = entry?;
        if entry.file_type()?.is_dir() {
            let file = entry.path().join("case.json");
            if file.is_file() {
                files.push(file);
            }
        }
    }
    files.sort();
    let mut source = String::from("pub(super) const CASES: &[&str] = &[\n");
    for file in files {
        source.push_str(&format!("include_str!({:?}),\n", file));
    }
    source.push_str("];\n");
    source.push_str("pub(super) const REFERENCE_MESHES: &[(&str, &str)] = &[\n");
    for entry in fs::read_dir(&root)? {
        let entry = entry?;
        let folder = entry.path().join("reference");
        if !folder.is_dir() {
            continue;
        }
        for file in fs::read_dir(folder)? {
            let file = file?.path();
            if file.extension().is_some_and(|ext| ext == "msh") {
                println!("cargo:rerun-if-changed={}", file.display());
                let key = file
                    .strip_prefix(&root)?
                    .to_string_lossy()
                    .replace('\\', "/");
                source.push_str(&format!("({key:?}, include_str!({file:?})),\n"));
            }
        }
    }
    source.push_str("];\n");
    fs::write(
        PathBuf::from(env::var("OUT_DIR")?).join("builtin_cases.rs"),
        source,
    )?;
    Ok(())
}
