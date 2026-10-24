fn main() {
    for path in ["src/compute/linear.wgsl", "src/compute/amg.wgsl"] {
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
