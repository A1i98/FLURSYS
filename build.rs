fn main() {
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

    slint_build::compile_with_config(
        "ui/main.slint",
        slint_build::CompilerConfiguration::new()
            .with_style("fluent".into())
            .embed_resources(slint_build::EmbedResourcesKind::EmbedFiles),
    )
    .expect("cannot compile FLURSYS Slint UI");
}
