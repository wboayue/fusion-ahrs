use std::path::PathBuf;

fn main() {
    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let fusion = manifest.join("../fusion-c/Fusion");

    if !fusion.join("FusionAhrs.c").exists() {
        panic!(
            "fusion-c submodule not found at {}; run `git submodule update --init`",
            fusion.display()
        );
    }

    println!("cargo:rerun-if-changed={}", fusion.display());
    println!("cargo:rerun-if-changed=shim.c");

    // Exact square roots and no FMA contraction so results are comparable
    // with the Rust implementation.
    cc::Build::new()
        .include(&fusion)
        .files(
            [
                "FusionAhrs.c",
                "FusionBias.c",
                "FusionCompass.c",
                "FusionConvention.c",
                "FusionRemap.c",
            ]
            .iter()
            .map(|f| fusion.join(f)),
        )
        .file("shim.c")
        .define("FUSION_USE_NORMAL_SQRT", None)
        .flag_if_supported("-ffp-contract=off")
        .flag_if_supported("-std=c99")
        .warnings(false)
        .compile("fusion");
}
