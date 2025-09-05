use st_mems_reg_config_conv::parser;
use std::path::Path;

fn main() {
    // Source file:
    // https://github.com/STMicroelectronics/st-mems-finite-state-machine/blob/main/examples/glance_detection/lis2duxs12/lis2duxs12_glance.json
    let input_file = Path::new("lis2duxs12_glance.json");
    let output_file = Path::new("src/glance_detection.rs");
    parser::generate_rs_from_json(&input_file, &output_file, "GLANCE_DETECTION", "LIS2DUXS12", false);

    println!("cargo:rerun-if-changed=lis2duxs12_glance.json");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
