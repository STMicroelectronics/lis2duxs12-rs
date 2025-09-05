use st_mems_reg_config_conv::parser;
use std::path::Path;

fn main() {
    //Source file:
    //https://github.com/STMicroelectronics/st-mems-machine-learning-core/blob/main/examples/activity_recognition_for_mobile/lis2duxs12/lis2duxs12_activity_recognition_for_mobile.json
    let input_file = Path::new("lis2duxs12_activity_recognition_for_mobile.json");
    let output_file = Path::new("src/mlc_activity_recognition.rs");
    parser::generate_rs_from_json(&input_file, &output_file, "ACTIVITY", "LIS2DUXS12", false);

    println!("cargo:rerun-if-changed=lis2duxs12_activity_recognition_for_mobile.json");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
