use st_mems_reg_config_conv::parser;
use std::path::Path;

fn main() {
    let input_file = Path::new("lis2duxs12_activity_recognition_for_mobile.ucf");
    let output_file = Path::new("src/mlc_activity_recognition.rs");
    parser::generate_rs_from_ucf(&input_file, &output_file, "ACTIVITY");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
