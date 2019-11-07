mod lib;

use lib::groove::objective::{*};
use lib::groove::vars::{*};
use lib::groove::objective_master::ObjectiveMaster;
use std::time::{Instant, Duration};
use lib::groove::groove::{OptimizationEngineOpen, OptimizationEngineNLopt};

fn main() {
    println!("{:?}", "yup");
    let mut vars = RelaxedIKVars::from_yaml_path("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml".to_string(), false, false);
    let om = ObjectiveMaster::relaxed_ik();

    let x = [0.0; 6];
    println!("{:?}", om.gradient_finite_diff(&x, &vars));
    println!("{:?}", om.gradient(&x, &vars));
}