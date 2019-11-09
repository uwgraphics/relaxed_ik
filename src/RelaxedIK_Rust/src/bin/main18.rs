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

    vars.goal_positions[0][0] += 0.00;

    let mut oe = OptimizationEngineOpen::new(6);
    let mut x = [ 3.14, -0.38, -1.2, -1.57, -1.57, -1.57 ];

    let start = Instant::now();
    for i in 0..10000 {
        vars.goal_positions[0][1] -= 0.000055;
        oe.optimize(&mut x, &vars, &om, 200);
    }
    let duration = start.elapsed();
    println!("{:?}", duration);
    println!("{:?}", x);

}