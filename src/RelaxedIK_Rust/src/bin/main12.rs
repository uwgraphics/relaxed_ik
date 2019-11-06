mod lib;

use lib::groove::vars::{Vars, RelaxedIKVars};
use lib::{utils_rust, spacetime};
use lib::groove::objective::{*};
use lib::groove::groove::{OptimizationEngineOpen, OptimizationEngineNLoptImmutable};
use nalgebra::geometry::UnitQuaternion;
use std::time::{Instant, Duration};
use crate::lib::groove::gradient::{ForwardFiniteDiff, CentralFiniteDiff, CentralFiniteDiff2, GradientFinder};
use crate::lib::groove::objective::{ObjectiveMasterRIKImmutable, ObjectiveMasterRIK, ObjectiveMasterRIKImmutableLite};

fn main() {
    let mut v1 = RelaxedIKVars::from_yaml_path("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml".to_string(), false, false);
    let mut v2 = RelaxedIKVars::from_yaml_path("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml".to_string(), false, false);
    let mut om = ObjectiveMasterRIKImmutableLite::get_standard_ik();

    v1.goal_positions[0][0] += -0.02;
    v2.goal_positions[0][0] += -0.02;
    let mut o = OptimizationEngineOpen::new(6);
    let start = Instant::now();
    let x = &mut [3.14, -0.38, -1.2, -1.57, -1.57, -1.57];
    for i in 0..10000 {
        x[0] += i as f64 * 0.0001;
        o.optimize_lite(x, &v1, &v2, &om, 100);
    }
    let duration = start.elapsed();
    println!("{:?}", duration);
    println!("{:?}", x);
}