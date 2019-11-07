mod lib;

use lib::groove::objective::{*};
use lib::groove::vars::{*};
use lib::groove::objective_master::ObjectiveMaster;
use std::time::{Instant, Duration};

fn main() {
    println!("{:?}", "yup");
    let mut vars = RelaxedIKVars::from_yaml_path("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml".to_string(), true, true);
    let om = ObjectiveMaster::standard_ik();
    let start = Instant::now();
    for i in 0..1000 {
        om.call_lite(&[0., 0., 0., 0., 0., 0.], &vars);
    }
    let duration = start.elapsed();
    println!("{:?}", duration);

    let start = Instant::now();
    for i in 0..1000 {
        om.gradient_finite_diff_lite(&[0., 0., 0., 0., 0., 0.], &vars);
    }
    let duration = start.elapsed();

    println!("{:?}", duration);
    println!("{:?}", om.gradient(&[0., 0., 0., 0., 0., 0.0], &vars));
    println!("{:?}", om.call(&[0., 0., 0., 0., 0., 0.], &vars));
    println!("{:?}", om.gradient_finite_diff(&[0., 0., 0., 0., 0., 0.], &vars));
    println!("{:?}", om.gradient_finite_diff_lite(&[0., 0., 0., 0., 0., 0.], &vars));
    println!("{:?}", om.gradient_lite(&[0., 0., 0., 0., 0., 0.], &vars));

    // println!("{:?}", );
    // let o = Objective::MatchEEPosGoalsObj(MatchEEPosGoals);
    // let o2 = Objective::MatchEEQuatGoalsObj(MatchEEQuatGoals);
    // let g = o2.gradient_lite(&[0.,0.,0.,0.,0.,0.], &vars, &vars.robot.get_ee_pos_and_quat_immutable(&[0.,0.,0.,0.,0.,0.]));
    // println!("{:?}", g);
}