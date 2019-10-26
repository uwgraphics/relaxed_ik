mod lib;

use lib::groove::vars::{Vars, RelaxedIKVars};
use lib::{utils_rust, spacetime};
use lib::groove::objective::{*};
use nalgebra::geometry::UnitQuaternion;
use std::time::{Instant, Duration};
use crate::lib::groove::gradient::{ForwardFiniteDiff, CentralFiniteDiff, GradientFinder};

fn main() {
    let mut v = RelaxedIKVars::from_yaml_path_with_init("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/sawyer_info.yaml", vec![1.,0.,0.,1.,0.,0.,0.], false, false);

    let arm = v.robot.arms[0].clone();

    let start = Instant::now();
    for i in 0..1000 {
        arm.get_frames_immutable(&[0., 0., 0., 0., 0., 0., 0.]);
    }
    let duration = start.elapsed();
    println!("{:?}", duration);

    let start = Instant::now();
    for i in 0..1000 {
        arm.get_ee_pos_and_quat_immutable(&[0., 0., 0., 0., 0., 0., 0.]);
    }
    let duration = start.elapsed();
    println!("{:?}", duration);

    let mut arm = v.robot.arms[0].clone();
    let start = Instant::now();
    for i in 0..1000 {
        arm.get_frames(&[0., 0., 0., 0., 0., 0., 0.]);
    }
    let duration = start.elapsed();
    println!("{:?}", duration);

    let mut arm = v.robot.arms[0].clone();
    let start = Instant::now();
    for i in 0..1000 {
        v.robot.get_ee_pos_and_quat_immutable(&[0., 0., 0., 0., 0., 0., 0.]);
    }
    let duration = start.elapsed();
    println!("{:?}", duration);
}