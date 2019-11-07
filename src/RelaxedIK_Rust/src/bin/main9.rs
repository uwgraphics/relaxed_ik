mod lib;

use lib::groove::vars::{Vars, RelaxedIKVars};
use lib::{utils_rust, spacetime};
use lib::groove::objective_old::{*};
use nalgebra::geometry::UnitQuaternion;
use std::time::{Instant, Duration};
use crate::lib::groove::gradient::{ForwardFiniteDiff, CentralFiniteDiff, GradientFinder};
use crate::lib::groove::objective_old::{ObjectiveMasterRIKImmutable, ObjectiveMasterRIK};

fn main() {
    let mut v = RelaxedIKVars::from_yaml_path_with_init("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/sawyer_info.yaml", vec![1.,1.,0.,0.,0.,0.,0.], false, false);
    let om1 = ObjectiveMasterRIK::get_standard_ik();
    let om2 = ObjectiveMasterRIKImmutable::get_standard_ik();
    let om3 = ObjectiveMasterRIKImmutableLite::get_standard_ik();
    println!("{}", om1.call(&[1.,1.,0.,0.,0.,0.,0.0], &mut v));
    println!("{}", om2.call(&[1.,1.,0.,0.,0.,0.,0.0], &v));
    println!("{}", om3.call(&[1.,1.,0.,0.,0.,0.,0.0], &v));

    // println!("{:?}", v.robot.arms[0].get_frames_immutable(&[0.,0.,0.,0.,1.,1.,2.]));
    // println!("{:?}", v.robot.get_frames_immutable(&[0.,0.,0.,0.,1.,1.,1.]));
    // println!("{:?}", v.robot.get_ee_quats(&[0.,0.,0.,0.,0.,0.,1.]));
    // println!("{:?}", v.robot.get_ee_positions(&[0.,0.,0.,0.,0.,0.,1.]));

    /*
    let start = Instant::now();
    for i in 0..10000 {
        om1.call(&[0.,0.,0.,0.,0.,0.01, 0.], &mut v);
    }
    let duration = start.elapsed();
    println!("{:?}", duration);

    let start = Instant::now();
    for i in 0..10000 {
        om3.call(&[0.,0.,0.,0.,0.,0.01, 0.], &mut v);
    }
    let duration = start.elapsed();
    println!("{:?}", duration);

    let start = Instant::now();
    for i in 0..10000 {
        om3.call(&[0.,0.,0.,0.,0.,0.01, 0.], &mut v);
    }
    let duration = start.elapsed();
    println!("{:?}", duration);
    */
}