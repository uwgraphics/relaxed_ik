mod lib;

use lib::groove::vars::{Vars, RelaxedIKVars};
use lib::{utils_rust, spacetime};
use lib::groove::objective_old::{*};
use nalgebra::geometry::UnitQuaternion;
use std::time::{Instant, Duration};
use crate::lib::groove::gradient::{ForwardFiniteDiff, CentralFiniteDiff, GradientFinder};

fn main() {
    let mut v = RelaxedIKVars::from_yaml_path_with_init("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml", vec![1.,0.,0.,1.,0.,0.], false, false);
    let om = ObjectiveMasterRIK::get_standard_ik();
    let start = Instant::now();
    for i in 0..400 {
        om.call(&[1.01, 0., 0., 1., 0., 0.1], &mut v);
    }
    let duration = start.elapsed();
    println!("{:?}", duration);

    let start = Instant::now();
    let g = |x: &[f64]| -> f64 {
        om.call(x, &mut v)
    };
    let duration = start.elapsed();
    println!("{:?}", duration);

    let start = Instant::now();
    let mut gradient_finder = ForwardFiniteDiff::new(6, g);
    let duration = start.elapsed();
    println!("{:?}", duration);


    let start = Instant::now();
    for i in 0..50 {
        gradient_finder.compute_gradient(&[1.01, 0., 0., 1., 0., 0.1]);
    }
    let duration = start.elapsed();
    println!("{:?}", duration);
    /*
    v.robot.get_frames(&[1.,0.,0.,1.,0.1,0.2]);
    println!("{:?}", match_ee_quat_goals_obj(&[1.,0.,0.,1.,0.,0.2], &mut v));

    let f: Vec<fn(&[f64], v: &mut RelaxedIKVars) -> f64> = vec![match_ee_pos_goals_obj, match_ee_quat_goals_obj];

    let start = Instant::now();
    for i in 0..1000 {
        v.robot.get_frames(&[1., 0., 0., 1., 0.1, 0.2]);
        for j in 0..f.len() {
            (f[j])(&[1., 0., 0., 1., 0.1, 0.2], &mut v);
        }
    }
    let duration = start.elapsed();
    println!("{:?}", duration);
    */
}


fn update(v: &mut RelaxedIKVars) {
    v.update(vec![2.,0.,0.,1.,0.,1.]);
}