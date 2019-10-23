pub mod lib;

use lib::{utils_rust, spacetime};
use std::time::{Duration, Instant};


fn main() {
    let axis = String::from("X");
    let rot_offset: nalgebra::Matrix3<f64> = nalgebra::Matrix3::identity();
    let disp: nalgebra::Vector3<f64> = nalgebra::Vector3::new(2.,0.,0.);

    let mut robot = spacetime::robot::Robot::from_yaml_path("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml");
    for i in 0..10 {
        robot.arms[0].get_frames(&[0., 0., 0., 0., 0.,0.,0.,0.]);
    }
    let start = Instant::now();
    for j in 0..10000 {
        for i in 0..7 {
            robot.arms[0].get_frames(&[0., 0., 0., 0., 0., 0., 0., 0.]);
            robot.arms[0].get_frames(&[0., 0., 0., 0., 0., 0., 0., 0.]);
            robot.arms[0].get_frames(&[0., 0., 0., 0., 0., 0., 0., 0.]);
        }
    }
    let duration = start.elapsed();
    println!("{:?}", robot.arms[0].out_rot_mats);
    println!("{:?}", robot.arms[0].out_positions);
    println!("{:?}", duration);
}