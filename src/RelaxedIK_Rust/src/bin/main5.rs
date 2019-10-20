use std::time::{Instant, Duration};
use nalgebra::{Matrix3, Rotation, U3, UnitQuaternion};
pub mod lib;

use lib::{utils_rust, spacetime};


fn main() {
    let m1: Matrix3<f64> = Matrix3::new_random();
    let mut m2: Matrix3<f64> = Matrix3::new_random();

    let mut robot = spacetime::robot::Robot::from_yaml_path("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml");
    // println!("{:?}", robot.arms[0].rot_offset_quats);

    // println!("{:?}", spacetime::arm::get_neg_rot_z(1.4));
    // println!("{:?}", spacetime::arm::get_neg_quat_z(1.4).to_rotation_matrix().matrix());
    let start = Instant::now();
    for i in 0..100 {
        robot.arms[0].get_frames(vec![0., 0., 0., 0., 0., 0.]);
    }
    let duration = start.elapsed();
    println!("{:?}", robot.arms[0].out_rot_mats);
    println!("{:?}", duration);
}