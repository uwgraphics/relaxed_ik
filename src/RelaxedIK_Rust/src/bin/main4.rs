use nalgebra::{Matrix3, Vector3, UnitQuaternion, Rotation, U3};
use std::time::{Instant, Duration};
pub mod lib;

use lib::{utils_rust, spacetime};

fn main() {
    let mut robot = spacetime::robot::Robot::from_yaml_path("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml");
    robot.arms[0].get_frames(&[1.,0.,0.,0.,0.,0.]);
    let mut uq: UnitQuaternion<f64> = UnitQuaternion::from_matrix(&robot.arms[0].out_rot_mats[5]);
    let mut rot_mat = uq.to_rotation_matrix();
    let start = Instant::now();
    for i in 0..1 {
        rot_mat = uq.to_rotation_matrix();
    }
    let duration = start.elapsed();
    let rot: Rotation<f64, U3> = Rotation::identity();

    println!("{:?}", duration);
    println!("{:?}", rot_mat * rot);
}