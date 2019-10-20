use std::time::{Instant, Duration};
use nalgebra::{Matrix3, Rotation, U3, UnitQuaternion, Quaternion, Vector3};
pub mod lib;

use lib::{utils_rust, spacetime};


fn main() {
    let m1: Matrix3<f64> = Matrix3::new_random();
    let mut m2: Matrix3<f64> = Matrix3::new_random();
    let mut robot = spacetime::robot::Robot::from_yaml_path("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml");

    let mut qa: Quaternion<f64> = Quaternion::new(-0.50563212, 0.48554394, -0.68368674,  0.20286862);
    let mut qb: Quaternion<f64> = Quaternion::new(-0.50563212, 0.48554394, -0.68368674,  0.20286862);
    let mut q1: UnitQuaternion<f64> = UnitQuaternion::from_quaternion(qa);
    let mut q2: UnitQuaternion<f64> = UnitQuaternion::from_quaternion(qb);

    let mut f = 1.0;

    let start = Instant::now();
    for i in 0..20 {
        f = utils_rust::transformations::angle_between(q1, q2);
    }
    let duration = start.elapsed();
    println!("{:?}", duration);
    println!("{:?}", q1.angle_to(&q2));
    println!("{:?}", utils_rust::transformations::angle_between(q1, q2));
    println!("{:?}", f);

}