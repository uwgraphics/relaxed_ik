pub mod lib;

use lib::{utils_rust, spacetime};
use std::time::{Duration, Instant};

fn main() {
    // let docs = utils_rust::yaml_utils::get_yaml_obj("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml");
    let ifp = utils_rust::yaml_utils::InfoFileParser::from_yaml_path("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml");
    println!("{:?}", ifp.velocity_limits);

    let mut robot = spacetime::robot::Robot::from_yaml_path("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml");
    println!("{:?}", robot.arms[0].axis_types[0] == String::from("z"));
    println!("{:?}", robot.subchain_indices);
    println!("{:?}", robot.arms[0].rot_offset_matrices);
    let mut list: Vec<nalgebra::Matrix3<f64>> = Vec::new();
    let mut test: nalgebra::Matrix3<f64> = nalgebra::Matrix3::identity();
    let mut test4: nalgebra::Matrix4<f64> = nalgebra::Matrix4::identity();
    let mut v: nalgebra::Vector3<f64> = robot.arms[0].disp_offset.clone();


    let start = Instant::now();
    for i in 0..7 {
        // String::from("X");
        // robot.arms[0].update_rot(1.57,String::from("X"));
        // &robot.arms[0].axis_types[0];
        // let m = crate::spacetime::arm::get_rot_x(1.57);
        // test = test*m*m*m*m*m*m*m*m;
        // test4 = test4 * test4 * test4 * test4 * test4;
        // list.push(m);
        v += robot.arms[0].disp_offset.clone();
    }
    let duration = start.elapsed();
    println!("{:?}", duration);
}