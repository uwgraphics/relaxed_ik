mod lib;

use lib::{utils_rust, spacetime};


fn main() {
    // let docs = utils_rust::yaml_utils::get_yaml_obj("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml");
    let ifp = utils_rust::yaml_utils::InfoFileParser::from_yaml_path("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml");
    println!("{:?}", ifp.velocity_limits);

    let a = spacetime::arm::Arm::new(ifp.axis_types[0].clone(), ifp.displacements[0].clone(),
                                        ifp.disp_offsets[0].clone(), ifp.joint_types[0].clone());

    let robot = spacetime::robot::Robot::from_yaml_path("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml");
    println!("{:?}", robot.subchain_indices)
}