mod lib;

use lib::{utils_rust, spacetime};


fn main() {
    println!("hi");
    // let docs = utils_rust::yaml_utils::get_yaml_obj("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml");
    let ifp = utils_rust::yaml_utils::InfoFileParser::from_yaml_path("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/hubo_info.yaml");
    println!("{:?}", ifp);
}