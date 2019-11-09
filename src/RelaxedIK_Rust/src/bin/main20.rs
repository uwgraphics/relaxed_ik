pub mod lib;
use crate::lib::relaxed_ik;

fn main() {
    let r = relaxed_ik::RelaxedIK::from_yaml_path("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml".to_string(), 0);
    let r2 = relaxed_ik::RelaxedIK::from_info_file_name("sawyer_info.yaml".to_string(), 0);
    let r3 = relaxed_ik::RelaxedIK::from_loaded(0);
}