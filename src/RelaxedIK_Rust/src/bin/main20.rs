pub mod lib;
use crate::lib::relaxed_ik;
use crate::lib::groove::groove::OptimizationEngineOpen;
use crate::lib::utils_rust::subscriber_utils::EEPoseGoalsSubscriber;

fn main() {
    let mut r = relaxed_ik::RelaxedIK::from_yaml_path("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml".to_string(), 0);
    let mut r2 = relaxed_ik::RelaxedIK::from_info_file_name("sawyer_info.yaml".to_string(), 0);
    let mut r3 = relaxed_ik::RelaxedIK::from_loaded(1);

    let x = r.solve_with_user_provided_goals(vec![vec![0.03,0.,0.]], vec![vec![1.,0.,0.,0.]]);
    println!("{:?}", x);
}