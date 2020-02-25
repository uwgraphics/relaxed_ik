pub mod lib;

use lib::utils_rust::yaml_utils::RobotCollisionSpecFileParser;
use lib::utils_rust::robot_collision_model::RobotCollisionModel;
use lib::utils_rust::file_utils::get_path_to_src;
use nalgebra::{UnitQuaternion, Quaternion, Vector3, Matrix3};
use std::time::{Instant, Duration};
use lib::utils_rust::transformations;

fn main() {
    let path_to_src = get_path_to_src();

    let fp = path_to_src + "RelaxedIK/Config/collision_files_rust/collision_example.yaml";

    // println!("{:?}", fp);
    let r = RobotCollisionSpecFileParser::from_yaml_path(fp);
    // println!("{:?}", r.spheres[0].coordinate_frame);

    let mut rcm = RobotCollisionModel::from_info_file_name("hubo_info.yaml".to_string());
    let mut q = UnitQuaternion::face_towards(&Vector3::new(0.,1.,0.), &Vector3::new(0.,0.,1.));

    let start = Instant::now();
    for i in 0..1000 {
        rcm.update_robot_transforms(&vec![0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.0], true, true);
        // let rot = Matrix3::new(1., 0., 0.,
         //                                    0., 0., -1.,
           //                                  0., 1., 0.);
        // let mut lookat = Vector3::new(1.0, 1.0, 1.0);
        // lookat = rot * lookat;
        // println!("{:?}", lookat);
        // q = UnitQuaternion::face_towards(&lookat, &Vector3::new(0.,0.,1.));
        // let q2 = UnitQuaternion::from_euler_angles(1.57, 0., 0.);
        // q = q * q2;
    }
    let stop = start.elapsed();

    println!("{:?}", rcm.collision_objects[2].curr_isometry);

    // println!("{:?}", q);
    println!("{:?}", stop);
}