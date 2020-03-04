pub mod lib;

use lib::utils_rust::self_collision_engine::SelfCollisionEngine;
use std::time::{Instant, Duration};
use lib::utils_rust::sampler::{ThreadRobotSampler, ThreadSampler};


fn main() {
    let mut sce = SelfCollisionEngine::from_info_file_name("ur5_info.yaml".to_string());

    let start = Instant::now();
    for i in 0..1000 {
        // let sample = sce.sampler.sample();
        // sce.collision_check(&sample.data.as_vec());
        let check = sce.collision_check(&vec![0.,0.,0.1,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]);
    }
    let end = start.elapsed();
    println!("{:?}", end);


    for i in 0..1 {
        // let sample = sce.sampler.sample();
        // println!("{:?}", sce.collision_check(&sample.data.as_vec()));
        let check = sce.collision_check(&vec![0.,0.,0.1,0.,0.,0.0,0.,0.,0.,0.,0.,0.,0.,0.,0.]);
        // println!("{:?}", sce.robot_shape_model.collision_objects[0].curr_isometry);
        // println!("{:?}", sce.robot_shape_model.collision_objects[2].curr_isometry);
        // println!("{:?}", sce.allowed_collision_matrix);
        println!("{:?}", check);
    }

}