use std::sync::{Arc, Mutex};
use std::thread;
use std::sync::mpsc::channel;
use rosrust;
use rosrust::api::Ros;
pub mod lib;
use nalgebra::{Vector3, UnitQuaternion, Quaternion};

use crate::lib::utils_rust::subscriber_utils::{*};

mod msg {
    rosrust::rosmsg_include!(std_msgs/UInt64, relaxed_ik / EEPoseGoals, geometry_msgs / Pose);
}

fn main() {
    rosrust::init("listener");

    let arc = Arc::new(Mutex::new(EEPoseGoalsSubscriber::new()));
    let arc2 = arc.clone();
    let subscriber = rosrust::subscribe("/relaxed_ik/ee_pose_goals", 3, move |v: msg::relaxed_ik::EEPoseGoals| {
        let mut g = arc2.lock().unwrap();
        g.pos_goals = Vec::new();
        g.quat_goals = Vec::new();

        let num_poses = v.ee_poses.len();

        for i in 0..num_poses {
            g.pos_goals.push( Vector3::new(v.ee_poses[i].position.x, v.ee_poses[i].position.y, v.ee_poses[i].position.z) );
            let tmp_q = Quaternion::new(v.ee_poses[i].orientation.w, v.ee_poses[i].orientation.x, v.ee_poses[i].orientation.y, v.ee_poses[i].orientation.z);
            g.quat_goals.push( UnitQuaternion::from_quaternion(tmp_q) );
        }
    });

    let rate = rosrust::rate(2000.);
    while rosrust::is_ok() {
        println!("{:?}", arc.lock().unwrap().quat_goals);
        rate.sleep();
    }
}