pub mod lib;
use crate::lib::relaxed_ik;
use crate::lib::utils_rust::subscriber_utils::EEPoseGoalsSubscriber;
use std::sync::{Arc, Mutex};
use rosrust;
use nalgebra::{Vector3, UnitQuaternion, Quaternion};
use crate::lib::utils_rust::subscriber_utils::{*};

mod msg {
    rosrust::rosmsg_include!(relaxed_ik / EEPoseGoals, relaxed_ik / JointAngles);
}

fn main() {
    rosrust::init("relaxed_ik");

    println!("solver initialized!");

    let mut r = relaxed_ik::RelaxedIK::from_loaded(1);

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
    let publisher = rosrust::publish("/relaxed_ik/joint_angle_solutions", 3).unwrap();

    let rate1 = rosrust::rate(100.);
    while arc.lock().unwrap().pos_goals.is_empty() {rate1.sleep();}

    let rate = rosrust::rate(3000.);
    while rosrust::is_ok() {
        let x = r.solve(&arc.lock().unwrap());
        println!("{:?}", x);

        let mut ja = msg::relaxed_ik::JointAngles::default();
        for i in 0..x.len() {
            ja.angles.data.push(x[i]);
        }
        publisher.send(ja);

        rate.sleep();
    }
}