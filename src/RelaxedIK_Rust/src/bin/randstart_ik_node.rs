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

    let mut r = relaxed_ik::RelaxedIK::from_loaded(0);

    let arc = Arc::new(Mutex::new(EEPoseGoalsSubscriber::new()));
    let arc2 = arc.clone();
    let subscriber = rosrust::subscribe("/relaxed_ik/ee_pose_goals", 100, move |v: msg::relaxed_ik::EEPoseGoals| {
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

    let rate = rosrust::rate(10000.);
    while rosrust::is_ok() {
        let x = r.solve_randstart(&arc.lock().unwrap());
        if x.0 {
            println!("{:?}", x.1);

            let mut ja = msg::relaxed_ik::JointAngles::default();
            for i in 0..x.1.len() {
                ja.angles.data.push(x.1[i]);
            }
            publisher.send(ja);
        }
        rate.sleep();
    }
}