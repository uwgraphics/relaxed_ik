use std::sync::{Arc, Mutex};
use std::thread;
use nalgebra::{Vector3, Quaternion, UnitQuaternion, Unit};

pub struct SingleValueSubscriber<T> {
    pub data: T
}
impl<T> SingleValueSubscriber<T> {
    pub fn new(data: T) -> Self {Self{data: data} }
}

/*
let arc = Arc::new(Mutex::new(SingleValueSubscriber{data: 0}));
let arc2 = arc.clone();
let subscriber = rosrust::subscribe("topic", 3, move |v: msg::std_msgs::UInt64| {
    let mut g = arc2.lock().unwrap();
    g.a = v.data;
});
*/

pub struct EEPoseGoalsSubscriber {
    pub pos_goals: Vec<Vector3<f64>>,
    pub quat_goals: Vec<UnitQuaternion<f64>>
}
impl EEPoseGoalsSubscriber {
    pub fn new() -> Self {
        let pos_goals: Vec<Vector3<f64>> = Vec::new();
        let quat_goals: Vec<UnitQuaternion<f64>> = Vec::new();
        Self{pos_goals, quat_goals}
    }
}
/*
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
*/

