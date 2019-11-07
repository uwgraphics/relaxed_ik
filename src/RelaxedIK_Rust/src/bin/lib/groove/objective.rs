use crate::lib::groove::{vars, tools};
use crate::lib::utils_rust::transformations::{*};
use nalgebra::geometry::{UnitQuaternion, Quaternion};
use std::cmp;
use crate::lib::groove::vars::RelaxedIKVars;


pub fn groove_loss(x_val: f64, t: f64, d: i32, c: f64, f: f64, g: i32) -> f64 {
    -( (-(x_val - t).powi(d)) / (2.0 * c.powi(2) ) ).exp() + f * (x_val - t).powi(g)
}

pub fn groove_loss_derivative(x_val: f64, t: f64, d: i32, c: f64, f: f64, g: i32) -> f64 {
    -( (-(x_val - t).powi(d)) / (2.0 * c.powi(2) ) ).exp() *  ((-d as f64 * (x_val - t)) /  (2.0 * c.powi(2))) + g as f64 * f * (x_val - t)
}

pub enum Objective {
    MatchEEPosGoalsObj(MatchEEPosGoals),
    MatchEEQuatGoalsObj(MatchEEQuatGoals)
}

impl Objective {
    pub fn call(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
        match *self {
            Objective::MatchEEPosGoalsObj(_) => self::MatchEEPosGoals::call(x, v, frames),
            Objective::MatchEEQuatGoalsObj(_) => self::MatchEEQuatGoals::call(x, v, frames)
        }
    }

    pub fn call_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        match *self {
            Objective::MatchEEPosGoalsObj(_) => self::MatchEEPosGoals::call_lite(x, v, ee_poses),
            Objective::MatchEEQuatGoalsObj(_) => self::MatchEEQuatGoals::call_lite(x, v, ee_poses)
        }
    }

    pub fn gradient(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> (f64, Vec<f64>) {
        match *self {
            Objective::MatchEEPosGoalsObj(_) => self.__finite_diff_gradient(x, v, frames),
            Objective::MatchEEQuatGoalsObj(_) => self.__finite_diff_gradient(x, v, frames)
        }
    }

    pub fn gradient_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> (f64, Vec<f64>) {
        match *self {
            Objective::MatchEEPosGoalsObj(_) => self.__finite_diff_gradient_lite(x, v, ee_poses),
            Objective::MatchEEQuatGoalsObj(_) => self.__finite_diff_gradient_lite(x, v, ee_poses)
        }
    }

    pub fn gradient_type(&self) -> usize {
        // manual diff = 0, finite diff = 1
        match *self {
            Objective::MatchEEPosGoalsObj(_) => 1,
            Objective::MatchEEQuatGoalsObj(_) => 1
        }
    }

    fn __finite_diff_gradient(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = Vec::new();
        let f_0 = self.call(x, v, frames);

        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.0000001;
            let frames_h = v.robot.get_frames_immutable(x_h.as_slice());
            let f_h = self.call(x_h.as_slice(), v, &frames_h);
            grad.push( (-f_0 + f_h) / 0.0000001);
        }

        (f_0, grad)
    }

    fn __finite_diff_gradient_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = Vec::new();
        let f_0 = self.call_lite(x, v, ee_poses);

        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.0000001;
            let ee_poses_h = v.robot.get_ee_pos_and_quat_immutable(x_h.as_slice());
            let f_h = self.call_lite(x_h.as_slice(), v, &ee_poses_h);
            grad.push( (-f_0 + f_h) / 0.0000001);
        }

        (f_0, grad)
    }
}

pub struct MatchEEPosGoals;
impl MatchEEPosGoals {
    pub fn call(x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..v.robot.num_chains {
            let last_elem = frames[i].0.len() - 1;
            x_val += ( frames[i].0[last_elem] - v.goal_positions[i] ).norm();
        }
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }

    pub fn call_lite(x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..v.robot.num_chains {
            x_val += ( ee_poses[i].0 - v.goal_positions[i] ).norm();
        }
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}

pub struct MatchEEQuatGoals;
impl MatchEEQuatGoals {
    pub fn call(x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..v.robot.num_chains {
            let e = Quaternion::new( -v.goal_quats[i].w, -v.goal_quats[i].i, -v.goal_quats[i].j, -v.goal_quats[i].k);
            let ee_quat2 = UnitQuaternion::from_quaternion(e);

            let last_elem = frames[i].1.len() - 1;
            let disp = angle_between(frames[i].1[last_elem], v.goal_quats[i]);
            let disp2 = angle_between(frames[i].1[last_elem], ee_quat2);
            x_val += disp.min(disp2);
        }
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }

    pub fn call_lite(x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..v.robot.num_chains {
            let e = Quaternion::new( -v.goal_quats[i].w, -v.goal_quats[i].i, -v.goal_quats[i].j, -v.goal_quats[i].k);
            let ee_quat2 = UnitQuaternion::from_quaternion(e);

            let disp = angle_between(ee_poses[i].1, v.goal_quats[i]);
            let disp2 = angle_between(ee_poses[i].1, v.goal_quats[i]);
            x_val += disp.min(disp2);
        }
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}