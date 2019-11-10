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
    MatchEEQuatGoalsObj(MatchEEQuatGoals),
    NNSelfCollisionObj(NNSelfCollision),
    JointLimitsObj(JointLimits),
    MinimizeVelocityObj(MinimizeVelocity),
    MinimizeAccelerationObj(MinimizeAcceleration),
    MinimizeJerkObj(MinimizeJerk)

}

impl Objective {
    pub fn call(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
        match *self {
            Objective::MatchEEPosGoalsObj(_) => self::MatchEEPosGoals::call(x, v, frames),
            Objective::MatchEEQuatGoalsObj(_) => self::MatchEEQuatGoals::call(x, v, frames),
            Objective::NNSelfCollisionObj(_) => self::NNSelfCollision::call(x, v, frames),
            Objective::JointLimitsObj(_) => self::JointLimits::call(x, v, frames),
            Objective::MinimizeVelocityObj(_) => self::MinimizeVelocity::call(x, v, frames),
            Objective::MinimizeAccelerationObj(_) => self::MinimizeAcceleration::call(x, v, frames),
            Objective::MinimizeJerkObj(_) => self::MinimizeJerk::call(x, v, frames)

        }
    }

    pub fn call_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        match *self {
            Objective::MatchEEPosGoalsObj(_) => self::MatchEEPosGoals::call_lite(x, v, ee_poses),
            Objective::MatchEEQuatGoalsObj(_) => self::MatchEEQuatGoals::call_lite(x, v, ee_poses),
            Objective::NNSelfCollisionObj(_) => self::NNSelfCollision::call_lite(x, v, ee_poses),
            Objective::JointLimitsObj(_) => self::JointLimits::call_lite(x, v, ee_poses),
            Objective::MinimizeVelocityObj(_) => self::MinimizeVelocity::call_lite(x, v, ee_poses),
            Objective::MinimizeAccelerationObj(_) => self::MinimizeAcceleration::call_lite(x, v, ee_poses),
            Objective::MinimizeJerkObj(_) => self::MinimizeJerk::call_lite(x, v, ee_poses)

        }
    }

    pub fn gradient(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> (f64, Vec<f64>) {
        match *self {
            Objective::MatchEEPosGoalsObj(_) => self.__finite_diff_gradient(x, v, frames),
            Objective::MatchEEQuatGoalsObj(_) => self.__finite_diff_gradient(x, v, frames),
            Objective::NNSelfCollisionObj(_) => self::NNSelfCollision::gradient(x, v, frames),
            Objective::JointLimitsObj(_) => self.__finite_diff_gradient(x, v, frames),
            Objective::MinimizeVelocityObj(_) => self.__finite_diff_gradient(x, v, frames),
            Objective::MinimizeAccelerationObj(_) => self.__finite_diff_gradient(x, v, frames),
            Objective::MinimizeJerkObj(_) => self.__finite_diff_gradient(x, v, frames)
        }
    }

    pub fn gradient_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> (f64, Vec<f64>) {
        match *self {
            Objective::MatchEEPosGoalsObj(_) => self.__finite_diff_gradient_lite(x, v, ee_poses),
            Objective::MatchEEQuatGoalsObj(_) => self.__finite_diff_gradient_lite(x, v, ee_poses),
            Objective::NNSelfCollisionObj(_) => self::NNSelfCollision::gradient_lite(x, v, ee_poses),
            Objective::JointLimitsObj(_) => self.__finite_diff_gradient_lite(x, v, ee_poses),
            Objective::MinimizeVelocityObj(_) => self.__finite_diff_gradient_lite(x, v, ee_poses),
            Objective::MinimizeAccelerationObj(_) => self.__finite_diff_gradient_lite(x, v, ee_poses),
            Objective::MinimizeJerkObj(_) => self.__finite_diff_gradient_lite(x, v, ee_poses)
        }
    }

    pub fn gradient_type(&self) -> usize {
        // manual diff = 0, finite diff = 1
        match *self {
            Objective::MatchEEPosGoalsObj(_) => 1,
            Objective::MatchEEQuatGoalsObj(_) => 1,
            Objective::NNSelfCollisionObj(_) => 0,
            Objective::JointLimitsObj(_) => 1,
            Objective::MinimizeVelocityObj(_) => 1,
            Objective::MinimizeAccelerationObj(_) => 1,
            Objective::MinimizeJerkObj(_) => 1
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
            // let e = Quaternion::new( -v.goal_quats[i].w, -v.goal_quats[i].i, -v.goal_quats[i].j, -v.goal_quats[i].k);
            // let ee_quat2 = UnitQuaternion::from_quaternion(e);

            let last_elem = frames[i].1.len() - 1;
            let tmp = Quaternion::new(-frames[i].1[last_elem].w, -frames[i].1[last_elem].i, -frames[i].1[last_elem].j, -frames[i].1[last_elem].k);
            let ee_quat2 = UnitQuaternion::from_quaternion(tmp);

            let disp = angle_between(v.goal_quats[i], frames[i].1[last_elem]);
            let disp2 = angle_between(v.goal_quats[i], ee_quat2);
            x_val += disp.min(disp2);
        }
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }

    pub fn call_lite(x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..v.robot.num_chains {
            // let e = Quaternion::new( -v.goal_quats[i].w, -v.goal_quats[i].i, -v.goal_quats[i].j, -v.goal_quats[i].k);
            // let ee_quat2 = UnitQuaternion::from_quaternion(e);
            let tmp = Quaternion::new(-ee_poses[i].1.w, -ee_poses[i].1.i, -ee_poses[i].1.j, -ee_poses[i].1.k);
            let ee_quat2 = UnitQuaternion::from_quaternion(tmp);


            let disp = angle_between(v.goal_quats[i], ee_poses[i].1);
            let disp2 = angle_between(v.goal_quats[i], ee_quat2);
            x_val += disp.min(disp2);
        }
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}

pub struct NNSelfCollision;
impl NNSelfCollision {
    pub fn call(x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
        let mut x_val = v.collision_nn.predict(&x.to_vec());
        groove_loss(x_val, 0., 2, 1.3, 0.002, 4)
    }

    pub fn call_lite(x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        let mut x_val = v.collision_nn.predict(&x.to_vec());
        groove_loss(x_val, 0., 2, 1.3, 0.002, 4)
    }

    pub fn gradient(x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> (f64, Vec<f64>) {
        let (x_val, mut grad) = v.collision_nn.gradient(&x.to_vec());
        let g_prime = groove_loss_derivative(x_val, 0., 2, 1.3, 0.002, 4);
        for i in 0..grad.len() {
            grad[i] *= g_prime;
        }
        (x_val, grad)
    }

    pub fn gradient_lite(x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> (f64, Vec<f64>) {
        let (x_val, mut grad) = v.collision_nn.gradient(&x.to_vec());
        let g_prime = groove_loss_derivative(x_val, 0., 2, 1.3, 0.002, 4);
        for i in 0..grad.len() {
            grad[i] *= g_prime;
        }
        (x_val, grad)
    }
}

pub struct JointLimits;
impl JointLimits {
    pub fn call(x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
        let mut sum = 0.0;
        let penalty_cutoff: f64 = 0.9;
        let a = 0.05 / (penalty_cutoff.powi(50));
        for i in 0..v.robot.num_dof {
            let l = v.robot.bounds[i][0];
            let u = v.robot.bounds[i][1];
            let r = (x[i] - l) / (u - l);
            let n = 2.0 * (r - 0.5);
            sum += a*n.powf(50.);
        }
        groove_loss(sum, 0.0, 2, 0.32950, 0.1, 2)
    }

    pub fn call_lite(x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        let mut sum = 0.0;
        let penalty_cutoff: f64 = 0.85;
        let a = 0.05 / (penalty_cutoff.powi(50));
        for i in 0..v.robot.num_dof {
            let l = v.robot.bounds[i][0];
            let u = v.robot.bounds[i][1];
            let r = (x[i] - l) / (u - l);
            let n = 2.0 * (r - 0.5);
            sum += a*n.powi(50);
        }
        groove_loss(sum, 0.0, 2, 0.32950, 0.1, 2)
    }
}

pub struct MinimizeVelocity;
impl MinimizeVelocity {
    pub fn call(x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
           x_val += (x[i] - v.xopt[i]).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }

    pub fn call_lite(x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
           x_val += (x[i] - v.xopt[i]).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }

}

pub struct MinimizeAcceleration;
impl MinimizeAcceleration {
    pub fn call(x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            let v1 = x[i] - v.xopt[i];
            let v2 = v.xopt[i] - v.prev_state[i];
            x_val += (v1 - v2).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }

    pub fn call_lite(x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            let v1 = x[i] - v.xopt[i];
            let v2 = v.xopt[i] - v.prev_state[i];
            x_val += (v1 - v2).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

pub struct MinimizeJerk;
impl MinimizeJerk {
    pub fn call(x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            let v1 = x[i] - v.xopt[i];
            let v2 = v.xopt[i] - v.prev_state[i];
            let v3 = v.prev_state[i] - v.prev_state2[i];
            let a1 = v1 - v2;
            let a2 = v2 - v3;
            x_val += (a1 - a2).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }

    pub fn call_lite(x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            let v1 = x[i] - v.xopt[i];
            let v2 = v.xopt[i] - v.prev_state[i];
            let v3 = v.prev_state[i] - v.prev_state2[i];
            let a1 = v1 - v2;
            let a2 = v2 - v3;
            x_val += (a1 - a2).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}