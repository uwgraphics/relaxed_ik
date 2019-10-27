use crate::lib::groove::{vars, tools};
use crate::lib::utils_rust::transformations::{*};
use nalgebra::geometry::{UnitQuaternion, Quaternion};
use std::cmp;
use crate::lib::groove::vars::RelaxedIKVars;

// (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g

// function groove_loss(x_val, t, d, c, f, g)
//    return (-2.718281828459^((-(x_val - t)^d) / (2.0 * c^2)) ) + f * (x_val - t)^g
// end



pub fn groove_loss(x_val: f64, t: f64, d: i32, c: f64, f: f64, g: i32) -> f64 {
    -( (-(x_val - t).powi(d)) / (2.0 * c.powi(2) ) ).exp() + f * (x_val - t).powi(g)
}

pub fn test(x: &[f64], v: &mut vars::RelaxedIKVars) -> f64 {
    v.robot.arms[0].get_frames(v.xopt.as_slice());
    v.xopt[0] + v.robot.arms[0].out_positions[3][2]
}

pub fn match_ee_pos_goals_obj(x: &[f64], v: &mut vars::RelaxedIKVars) -> f64 {
    let mut x_val = 0.0;
    for i in 0..v.robot.num_chains {
        let last_elem = v.robot.arms[i].out_positions.len() - 1;
        x_val += (v.robot.arms[i].out_positions[last_elem] - v.goal_positions[i]).norm()
    }
    groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
}

pub fn match_ee_pos_goals_obj_immutable(x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)> )  -> f64 {
    let mut x_val = 0.0;
    for i in 0..v.robot.num_chains {
        let last_elem = frames[i].0.len() - 1;
       // x_val += (v.robot.arms[i].out_positions[v.robot.arms[i].num_dof] - v.goal_positions[i]).norm()
        x_val += ( frames[i].0[last_elem] - v.goal_positions[i] ).norm();
    }
    groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
}

pub fn match_ee_pos_goals_obj_immutable_lite(x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)> ) -> f64 {
    let mut x_val = 0.0;
    for i in 0..v.robot.num_chains {
        x_val += ( ee_poses[i].0 - v.goal_positions[i] ).norm();
    }
    groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
}

pub fn match_ee_quat_goals_obj(x: &[f64], v: &mut vars::RelaxedIKVars) -> f64 {
    let mut x_val = 0.0;
    for i in 0..v.robot.num_chains {
        let e = Quaternion::new( -v.goal_quats[i].w, -v.goal_quats[i].i, -v.goal_quats[i].j, -v.goal_quats[i].k);
        let ee_quat2 = UnitQuaternion::from_quaternion(e);

        let last_elem = v.robot.arms[i].out_rot_quats.len() - 1;
        let disp = angle_between(v.robot.arms[i].out_rot_quats[last_elem], v.goal_quats[i]);
        let disp2 = angle_between(v.robot.arms[i].out_rot_quats[last_elem], v.goal_quats[i]);
        x_val += disp.min(disp2);
    }
    groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
}

pub fn match_ee_quat_goals_obj_immutable(x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)> ) -> f64 {
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

pub fn match_ee_quat_goals_obj_immutable_lite(x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)> ) -> f64 {
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


pub struct ObjectiveMasterRIK {
    objectives: Vec<fn(&[f64], v: &mut RelaxedIKVars) -> f64>,
    weight_priors: Vec<f64>
}
impl ObjectiveMasterRIK {
    pub fn get_standard_ik() -> Self {
        let objectives: Vec<fn(&[f64], v: &mut RelaxedIKVars) -> f64> = vec![match_ee_pos_goals_obj, match_ee_quat_goals_obj];
        let weight_priors = vec![2.0, 1.5];
        ObjectiveMasterRIK{objectives, weight_priors}
    }

    pub fn call(&self, x: &[f64], v: &mut RelaxedIKVars) -> f64 {
        v.robot.get_frames(x);
        let mut sum = 0.0;
        for i in 0..self.weight_priors.len() {
            sum += self.weight_priors[i] * (self.objectives[i])(x, v);
        }
        sum
    }
}


pub struct ObjectiveMasterRIKImmutable {
    objectives: Vec<fn(&[f64], v: &RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64>,
    weight_priors: Vec<f64>
}
impl ObjectiveMasterRIKImmutable {
    pub fn get_standard_ik() -> Self {
        let objectives: Vec<fn(&[f64], &RelaxedIKVars, &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64> =
            vec![match_ee_pos_goals_obj_immutable, match_ee_quat_goals_obj_immutable];
        let weight_priors = vec![2.0, 1.5];
        ObjectiveMasterRIKImmutable{objectives, weight_priors}
    }

    pub fn call(&self, x: &[f64], v: &RelaxedIKVars) -> f64 {
        let frames = v.robot.get_frames_immutable(x);
        let mut sum = 0.0;
        for i in 0..self.weight_priors.len() {
            sum += self.weight_priors[i] * (self.objectives[i])(x, v, &frames);
        }
        sum
    }
}

pub struct ObjectiveMasterRIKImmutableLite {
    objectives: Vec<fn(&[f64], v: &RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64>,
    weight_priors: Vec<f64>
}
impl ObjectiveMasterRIKImmutableLite {
    pub fn get_standard_ik() -> Self {
        let objectives: Vec<fn(&[f64], &RelaxedIKVars, &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64> =
            vec![match_ee_pos_goals_obj_immutable_lite, match_ee_quat_goals_obj_immutable_lite];
        let weight_priors = vec![2.0, 1.5];
        ObjectiveMasterRIKImmutableLite{objectives, weight_priors}
    }

    pub fn call(&self, x: &[f64], v: &RelaxedIKVars) -> f64 {
        let ee_poses = v.robot.get_ee_pos_and_quat_immutable(x);
        let mut sum = 0.0;
        for i in 0..self.weight_priors.len() {
            sum += self.weight_priors[i] * (self.objectives[i])(x, v, &ee_poses);
        }
        sum
    }
}


