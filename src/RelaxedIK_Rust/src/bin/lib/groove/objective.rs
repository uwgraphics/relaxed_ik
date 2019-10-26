use crate::lib::groove::{vars, tools};
use crate::lib::utils_rust::transformations::{*};
use nalgebra::geometry::{UnitQuaternion, Quaternion};
use std::cmp;
use crate::lib::groove::vars::RelaxedIKVars;

/*
pub struct Objective<'a, G: GradientFinder<'a>> {
    pub dim: usize,
    pub f: &'a dyn Fn(&[f64]) -> f64,
    pub gradient_finder: G
}

impl<'a, G: GradientFinder<'a>> Objective<'a, G> {
    pub fn new(dim: usize, f: &'a dyn Fn(&[f64]) -> f64) -> Self {
        let gradient_finder = G::new(dim, f);
        Self{dim, f, gradient_finder}
    }

    pub fn objective(&self, x: &[f64]) -> f64 {
        (self.f)(x)
    }

    pub fn compute_gradient(&mut self, x: &[f64]) {
        self.gradient_finder.compute_gradient(x);
    }

    pub fn compute_and_return_gradient(&mut self, x: &[f64]) -> Vec<f64> {
        self.gradient_finder.compute_and_return_gradient(x)
    }
}
*/
/*
pub trait ObjectiveTrait<V, T> {
    fn new(t: T) -> Self;
    fn call(&self, x: &[f64], vars: &V) -> f64;
}

pub struct Objective<V,T> {v: PhantomData<V>, t: T}
impl<V,T> ObjectiveTrait<V,T> for Objective<V,T> {
    fn new(t: T) -> Self {
        Objective{v: PhantomData, t}
    }
    fn call(&self, x: &[f64], vars: &V) -> f64 {
        1.0
    }
}

pub struct TestObjective<V,T> {v: PhantomData<V>, t: T}
impl<V,T> ObjectiveTrait<V,T> for TestObjective<V,T> {
    fn new(t: T) -> Self {
        TestObjective{v: PhantomData, t}
    }
    fn call(&self, x: &[f64], vars: &V) -> f64 {
        2.0
    }
}
*/
/*
pub struct Objective<'a, V, T> {
    pub f: &'a FnMut(&[f64]) -> f64,
    pub tools: T,
    pub v: PhantomData<V>
}

impl<'a, V, T>  Objective<'a, V, T>
{
    pub fn test(tools: T) -> Self {
        Objective{f: &|x: &[f64]| 1.0, tools, v: PhantomData}
    }
}
*/
/*
pub struct Objective<F>
where F: FnMut(&[f64], &vars::Vars, &mut tools::RelaxedIKTools) -> f64
{
    pub f: F
}
*/

pub fn test(x: &[f64], v: &mut vars::RelaxedIKVars) -> f64 {
    v.robot.arms[0].get_frames(v.xopt.as_slice());
    v.xopt[0] + v.robot.arms[0].out_positions[3][2]
}

pub fn match_ee_pos_goals_obj(x: &[f64], v: &mut vars::RelaxedIKVars) -> f64 {
    let mut x_val = 0.0;
    for i in 0..v.robot.num_chains {
       x_val += (v.robot.arms[i].out_positions[v.robot.arms[i].num_dof] - v.goal_positions[i]).norm()
    }
    x_val
}

pub fn match_ee_quat_goals_obj(x: &[f64], v: &mut vars::RelaxedIKVars) -> f64 {
    let mut x_val = 0.0;
    for i in 0..v.robot.num_chains {
        let e = Quaternion::new( -v.goal_quats[i].w, -v.goal_quats[i].i, -v.goal_quats[i].j, -v.goal_quats[i].k);
        let ee_quat2 = UnitQuaternion::from_quaternion(e);

        let disp = angle_between(v.robot.arms[i].out_rot_quats[v.robot.arms[i].num_dof], v.goal_quats[i]);
        let disp2 = angle_between(v.robot.arms[i].out_rot_quats[v.robot.arms[i].num_dof], v.goal_quats[i]);
        x_val += disp.min(disp2);
    }
    x_val
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

