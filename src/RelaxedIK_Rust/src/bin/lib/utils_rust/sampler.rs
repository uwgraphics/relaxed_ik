use nalgebra::DVector;
extern crate rand;
use rand::distributions::{Distribution, Uniform};
use rand::{Rng};
use rand::rngs::{ThreadRng, StdRng};
use crate::lib::spacetime::robot::Robot;
use std::marker::{Sync, Send};


pub trait Sampler {
    fn sample(&mut self) -> DVector<f64>;
}

pub trait ThreadSampler: Send + Sync {
    fn sample(&self) -> DVector<f64>;
}

pub struct NullSampler2D;
impl Sampler for NullSampler2D {
    fn sample(&mut self) -> DVector<f64> {
        return DVector::from_element(2, 0.0);
    }
}
impl ThreadSampler for NullSampler2D {
    fn sample(&self) -> DVector<f64> {
        return DVector::from_element(2, 0.0);
    }
}

pub struct NullSamplerND {
    pub dim: usize
}
impl Sampler for NullSamplerND {
    fn sample(&mut self) -> DVector<f64> {
        return DVector::from_element(self.dim, 0.0);
    }
}
impl ThreadSampler for NullSamplerND {
    fn sample(&self) -> DVector<f64> {
        return DVector::from_element(self.dim, 0.0);
    }
}

pub struct RangeSampler {
    pub upper_bound: f64,
    pub lower_bound: f64,
    pub dim: usize,
    rng: ThreadRng,
    u: Uniform<f64>
}
impl RangeSampler {
    pub fn new(lower_bound: f64, upper_bound: f64, dim: usize) -> Self {
        let mut rng = rand::thread_rng();
        let u = Uniform::from(lower_bound..upper_bound);
        Self{upper_bound, lower_bound, dim, rng, u}
    }
}
impl Sampler for RangeSampler {
    fn sample(&mut self) -> DVector<f64> {
        let mut v =  DVector::from_element(self.dim, 0.0);
        for i in 0..self.dim {
            v[i] = self.u.sample(&mut self.rng);
        }
        v
    }
}


pub struct ThreadRangeSampler {
    pub upper_bound: f64,
    pub lower_bound: f64,
    pub dim: usize
}
impl ThreadRangeSampler {
    pub fn new(lower_bound: f64, upper_bound: f64, dim: usize) -> Self {
        Self{upper_bound, lower_bound, dim}
    }
}
impl ThreadSampler for ThreadRangeSampler {
    fn sample(&self) -> DVector<f64> {
        let mut v =  DVector::from_element(self.dim, 0.0);
        let mut rng = rand::thread_rng();
        for i in 0..self.dim {
            v[i] = rng.gen_range(self.lower_bound, self.upper_bound);
        }
        v
    }
}



pub struct RobotSampler {
    pub robot: Robot,
    pub dim: usize,
    rng: ThreadRng,
    us: Vec<Uniform<f64>>
}
impl RobotSampler {
    pub fn new(robot: Robot) -> Self {
        let mut rng = rand::thread_rng();
        let num_dof = robot.num_dof;
        // let us = Uniform::from(lower_bound..upper_bound);
        let mut us: Vec<Uniform<f64>> = Vec::new();
        for i in 0..robot.num_dof {
            let u = Uniform::from(robot.bounds[i][0]..robot.bounds[i][1]);
            us.push(u);
        }
        Self{robot, dim: num_dof, rng, us}
    }
}
impl Sampler for RobotSampler {
    fn sample(&mut self) -> DVector<f64> {
        let mut v =  DVector::from_element(self.dim, 0.0);
        for i in 0..self.dim {
            v[i] = self.us[i].sample(&mut self.rng);
        }
        v
    }
}


pub struct ThreadRobotSampler {
    pub robot: Robot,
    pub dim: usize
}
impl ThreadRobotSampler {
    pub fn new(robot: Robot) -> Self {
        let num_dof = robot.num_dof;
        Self{robot, dim: num_dof}
    }
}
impl ThreadSampler for ThreadRobotSampler {
    fn sample(&self) -> DVector<f64> {
        let mut rng = rand::thread_rng();
        let mut v =  DVector::from_element(self.dim, 0.0);
        for i in 0..self.dim {
            v[i] = rng.gen_range(self.robot.bounds[i][0], self.robot.bounds[i][1]);
        }
        v
    }
}