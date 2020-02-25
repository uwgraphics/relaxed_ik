use crate::lib::groove::gradient::{ForwardFiniteDiff, CentralFiniteDiff, GradientFinder, ForwardFiniteDiffImmutable, CentralFiniteDiffImmutable, GradientFinderImmutable};
use crate::lib::groove::vars::{RelaxedIKVars};
use optimization_engine::{constraints::*, panoc::*, *};
use nlopt::*;
use crate::lib::groove::objective_master::ObjectiveMaster;

pub struct OptimizationEngineOpen {
    dim: usize,
    cache: PANOCCache
}
impl OptimizationEngineOpen {
    pub fn new(dim: usize) -> Self {
        let mut cache = PANOCCache::new(dim, 1e-14, 10);
        OptimizationEngineOpen { dim, cache }
    }

    pub fn optimize(&mut self, x: &mut [f64], v: &RelaxedIKVars, om: &ObjectiveMaster, max_iter: usize) {
        let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
            let (my_obj, my_grad) = om.gradient(u, v);
            for i in 0..my_grad.len() {
                grad[i] = my_grad[i];
            }
            Ok(())
        };

        let f = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
            *c = om.call(u, v);
            Ok(())
        };

        // let bounds = NoConstraints::new();
        let bounds = Rectangle::new(Option::from(v.robot.lower_bounds.as_slice()), Option::from(v.robot.upper_bounds.as_slice()));

        /* PROBLEM STATEMENT */
        let problem = Problem::new(&bounds, df, f);
        let mut panoc = PANOCOptimizer::new(problem, &mut self.cache).with_max_iter(max_iter).with_tolerance(0.0005);
        // let mut panoc = PANOCOptimizer::new(problem, &mut self.cache);

        // Invoke the solver
        let status = panoc.solve(x);

        // println!("Panoc status: {:#?}", status);
        // println!("Panoc solution: {:#?}", x);
    }
}

pub struct OptimizationEngineNLopt;
impl OptimizationEngineNLopt {
    pub fn new() -> Self { OptimizationEngineNLopt{} }

    pub fn optimize(&mut self, x_out: &mut [f64], v: &RelaxedIKVars, om: &ObjectiveMaster, max_iter: u32) {
        let num_dim = v.robot.num_dof;

        let obj_f = |x: &[f64], _gradient: Option<&mut [f64]>, _params: &mut ()| -> f64 {
            let (my_f, my_grad) = om.gradient(x, v);
            if _gradient.is_none() {
            } else {
                let g = _gradient.unwrap();
                for i in 0..my_grad.len() {
                    g[i] = my_grad[i];
                }
            }
            my_f
        };

        let mut opt = Nlopt::new(Algorithm::Slsqp, num_dim, obj_f, Target::Minimize, ());
        opt.set_ftol_rel(0.000001);
        opt.set_maxeval(max_iter);

        // let mut x_init = x_out.to_vec();
        let res = opt.optimize(x_out);

        // println!("X vals: {:?}\n", &x_out[..num_dim]);

    }
}