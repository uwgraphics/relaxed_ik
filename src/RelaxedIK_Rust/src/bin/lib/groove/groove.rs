use crate::lib::groove::objective::{ObjectiveMasterRIK, ObjectiveMasterRIKImmutable, ObjectiveMasterRIKImmutableLite};
use crate::lib::groove::gradient::{ForwardFiniteDiff, CentralFiniteDiff, GradientFinder, ForwardFiniteDiffImmutable, CentralFiniteDiffImmutable, GradientFinderImmutable};
use crate::lib::groove::vars::{RelaxedIKVars};
use optimization_engine::{constraints::*, panoc::*, *};
use nlopt::*;

pub struct OptimizationEngineOpen {
    dim: usize,
    cache: PANOCCache
}
impl OptimizationEngineOpen {
    pub fn new(dim: usize) -> Self {
        let mut cache = PANOCCache::new(dim, 1e-14, 10);
        OptimizationEngineOpen{dim, cache}
    }

    pub fn optimize(&mut self, x_out: &mut [f64], v1: &RelaxedIKVars, v2: &RelaxedIKVars, om: &ObjectiveMasterRIKImmutable, max_iter: usize) {
        let mut obj1 = |x: &[f64]| om.call(x, v1);
        let mut obj2 = |x: &[f64]| om.call(x, v2);
        let mut gradient_finder = ForwardFiniteDiffImmutable::new(self.dim, obj1);

        let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
            let my_grad = gradient_finder.compute_gradient_immutable(u);
            for i in 0..my_grad.len() {
                grad[i] = my_grad[i];
            }
            Ok(())
        };

        let f = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
            *c = obj2(u);
            Ok(())
        };

        let bounds = NoConstraints::new();

        /* PROBLEM STATEMENT */
        let problem = Problem::new(&bounds, df, f);
        let mut panoc = PANOCOptimizer::new(problem, &mut self.cache).with_max_iter(max_iter).with_tolerance(0.00005);

        // Invoke the solver
        let status = panoc.solve(x_out);

        //println!("Panoc status: {:#?}", status);
        //println!("Panoc solution: {:#?}", x_out);

    }

    pub fn optimize_lite(&mut self, x_out: &mut [f64], v1: &RelaxedIKVars, v2: &RelaxedIKVars, om: &ObjectiveMasterRIKImmutableLite, max_iter: usize) {
        let mut obj1 = |x: &[f64]| om.call(x, v1);
        let mut obj2 = |x: &[f64]| om.call(x, v2);
        let mut gradient_finder = ForwardFiniteDiffImmutable::new(self.dim, obj1);

        let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
            let my_grad = gradient_finder.compute_gradient_immutable(u);
            for i in 0..my_grad.len() {
                grad[i] = my_grad[i];
            }
            Ok(())
        };

        let f = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
            *c = obj2(u);
            Ok(())
        };

        let bounds = NoConstraints::new();

        /* PROBLEM STATEMENT */
        let problem = Problem::new(&bounds, df, f);
        let mut panoc = PANOCOptimizer::new(problem, &mut self.cache).with_max_iter(max_iter).with_tolerance(0.00005);

        // Invoke the solver
        let status = panoc.solve(x_out);

        //println!("Panoc status: {:#?}", status);
        //println!("Panoc solution: {:#?}", x_out);

    }
}



pub struct OptimizationEngineNLoptImmutable;
impl OptimizationEngineNLoptImmutable {
    pub fn new() -> Self { OptimizationEngineNLoptImmutable{} }

    pub fn optimize(&mut self, x_out: &mut [f64], v1: &RelaxedIKVars, v2: &RelaxedIKVars, om: &ObjectiveMasterRIKImmutable, max_iter: u32) {
        let num_dim = v1.robot.num_dof;
        let obj = |x: &[f64]| om.call(x, v1);
        let mut gradient_finder= ForwardFiniteDiffImmutable::new(v1.robot.num_dof, obj);

        // ForwardFiniteDiff<impl FnMut(&[f64]) -> f64>
        // println!("{:?}", workspace.gradient_finder.out_grad);

        let obj_f = |x: &[f64], _gradient: Option<&mut [f64]>, _params: &mut ()| -> f64 {
            if _gradient.is_none() {
            } else {
                let mut grad = gradient_finder.compute_gradient_immutable(x);
                let g = _gradient.unwrap();
                for i in 0..grad.len() {
                    g[i] = grad[i];
                }
            }
            obj(x)
        };

        let mut opt = Nlopt::new(Algorithm::Slsqp, num_dim, obj_f, Target::Minimize, ());
        opt.set_ftol_rel(0.0000001);

        let mut x_init = x_out.to_vec();
        let res = opt.optimize(x_init.as_mut_slice());

        // println!("Result: {:?}", res);
        // println!("X vals: {:?}\n", &x_init[..num_dim]);
    }

    pub fn optimize_lite(&mut self, x_out: &mut [f64], v1: &RelaxedIKVars, v2: &RelaxedIKVars, om: &ObjectiveMasterRIKImmutableLite, max_iter: u32) {
        let num_dim = v1.robot.num_dof;
        let obj = |x: &[f64]| om.call(x, v1);
        let mut gradient_finder= ForwardFiniteDiffImmutable::new(v1.robot.num_dof, obj);

        // ForwardFiniteDiff<impl FnMut(&[f64]) -> f64>
        // println!("{:?}", workspace.gradient_finder.out_grad);

        let obj_f = |x: &[f64], _gradient: Option<&mut [f64]>, _params: &mut ()| -> f64 {
            if _gradient.is_none() {
            } else {
                let mut grad = gradient_finder.compute_gradient_immutable(x);
                let g = _gradient.unwrap();
                for i in 0..grad.len() {
                    g[i] = grad[i];
                }
            }
            obj(x)
        };

        let mut opt = Nlopt::new(Algorithm::Slsqp, num_dim, obj_f, Target::Minimize, ());
        opt.set_ftol_rel(0.0000001);

        let mut x_init = x_out.to_vec();
        let res = opt.optimize(x_init.as_mut_slice());

        // println!("Result: {:?}", res);
        // println!("X vals: {:?}\n", &x_init[..num_dim]);
    }
}

/*
pub struct OptimizationEngineNLopt;
impl OptimizationEngineNLopt {
    pub fn new() -> Self { OptimizationEngineNLopt{} }

    /*
    pub fn optimize(&mut self, x_out: &mut [f64], v1: &mut RelaxedIKVars, v2: &mut RelaxedIKVars, om: &ObjectiveMasterRIK, max_iter: u32) {
        let num_dim = v1.robot.num_dof;
        let obj = |x: &[f64]| om.call(x, v1);
        let mut gradient_finder= ForwardFiniteDiffImmutable::new(v1.robot.num_dof, obj);

        // ForwardFiniteDiff<impl FnMut(&[f64]) -> f64>
        // println!("{:?}", workspace.gradient_finder.out_grad);

        let obj_f = |x: &[f64], _gradient: Option<&mut [f64]>, _params: &mut ()| -> f64 {
            if _gradient.is_none() {
            } else {
                let mut grad = gradient_finder.compute_gradient_immutable(x);
                let g = _gradient.unwrap();
                for i in 0..grad.len() {
                    g[i] = grad[i];
                }
            }
            obj(x)
        };

        let mut opt = Nlopt::new(Algorithm::Slsqp, num_dim, obj_f, Target::Minimize, ());
        opt.set_ftol_rel(0.0000001);

        let mut x_init = x_out.to_vec();
        let res = opt.optimize(x_init.as_mut_slice());

        // println!("Result: {:?}", res);
        // println!("X vals: {:?}\n", &x_init[..num_dim]);
    }
    */

    pub fn optimize_lite(&mut self, x_out: &mut [f64], v1: &mut RelaxedIKVars, v2: &mut RelaxedIKVars, om: &ObjectiveMasterRIK, max_iter: u32) {
        let num_dim = v1.robot.num_dof;
        let obj = |x: &[f64]| om.call(x, v1);
        let mut gradient_finder= ForwardFiniteDiff2::new(v1.robot.num_dof, &obj);
        let mut workspace = OptimizationWorkspace{vars: v2, gradient_finder: &gradient_finder};

        // ForwardFiniteDiff<impl FnMut(&[f64]) -> f64>
        // println!("{:?}", workspace.gradient_finder.out_grad);

        let obj_f = |x: &[f64], _gradient: Option<&mut [f64]>, _params: &mut OptimizationWorkspace| -> f64 {
            if _gradient.is_none() {
            } else {
                _params.gradient_finder.compute_gradient(x);
                let mut grad = _params.gradient_finder.out_grad;
                let g = _gradient.unwrap();
                for i in 0..grad.len() {
                    g[i] = grad[i];
                }
            }
            obj(x)
        };

        let mut opt = Nlopt::new(Algorithm::Slsqp, num_dim, obj_f, Target::Minimize,  workspace);
        opt.set_ftol_rel(0.0000001);

        let mut x_init = x_out.to_vec();
        let res = opt.optimize(x_init.as_mut_slice());

        // println!("Result: {:?}", res);
        // println!("X vals: {:?}\n", &x_init[..num_dim]);
    }
}

pub struct OptimizationWorkspace<'a> {
    pub vars: &'a mut RelaxedIKVars,
    pub gradient_finder: ForwardFiniteDiff2<'a, &'a Fn(&[f64]) -> f64 >
}
*/