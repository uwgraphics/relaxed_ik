use crate::lib::groove::objective::ObjectiveMasterRIK;
use crate::lib::groove::gradient::{ForwardFiniteDiff, CentralFiniteDiff, GradientFinder};
use crate::lib::groove::vars::{RelaxedIKVars};
use optimization_engine::{constraints::*, panoc::*, *};

// pub fn optimize_open(vars: &mut RelaxedIKVars, om: &mut ObjectiveMaster) {
//
// }

pub struct OptimizationEngineOpen {
    dim: usize,
    cache: PANOCCache
}
impl OptimizationEngineOpen {
    pub fn new(dim: usize) -> Self {
        let mut cache = PANOCCache::new(dim, 1e-14, 10);
        OptimizationEngineOpen{dim, cache}
    }

    /*
    pub fn optimize(&mut self, x_out: &mut [f64], v1: &mut RelaxedIKVars, v2: &mut RelaxedIKVars, om: &mut ObjectiveMasterRIK, max_iter: usize) {
        let mut obj1 = |x: &[f64]| om.call(x_out.to_vec().as_slice(), v1);
        let mut obj2 = |x: &[f64]| om.call(x_out.to_vec().as_slice(), v2);
        let mut gradient_finder = ForwardFiniteDiff::new(self.dim, obj1);

        let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
            gradient_finder.compute_gradient(u);
            for i in 0..gradient_finder.out_grad.len() {
                grad[i] = gradient_finder.out_grad[i];
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
        let mut panoc = PANOCOptimizer::new(problem, &mut self.cache).with_max_iter(max_iter);

        // Invoke the solver
        let status = panoc.solve(x_out);

        println!("Panoc status: {:#?}", status);
        println!("Panoc solution: {:#?}", x_out);

    }
    */
}


// pub struct OptimizationEngineNLopt {
//
// }