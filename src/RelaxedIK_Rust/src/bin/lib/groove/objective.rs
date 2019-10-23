// use crate::utils_rust::gradient_utils::{ForwardFiniteDiff, CentralFiniteDiff, GradientFinder};

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



