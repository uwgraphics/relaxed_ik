mod lib;

use lib::groove::vars::{Vars, RelaxedIKVars};
use lib::{utils_rust, spacetime};
use lib::groove::objective_old::{*};
use lib::groove::groove_old::OptimizationEngineOpen;
use nalgebra::geometry::UnitQuaternion;
use std::time::{Instant, Duration};
use crate::lib::groove::gradient::{ForwardFiniteDiff, CentralFiniteDiff, CentralFiniteDiff2, GradientFinder};
use crate::lib::groove::objective_old::{ObjectiveMasterRIKImmutable, ObjectiveMasterRIK, ObjectiveMasterRIKImmutableLite};

pub fn approximate_gradient(x0: &[f64], f: &dyn Fn(&[f64]) -> f64, grad: &mut [f64]) {
    let n = x0.len();
    let mut x0 = x0.to_vec();
    let eps = std::f64::EPSILON.powf(1.0 / 3.0);
    for i in 0..n {
        let x0i = x0[i];
        x0[i] = x0i - eps;
        let fl = f(&x0);
        x0[i] = x0i + eps;
        let fh = f(&x0);
        grad[i] = (fh - fl) / (2.0 * eps);
        x0[i] = x0i;
    }
}


fn main() {
    let mut v1 = RelaxedIKVars::from_yaml_path("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml", false, false);
    let mut v2 = RelaxedIKVars::from_yaml_path("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml", false, false);
    let om = ObjectiveMasterRIKImmutableLite::get_standard_ik();
    let om2 = ObjectiveMasterRIK::get_standard_ik();

    println!("{:?}", v1.goal_positions);
    let mut o = OptimizationEngineOpen::new(v1.robot.num_dof);
    // let mut x_out = [3.0, -0.38, -1.2, -1.57, -1.57, -1.57];
    let mut x_out = [1.0, -0.0, -1.4, -1.11, -1.57, -1.57];
    o.optimize_lite(&mut x_out, &v1, &v2, &om, 10000);

    // println!("{:?}", om.call(&x_out, &v2));

    // println!("{:?}", v1.goal_positions);
    // v1.goal_positions[0].x += 0.3;
    // println!("{:?}", v1.goal_positions);


    /*
    let mut f = |x: &[f64]| om2.call(x, &mut v1);
    let mut grad_finder = ForwardFiniteDiff::new(6, f);

    let start = Instant::now();
        for i in 0..10000 {
            grad_finder.compute_gradient(&[3., 0., 0., 0., 0., 0.0]);
        }
    let duration = start.elapsed();
    println!("{:?}", duration);

    println!("{:?}", grad_finder.out_grad);
    */
}