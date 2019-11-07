
mod lib;

use lib::groove::vars::Vars;
use lib::{utils_rust, spacetime};
use lib::groove::tools::RelaxedIKTools;
use lib::groove::objective_old::*;
use std::marker::PhantomData;
use std::time::{Instant, Duration};

fn main() {
    let mut tools1 = RelaxedIKTools::from_yaml_path("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/info_files/ur5_info.yaml");
    // let mut vars = Vars::new(vec![0.,0.,0.,0.,0.,0.]);

    // vars.update(vec![1.,1.,1.,1.,1.,1.]);
    // tools1.robot.arms[0].get_frames(&[0.,0.,0.,0.,0.,0.]);
    // println!("{:?}", tools1.robot.split_into_subchains(&[1.,0.,0.,0.,0.,0.,0.,2.,0.,0.,0.,0.,0.,0.,0.]));
    let start = Instant::now();
    for i in 0..1000 {
        tools1.robot.get_frames(&[0., 0., 0., 0., 0., 0.]);
    }
    let duration = start.elapsed();
    println!("{:?}", duration);

    let start = Instant::now();
    for i in 0..1000 {
        // tools1.robot.arms[0].get_frames(&[1., 0., 0., 0., 0., 0.]);
    }
    let duration = start.elapsed();
    println!("{:?}", duration);
    println!("{:?}", tools1.robot.arms[0].out_positions);
    // println!("{:?}", tools1.robot.arms[0].out_positions);

    // let to1: TestObjective<Vars, RelaxedIKTools> = TestObjective::new(tools1.clone());
    // let to2: TestObjective<Vars, RelaxedIKTools> = TestObjective::new(tools1.clone());
    // to2.call(&[0.,0.,0.,0.,0.,0.], &vars);
    // to1.call(&[0.,0.,0.,0.,0.,0.], &vars);

    // let mut o = Objective{f: test};
    // let mut o2 = Objective{f: test2};

    // println!("{:?}", (o.f)(&[1.,1.], &vars, &mut tools1));
    // vars.update(vec![2.,1.,1.,1.,1.,1.]);
    // println!("{:?}", (o2.f)(&[1.,1.], &vars, &mut tools1));

    // let v = [test, test2, test, test2];
    // let mut v2: Vec<fn(&[f64], &Vars, &mut RelaxedIKTools) -> f64> = Vec::new();
    //
    /*
    for i in 0..4 {
        vars.update(vec![2. + i as f64,1.,1.,1.,1.,1.]);
        println!("{:?}", (v[i])(&[1.,1.], &vars, &mut tools1));
    }
    */
}