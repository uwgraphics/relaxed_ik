mod lib;

use crate::lib::utils_rust::yaml_utils::{CollisionNeuralNetParser};
use nalgebra;
use crate::lib::groove::collision_nn::{CollisionNN, get_relu_jacobian};
use std::time::{Duration, Instant};

fn test(x: f64) -> f64 {
    2.0 * x
}

fn relu(x: f64) -> f64 {
    x.max(0.0)
}

fn main() {
    let mut cnn = CollisionNN::from_yaml_path("/home/rakita/catkin_ws/src/relaxed_ik/src/RelaxedIK/Config/collision_nn_rust/hubo_description_nn.yaml".to_string());
    let x = vec![0., 0., 0., 0., 0., 0.,1.,0., 0., 0., 0., 0., 0.,1.,0.];
    let start = Instant::now();
    for i in 0..1000 {
        cnn.predict(&x);
    }
    let duration = start.elapsed();
    println!("{:?}", duration);


    let start = Instant::now();
    for i in 0..1000 {
        cnn.gradient(&x);
    }
    let duration = start.elapsed();
    println!("{:?}", duration);

    let start = Instant::now();
    for i in 0..1000 {
        cnn.gradient2(&x);
    }
    let duration = start.elapsed();
    println!("{:?}", duration);

    let start = Instant::now();
    for i in 0..1000 {
        cnn.gradient_finite_diff(&x);
    }
    let duration = start.elapsed();
    println!("{:?}", duration);

    println!("{:?}", cnn.predict(&x));
    println!("{:?}", cnn.gradient_finite_diff(&x));
    println!("{:?}", cnn.gradient(&x));
    println!("{:?}", cnn.gradient2(&x));

}