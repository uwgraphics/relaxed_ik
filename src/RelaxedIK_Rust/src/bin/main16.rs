use nalgebra::Vector6;
use std::time::{Instant, Duration};

fn main() {
    let v1 = Vector6::new(0.,1.,2.,3.,4.,5.);
    let v2 = Vector6::new(0.,1.,2.1,3.,4.,5.);

    let mut e: f64 = (v1 - v2).norm();
    let start = Instant::now();
    for i in 0..5000 {
        e += ((v1 - v2).norm() as f64).exp() / 2.0f64.powi(2);
    }
    let duration = start.elapsed();
    println!("{:?}", duration);
    println!("{:?}", e);

}