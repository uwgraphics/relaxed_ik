use nalgebra::{Vector3, DVector};
use std::time::Instant;

fn main() {
    let d = Vector3::new(1.2,1.2,0.);
    let d2 = Vector3::new(1.,1.,0.);
    let mut d3 = DVector::from_vec(vec![0.,0.,0.,0.,0.,0.]);
    let mut d4 = DVector::from_vec(vec![0.,2.,2.,0.,0.,0.]);

    let mut v = (d2 - d).norm();
    let start = Instant::now();
    for i in 0..700 {
        v += i as f64 + 2.0*(d - d2).norm();
        // v = (&d4 - &d3).norm();
    }
    let duration = start.elapsed();
    println!("{:?}", duration);
    println!("{:?}", v);

    // println!("{:?}", (d2 - d).norm() );
}