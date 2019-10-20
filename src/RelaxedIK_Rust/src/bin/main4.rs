use nalgebra::{Matrix3, Vector3};
use std::time::{Instant, Duration};
pub mod lib;

use lib::{utils_rust, spacetime};

fn main() {
    let mut f: Vec<&dyn Fn(f64) -> nalgebra::Matrix3<f64>> = Vec::new();
    f.push(&spacetime::arm::get_rot_x);
    f.push(&spacetime::arm::get_rot_y);
    f.push(&spacetime::arm::get_rot_z);
    f.push(&spacetime::arm::get_rot_x);
    f.push(&spacetime::arm::get_rot_y);

    let start = Instant::now();
    spacetime::arm::get_rot_x(1.0);
    let duration = start.elapsed();
    println!("{:?}", duration);
}