use ndarray::{Array, OwnedRepr, ArrayBase};
use ndarray_rand::RandomExt;
use ndarray_rand::rand_distr::Uniform;
use std::time::{Duration, Instant};
use nalgebra::{DVector, DMatrix, Matrix, U50, MatrixArray};

fn multiply_vector(scalar: f64, vec: &Vec<f64>) -> Vec<f64> {
    let mut out: Vec<f64> = Vec::new();
    for i in 0..vec.len() {
        out.push(vec[i] * scalar);
    }
    out
}

fn main() {
    let dim = 20;
    let mut e = Array::random((70, dim), Uniform::new(0., 10.));
    let mut a = Array::random((dim, dim), Uniform::new(0., 10.));
    let mut b = Array::random((dim, dim), Uniform::new(0., 10.));
    let mut x = Array::random((dim, 1), Uniform::new(0., 10.));
    let start = Instant::now();
    for i in 0..1000 {
        for i in 0..7 {
            x = a.dot(&x)
        }
    }
    println!("{:?}", x);
    let duration = start.elapsed();
    println!("{:?}", duration);

    a[[1,1]] = 1.0;
    // println!("{:?}", a);

    let mut d = Array::random((dim), Uniform::new(0., 10.));
    // println!("{:?}", &d);
    let start = Instant::now();
    for i in 0..1000 {
        &d + &d;
    }
    let duration = start.elapsed();
    // println!("{:?}", duration);

    let test_vec = vec![0.,1.,2.,3.,4.,5.,6.];
    let mut d = Array::random((dim), Uniform::new(0., 10.));
    // println!("{:?}", &d);
    let start = Instant::now();
    for i in 0..1000 {
        multiply_vector(3.0, &test_vec);
    }
    let duration = start.elapsed();
    // println!("{:?}", duration);

    let mut dynamic_v = DVector::from_element(10, 0.0);
    dynamic_v[0] = 10.0;
    let start = Instant::now();
    for i in 0..1000 {
        &dynamic_v * 3.0;
    }
    let duration = start.elapsed();
    // println!("{:?}", duration);

    let mut dynamic_m = DMatrix::from_element(dim, dim, 0.5);
    let mut dynamic_v = DVector::from_element(dim, 1.001);
    let mut x = DVector::from_element(dim, 1.0);
    let start = Instant::now();
    for i in 0..1000 {
        for i in 0..7 {
            x = &dynamic_m * &dynamic_v + &dynamic_v;
        }
    };
    let duration = start.elapsed();
    println!("{:?}", duration);

    println!("{:?}", x);
    // let dynamic_m = DMatrix::from_element(dim, dim, 0.0);

    // Example Output:
    // [[  8.6900,   6.9824,   3.8922,   6.5861,   2.4890],
    //  [  0.0914,   5.5186,   5.8135,   5.2361,   3.1879]]
    let start = Instant::now();

    let a = 1.0;
    let b = 1.0;
    let mut c = 1.0;
    for i in 0..10000 {
        c = c - a - b;
    }

    // let mut m1 = DMatrix::from_element(2, 3, 1.1);
    // let mut m2 = DMatrix::from_element(3, 2, 1.1);
    // println!("{:?}", m1 * m2);

    // println!("{:?}", m);
}