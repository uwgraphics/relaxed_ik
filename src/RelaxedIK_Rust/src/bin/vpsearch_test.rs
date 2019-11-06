extern crate vpsearch;
use std::time::{Instant, Duration};
// use rand::distributions::{Distribution, Uniform};


fn rand_data() -> [f64; 3] {
    rand::random()
}

#[derive(Copy, Clone)]
struct Point {
    x: f32, y: f32,
}

impl vpsearch::MetricSpace for Point {
    type UserData = ();
    type Distance = f32;

    fn distance(&self, other: &Self, _: &Self::UserData) -> Self::Distance {
        let dx = self.x - other.x;
        let dy = self.y - other.y;

        (dx*dx + dy*dy).sqrt() // sqrt is required
    }
}

#[derive(Copy, Clone)]
struct MultiPoint<'a> {
    p: &'a [f64]
}

impl<'a> vpsearch::MetricSpace for MultiPoint<'a> {
    type UserData = ();
    type Distance = f64;

    fn distance(&self, other: &Self, _: &Self::UserData) -> Self::Distance {
        let mut sum = 0.0;
        for i in 0..self.p.len() {
            sum += (other.p[i] - self.p[i]).powi(2);
        }
        sum.sqrt()
    }
}

fn main() {
    let num_points = 100000;
    let mut rand_points: Vec<Vec<f64>> = Vec::new();
    for i in 0..num_points {
        let d = rand_data();
        rand_points.push(d.to_vec());
    }
    let mut points: Vec<MultiPoint> = Vec::new();
    for i in 0..num_points {
        points.push(MultiPoint{p: rand_points[i].as_slice()})
    }
    // println!("{:?}", rand_points);
    // println!("{:?}", points[0]);

    // let points = vec![MultiPoint{ p: &[1.,2.,3.]}, MultiPoint{p: &[4.,5.,6.]}, MultiPoint{p: &[7.,8.,9.]}  ];
    let vp = vpsearch::Tree::new(&points);
    println!("got here");

    let (index, dis) = vp.find_nearest(&MultiPoint{p: &[0.,0.,0.]});

    let start = Instant::now();
    for i in 0..1000 {
        let (index, dis) = vp.find_nearest(&MultiPoint { p: &[0.,0.,0.] });
    }
    let duration = start.elapsed();
    println!("{:?}", duration);

    println!("{:?}", dis);
    println!("{:?}", rand_points[index]);

    /*
    let points = vec![Point{x:2.0,y:3.0}, Point{x:0.0,y:1.0}, Point{x:4.0,y:5.0}];

    let vp = vpsearch::Tree::new(&points);
    let (index, _) = vp.find_nearest(&Point { x: 1.0, y: 2.0 });

    let start = Instant::now();
    for i in 0..100 {
        let (index, _) = vp.find_nearest(&Point { x: 1.0, y: 2.0 });
    }
    let duration = start.elapsed();
    println!("{:?}", duration);

    println!("The nearest point is at ({}, {})", points[index].x, points[index].y);
    */
}