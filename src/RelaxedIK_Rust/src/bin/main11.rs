
// return (    -2.718281828459^(  (-(x_val - t)^d) / (2.0 * c^2) ) ) + f * (x_val - t)^g

pub fn groove_loss(x_val: f64, t: f64, d: i32, c: f64, f: f64, g: i32) -> f64 {
    ( (-(x_val - t).powi(d)) / (2.0 * c.powi(2) ) ).exp() + f * (x_val - t).powi(g)
}

fn main() {
    let mut o: Option<&[f64]> = Option::Some(&[1.,2.,3.]);
    println!("{:?}", o);
    let mut o: Option<&[f64]> = Option::Some(&[1.,2.,3.]);
    println!("{:?}", o);
}