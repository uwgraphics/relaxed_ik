pub fn forward_finite_diff1(f: &dyn Fn(&Vec<f64>) -> f64, x: &Vec<f64>, h: f64, output_grad: &mut Vec<f64>) {
    assert_eq!(x.len(), output_grad.len());
    let val_0 = f(x);
    for (i, val) in x.iter().enumerate() {
        let mut x_h = x.clone();
        x_h[i] += h;
        output_grad[i] = (-val_0 + f(&x_h)) / h;
    }
}

pub fn backward_finite_diff1(f: &dyn Fn(&Vec<f64>) -> f64, x: &Vec<f64>, h: f64, output_grad: &mut Vec<f64>) {
    assert_eq!(x.len(), output_grad.len());
    let val_0 = f(x);
    for (i, val) in x.iter().enumerate() {
        let mut x_h = x.clone();
        x_h[i] -= h;
        output_grad[i] = (val_0 - f(&x_h)) / h;
    }
}

pub fn central_finite_diff1(f: &dyn Fn(&Vec<f64>) -> f64, x: &Vec<f64>, h: f64, output_grad: &mut Vec<f64>) {
    assert_eq!(x.len(), output_grad.len());
    let mut x_hf = x.clone();
    let mut x_hb = x.clone();

    for (i, val) in x.iter().enumerate() {
        x_hf = x.clone();
        x_hb = x.clone();
        x_hf[i] += h;
        x_hb[i] -= h;
        output_grad[i] = (-0.5 * f(&x_hb) + 0.5 * f(&x_hf)) / h
    }
}

pub fn central_finite_diff2(f: &dyn Fn(&Vec<f64>) -> f64, x: &Vec<f64>, h: f64, output_grad: &mut Vec<f64>) {
    assert_eq!(x.len(), output_grad.len());

    let mut x_hf1 = x.clone();
    let mut x_hb1 = x.clone();
    let mut x_hf2 = x.clone();
    let mut x_hb2 = x.clone();

    for (i, val) in x.iter().enumerate() {
        x_hf1 = x.clone();
        x_hb1 = x.clone();
        x_hf2 = x.clone();
        x_hb2 = x.clone();
        x_hf1[i] += h;
        x_hb1[i] -= h;
        x_hf2[i] += 2. * h;
        x_hb2[i] -= 2. * h;
        output_grad[i] = ((1. / 12.) * f(&x_hb2) + -(2. / 3.) * f(&x_hb1) + (2. / 3.) * f(&x_hf1) - (1. / 12.) * f(&x_hf2)) / h;
    }
}
