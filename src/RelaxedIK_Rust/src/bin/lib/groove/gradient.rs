
/*
pub fn forward_finite_diff1(f: &impl FnMut(&[f64]) -> f64, x: &[f64], h: f64, output_grad: &mut [f64]) {
    assert_eq!(x.len(), output_grad.len());
    let val_0 = f(x);
    for (i, val) in x.iter().enumerate() {
        let mut x_h: Vec<f64> = (*x.clone()).to_vec();
        x_h[i] += h;
        output_grad[i] = (-val_0 + f(&x_h)) / h;
    }
}

pub fn backward_finite_diff1(f: &dyn FnMut(&Vec<f64>) -> f64, x: &Vec<f64>, h: f64, output_grad: &mut Vec<f64>) {
    assert_eq!(x.len(), output_grad.len());
    let val_0 = f(x);
    for (i, val) in x.iter().enumerate() {
        let mut x_h = x.clone();
        x_h[i] -= h;
        output_grad[i] = (val_0 - f(&x_h)) / h;
    }
}

pub fn central_finite_diff1(f: &dyn FnMut(&Vec<f64>) -> f64, x: &Vec<f64>, h: f64, output_grad: &mut Vec<f64>) {
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

pub fn central_finite_diff2(f: &dyn FnMut(&Vec<f64>) -> f64, x: &Vec<f64>, h: f64, output_grad: &mut Vec<f64>) {
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
*/

/*
pub trait GradientFinder<'a> {
    fn new(dim: usize, f: &'a dyn FnMut(&[f64]) -> f64) -> Self;
    fn compute_gradient(&mut self, x: &[f64]);
    fn compute_and_return_gradient(&mut self, x: &[f64]) -> Vec<f64>;
    fn reset(&mut self, x: &[f64]);
}

pub struct ForwardFiniteDiff<'a> {
    pub dim: usize,
    pub f: &'a dyn FnMut(&[f64]) -> f64,
    pub h: f64,
    pub out_grad: Vec<f64>,
    __x_h: Vec<f64>
}
impl<'a> GradientFinder<'a> for ForwardFiniteDiff<'a> {
    fn new(dim: usize, f: &'a dyn FnMut(&[f64]) -> f64) -> ForwardFiniteDiff<'a> {
        let mut out_grad: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            out_grad.push(0.0);
        }

        let mut __x_h: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            __x_h.push(0.0);
        }
        ForwardFiniteDiff{dim, f, h: 0.00000000001, out_grad, __x_h}
    }

    fn compute_gradient(&mut self, x: &[f64]) {
        let val_0 = (self.f)(x);
        for (i, val) in x.iter().enumerate() {
            self.reset(x);
            self.__x_h[i] += self.h;
            self.out_grad[i] = (-val_0 + (self.f)(&self.__x_h)) / self.h;
        }
    }

    fn compute_and_return_gradient(&mut self, x: &[f64]) -> Vec<f64> {
        self.compute_gradient(x);
        self.out_grad.clone()
    }

    fn reset(&mut self, x: &[f64]) {
        for i in 0..self.dim {
            self.__x_h[i] = x[i];
        }
    }
}

pub struct CentralFiniteDiff<'a> {
    pub dim: usize,
    pub f: &'a dyn FnMut(&[f64]) -> f64,
    pub h: f64,
    pub out_grad: Vec<f64>,
    __x_hf: Vec<f64>,
    __x_hb: Vec<f64>
}
impl<'a> GradientFinder<'a> for CentralFiniteDiff<'a> {
    fn new(dim: usize, f: &'a dyn FnMut(&[f64]) -> f64) -> CentralFiniteDiff<'a> {
        let mut out_grad: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            out_grad.push(0.0);
        }

        let mut __x_hf: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            __x_hf.push(0.0);
        }
        let mut __x_hb: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            __x_hb.push(0.0);
        }

        CentralFiniteDiff{dim, f, h: 0.00000000001, out_grad, __x_hf, __x_hb}
    }

    fn compute_gradient(&mut self, x: &[f64]) {
        let val_0 = (self.f)(x);
        for (i, val) in x.iter().enumerate() {
            self.reset(x);
            self.__x_hf[i] += self.h;
            self.__x_hb[i] -= self.h;
            self.out_grad[i] = (-0.5 * (self.f)(&self.__x_hb) + 0.5 * (self.f)(&self.__x_hf)) / self.h;
        }
    }

    fn compute_and_return_gradient(&mut self, x: &[f64]) -> Vec<f64> {
        self.compute_gradient(x);
        self.out_grad.clone()
    }

    fn reset(&mut self, x: &[f64]) {
        for i in 0..self.dim {
            self.__x_hf[i] = x[i];
            self.__x_hb[i] = x[i];
        }
    }
}

pub struct CentralFiniteDiff2<'a> {
    pub dim: usize,
    pub f: &'a dyn FnMut(&[f64]) -> f64,
    pub h: f64,
    pub out_grad: Vec<f64>,
    __x_hf1: Vec<f64>,
    __x_hb1: Vec<f64>,
    __x_hf2: Vec<f64>,
    __x_hb2: Vec<f64>
}
impl<'a> GradientFinder<'a> for CentralFiniteDiff2<'a> {
    fn new(dim: usize, f: &'a dyn FnMut(&[f64]) -> f64) -> CentralFiniteDiff2<'a> {
        let mut out_grad: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            out_grad.push(0.0);
        }

        let mut __x_hf1: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            __x_hf1.push(0.0);
        }
        let mut __x_hb1: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            __x_hb1.push(0.0);
        }
        let mut __x_hf2: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            __x_hf2.push(0.0);
        }
        let mut __x_hb2: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            __x_hb2.push(0.0);
        }

        CentralFiniteDiff2{dim, f, h: 0.00000000001, out_grad, __x_hf1, __x_hb1, __x_hf2, __x_hb2}
    }

    fn compute_gradient(&mut self, x: &[f64]) {
        let val_0 = (self.f)(x);
        for (i, val) in x.iter().enumerate() {
            self.reset(x);
            self.__x_hf1[i] += self.h;
            self.__x_hb1[i] -= self.h;
            self.__x_hf2[i] += 2.0*self.h;
            self.__x_hb2[i] -= 2.0*self.h;
            self.out_grad[i] = ((1. / 12.) * (self.f)(&self.__x_hb2) + -(2. / 3.) * (self.f)(&self.__x_hb1) + (2. / 3.) * (self.f)(&self.__x_hf1) - (1. / 12.) * (self.f)(&self.__x_hf2)) / self.h;
        }
    }

    fn compute_and_return_gradient(&mut self, x: &[f64]) -> Vec<f64> {
        self.compute_gradient(x);
        self.out_grad.clone()
    }

    fn reset(&mut self, x: &[f64]) {
        for i in 0..self.dim {
            self.__x_hf1[i] = x[i];
            self.__x_hb1[i] = x[i];
            self.__x_hf2[i] = x[i];
            self.__x_hb2[i] = x[i];
        }
    }
}
*/

pub trait GradientFinder<F>
where F: FnMut(&[f64]) -> f64 {
    fn new(dim: usize, f: F) -> Self;
    fn compute_gradient(&mut self, x: &[f64]);
    fn compute_and_return_gradient(&mut self, x: &[f64]) -> Vec<f64>;
    fn reset(&mut self, x: &[f64]);
}

pub struct ForwardFiniteDiff<F>
where F: FnMut(&[f64]) -> f64
{
    pub dim: usize,
    pub f: F,
    pub h: f64,
    pub out_grad: Vec<f64>,
    __x_h: Vec<f64>
}
impl<F> GradientFinder<F> for ForwardFiniteDiff<F>
where F: FnMut(&[f64]) -> f64
{
    fn new(dim: usize, f: F) -> Self {
        let mut out_grad: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            out_grad.push(0.0);
        }

        let mut __x_h: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            __x_h.push(0.0);
        }
        ForwardFiniteDiff{dim, f, h: 0.00000000001, out_grad, __x_h}
    }

    fn compute_gradient(&mut self, x: &[f64]) {
        let val_0 = (self.f)(x);
        for (i, val) in x.iter().enumerate() {
            self.reset(x);
            self.__x_h[i] += self.h;
            self.out_grad[i] = (-val_0 + (self.f)(&self.__x_h)) / self.h;
        }
    }

    fn compute_and_return_gradient(&mut self, x: &[f64]) -> Vec<f64> {
        self.compute_gradient(x);
        self.out_grad.clone()
    }

    fn reset(&mut self, x: &[f64]) {
        for i in 0..self.dim {
            self.__x_h[i] = x[i];
        }
    }
}

pub struct CentralFiniteDiff<F>
where F: FnMut(&[f64]) -> f64
{
    pub dim: usize,
    pub f: F,
    pub h: f64,
    pub out_grad: Vec<f64>,
    __x_hf: Vec<f64>,
    __x_hb: Vec<f64>
}
impl<F> GradientFinder<F> for CentralFiniteDiff<F>
where F: FnMut(&[f64]) -> f64
{
    fn new(dim: usize, f: F) -> CentralFiniteDiff<F> {
        let mut out_grad: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            out_grad.push(0.0);
        }

        let mut __x_hf: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            __x_hf.push(0.0);
        }
        let mut __x_hb: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            __x_hb.push(0.0);
        }

        CentralFiniteDiff{dim, f, h: 0.00000000001, out_grad, __x_hf, __x_hb}
    }

    fn compute_gradient(&mut self, x: &[f64]) {
        let val_0 = (self.f)(x);
        for (i, val) in x.iter().enumerate() {
            self.reset(x);
            self.__x_hf[i] += self.h;
            self.__x_hb[i] -= self.h;
            self.out_grad[i] = (-0.5 * (self.f)(&self.__x_hb) + 0.5 * (self.f)(&self.__x_hf)) / self.h;
        }
    }

    fn compute_and_return_gradient(&mut self, x: &[f64]) -> Vec<f64> {
        self.compute_gradient(x);
        self.out_grad.clone()
    }

    fn reset(&mut self, x: &[f64]) {
        for i in 0..self.dim {
            self.__x_hf[i] = x[i];
            self.__x_hb[i] = x[i];
        }
    }
}

pub struct CentralFiniteDiff2<F>
where F: FnMut(&[f64]) -> f64
{
    pub dim: usize,
    pub f: F,
    pub h: f64,
    pub out_grad: Vec<f64>,
    __x_hf1: Vec<f64>,
    __x_hb1: Vec<f64>,
    __x_hf2: Vec<f64>,
    __x_hb2: Vec<f64>
}
impl<F> GradientFinder<F> for CentralFiniteDiff2<F>
where F: FnMut(&[f64]) -> f64
{
    fn new(dim: usize, f: F) -> CentralFiniteDiff2<F> {
        let mut out_grad: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            out_grad.push(0.0);
        }

        let mut __x_hf1: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            __x_hf1.push(0.0);
        }
        let mut __x_hb1: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            __x_hb1.push(0.0);
        }
        let mut __x_hf2: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            __x_hf2.push(0.0);
        }
        let mut __x_hb2: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            __x_hb2.push(0.0);
        }

        CentralFiniteDiff2{dim, f, h: 0.00000000001, out_grad, __x_hf1, __x_hb1, __x_hf2, __x_hb2}
    }

    fn compute_gradient(&mut self, x: &[f64]) {
        let val_0 = (self.f)(x);
        for (i, val) in x.iter().enumerate() {
            self.reset(x);
            self.__x_hf1[i] += self.h;
            self.__x_hb1[i] -= self.h;
            self.__x_hf2[i] += 2.0*self.h;
            self.__x_hb2[i] -= 2.0*self.h;
            self.out_grad[i] = ((1. / 12.) * (self.f)(&self.__x_hb2) + -(2. / 3.) * (self.f)(&self.__x_hb1) + (2. / 3.) * (self.f)(&self.__x_hf1) - (1. / 12.) * (self.f)(&self.__x_hf2)) / self.h;
        }
    }

    fn compute_and_return_gradient(&mut self, x: &[f64]) -> Vec<f64> {
        self.compute_gradient(x);
        self.out_grad.clone()
    }

    fn reset(&mut self, x: &[f64]) {
        for i in 0..self.dim {
            self.__x_hf1[i] = x[i];
            self.__x_hb1[i] = x[i];
            self.__x_hf2[i] = x[i];
            self.__x_hb2[i] = x[i];
        }
    }
}

pub fn t(x: &[f64]) -> f64 {
    x[0] * x[1]
}

pub fn t2(x: &[f64], a: &f64) -> f64 {
    *a * x[0] * x[1]
}

pub fn get_obj() -> ForwardFiniteDiff<fn(&[f64]) -> f64>
{
    ForwardFiniteDiff::new(2, t)
}

