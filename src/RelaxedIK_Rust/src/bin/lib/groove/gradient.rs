
pub trait GradientFinderImmutable<F>
where F: Fn(&[f64]) -> f64 {
    fn new(dim: usize, f: F) -> Self;
    fn compute_gradient(&mut self, x: &[f64]);
    fn compute_and_return_gradient(&mut self, x: &[f64]) -> Vec<f64>;
    fn compute_gradient_immutable(&self, x: &[f64]) -> Vec<f64>;
    fn reset(&mut self, x: &[f64]);
}

pub struct ForwardFiniteDiffImmutable<F>
where F: Fn(&[f64]) -> f64
{
    pub dim: usize,
    pub f: F,
    pub h: f64,
    pub out_grad: Vec<f64>,
    __x_h: Vec<f64>
}
impl<F> GradientFinderImmutable<F> for ForwardFiniteDiffImmutable<F>
where F: Fn(&[f64]) -> f64
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
        ForwardFiniteDiffImmutable{dim, f, h: 0.00001, out_grad, __x_h}
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

    fn compute_gradient_immutable(&self, x: &[f64]) -> Vec<f64> {
        let mut out: Vec<f64> = Vec::new();
        let val_0 = (self.f)(x);
        for i in 0..x.len() {
            let mut x_h = x.clone().to_vec();
            x_h[i] += self.h;
            out.push((-val_0 + (self.f)(x_h.as_slice())) / self.h);
        }
        out
    }

    fn reset(&mut self, x: &[f64]) {
        for i in 0..self.dim {
            self.__x_h[i] = x[i];
        }
    }
}

pub struct CentralFiniteDiffImmutable<F>
where F: Fn(&[f64]) -> f64
{
    pub dim: usize,
    pub f: F,
    pub h: f64,
    pub out_grad: Vec<f64>,
    __x_hf: Vec<f64>,
    __x_hb: Vec<f64>
}
impl<F> GradientFinderImmutable<F> for CentralFiniteDiffImmutable<F>
where F: Fn(&[f64]) -> f64
{
    fn new(dim: usize, f: F) -> Self {
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

        CentralFiniteDiffImmutable{dim, f, h: 0.0001, out_grad, __x_hf, __x_hb}
    }

    fn compute_gradient(&mut self, x: &[f64]) {
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

    fn compute_gradient_immutable(&self, x: &[f64]) -> Vec<f64> {
        let mut out: Vec<f64> = Vec::new();
        for (i, val) in x.iter().enumerate() {
            let mut x_hf = x.clone().to_vec();
            let mut x_hb = x.clone().to_vec();
            x_hf[i] += self.h;
            x_hb[i] -= self.h;
            out.push( (-0.5 * (self.f)(x_hb.as_slice()) + 0.5 * (self.f)(x_hf.as_slice())) / self.h);
        }
        out
    }


    fn reset(&mut self, x: &[f64]) {
        for i in 0..self.dim {
            self.__x_hf[i] = x[i];
            self.__x_hb[i] = x[i];
        }
    }
}

pub struct CentralFiniteDiff2Immutable<F>
where F: Fn(&[f64]) -> f64
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
impl<F> GradientFinderImmutable<F> for CentralFiniteDiff2Immutable<F>
where F: Fn(&[f64]) -> f64
{
    fn new(dim: usize, f: F) -> Self {
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

        CentralFiniteDiff2Immutable{dim, f, h: 0.0001, out_grad, __x_hf1, __x_hb1, __x_hf2, __x_hb2}
    }

    fn compute_gradient(&mut self, x: &[f64]) {
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

    fn compute_gradient_immutable(&self, x: &[f64]) -> Vec<f64> {
        let mut out: Vec<f64> = Vec::new();
        for (i, val) in x.iter().enumerate() {
            let mut x_hf1 = x.clone().to_vec();
            let mut x_hb1 = x.clone().to_vec();
            let mut x_hf2 = x.clone().to_vec();
            let mut x_hb2 = x.clone().to_vec();
            x_hf1[i] += self.h;
            x_hb1[i] -= self.h;
            x_hf2[i] += 2.0*self.h;
            x_hb2[i] -= 2.0*self.h;
            out.push( ((1. / 12.) * (self.f)(x_hb2.as_slice()) + -(2. / 3.) * (self.f)(x_hb1.as_slice()) + (2. / 3.) * (self.f)(x_hf1.as_slice()) - (1. / 12.) * (self.f)(x_hf2.as_slice())) / self.h);
        }
        out
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
        ForwardFiniteDiff{dim, f, h: 0.00001, out_grad, __x_h}
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

        CentralFiniteDiff{dim, f, h: 0.0001, out_grad, __x_hf, __x_hb}
    }

    fn compute_gradient(&mut self, x: &[f64]) {
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

        CentralFiniteDiff2{dim, f, h: 0.0001, out_grad, __x_hf1, __x_hb1, __x_hf2, __x_hb2}
    }

    fn compute_gradient(&mut self, x: &[f64]) {
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







/*
pub trait GradientFinder2<'a, F>
where F: FnMut(&[f64]) -> f64 {
    fn new(dim: usize, f: &'a F) -> Self;
    fn compute_gradient(&mut self, x: &[f64]);
    fn compute_and_return_gradient(&mut self, x: &[f64]) -> Vec<f64>;
    fn reset(&mut self, x: &[f64]);
}

pub struct ForwardFiniteDiff2<'a, F>
where F: FnMut(&[f64]) -> f64
{
    pub dim: usize,
    pub f: &'a F,
    pub h: f64,
    pub out_grad: Vec<f64>,
    __x_h: Vec<f64>
}
impl<'a, F> GradientFinder2<'a, F> for ForwardFiniteDiff2<'a, F>
where F: FnMut(&[f64]) -> f64
{
    fn new(dim: usize, f: &'a F) -> Self {
        let mut out_grad: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            out_grad.push(0.0);
        }

        let mut __x_h: Vec<f64> = Vec::with_capacity(dim);
        for i in 0..dim {
            __x_h.push(0.0);
        }
        ForwardFiniteDiff2{dim, f, h: 0.00001, out_grad, __x_h}
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
*/
