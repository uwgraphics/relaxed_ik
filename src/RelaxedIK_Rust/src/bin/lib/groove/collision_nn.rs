use crate::lib::utils_rust::yaml_utils::NeuralNetParser;
use crate::lib::utils_rust::{geometry_utils, yaml_utils};
use crate::lib::spacetime::robot::Robot;
use nalgebra::{DMatrix, DVector};

fn relu(x: f64) -> f64 {
    x.max(0.0)
}

fn relu_prime(x: f64) -> f64 {
    if x<= 0.0 {
        return 0.0
    } else {
        return 1.0
    }
}

pub fn get_relu_jacobian(x: &DMatrix<f64>) -> DMatrix<f64> {
    let mut out: DMatrix<f64> = DMatrix::from_element(x.shape().1, x.shape().1, 0.0);
    for i in 0..x.shape().1 {
        out[(i,i)] = relu_prime(x[i]);
    }

    out
}

pub fn get_relu_jacobian_mul(x: &DMatrix<f64>, c: &DMatrix<f64>) -> DMatrix<f64> {
    // let mut out: DMatrix<f64> = DMatrix::from_element(x.shape().1, x.shape().1, 0.0);
    let mut out: DMatrix<f64> = DMatrix::from_element(c.shape().0, c.shape().1, 0.0);
    for i in 0..c.shape().0 {
        for j in 0..c.shape().1 {
            // out[(i,i)] = relu_prime(x[i]) * c[(i,i)];
            out[(i,j)] = relu_prime(x[i]) * c[(i,j)];
        }
    }

    out
}

pub fn state_to_jt_pt_vec(x: &Vec<f64>, robot: &Robot) -> Vec<f64> {
    let mut out_vec: Vec<f64> = Vec::new();
    let frames = robot.get_frames_immutable(x.as_slice());
    for i in 0..frames.len() {
        for j in 0..frames[i].0.len() {
            out_vec.push(frames[i].0[j][0]);
            out_vec.push(frames[i].0[j][1]);
            out_vec.push(frames[i].0[j][2]);
        }
    }
    out_vec
}



pub struct CollisionNN {
    pub coef_matrices: Vec<DMatrix<f64>>,
    pub intercept_vectors: Vec<DMatrix<f64>>,
    pub split_point: f64,
    pub input_length: usize,
    pub result: f64,
    __x_proxy: DMatrix<f64>,
    __intermediate_vecs: Vec<DMatrix<f64>>
}

impl CollisionNN {
    pub fn from_yaml_path(fp: String) -> Self {
        let parser = NeuralNetParser::from_yaml_path(fp.clone());
        let input_length = parser.coefs[0].len();
        let __x_proxy: DMatrix<f64> = DMatrix::from_element(1, input_length, 0.0);
        let mut __intermediate_vecs: Vec<DMatrix<f64>> = Vec::new();
        let mut result = 0.0;

        for i in 0..parser.intercept_vectors.len() {
            __intermediate_vecs.push(parser.intercept_vectors[i].clone());
        }

        Self{coef_matrices: parser.coef_matrices.clone(), intercept_vectors: parser.intercept_vectors.clone(), split_point: parser.split_point, input_length, result, __x_proxy, __intermediate_vecs}
    }

    pub fn predict_mutable(&mut self, x: Vec<f64>) {
        for i in 0..self.input_length {
            self.__x_proxy[i] = x[i];
        }

        self.__intermediate_vecs[0] = &self.__x_proxy * &self.coef_matrices[0] + &self.intercept_vectors[0];
        self.__intermediate_vecs[0].apply(relu);


        for i in 1..self.coef_matrices.len() {
            self.__intermediate_vecs[i] =  &self.__intermediate_vecs[i-1] * &self.coef_matrices[i] + &self.intercept_vectors[i];
            self.__intermediate_vecs[i].apply(relu);
        }

        self.result = self.__intermediate_vecs[self.input_length-1][0];
    }

    pub fn predict(&self, x: &Vec<f64>) -> f64 {
        let mut x_vec = DMatrix::from_element(1, self.input_length, 0.0);
        for i in 0..self.input_length {
            x_vec[i] = x[i];
        }
        for i in 0..self.coef_matrices.len() {
            x_vec =  x_vec * &self.coef_matrices[i] + &self.intercept_vectors[i];
            x_vec.apply(relu);
        }

        x_vec[0]
    }

    pub fn in_collision(&self, x: &Vec<f64>) -> bool {
        let p = self.predict(x);
        if p > self.split_point {
            return true;
        } else {
            return false;
        }
    }

    pub fn gradient2(&self, x: &Vec<f64>) -> (f64, Vec<f64>) {
        let mut out: Vec<f64> = Vec::new();
        let mut grad: DMatrix<f64> = DMatrix::from_element(1, x.len(), 0.0);

        let mut x_vec = DMatrix::from_element(1, self.input_length, 0.0);
        for i in 0..self.input_length {
            x_vec[i] = x[i];
        }

        let mut first = true;
        for i in 0..self.coef_matrices.len() {
            x_vec =  x_vec * &self.coef_matrices[i] + &self.intercept_vectors[i];
            x_vec.apply(relu);
            let j = get_relu_jacobian(&x_vec);
            if first {
                grad = j * &self.coef_matrices[i].transpose();
                first = false;
            } else {
                grad = (j * &self.coef_matrices[i].transpose()) * grad;
            }
        }

        for i in 0..grad.len() {
            out.push(grad[i]);
        }
        (x_vec[0], out)
    }

    pub fn gradient(&self, x: &Vec<f64>) -> (f64, Vec<f64>) {
        let mut out: Vec<f64> = Vec::new();
        let mut grad: DMatrix<f64> = DMatrix::from_element(1, x.len(), 0.0);

        let mut x_vec = DMatrix::from_element(1, self.input_length, 0.0);
        for i in 0..self.input_length {
            x_vec[i] = x[i];
        }

        let mut first = true;
        for i in 0..self.coef_matrices.len() {
            x_vec =  x_vec * &self.coef_matrices[i] + &self.intercept_vectors[i];
            x_vec.apply(relu);
            if first {
                let j = get_relu_jacobian_mul(&x_vec, &self.coef_matrices[i].transpose());
                grad = j;
                first = false;
            } else {
                let j = get_relu_jacobian_mul(&x_vec, &self.coef_matrices[i].transpose());
                grad = j * grad;
            }
        }

        for i in 0..grad.len() {
            out.push(grad[i]);
        }
        (x_vec[0], out)
    }

    pub fn gradient_finite_diff(&self, x: &Vec<f64>) -> (f64, Vec<f64>) {
        let mut out: Vec<f64> = Vec::new();

        let f_0 = self.predict(&x);
        for i in 0..x.len() {
            let mut x_h = x.clone();
            x_h[i] += 0.000001;
            let f_h = self.predict(&x_h);
            out.push( (-f_0 + f_h) / 0.000001);
        }

        (f_0, out)
    }
}

pub struct CollisionNNJointPoint {
    pub coef_matrices: Vec<DMatrix<f64>>,
    pub intercept_vectors: Vec<DMatrix<f64>>,
    pub split_point: f64,
    pub input_length: usize,
    pub result: f64,
    __x_proxy: DMatrix<f64>,
    __intermediate_vecs: Vec<DMatrix<f64>>
}

impl CollisionNNJointPoint {
    pub fn from_yaml_path(fp: String) -> Self {
        let parser = NeuralNetParser::from_yaml_path(fp.clone());
        let input_length = parser.coefs[0].len();
        let __x_proxy: DMatrix<f64> = DMatrix::from_element(1, input_length, 0.0);
        let mut __intermediate_vecs: Vec<DMatrix<f64>> = Vec::new();
        let mut result = 0.0;

        for i in 0..parser.intercept_vectors.len() {
            __intermediate_vecs.push(parser.intercept_vectors[i].clone());
        }

        Self{coef_matrices: parser.coef_matrices.clone(), intercept_vectors: parser.intercept_vectors.clone(), split_point: parser.split_point, input_length, result, __x_proxy, __intermediate_vecs}
    }

    pub fn predict(&self, x: &Vec<f64>, robot: &Robot) -> f64 {
        let jt_pt_vec = state_to_jt_pt_vec(x, robot);
        let mut x_vec = DMatrix::from_element(1, jt_pt_vec.len(), 0.0);
        for i in 0..jt_pt_vec.len() {
            x_vec[i] = jt_pt_vec[i];
        }
        for i in 0..self.coef_matrices.len() {
            x_vec =  x_vec * &self.coef_matrices[i] + &self.intercept_vectors[i];
            x_vec.apply(relu);
        }
        x_vec[0]
    }

    pub fn in_collision(&self, x: &Vec<f64>, robot: &Robot) -> bool {
        let p = self.predict(x, robot);
        if p > self.split_point {
            return true;
        } else {
            return false;
        }
    }

    pub fn gradient_finite_diff(&self, x: &Vec<f64>, robot: &Robot) -> (f64, Vec<f64>) {
        let mut out: Vec<f64> = Vec::new();

        let f_0 = self.predict(&x, robot);
        for i in 0..x.len() {
            let mut x_h = x.clone();
            x_h[i] += 0.000001;
            let f_h = self.predict(&x_h, robot);
            out.push( (-f_0 + f_h) / 0.000001);
        }

        (f_0, out)
    }
}




