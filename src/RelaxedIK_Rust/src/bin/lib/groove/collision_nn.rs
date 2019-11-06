use crate::lib::utils_rust::yaml_utils::CollisionNeuralNetParser;
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

pub struct CollisionNN {
    pub coef_matrices: Vec<DMatrix<f64>>,
    pub intercept_vectors: Vec<DMatrix<f64>>,
    pub input_length: usize,
    pub result: f64,
    __x_proxy: DMatrix<f64>,
    __intermediate_vecs: Vec<DMatrix<f64>>
}

impl CollisionNN {
    pub fn from_yaml_path(fp: String) -> Self {
        let parser = CollisionNeuralNetParser::from_yaml_path(fp.clone());
        let input_length = parser.coefs[0].len();
        let __x_proxy: DMatrix<f64> = DMatrix::from_element(1, input_length, 0.0);
        let mut __intermediate_vecs: Vec<DMatrix<f64>> = Vec::new();
        let mut result = 0.0;

        for i in 0..parser.intercept_vectors.len() {
            __intermediate_vecs.push(parser.intercept_vectors[i].clone());
        }

        Self{coef_matrices: parser.coef_matrices.clone(), intercept_vectors: parser.intercept_vectors.clone(), input_length, result, __x_proxy, __intermediate_vecs}
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

    pub fn gradient(&self, x: &Vec<f64>) -> Vec<f64> {
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
        out
    }

    pub fn gradient_finite_diff(&self, x: &Vec<f64>) -> Vec<f64> {
        let mut out: Vec<f64> = Vec::new();

        let f_0 = self.predict(&x);
        for i in 0..x.len() {
            let mut x_h = x.clone();
            x_h[i] += 0.000001;
            let f_h = self.predict(&x_h);
            out.push( (-f_0 + f_h) / 0.000001);
        }

        out
    }
}

