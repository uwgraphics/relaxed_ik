use nalgebra::{Vector3, UnitQuaternion, Quaternion, Vector4};

pub fn quaternion_log(q: UnitQuaternion<f64>) -> Vector3<f64> {
    let mut out_vec: Vector3<f64> = Vector3::new(q.i, q.j, q.k);
    if q.w.abs() < 1.0 {
        let a = q.w.acos();
        let sina = a.sin();
        if sina.abs() >= 0.005 {
            let c = a / sina;
            out_vec *= c;
        }
    }
    out_vec
}

pub fn quaternion_exp(v: Vector3<f64>) -> UnitQuaternion<f64> {
    let mut qv: Vector4<f64> = Vector4::new(1.0, v[0], v[1], v[2]);
    let a = qv.norm();
    let sina = a.sin();
    if sina.abs() >= 0.005 {
        let c = sina/a;
        qv *= c;
    }
    UnitQuaternion::from_quaternion(Quaternion::new(a.cos(), qv[1], qv[2], qv[3]))
}

pub fn quaternion_disp(q: UnitQuaternion<f64>, q_prime: UnitQuaternion<f64>) -> Vector3<f64> {
    quaternion_log( q.inverse()*q_prime )
}

pub fn quaternion_dispQ(q: UnitQuaternion<f64>, q_prime: UnitQuaternion<f64>) -> UnitQuaternion<f64> {
    q.inverse()*q_prime
}

pub fn angle_between(q: UnitQuaternion<f64>, q_prime: UnitQuaternion<f64>) -> f64 {
    quaternion_disp(q, q_prime).norm() * 2.0
}
