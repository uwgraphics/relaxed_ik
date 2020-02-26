use ncollide3d::query;
use ncollide3d::query::{Proximity, PointQuery};
use ncollide3d::shape::FeatureId;
use ncollide3d::shape::{Ball, Cuboid, Cylinder, Capsule, Cone, ConvexHull, Shape, TriMesh};
use ncollide3d::bounding_volume::{self, BoundingVolume, BoundingSphere, AABB};
use ncollide3d::transformation;
use crate::lib::utils_rust::transformations;
use std::boxed::Box;
use nalgebra::{UnitQuaternion, Vector3, Translation3, Quaternion, Isometry3, DVector, Rotation3, Matrix3};
use std::borrow::BorrowMut;


pub struct CollisionObject {
    pub shape: Box<dyn Shape<f64>>,
    pub bounding_sphere: BoundingSphere<f64>,
    pub base_bounding_sphere: BoundingSphere<f64>,
    pub bounding_aabb: AABB<f64>,
    pub base_bounding_aabb: AABB<f64>,
    pub curr_translation: Translation3<f64>,
    // pub curr_bounding_sphere_translation: Translation3<f64>,
    // pub curr_bounding_aabb_translation: Translation3<f64>,
    pub curr_orientation: UnitQuaternion<f64>,
    // pub curr_bounding_sphere_orientation: UnitQuaternion<f64>,
    // pub curr_bounding_aabb_orientation: UnitQuaternion<f64>,
    pub curr_isometry: Isometry3<f64>,
}

impl CollisionObject {
    pub fn new(shape: Box<dyn Shape<f64>>) -> Self {
        let bounding_sphere = bounding_volume::bounding_sphere(&(*shape), &nalgebra::Isometry3::identity());
        let base_bounding_sphere = bounding_volume::bounding_sphere(&(*shape), &nalgebra::Isometry3::identity());
        let bounding_aabb = bounding_volume::aabb(&(*shape), &nalgebra::Isometry3::identity());
        let base_bounding_aabb = bounding_volume::aabb(&(*shape), &nalgebra::Isometry3::identity());

        let curr_translation = Translation3::new(0.,0.,0.);
        // let curr_bounding_sphere_translation = Translation3::new(0.,0.,0.);
        // let curr_bounding_aabb_translation = Translation3::new(0.,0.,0.);

        let curr_orientation = UnitQuaternion::from_quaternion(Quaternion::new(1.0, 0.0, 0.0, 0.0));
        // let curr_bounding_sphere_orientation = UnitQuaternion::from_quaternion(Quaternion::new(1.0, 0.0, 0.0, 0.0));
        // let curr_bounding_aabb_orientation = UnitQuaternion::from_quaternion(Quaternion::new(1.0, 0.0, 0.0, 0.0));

        let curr_isometry = Isometry3::from_parts(curr_translation.clone(), curr_orientation.clone());

        Self {shape, bounding_sphere, base_bounding_sphere, bounding_aabb, base_bounding_aabb, curr_translation,
            curr_orientation, curr_isometry}
    }

    pub fn new_capsule(half_height: f64, radius: f64) -> Self {
        // half height refers to JUST the cylindrical part.  The TOTAL half height of the capsule will be half_height + radius.
        let shape = Capsule::new(half_height, radius);
        CollisionObject::new(Box::new(shape))
    }

    pub fn new_cuboid(x_half: f64, y_half: f64, z_half: f64) -> Self {
        let cuboid = Cuboid::new(Vector3::new(x_half, y_half, z_half));
        CollisionObject::new(Box::new(cuboid))
    }

    pub fn new_ball(radius: f64) -> Self {
        let ball = Ball::new(radius);
        CollisionObject::new(Box::new(ball))
    }

    pub fn set_curr_translation(&mut self, x: f64, y: f64, z: f64) {
        self.curr_translation.vector[0] = x; self.curr_translation.vector[1] = y; self.curr_translation.vector[2] = z;
        self.update_curr_isometry();
    }

    pub fn set_curr_orientation(&mut self, w: f64, x: f64, y: f64, z: f64) {
        self.curr_orientation = UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z));
        self.update_curr_isometry();
    }

    pub fn set_curr_transform(&mut self, tx: f64, ty: f64, tz: f64, rw: f64, rx: f64, ry: f64, rz: f64) {
        self.set_curr_translation(tx, ty, tz);
        self.set_curr_orientation(rw, rx, ry, rz);
        self.update_curr_isometry();
    }

    pub fn align_object_with_vector_vec3(&mut self, vector: &Vector3<f64>) {
        // Aligns the Y axis of the object with the given vector.  Most useful for the capsule object
        // to align it with robot links
        /*
        let mut y_vec = vector.clone();
        y_vec = &y_vec / y_vec.norm();
        y_vec[0] += 0.000000000001;
        y_vec[1] += 0.000000000001;
        y_vec[2] += 0.000000000001;
        let mut tmp = DVector::from_element(3, 0.0);
        tmp[2] = 1.0;
        if y_vec[2] == 1.0 {
            tmp[2] = 0.0;
            tmp[0] = 1.0;
        }
        let mut x_vec = y_vec.cross(&tmp);
        x_vec = &x_vec / x_vec.norm();
        let mut z_vec = x_vec.cross(&y_vec);
        z_vec = &z_vec / z_vec.norm();
        let mat = Matrix3::new(x_vec[0], y_vec[0], z_vec[0], x_vec[1], y_vec[1], z_vec[1], x_vec[2], y_vec[2], z_vec[2]);
        // let mat = Matrix3::new(x_vec[0], x_vec[1], x_vec[2], y_vec[0], y_vec[1], y_vec[2], z_vec[0], z_vec[1], z_vec[2]);
        let unit_quat = UnitQuaternion::from_matrix(&mat);
        */
        let mut unit_quat = UnitQuaternion::face_towards(vector, &Vector3::new(0.0000000001,0.00000000001,1.000000000000000001));
        let q2 = UnitQuaternion::from_euler_angles(std::f64::consts::FRAC_PI_2, 0., 0.);
        unit_quat = unit_quat * q2;
        self.set_curr_orientation(unit_quat.w, unit_quat.i, unit_quat.j, unit_quat.k);
    }

    pub fn align_object_with_vector(&mut self, vector: Vec<f64>) {
        // let mut dvec = DVector::from_element(3, 0.0);
        // dvec[0] = vector[0]; dvec[1] = vector[1]; dvec[2] = vector[2];
        let mut vec3 = Vector3::new(vector[0], vector[1], vector[2]);
        self.align_object_with_vector_vec3(&vec3);
    }

    pub fn update_curr_isometry(&mut self) {
        self.curr_isometry = Isometry3::from_parts(self.curr_translation.clone(), self.curr_orientation.clone());
    }

    pub fn update_bounding_sphere(&mut self) {
        // let mut rel_translation = self.curr_translation.vector - self.curr_bounding_sphere_translation.vector;
        // let rel_translation = Translation3::new(rel_translation[0], rel_translation[1], rel_translation[2]);
        // let rel_orientation = transformations::quaternion_dispQ(self.curr_bounding_sphere_orientation, self.curr_orientation);
        // let rel_isometry = Isometry3::from_parts(rel_translation, UnitQuaternion::identity());
        // self.bounding_sphere = self.bounding_sphere.transform_by(&rel_isometry);
        self.bounding_sphere = self.base_bounding_sphere.transform_by(&self.curr_isometry);
        // self.curr_bounding_sphere_translation = self.curr_translation.clone();
        // self.curr_bounding_sphere_orientation = self.curr_orientation.clone();
    }

    pub fn update_bounding_aabb(&mut self) {
        // let mut rel_translation = self.curr_translation.vector - self.curr_bounding_aabb_translation.vector;
        // let rel_translation = Translation3::new(rel_translation[0], rel_translation[1], rel_translation[2]);
        // let rel_orientation = transformations::quaternion_dispQ(self.curr_bounding_aabb_orientation, self.curr_orientation);
        // let rel_isometry = Isometry3::from_parts(rel_translation, rel_orientation);
        // self.bounding_aabb = self.bounding_aabb.transform_by(&rel_isometry);
        // self.bounding_aabb = bounding_volume::aabb(&(*self.shape), &self.curr_isometry);
        self.bounding_aabb = self.base_bounding_aabb.transform_by(&self.curr_isometry);
        // self.curr_bounding_aabb_translation = self.curr_translation.clone();
        // self.curr_bounding_aabb_orientation = self.curr_orientation.clone();
    }

    pub fn update_all_bounding_volumes(&mut self) {
        self.update_bounding_sphere();
        self.update_bounding_aabb();
    }

    pub fn proximity_check(&self, other: &CollisionObject) -> Proximity {
        return query::proximity(&self.curr_isometry, &(*self.shape), &other.curr_isometry, &(*other.shape), 0.1);
    }

    pub fn bounding_aabb_intersect_check(&self, other: &CollisionObject) -> bool {
        self.bounding_aabb.intersects(&other.bounding_aabb)
    }

    pub fn bounding_sphere_intersect_check(&self, other: &CollisionObject) -> bool {
        self.bounding_sphere.intersects(&other.bounding_sphere)
    }
}



