pub mod lib;

use nalgebra::{Isometry3, Vector3, Point3, Quaternion, UnitQuaternion, Translation3, DVector};
use lib::utils_rust::self_collision_engine::SelfCollisionEngine;
use lib::utils_rust::collision_object::CollisionObject;
use ncollide3d::query;
use ncollide3d::query::{Proximity, PointQuery};
use ncollide3d::shape::FeatureId;
use ncollide3d::shape::{Ball, Cuboid, Cylinder, Capsule, Cone, ConvexHull, Shape, TriMesh};
use ncollide3d::bounding_volume::{self, BoundingVolume, BoundingSphere, AABB};
use ncollide3d::transformation;


fn main() {
    let mut c = CollisionObject::new_capsule(0.4, 0.1);
    let mut c2 = CollisionObject::new_cuboid(2.0, 0.1, 0.1);

    c.set_curr_translation(0., 0., 0.);
    c.align_object_with_vector( vec![0.33, 0.33, 0.33] );
    println!("{:?}", c.curr_orientation);
    c.update_all_bounding_volumes();
    c2.set_curr_translation(0.0, 0.8, 0.0);
    c.update_all_bounding_volumes();
    c2.update_all_bounding_volumes();
    println!("{:?}", c2.bounding_aabb);
    println!("{:?}", c.bounding_aabb);
    // println!("{:?}", c.bounding_aabb);
    let mut check = c.bounding_aabb_intersect_check(&c2);
    println!("{:?}", check);
    let mut check = c.proximity_check(&c2);
    println!("{:?}", check);
}
