
use crate::lib::spacetime::robot::Robot;
use crate::lib::utils_rust::yaml_utils::{RobotCollisionSpecFileParser, InfoFileParser};
use crate::lib::utils_rust::file_utils::get_path_to_src;
use crate::lib::utils_rust::collision_object::CollisionObject;
use crate::lib::utils_rust::transformations;
use nalgebra::{UnitQuaternion, Vector3, UnitComplex};
use ncollide3d::query::{Proximity, PointQuery};

#[derive(Clone, Debug)]
pub struct RobotLinkShapeInfo {
    pub link_type: usize, // 0 = auto capsule, 1 = user defined shape
    pub chain_idx: usize,
    pub joint_idx: usize,
    pub stationary: bool,
    pub shape_type: usize, // 0 = capsule, 1 = sphere, 2 = cuboid
    pub init_quat: UnitQuaternion<f64>,
    pub local_position: Vector3<f64>
}

impl RobotLinkShapeInfo {
    pub fn new(link_type: usize, chain_idx: usize, joint_idx: usize, stationary: bool, shape_type: usize, init_quat: UnitQuaternion<f64>, local_position: Vector3<f64>) -> Self  {
        Self {link_type, chain_idx, joint_idx, stationary, shape_type, init_quat, local_position}
    }
}

pub struct RobotShapeModel {
    pub robot: Robot,
    pub robot_collision_specs_file: RobotCollisionSpecFileParser,
    pub collision_objects: Vec<CollisionObject>,
    pub link_info_arr: Vec<RobotLinkShapeInfo>
}

impl RobotShapeModel {
    pub fn from_robot_and_specs(robot: &Robot, specs: &RobotCollisionSpecFileParser, starting_config: &Vec<f64>) -> Self {
        let mut collision_objects: Vec<CollisionObject> = Vec::new();
        let mut link_info_arr: Vec<RobotLinkShapeInfo> = Vec::new();


        let frames = robot.get_frames_immutable(starting_config);
        for i in 0..frames.len() {
            for j in 0..frames[i].0.len()-1 {
                let link_vector = (frames[i].0[j +1] - frames[i].0[j]);
                let link_length = link_vector.norm();
                if link_length > specs.robot_link_radius {
                    let link_midpoint = (frames[i].0[j +1] + frames[i].0[j]) / 2.0;
                    let mut collision_object = CollisionObject::new_capsule(((link_length*1.01)/2.0 - specs.robot_link_radius).max(0.000001), specs.robot_link_radius);
                    collision_object.set_curr_translation(link_midpoint[0], link_midpoint[1], link_midpoint[2]);
                    collision_object.align_object_with_vector(vec![link_vector[0], link_vector[1], link_vector[2]]);
                    collision_object.update_all_bounding_volumes();
                    link_info_arr.push(RobotLinkShapeInfo::new(0, i, j, false, 0, collision_object.curr_orientation.clone(), Vector3::identity()));
                    collision_objects.push(collision_object);
                }
            }
        }


        for i in 0..specs.spheres.len() {
            let mut sphere = CollisionObject::new_ball(specs.spheres[i].radius);
            sphere.update_all_bounding_volumes();

            let idx = Robot::get_index_from_joint_order(&robot.joint_ordering, &specs.spheres[i].coordinate_frame);
            if specs.spheres[i].coordinate_frame == "static".to_string() {
                link_info_arr.push(RobotLinkShapeInfo::new(1, usize::max_value(), usize::max_value(), true, 1, UnitQuaternion::identity(), Vector3::new(specs.spheres[i].tx, specs.spheres[i].ty, specs.spheres[i].tz)));
                sphere.set_curr_translation(specs.spheres[i].tx, specs.spheres[i].ty, specs.spheres[i].tz);
            } else {
                let mut chain_idx = 0 as usize;
                let mut joint_idx = 0 as usize;
                let mut found = false;

                for j in 0..robot.num_chains {
                    for k in 0..robot.subchain_indices[j].len() {
                        if !found && idx == robot.subchain_indices[j][k] {
                            found = true;
                            chain_idx = j; joint_idx = k;
                        }
                    }
                }

                let mut sphere_position = Vector3::new(specs.spheres[i].tx, specs.spheres[i].ty, specs.spheres[i].tz) + frames[chain_idx].0[joint_idx];
                sphere_position = frames[chain_idx].1[joint_idx + 1] * sphere_position;
                sphere.set_curr_translation(sphere_position[0], sphere_position[1], sphere_position[2]);

                link_info_arr.push(RobotLinkShapeInfo::new(1, chain_idx, joint_idx, false, 1, UnitQuaternion::identity(), Vector3::new(specs.spheres[i].tx, specs.spheres[i].ty, specs.spheres[i].tz)));
            }
            collision_objects.push(sphere);
        }

        for i in 0..specs.cuboids.len() {
            let mut cuboid = CollisionObject::new_cuboid(specs.cuboids[i].x_halflength, specs.cuboids[i].y_halflength, specs.cuboids[i].z_halflength);
            let mut q = UnitQuaternion::from_euler_angles(specs.cuboids[i].rx, specs.cuboids[i].ry, specs.cuboids[i].rz);
            cuboid.set_curr_orientation(q.w, q.i, q.j, q.k);
            cuboid.update_all_bounding_volumes();

            let idx = Robot::get_index_from_joint_order(&robot.joint_ordering, &specs.cuboids[i].coordinate_frame);
            if specs.cuboids[i].coordinate_frame == "static".to_string() {
                link_info_arr.push(RobotLinkShapeInfo::new(1, usize::max_value(), usize::max_value(), true, 2, q.clone(), Vector3::new(specs.cuboids[i].tx, specs.cuboids[i].ty, specs.cuboids[i].tz)));
                cuboid.set_curr_translation(specs.cuboids[i].tx, specs.cuboids[i].ty, specs.cuboids[i].tz);
            } else {
                let mut chain_idx = 0 as usize;
                let mut joint_idx = 0 as usize;

                // link_info_arr.push(RobotCollisionLinkInfo::new(1, chain_idx, joint_idx, false, 2, q.clone()));

                let mut found = false;
                for j in 0..robot.num_chains {
                    for k in 0..robot.subchain_indices[j].len() {
                        if !found && idx == robot.subchain_indices[j][k] {
                            found = true;
                            chain_idx = j; joint_idx = k;
                        }
                    }
                }

                let mut cube_position = Vector3::new(specs.cuboids[i].tx, specs.cuboids[i].ty, specs.cuboids[i].tz) + frames[chain_idx].0[joint_idx];
                cube_position = frames[chain_idx].1[joint_idx + 1] * cube_position;
                cuboid.set_curr_translation(cube_position[0], cube_position[1], cube_position[2]);

                link_info_arr.push(RobotLinkShapeInfo::new(1, chain_idx, joint_idx, false, 2, q.clone(), Vector3::new(specs.cuboids[i].tx, specs.cuboids[i].ty, specs.cuboids[i].tz)));
            }

            collision_objects.push(cuboid);
        }

        Self {robot: robot.clone(), robot_collision_specs_file: specs.clone(), collision_objects, link_info_arr}
    }

    pub fn from_yaml_path(fp: String) -> Self {
        // yaml path to info file
        let robot = Robot::from_yaml_path(fp.clone());
        let ifp = InfoFileParser::from_yaml_path(fp.clone());
        let collision_file_name = ifp.collision_file_name;
        let fp2 = get_path_to_src() + "RelaxedIK/Config/collision_files_rust/" + collision_file_name.as_str();
        let robot_collision_specs_file = RobotCollisionSpecFileParser::from_yaml_path(fp2.clone());
        RobotShapeModel::from_robot_and_specs(&robot, &robot_collision_specs_file, &ifp.starting_config)
    }

    pub fn from_info_file_name(info_file_name: String) -> Self {
         let path_to_src = get_path_to_src();
         let fp = path_to_src + "RelaxedIK/Config/info_files/" + info_file_name.as_str();
         RobotShapeModel::from_yaml_path(fp)
     }

    pub fn collision_check_full_shapes(&self, idx1: usize, idx2: usize) -> bool {
        let proximity = self.collision_objects[idx1].proximity_check(&self.collision_objects[idx2]);
        if proximity == Proximity::Intersecting {
            return true;
        } else {
            return false
        }
    }

    pub fn collision_check_bounding_aabbs(&self, idx1: usize, idx2: usize) -> bool {
        self.collision_objects[idx1].bounding_aabb_intersect_check(&self.collision_objects[idx2])
    }

    pub fn collision_check_bounding_spheres(&self, idx1: usize, idx2: usize) -> bool {
        self.collision_objects[idx1].bounding_sphere_intersect_check(&self.collision_objects[idx2])
    }

    pub fn update_robot_transforms(&mut self, state: &Vec<f64>) {
        let frames = self.robot.get_frames_immutable(state);
        self.update_robot_transforms_from_frames(&frames);
    }

    pub fn update_robot_transforms_from_frames(&mut self, frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>) {
        for i in 0..self.link_info_arr.len() {
            self.update_robot_link_transform_from_frames(i, frames);
        }
    }

    pub fn update_robot_link_transform(&mut self, link_idx: usize, state: &Vec<f64>) {
        let frames = self.robot.get_frames_immutable(state);
        self.update_robot_link_transform_from_frames(link_idx, &frames);
    }

    pub fn update_robot_link_transform_from_frames(&mut self, link_idx: usize, frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>) {
        if self.link_info_arr[link_idx].link_type == 0 { // if it's an auto capsule...
            let chain_idx = self.link_info_arr[link_idx].chain_idx;
            let joint_idx = self.link_info_arr[link_idx].joint_idx;
            let link_vector = (frames[chain_idx].0[joint_idx + 1] - frames[chain_idx].0[joint_idx]);
            let link_midpoint = (frames[chain_idx].0[joint_idx + 1] + frames[chain_idx].0[joint_idx]) / 2.0;
            // println!("{:?}", link_midpoint);
            self.collision_objects[link_idx].set_curr_translation(link_midpoint[0], link_midpoint[1], link_midpoint[2]);
            self.collision_objects[link_idx].align_object_with_vector(vec![link_vector[0], link_vector[1], link_vector[2]]);
        } else { // else, if it's a user supplied shape...
            if !self.link_info_arr[link_idx].stationary { // if it's not a stationary shape...
                let parent_chain_idx = self.link_info_arr[link_idx].chain_idx;
                let parent_joint_idx = self.link_info_arr[link_idx].joint_idx;
                let curr_quat = frames[parent_chain_idx].1[parent_joint_idx + 1];
                let new_position = curr_quat * (self.link_info_arr[link_idx].local_position) + frames[parent_chain_idx].0[parent_joint_idx];
                self.collision_objects[link_idx].set_curr_translation(new_position[0], new_position[1], new_position[2]);
                if self.link_info_arr[link_idx].shape_type == 0 ||  self.link_info_arr[link_idx].shape_type == 2 { // if it's capsule or cuboid, have to update orientation
                    let init_quat = self.link_info_arr[link_idx].init_quat.clone();
                    let disp_quat = transformations::quaternion_dispQ(init_quat, curr_quat.clone());
                    let new_quat = self.link_info_arr[link_idx].init_quat * disp_quat;
                    self.collision_objects[link_idx].set_curr_orientation(new_quat.w, new_quat.i, new_quat.j, new_quat.k);
                }
            }
        }
    }

    pub fn update_all_bounding_spheres(&mut self) {
        let l = self.link_info_arr.len();
        for i in 0..l {
            self.collision_objects[i].update_bounding_sphere();
        }
    }

    pub fn update_all_bounding_aabbs(&mut self) {
        let l = self.link_info_arr.len();
        for i in 0..l {
            self.collision_objects[i].update_bounding_aabb();
        }
    }

    pub fn update_bounding_sphere(&mut self, idx: usize) {
        self.collision_objects[idx].update_bounding_sphere();
    }

    pub fn update_bounding_aabb(&mut self, idx: usize) {
        self.collision_objects[idx].update_bounding_aabb();
    }
}