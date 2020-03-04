use crate::lib::utils_rust::robot_shape_model::RobotShapeModel;
use crate::lib::spacetime::robot::Robot;
use crate::lib::utils_rust::yaml_utils::{RobotCollisionSpecFileParser, InfoFileParser};
use crate::lib::utils_rust::file_utils::get_path_to_src;
use crate::lib::utils_rust::collision_object::CollisionObject;
use crate::lib::utils_rust::transformations;
use crate::lib::utils_rust::sampler::{ThreadRobotSampler, ThreadSampler};
use nalgebra::{UnitQuaternion, Vector3, UnitComplex};
use ncollide3d::query::{Proximity, PointQuery};

pub struct SelfCollisionEngine {
    pub robot_shape_model: RobotShapeModel,
    pub allowed_collision_matrix: Vec<Vec<bool>>,
    pub sampler: ThreadRobotSampler,
    pub link_pair_idxs: Vec<(usize, usize)>,
    pub collision_pair_check_order: Vec<usize>
    // pub last_collision_pair_idx: usize
}

impl SelfCollisionEngine {
    pub fn from_robot_shape_model(robot_shape_model: RobotShapeModel) -> Self {
        let num_collision_links = robot_shape_model.link_info_arr.len();
        let allowed_collision_matrix = vec![vec![true; num_collision_links]; num_collision_links];
        let sampler = ThreadRobotSampler::new(robot_shape_model.robot.clone());
        let link_pair_idxs = SelfCollisionEngine::get_all_link_pair_idxs(num_collision_links);
        // let last_collision_pair_idx = 0 as usize;
        let l = link_pair_idxs.len();
        let mut collision_pair_check_order: Vec<usize> = Vec::new();
        for i in 0..l {
            collision_pair_check_order.push(i);
        }
        let mut sce = Self {robot_shape_model, allowed_collision_matrix, sampler, link_pair_idxs, collision_pair_check_order};
        // let mut sce = Self {robot_shape_model, allowed_collision_matrix, sampler, link_pair_idxs, last_collision_pair_idx};
        sce.calibrate_allowed_collision_matrix(1000);
        sce
    }

    pub fn from_yaml_path(fp: String) -> Self {
        // yaml path to info file
        let robot = Robot::from_yaml_path(fp.clone());
        let ifp = InfoFileParser::from_yaml_path(fp.clone());
        let collision_file_name = ifp.collision_file_name;
        let fp2 = get_path_to_src() + "RelaxedIK/Config/collision_files_rust/" + collision_file_name.as_str();
        let robot_collision_specs_file = RobotCollisionSpecFileParser::from_yaml_path(fp2.clone());
        let robot_shape_model = RobotShapeModel::from_robot_and_specs(&robot, &robot_collision_specs_file, &ifp.starting_config);
        SelfCollisionEngine::from_robot_shape_model(robot_shape_model)
    }

    pub fn from_info_file_name(info_file_name: String) -> Self {
        let path_to_src = get_path_to_src();
        let fp = path_to_src + "RelaxedIK/Config/info_files/" + info_file_name.as_str();
        let robot_shape_model = RobotShapeModel::from_yaml_path(fp);
        SelfCollisionEngine::from_robot_shape_model(robot_shape_model)
     }

    pub fn calibrate_allowed_collision_matrix(&mut self, num_samples: usize) {
        let num_collision_links = self.robot_shape_model.link_info_arr.len();
        let mut collision_count_matrix = vec![vec![0.0; num_collision_links]; num_collision_links];
        let mut total_count = 0.0;
        for i in 0..num_samples {
            let sample = self.sampler.sample();
            self.robot_shape_model.update_robot_transforms(&sample.data.as_vec());
            for j in 0..num_collision_links {
                for k in 0..num_collision_links {
                    let in_collision = self.robot_shape_model.collision_check_full_shapes(j, k);
                    if in_collision {
                        collision_count_matrix[j][k] += 1.0;
                    }
                }
            }
            total_count += 1.0;
        }

        let mut allowed_collision_matrix = vec![vec![true; num_collision_links]; num_collision_links];

        for i in 0..num_collision_links {
            for j in 0..num_collision_links {
                if collision_count_matrix[i][j] / total_count > 0.98 {
                    allowed_collision_matrix[i][j] = false;
                }
            }
        }

        self.allowed_collision_matrix = allowed_collision_matrix;
    }

    /*
    pub fn collision_check(&mut self, state: &Vec<f64>) -> bool {
        self.robot_shape_model.update_robot_transforms(state);
        let l = self.link_pair_idxs.len();

        // start with the pair that was last seen to be in collision to try it as a short circuit
        let link_pair = self.link_pair_idxs[self.last_collision_pair_idx];
        if self.allowed_collision_matrix[link_pair.0][link_pair.1] {
            let in_collision = self.robot_shape_model.collision_check_full_shapes(link_pair.0, link_pair.1);
            if in_collision {
                return true;
            }
        }

        /*
        if self.allowed_collision_matrix[link_pair.0][link_pair.1] {
            self.robot_shape_model.update_bounding_sphere(link_pair.0);
            self.robot_shape_model.update_bounding_sphere(link_pair.1);
            let in_collision_bounding_sphere = self.robot_shape_model.collision_check_bounding_spheres(link_pair.0, link_pair.1);
            if in_collision_bounding_sphere { // bounding aabb check...
                self.robot_shape_model.update_bounding_aabb(link_pair.0);
                self.robot_shape_model.update_bounding_aabb(link_pair.1);
                let in_collision_bounding_aabb = self.robot_shape_model.collision_check_bounding_aabbs(link_pair.0, link_pair.1);
                if in_collision_bounding_aabb { // full shape check...
                    let in_collision = self.robot_shape_model.collision_check_full_shapes(link_pair.0, link_pair.1);
                    if in_collision {
                        return true;
                    }
                }
            }
        }
        */
        for i in 0..l {
            let link_pair = self.link_pair_idxs[i];
            if self.allowed_collision_matrix[link_pair.0][link_pair.1] && !(i == self.last_collision_pair_idx){ // bounding sphere check...
                self.robot_shape_model.update_bounding_sphere(link_pair.0);
                self.robot_shape_model.update_bounding_sphere(link_pair.1);
                let in_collision_bounding_sphere = self.robot_shape_model.collision_check_bounding_spheres(link_pair.0, link_pair.1);
                if in_collision_bounding_sphere { // bounding aabb check...
                    self.robot_shape_model.update_bounding_aabb(link_pair.0);
                    self.robot_shape_model.update_bounding_aabb(link_pair.1);
                    let in_collision_bounding_aabb = self.robot_shape_model.collision_check_bounding_aabbs(link_pair.0, link_pair.1);
                    if in_collision_bounding_aabb { // full shape check...
                        let in_collision = self.robot_shape_model.collision_check_full_shapes(link_pair.0, link_pair.1);
                        if in_collision {
                            self.last_collision_pair_idx = i;
                            return true;
                        }
                    }
                }
            }
        }

        return false;
    }
    */

    pub fn collision_check(&mut self, state: &Vec<f64>) -> bool {
        let frames = self.robot_shape_model.robot.get_frames_immutable(state);
        let mut updated_link_transform: Vec<usize> = Vec::new();
        let mut updated_link_bounding_sphere: Vec<usize> = Vec::new();
        let mut updated_link_bounding_aabb: Vec<usize> = Vec::new();
        let l = self.link_pair_idxs.len();

        for i in 0..l {
            let curr_link_pair_idx = self.collision_pair_check_order[i];
            let link_pair = self.link_pair_idxs[curr_link_pair_idx];
            if self.allowed_collision_matrix[link_pair.0][link_pair.1] {
                if !updated_link_transform.contains(&link_pair.0) {
                    self.robot_shape_model.update_robot_link_transform_from_frames(link_pair.0, &frames);
                    updated_link_transform.push(link_pair.0);
                }
                if !updated_link_transform.contains(&link_pair.1) {
                    self.robot_shape_model.update_robot_link_transform_from_frames(link_pair.1, &frames);
                    updated_link_transform.push(link_pair.1);
                }

                if !updated_link_bounding_sphere.contains(&link_pair.0) {
                    self.robot_shape_model.update_bounding_sphere(link_pair.0);
                    updated_link_bounding_sphere.push(link_pair.0);
                }
                if !updated_link_bounding_sphere.contains(&link_pair.1) {
                    self.robot_shape_model.update_bounding_sphere(link_pair.1);
                    updated_link_bounding_sphere.push(link_pair.1);
                }

                let in_collision_bounding_sphere = self.robot_shape_model.collision_check_bounding_spheres(link_pair.0, link_pair.1);

                if in_collision_bounding_sphere {
                    if !updated_link_bounding_aabb.contains(&link_pair.0) {
                        self.robot_shape_model.update_bounding_aabb(link_pair.0);
                        updated_link_bounding_aabb.push(link_pair.0);
                    }
                    if !updated_link_bounding_aabb.contains(&link_pair.1) {
                        self.robot_shape_model.update_bounding_aabb(link_pair.1);
                        updated_link_bounding_aabb.push(link_pair.1);
                    }

                    let in_collision_bounding_aabb = self.robot_shape_model.collision_check_bounding_aabbs(link_pair.0, link_pair.1);

                    if in_collision_bounding_aabb {
                        let in_collision = self.robot_shape_model.collision_check_full_shapes(link_pair.0, link_pair.1);
                        if in_collision {
                            self.update_collision_pair_check_order(curr_link_pair_idx);
                            return true;
                        }
                    }
                }
            }
        }

        false
    }

    pub fn update_collision_pair_check_order(&mut self, most_recent_collision_pair_idx: usize) {
        let mut array_idx = usize::max_value();
        let l = self.link_pair_idxs.len();
        for i in 0..l {
            if most_recent_collision_pair_idx == self.collision_pair_check_order[i] {
                array_idx = i;
                break;
            }
        }

        for i in 0..(array_idx) {
            self.collision_pair_check_order[array_idx - i] = self.collision_pair_check_order[array_idx - i - 1];
        }


        self.collision_pair_check_order[0] = most_recent_collision_pair_idx;
    }

    pub fn get_all_link_pair_idxs(num_links: usize) -> Vec<(usize, usize)> {
        let mut all_link_pair_idxs: Vec<(usize, usize)> = Vec::new();
        for i in 0..num_links {
            for j in 0..num_links {
                if !all_link_pair_idxs.contains(&(i,j)) && !all_link_pair_idxs.contains(&(j,i)) && !(i == j) {
                    all_link_pair_idxs.push((i,j));
                }
            }
        }
        all_link_pair_idxs
    }
}
