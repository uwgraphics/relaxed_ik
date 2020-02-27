use crate::lib::spacetime::arm;
use crate::lib::utils_rust::{geometry_utils, yaml_utils};

#[derive(Clone, Debug)]
pub struct Robot {
    pub arms: Vec<arm::Arm>,
    pub joint_names: Vec<Vec<String>>,
    pub joint_ordering: Vec<String>,
    pub num_chains: usize,
    pub num_dof: usize,
    pub subchain_indices: Vec<Vec<usize>>,
    pub bounds: Vec< [f64; 2] >,
    pub lower_bounds: Vec<f64>,
    pub upper_bounds: Vec<f64>,
    pub velocity_limits: Vec<f64>,
    __subchain_outputs: Vec<Vec<f64>>
}

impl Robot {
    pub fn from_info_file_parser(ifp: &yaml_utils::InfoFileParser) -> Robot {
        let num_chains = ifp.axis_types.len();
        let num_dof = ifp.velocity_limits.len();

        let mut arms: Vec<arm::Arm> = Vec::new();
        for i in 0..num_chains {
            let a = arm::Arm::new(ifp.axis_types[i].clone(), ifp.displacements[i].clone(),
                              ifp.disp_offsets[i].clone(), ifp.rot_offsets[i].clone(), ifp.joint_types[i].clone());
            arms.push(a);
        }

        let subchain_indices = Robot::get_subchain_indices(&ifp.joint_names, &ifp.joint_ordering);

        let mut __subchain_outputs: Vec<Vec<f64>> = Vec::new();
        for i in 0..subchain_indices.len() {
            let v: Vec<f64> = Vec::new();
            __subchain_outputs.push(v);
            for j in 0..subchain_indices[i].len() {
                __subchain_outputs[i].push(0.0);
            }
        }

        let mut upper_bounds: Vec<f64> = Vec::new();
        let mut lower_bounds: Vec<f64> = Vec::new();
        for i in 0..ifp.joint_limits.len() {
            upper_bounds.push(ifp.joint_limits[i][1].clone());
            lower_bounds.push(ifp.joint_limits[i][0].clone());
        }

        Robot{arms, joint_names: ifp.joint_names.clone(), joint_ordering: ifp.joint_ordering.clone(),
            num_chains, num_dof, subchain_indices, bounds: ifp.joint_limits.clone(), lower_bounds, upper_bounds, velocity_limits: ifp.velocity_limits.clone(), __subchain_outputs}
    }

    pub fn from_yaml_path(fp: String) -> Robot {
        let ifp = yaml_utils::InfoFileParser::from_yaml_path(fp);
        Robot::from_info_file_parser(&ifp)
    }

    pub fn split_into_subchains(&self, x: &[f64]) -> Vec<Vec<f64>>{
        let mut out_subchains: Vec<Vec<f64>> = Vec::new();
        for i in 0..self.num_chains {
            let s: Vec<f64> = Vec::new();
            out_subchains.push(s);
            for j in 0..self.subchain_indices[i].len() {
                out_subchains[i].push( x[self.subchain_indices[i][j]] );
            }
        }
        out_subchains
    }

    pub fn split_into_subchains_inplace(&mut self, x: &[f64]) {
        // let mut out_subchains: Vec<Vec<f64>> = Vec::new();
        for i in 0..self.num_chains {
            let s: Vec<f64> = Vec::new();
            // out_subchains.push(s);
            for j in 0..self.subchain_indices[i].len() {
                self.__subchain_outputs[i][j] = x[self.subchain_indices[i][j]];
            }
        }
    }

    pub fn get_frames(&mut self, x: &[f64]) {
        self.split_into_subchains_inplace(x);
        for i in 0..self.num_chains {
            self.arms[i].get_frames(self.__subchain_outputs[i].as_slice());
        }
    }

    pub fn get_frames_immutable(&self, x: &[f64]) -> Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)> {
        let mut out: Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)> = Vec::new();
        let subchains = self.split_into_subchains(x);
        for i in 0..self.num_chains {
            out.push( self.arms[i].get_frames_immutable( subchains[i].as_slice() ) );
        }
        out
    }

    pub fn get_ee_pos_and_quat_immutable(&self, x: &[f64]) -> Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)> {
        let mut out: Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)> = Vec::new();
        let subchains = self.split_into_subchains(x);
        for i in 0..self.num_chains {
            out.push( self.arms[i].get_ee_pos_and_quat_immutable( subchains[i].as_slice() ) );
        }
        out
    }

    pub fn get_ee_positions(&mut self, x: &[f64]) -> Vec<nalgebra::Vector3<f64>> {
        let mut out: Vec<nalgebra::Vector3<f64>> = Vec::new();
        self.split_into_subchains_inplace(x);
        for i in 0..self.num_chains {
            out.push(self.arms[i].get_ee_position(self.__subchain_outputs[i].as_slice()));
        }
        out
    }

    pub fn get_ee_rot_mats(&mut self, x: &[f64]) -> Vec<nalgebra::Matrix3<f64>> {
        let mut out: Vec<nalgebra::Matrix3<f64>> = Vec::new();
        self.split_into_subchains_inplace(x);
        for i in 0..self.num_chains {
            out.push(self.arms[i].get_ee_rot_mat(self.__subchain_outputs[i].as_slice()));
        }
        out
    }

    pub fn get_ee_quats(&mut self, x: &[f64]) -> Vec<nalgebra::UnitQuaternion<f64>> {
        let mut out: Vec<nalgebra::UnitQuaternion<f64>> = Vec::new();
        self.split_into_subchains_inplace(x);
        for i in 0..self.num_chains {
            out.push(self.arms[i].get_ee_quat(self.__subchain_outputs[i].as_slice()));
        }
        out
    }

    fn get_subchain_indices(joint_names: &Vec<Vec<String>>, joint_ordering: &Vec<String>) -> Vec<Vec<usize>> {
        let mut out: Vec<Vec<usize>> = Vec::new();

        let num_chains = joint_names.len();
        for i in 0..num_chains {
            let v: Vec<usize> = Vec::new();
            out.push(v);
        }

        for i in 0..num_chains {
            for j in 0..joint_names[i].len() {
                let idx = Robot::get_index_from_joint_order(joint_ordering, &joint_names[i][j]);
                if  idx == 101010101010 {
                } else {
                    out[i].push(idx);
                }
            }
        }
        out
    }

    pub fn get_index_from_joint_order(joint_ordering: &Vec<String>, joint_name: &String) -> usize {
        for i in 0..joint_ordering.len() {
            if *joint_name == joint_ordering[i] {
                return i
            }
        }
        101010101010
    }

}

