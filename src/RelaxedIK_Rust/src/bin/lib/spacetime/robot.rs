use crate::lib::spacetime::arm;
use crate::utils_rust::{geometry_utils, yaml_utils};


pub struct Robot {
    pub arms: Vec<arm::Arm>,
    pub joint_names: Vec<Vec<String>>,
    pub joint_ordering: Vec<String>,
    pub num_chains: usize,
    pub num_dof: usize,
    pub subchain_indices: Vec<Vec<usize>>,
    pub bounds: Vec< [f64; 2] >,
    pub velocity_limits: Vec<f64>
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


        Robot{arms, joint_names: ifp.joint_names.clone(), joint_ordering: ifp.joint_ordering.clone(),
            num_chains, num_dof, subchain_indices, bounds: ifp.joint_limits.clone(), velocity_limits: ifp.velocity_limits.clone()}
    }

    pub fn from_yaml_path(fp: &str) -> Robot {
        let ifp = yaml_utils::InfoFileParser::from_yaml_path(fp);
        Robot::from_info_file_parser(&ifp)
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
                out[i].push(idx);
            }
        }

        out
    }

    fn get_index_from_joint_order(joint_ordering: &Vec<String>, joint_name: &String) -> usize {
        for i in 0..joint_ordering.len() {
            if *joint_name == joint_ordering[i] {
                return i
            }
        }
        0
    }
}

