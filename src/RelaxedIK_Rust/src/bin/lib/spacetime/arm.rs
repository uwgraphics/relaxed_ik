use nalgebra;

pub struct Arm {
    pub joint_types: Vec<String>,
    pub starting_config: Vec<f64>,
    pub num_dof: usize
}

impl Arm{
    pub fn new(joint_types: Vec<String>, starting_config: Vec<f64>) -> Arm {
        let num_dof = joint_types.len();
        Arm{joint_types, starting_config, num_dof}
    }

    pub fn from_yaml_file_path(fp: &str) {

    }

}