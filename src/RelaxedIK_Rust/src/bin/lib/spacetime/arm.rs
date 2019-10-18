use nalgebra;

pub struct Arm {
    pub axis_types: Vec<String>,
    pub displacements: Vec<nalgebra::Vector3<f64>>,
    pub disp_offset: nalgebra::Vector3<f64>,
    pub joint_types: Vec<String>,
    pub num_dof: usize
}

impl Arm{
    pub fn new(axis_types: Vec<String>,
        displacements: Vec<nalgebra::Vector3<f64>>, disp_offset: nalgebra::Vector3<f64>,
        joint_types: Vec<String>) -> Arm {

        let num_dof = axis_types.len();

        Arm{axis_types, displacements, disp_offset, joint_types, num_dof}
    }
}