use crate::lib::spacetime::robot::{Robot};
use crate::lib::spacetime::arm::{Arm};

#[derive(Clone, Debug)]
pub struct RelaxedIKTools {
    pub robot: Robot
}

impl RelaxedIKTools {
    pub fn new(robot: Robot) -> Self {
        RelaxedIKTools{robot}
    }

    pub fn from_yaml_path(fp: String) -> Self {
        let robot = Robot::from_yaml_path(fp.clone());
        RelaxedIKTools{robot}
    }
}