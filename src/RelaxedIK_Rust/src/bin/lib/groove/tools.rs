use crate::spacetime::robot::{Robot};
use crate::spacetime::arm::{Arm};

#[derive(Clone, Debug)]
pub struct RelaxedIKTools {
    pub robot: Robot
}

impl RelaxedIKTools {
    pub fn new(robot: Robot) -> Self {
        RelaxedIKTools{robot}
    }

    pub fn from_yaml_path(fp: &str) -> Self {
        let robot = Robot::from_yaml_path(fp);
        RelaxedIKTools{robot}
    }
}