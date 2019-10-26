use nalgebra::{UnitQuaternion, Vector3, Quaternion};
use crate::lib::utils_rust::yaml_utils::{get_yaml_obj, InfoFileParser};
use crate::lib::spacetime::robot::Robot;


#[derive(Clone, Debug)]
pub struct Vars {
    pub init_state: Vec<f64>,
    pub xopt: Vec<f64>,
    pub prev_state: Vec<f64>,
    pub prev_state2: Vec<f64>,
    pub prev_state3: Vec<f64>
}
impl Vars {
    pub fn new(init_state: Vec<f64>) -> Self {
        Vars{init_state: init_state.clone(), xopt: init_state.clone(), prev_state: init_state.clone(),
            prev_state2: init_state.clone(), prev_state3: init_state.clone()}
    }

    pub fn update(&mut self, xopt: Vec<f64>) {
        self.prev_state3 = self.prev_state2.clone();
        self.prev_state2 = self.prev_state.clone();
        self.prev_state = self.xopt.clone();
        self.xopt = xopt.clone();
    }
}

#[derive(Clone, Debug)]
pub struct RelaxedIKVars {
    pub robot: Robot,
    pub init_state: Vec<f64>,
    pub xopt: Vec<f64>,
    pub prev_state: Vec<f64>,
    pub prev_state2: Vec<f64>,
    pub prev_state3: Vec<f64>,
    pub goal_positions: Vec<Vector3<f64>>,
    pub goal_quats: Vec<UnitQuaternion<f64>>,
    pub init_ee_positions: Vec<Vector3<f64>>,
    pub init_ee_quats: Vec<UnitQuaternion<f64>>,
    pub position_mode_relative: bool, // if false, will be absolute
    pub rotation_mode_relative: bool, // if false, will be absolute
}
impl RelaxedIKVars {
    pub fn from_yaml_path(fp: &str, position_mode_relative: bool, rotation_mode_relative: bool) -> Self {
        let ifp = InfoFileParser::from_yaml_path(fp);
        let mut robot = Robot::from_yaml_path(fp);
        let num_chains = ifp.joint_names.len();

        let mut goal_positions: Vec<Vector3<f64>> = Vec::new();
        let mut goal_quats: Vec<UnitQuaternion<f64>> = Vec::new();

        for i in 0..num_chains {
            goal_positions.push(nalgebra::Vector3::identity());
            goal_quats.push(nalgebra::UnitQuaternion::identity());
        }

        let init_ee_positions = robot.get_ee_positions(ifp.starting_config.as_slice());
        let init_ee_quats = robot.get_ee_quats(ifp.starting_config.as_slice());

        RelaxedIKVars{robot, init_state: ifp.starting_config.clone(), xopt: ifp.starting_config.clone(),
            prev_state: ifp.starting_config.clone(), prev_state2: ifp.starting_config.clone(), prev_state3: ifp.starting_config.clone(),
            goal_positions, goal_quats, init_ee_positions, init_ee_quats, position_mode_relative, rotation_mode_relative}
    }

    pub fn from_yaml_path_with_init(fp: &str, init_state: Vec<f64>, position_mode_relative: bool, rotation_mode_relative: bool) -> Self {
        let mut robot = Robot::from_yaml_path(fp);
        let num_chains = robot.num_chains;

        let mut goal_positions: Vec<Vector3<f64>> = Vec::new();
        let mut goal_quats: Vec<UnitQuaternion<f64>> = Vec::new();

        let init_ee_positions = robot.get_ee_positions(init_state.as_slice());
        let init_ee_quats = robot.get_ee_quats(init_state.as_slice());

        for i in 0..num_chains {
            if position_mode_relative {
                goal_positions.push(nalgebra::Vector3::identity());
            } else {
                goal_positions.push(init_ee_positions[i]);
            }

            if rotation_mode_relative {
                goal_quats.push(nalgebra::UnitQuaternion::identity());
            } else {
                goal_quats.push(init_ee_quats[i]);
            }
        }

        RelaxedIKVars{robot, init_state: init_state.clone(), xopt: init_state.clone(),
            prev_state: init_state.clone(), prev_state2: init_state.clone(), prev_state3: init_state.clone(),
            goal_positions, goal_quats, init_ee_positions, init_ee_quats, position_mode_relative, rotation_mode_relative}

    }

    pub fn update(&mut self, xopt: Vec<f64>) {
        self.prev_state3 = self.prev_state2.clone();
        self.prev_state2 = self.prev_state.clone();
        self.prev_state = self.xopt.clone();
        self.xopt = xopt.clone();
    }
}
