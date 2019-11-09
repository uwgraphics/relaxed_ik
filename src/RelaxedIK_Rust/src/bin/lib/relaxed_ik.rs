use crate::lib::groove::vars::RelaxedIKVars;
use crate::lib::groove::groove::{OptimizationEngineOpen, OptimizationEngineNLopt};
use crate::lib::groove::objective_master::ObjectiveMaster;
use crate::lib::utils_rust::file_utils::{*};

pub struct RelaxedIK {
    pub vars: RelaxedIKVars,
    pub groove: OptimizationEngineOpen,
    pub om: ObjectiveMaster
}

impl RelaxedIK {
    pub fn from_info_file_name(info_file_name: String, mode: usize) -> Self {
        let path_to_src = get_path_to_src();
        let fp = path_to_src + "RelaxedIK/Config/info_files/" + info_file_name.as_str();
        RelaxedIK::from_yaml_path(fp.clone(), mode.clone())
    }

    pub fn from_yaml_path(fp: String, mode: usize) -> Self {
        let vars = RelaxedIKVars::from_yaml_path(fp.clone(), true, true);
        let mut om = ObjectiveMaster::relaxed_ik();
        if mode == 0 {
            om = ObjectiveMaster::standard_ik();
        }
        let groove = OptimizationEngineOpen::new(vars.robot.num_dof);

        Self{vars, groove, om}
    }

    pub fn from_loaded(mode: usize) -> Self {
        let path_to_src = get_path_to_src();
        let fp1 = path_to_src +  "RelaxedIK/Config/loaded_robot";
        let info_file_name = get_file_contents(fp1);
        RelaxedIK::from_info_file_name(info_file_name.clone(), mode.clone())
    }
}