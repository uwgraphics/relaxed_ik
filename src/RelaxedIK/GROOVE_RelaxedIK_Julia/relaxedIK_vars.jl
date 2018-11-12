include("../GROOVE_Julia/vars.jl")
include
using YAML

mutable struct RelaxedIK_vars
    vars
    robot
end

function RelaxedIK_vars(path_to_src, info_file_name, init_state, objectives, grad_types, weight_priors, inequality_constraints, equality_constraints)

    y = info_file_name_to_yaml_block(path_to_src, info_file_name)
    # bounds will be set up manually from yaml before making vars object
end

function info_file_name_to_yaml_block(path_to_src, info_file_name)
    f = open(path_to_src * "/RelaxedIK/Config/info_files/ur5_info.yaml")
    y = YAML.load(f)
    return y
end

function yaml_block_to_robot(y)
    arms = yaml_blcok_to_arms(y)
end

function yaml_block_to_arms(y)

end

path_to_src = "/home/rakita/catkin_ws/src/relaxed_ik/src/"
