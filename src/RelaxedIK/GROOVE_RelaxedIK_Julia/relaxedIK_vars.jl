include("../GROOVE_Julia/vars.jl")
include("../Spacetime_Julia/arm.jl")
include("../Spacetime_Julia/robot.jl")

using YAML
using Rotations
using StaticArrays

mutable struct RelaxedIK_vars
    vars
    robot
    position_mode
    rotation_mode
    goal_positions
    goal_quats
    init_ee_positions
    init_ee_quats
end

function RelaxedIK_vars(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types; position_mode = "relative", rotation_mode = "relative")

    y = info_file_name_to_yaml_block(path_to_src, info_file_name)

    robot = yaml_block_to_robot(y)
    vars = Vars(y["starting_config"], objectives, grad_types, weight_priors, inequality_constraints, [], equality_constraints, [], y["joint_limits"])

    robot.getFrames(y["starting_config"])

    num_chains = robot.num_chains

    goal_positions = []
    goal_quats = []
    init_ee_positions = []
    init_ee_quats = []

    for i in 1:num_chains
        push!(init_ee_positions, robot.arms[i].out_pts[end])
        push!(init_ee_quats, Quat(robot.arms[i].out_frames[end]))
        push!(goal_positions, SVector(0.,0.,0.))
        push!(goal_quats, Quat(1.,0.,0.,0.))
    end

    rv = RelaxedIK_vars(vars, robot, position_mode, rotation_mode, goal_positions, goal_quats, init_ee_positions, init_ee_quats)

    populate_vars!(vars, rv)

    return rv
end

function update!(relaxedIK_vars)

end

function info_file_name_to_yaml_block(path_to_src, info_file_name)
    f = open(path_to_src * "/RelaxedIK/Config/info_files/ur5_info.yaml")
    y = YAML.load(f)
    return y
end

function yaml_block_to_robot(y)
    arms = yaml_block_to_arms(y)
    robot = Robot(arms, y["joint_names"], y["joint_ordering"], y["joint_limits"], y["velocity_limits"])
    return robot
end

function yaml_block_to_arms(y)
    num_chains = length(y["joint_names"])

    arms = []

    for i=1:num_chains
        a = Arm(y["axis_types"][i], y["displacements"][i],  y["disp_offsets"][i],
            y["rot_offsets"][i], y["joint_types"][i], false)
        push!(arms, a)
    end

    return arms
end

path_to_src = "/home/rakita/catkin_ws/src/relaxed_ik/src/"
f = open(path_to_src * "/RelaxedIK/Config/info_files/hubo_info.yaml")

y = YAML.load(f)

using BenchmarkTools

# arms = yaml_block_to_arms(y)
# robot = yaml_block_to_robot(y)
# robot.getFrames([0., 0., 0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.])
# println(robot.subchain_indices[2][1])
# println(robot.out_subchains)
# println(robot.arms[1].out_pts)
# println(robot.arms[2].out_pts)

# @btime arms[1].getFrames([0.,0.,0.,0.,0.,0.])
# println(arms[1].out_pts)

vars = RelaxedIK_vars(path_to_src, "hubo_info.yaml", [], [], [], [], [], [], [])
