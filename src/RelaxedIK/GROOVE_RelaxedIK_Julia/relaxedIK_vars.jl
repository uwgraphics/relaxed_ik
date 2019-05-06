include("../GROOVE_Julia/vars.jl")
include("../Spacetime_Julia/arm.jl")
include("../Spacetime_Julia/robot.jl")
include("../Utils_Julia/nn_utils.jl")

using YAML
using Rotations
using StaticArrays
using Flux
using BSON

mutable struct RelaxedIK_vars
    vars
    robot
    position_mode
    rotation_mode
    goal_positions
    goal_quats
    goal_positions_relative
    goal_quats_relative
    init_ee_positions
    init_ee_quats
    joint_pts
    nn_model
    w
    nn_t
    nn_c
    nn_f
    additional_vars
end


function RelaxedIK_vars(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types; position_mode = "relative", rotation_mode = "relative", preconfigured=false)

    y = info_file_name_to_yaml_block(path_to_src, info_file_name)

    robot = yaml_block_to_robot(y)
    vars = Vars(y["starting_config"], objectives, grad_types, weight_priors, inequality_constraints, [], equality_constraints, [], y["joint_limits"])

    robot.getFrames(y["starting_config"])

    num_chains = robot.num_chains

    goal_positions = []
    goal_quats = []
    goal_positions_relative = []
    goal_quats_relative = []
    init_ee_positions = Array{SArray{Tuple{3},Float64,1,3},1}()
    init_ee_quats = Array{Quat{Float64},1}()

    for i in 1:num_chains
        push!(init_ee_positions, robot.arms[i].out_pts[end])
        push!(init_ee_quats, Quat(robot.arms[i].out_frames[end]))
        push!(goal_positions, SVector(0.0,0.0,0.0))
        push!(goal_quats, rand(Quat))
        push!(goal_positions_relative, SVector(0.,0.,0.))
        push!(goal_quats_relative, Quat(1.,0.,0.,0.))
    end

    if preconfigured == false
        collision_nn_file_name = y["collision_nn_file"]
        w = BSON.load(path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name)[:w]
        model = (x) -> predict(w, x)[1]
        #function model_nn(x, model, state_to_joint_pts_closure)
        #    return model(state_to_joint_pts_closure(x))
        #end
        fp = open(path_to_src * "/RelaxedIK/Config/collision_nn/" * collision_nn_file_name * "_params", "r")
        nn_params_line = readline(fp)
        split_arr = split(nn_params_line, ",")
        t_val = parse(Float64, split_arr[1])
        c_val = parse(Float64, split_arr[2])
        f_val = parse(Float64, split_arr[3])

        rv = RelaxedIK_vars(vars, robot, position_mode, rotation_mode, goal_positions, goal_quats, goal_positions_relative, goal_quats_relative, init_ee_positions, init_ee_quats, 0, model, w, t_val, c_val, f_val, 0)
        initial_joint_points = state_to_joint_pts_withreturn(rand(length(vars.init_state)), rv)
        rv.joint_pts = initial_joint_points

        # state_to_joint_pts_closure = (x) -> state_to_joint_pts(x, rv)
        # collision_nn = (x)-> model_nn(x, model, state_to_joint_pts_closure)
        # rv.collision_nn = collision_nn
    else
        rv = RelaxedIK_vars(vars, robot, position_mode, rotation_mode, goal_positions, goal_quats, goal_positions_relative, goal_quats_relative, init_ee_positions, init_ee_quats, 0, 0, 0, 0, 0, 0, 0)
    end

    populate_vars!(vars, rv)

    return rv
end

function update_relaxedIK_vars!(relaxedIK_vars, xopt)
    update!(relaxedIK_vars.vars, xopt)
end

function info_file_name_to_yaml_block(path_to_src, info_file_name)
    f = open(path_to_src * "/RelaxedIK/Config/info_files/" * info_file_name)
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
            y["rot_offsets"][i], y["joint_types"][i], true)
        push!(arms, a)
    end

    return arms
end
