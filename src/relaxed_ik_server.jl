#!/usr/bin/env julia
#

include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_objective.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_vars.jl")
include("RelaxedIK/Utils_Julia/nn_utils.jl")
include("RelaxedIK/Utils_Julia/solver_output.jl")
using YAML
using RobotOS
using Rotations
using BenchmarkTools
using ForwardDiff
using Knet
using Dates
using Distributions
@rosimport relaxed_ik.msg : EEPoseGoals, JointAngles
@rosimport relaxed_ik.srv: RelaxedIKSolution, IKSolution, SelfCollisionQuery
@rosimport std_msgs.msg: Float64MultiArray, Bool, Float32, Int8
@rosimport geometry_msgs.msg: Point, Quaternion, Pose

rostypegen()
using .relaxed_ik.msg
using .relaxed_ik.srv
using .std_msgs.msg
using .geometry_msgs.msg

function get_rand_state_with_bounds(bounds)
    sample = Array{Float64, 1}()
    for b in bounds
        push!(sample, rand(Uniform(b[1], b[2])))
    end
    return sample
end

function rik_handler(relaxedIK, req)
    pose_goals = req.ee_pose_goals.ee_poses

    pos_goals = []
    quat_goals = []

    for i = 1:num_chains
        p = pose_goals[i]

        pos_x = p.position.x
        pos_y = p.position.y
        pos_z = p.position.z

        quat_w = p.orientation.w
        quat_x = p.orientation.x
        quat_y = p.orientation.y
        quat_z = p.orientation.z

        push!(pos_goals, [pos_x, pos_y, pos_z])
        push!(quat_goals, Quat(quat_w, quat_x, quat_y, quat_z))
    end

    xopt = solve(relaxedIK, pos_goals, quat_goals)
    println(xopt)

    ja = JointAngles()
    for i = 1:length(xopt)
        push!(ja.angles.data, xopt[i])
    end

    return RelaxedIKSolutionResponse(ja)
end

function ik_handler(relaxedIK_base, req)
    pose_goals = req.ee_pose_goals.ee_poses

    pos_goals = []
    quat_goals = []

    for i = 1:num_chains
        p = pose_goals[i]

        pos_x = p.position.x
        pos_y = p.position.y
        pos_z = p.position.z

        quat_w = p.orientation.w
        quat_x = p.orientation.x
        quat_y = p.orientation.y
        quat_z = p.orientation.z

        push!(pos_goals, [pos_x, pos_y, pos_z])
        push!(quat_goals, Quat(quat_w, quat_x, quat_y, quat_z))
    end

    xopt, try_idx, valid_sol, pos_error, rot_error = solve_precise(relaxedIK_base, pos_goals, quat_goals, max_iter=40, max_tries=2)
    println(xopt)

    ja = JointAngles()
    for i = 1:length(xopt)
        push!(ja.angles.data, xopt[i])
    end

    # ret = IKSolutionResponse((ja, valid_sol))
    # ret.joint_angles = ja
    # ret.success = valid_sol
    # return ret
    return IKSolutionResponse(ja, valid_sol)
end

function ik_rand_start_handler(relaxedIK_base, req)
    pose_goals = req.ee_pose_goals.ee_poses

    pos_goals = []
    quat_goals = []

    for i = 1:num_chains
        p = pose_goals[i]

        pos_x = p.position.x
        pos_y = p.position.y
        pos_z = p.position.z

        quat_w = p.orientation.w
        quat_x = p.orientation.x
        quat_y = p.orientation.y
        quat_z = p.orientation.z

        push!(pos_goals, [pos_x, pos_y, pos_z])
        push!(quat_goals, Quat(quat_w, quat_x, quat_y, quat_z))
    end

    prev_state = get_rand_state_with_bounds(relaxedIK_base.relaxedIK_vars.robot.bounds)
    # prev_state = rand(6)
    # println(prev_state)
    xopt, try_idx, valid_sol, pos_error, rot_error = solve_precise(relaxedIK_base, pos_goals, quat_goals, prev_state = prev_state, max_tries = 2, pos_tol = req.ee_pos_precision, rot_tol = req.ee_rot_precision, max_iter=40)
    println(xopt)

    ja = JointAngles()
    for i = 1:length(xopt)
        push!(ja.angles.data, xopt[i])
    end

    # ret = IKSolutionResponse()
    # ret.joint_angles = ja
    # ret.success = valid_sol
    # return ret
    return IKSolutionResponse(ja, valid_sol)
end

function selfcollision_handler(relaxedIK, req)
    x = req.query_state.angles.data
    # println(query_state)
    #x = []
    #for i = 1:length(query_state)
    #    push!(x, query_state[i].data)
    #end
    #println(x)
    vars = relaxedIK.relaxedIK_vars
    state_to_joint_pts_inplace(x, vars)

    val = groove_loss(  vars.nn_model2( vars.joint_pts ) , vars.nn_t2, 2, vars.nn_c2, vars.nn_f2, 2 )
    b = BoolMsg()
    if val > 1.0
        b.data = true
    else
        b.data = false
    end
    return b
end

function reset_solver(relaxedIK, relaxedIK_base, path_to_src, loaded_robot, req)
    relaxedIK = get_standard(path_to_src, loaded_robot)
    relaxedIK_base = get_base_ik(path_to_src, loaded_robot)
end

path_to_src = Base.source_dir()
loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
loaded_robot = readline(loaded_robot_file)
close(loaded_robot_file)

relaxedIK = get_standard(path_to_src, loaded_robot)
relaxedIK_base = get_base_ik(path_to_src, loaded_robot)
num_chains = relaxedIK.relaxedIK_vars.robot.num_chains

#=
if num_chains == 2
    relaxedIK = get_bimanual(path_to_src, loaded_robot)
    relaxedIK_base = get_bimanual_base_ik(path_to_src, loaded_robot)
elseif num_chains == 3
    relaxedIK = get_3chain(path_to_src, loaded_robot)
    relaxedIK_base = get_3chain_base_ik(path_to_src, loaded_robot)
elseif num_chains == 4
    relaxedIK = get_4chain(path_to_src, loaded_robot)
    relaxedIK_base = get_4chain_base_ik(path_to_src, loaded_robot)
elseif num_chains == 5
    relaxedIK = get_5chain(path_to_src, loaded_robot)
    relaxedIK_base = get_5chain_base_ik(path_to_src, loaded_robot)
end
=#

for i = 1:10
    pos_goals = []
    quat_goals = []
    for j = 1:num_chains
        push!(pos_goals, [0.,0.,0.])
        push!(quat_goals, Quat(1.,0.,0.,0.) )
    end
    solve(relaxedIK, pos_goals, quat_goals)
    solve(relaxedIK_base, pos_goals, quat_goals)
end

init_node("relaxed_ik_server")
println("RelaxedIK Server Has Been Initialized!")

rik_handler_c = req->rik_handler(relaxedIK, req)
rik_service = Service("relaxed_ik_solution", RelaxedIKSolution, rik_handler_c)
ik_handler_c = req->ik_handler(relaxedIK_base, req)
ik_service = Service("ik_solution", IKSolution, ik_handler_c)
ik_rs_handler_c = req->ik_rand_start_handler(relaxedIK_base, req)
ik_rs_service = Service("ik_rand_start_solution", IKSolution, ik_rs_handler_c)
selfcollision_handler_c = req->selfcollision_handler(relaxedIK, req)
self_collision_service = Service("self_collision_query", SelfCollisionQuery, selfcollision_handler_c)

spin()
