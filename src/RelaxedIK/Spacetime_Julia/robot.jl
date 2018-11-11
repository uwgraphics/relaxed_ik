include("arm.jl")
using BenchmarkTools

mutable struct Robot
    arms
    full_joint_lists
    joint_order
    num_chains
    num_dof
    subchain_indices
    bounds
    velocity_limitss
    get_frame_funcs
    out_subchains
    split_state_into_subchains_c
    getFrames
end

function Robot(arms, full_joint_lists, joint_order)
    num_chains = length(arms)
    num_dof = length(joint_order)

    subchain_indices = Array{Array{Int64, 1}, 1}()
    out_subchains = Array{Array{Float64, 1},1}()
    for i in 1:num_chains
        push!(out_subchains, [])
        for j in 1:length(arms[i].axis_types)
            push!(out_subchains[i], 0.0)
        end
    end

    robot = Robot(arms, full_joint_lists, joint_order, num_chains, num_dof, subchain_indices,0,0,0, out_subchains, 0,0)
    initialize_subchain_indices!(robot)

    robot.split_state_into_subchains_c = split_state_into_subchains_closure(robot)
    get_frame_funcs = []
    for i in 1:length(arms)
        push!(get_frame_funcs, arms[i].getFrames)
    end
    robot.get_frame_funcs = get_frame_funcs
    robot.getFrames = getFrames_c(robot)

    return robot
end

function get_index_from_joint_order(robot, jt_name)
    for j in 1:length(robot.joint_order)
        curr_joint_name = robot.joint_order[j]
        if curr_joint_name == jt_name
            return j
        end
    end
    return Nothing
end

function initialize_subchain_indices!(robot)
    for i in 1:robot.num_chains
        push!(robot.subchain_indices, [])
    end

    for i in 1:robot.num_chains
        for j in robot.full_joint_lists[i]
            idx = get_index_from_joint_order(robot, j)
            if idx != Nothing
                push!(robot.subchain_indices[i], idx)
            end
        end
    end
end

function initialize_bounds(robot)
    bounds = []
    for i in 1:robot.num_dof
        push!(bounds, (0.0,0.0) )
    end

    for i in 1:length(robot.arms)
        sub_bounds = robot.arms[i].joint_limits
        for j in 1:length(sub_bounds)
            idx = robot.subchain_indices[i][j]
            bounds[idx] = sub_bounds[j]
        end
    end
    robot.bounds = bounds
end

function initialize_velocity_limits(robot)
    velocity_limits = []
    for i in 1:robot.num_dof
        push!(bounds, 0.0)
    end

    for i in 1:length(robot.arms)
        sub_vl = robot.arms[i].velocity_limits
        for j in 1:length(sub_vl)
            idx = robot.subchain_indices[i][j]
            velocity_limits[idx] = sub_vl[j]
        end
    end
    robot.velocity_limits = velocity_limits
end

function split_state_into_subchains(x, num_chains, subchain_indices, out_subchains)
    # out_subchains = Array{Array{Float64, 1},1}()
    for i in 1:length(num_chains)
        # subchain = Array{Float64, 1}()
        for j in 1:length(subchain_indices[i])
            # push!(subchain, x[s])
            out_subchains[i][j] = x[subchain_indices[i][j]]
        end
        # push!(subchains, subchain)
    end
    # return subchains
end

function split_state_into_subchains_closure(robot)
    f = x->split_state_into_subchains(x, robot.num_chains, robot.subchain_indices, robot.out_subchains)
    return f
end

function getFrames(x, robot)
    robot.split_state_into_subchains_c(x)
    out_subchains = robot.out_subchains
    for i in 1:length(robot.num_chains)
        robot.get_frame_funcs[i](out_subchains[i])
    end
end

function getFrames_c(robot)
    f = x->getFrames(x, robot)
    return f
end

#=
axis_types = ["-z","y","y","y","z","y"]
displacements = [[0.0, 0.13585, 0.0], [0.0, -0.1197, 0.425], [0.0, 0.0, 0.39225], [0, 0.093, 0], [0, 0, 0.09465], [0.0,0.0823,0.0]]
disp_offset = [0., 0., 0.089159]
rot_offsets = [[0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0]]
# rot_offsets = [eulerTupleTo3x3(t) for t in rot_offsets]
joint_types = ["revolute","revolute","revolute","revolute","revolute","revolute"]
# ur5 = Arm(axis_types,displacements,displacements,disp_offset,rot_offsets, joint_types,0,0,false)
ur5 = Arm(axis_types, displacements, disp_offset, rot_offsets,joint_types,0,0,false)
arms = [ur5]
joint_names = [ ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"] ]
joint_ordering = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

using BenchmarkTools

robot = Robot(arms, joint_names, joint_ordering)
robot.getFrames( [1.,1.,1.,1.,1.,1.1]  )
println(robot.arms[1].out_pts)

function test_func(x)
    robot.getFrames(x)
    ee_pt = robot.arms[1].out_pts[end]
    return norm(ee_pt)
end

using ReverseDiff, BenchmarkTools, Calculus

println(ForwardDiff.gradient(test_func, [1.,1.,1.,1.,1.,1.1] ))

#@btime robot.get_frame_funcs[1]([1.,1.,1.,1.,1.,1.1])
# show(robot.arms[1].out_pts)
#func = getFrames_closure(robot)
#func([1.,1.,1.,1.,1.,1.1])
#os = robot.out_subchains
#si = robot.subchain_indices
#split_state_into_subchains([1.,1.,1.,1.,1.,1.1], 1, si, os)
# @btime split_state_into_subchains(robot, [1.,1.,1.,1.,1.,1.])
#@btime robot.split_state_into_subchains_c([1.,1.,1.,1.2,1.,1.1])
#@btime robot.out_subchains
=#
