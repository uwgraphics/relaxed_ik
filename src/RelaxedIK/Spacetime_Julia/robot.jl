using BenchmarkTools
include("arm.jl")


mutable struct Robot
    arms
    full_joint_lists
    joint_order
    num_chains
    num_dof
    subchain_indices
    frame_subchain_indices # the indices in arm chain frames where the joints in the x array are situated
    frame_subchain_indices_concat # all vectors glued together in the format of nn inputs
    x_subchain_indices # the values in x go to which indicies in their chains
    x_frame_indices # the values in x go to which indices in frames
    x_frame_chain_inclusion
    bounds
    velocity_limits
    get_frame_funcs
    out_subchains
    split_state_into_subchains_c
    getFrames
    getFramesf
    total_num_joint_pts
    linear_jacobian
end

function Robot(arms, full_joint_lists, joint_order, bounds, velocity_limits)
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

    robot = Robot(arms, full_joint_lists, joint_order, num_chains, num_dof, subchain_indices,0,0,0,0,0,bounds,velocity_limits,0, out_subchains, 0,0,0,0,0)
    initialize_subchain_indices!(robot)

    robot.split_state_into_subchains_c = split_state_into_subchains_closure(robot)
    get_frame_funcs = []
    for i in 1:length(arms)
        push!(get_frame_funcs, arms[i].getFrames)
    end
    robot.get_frame_funcs = get_frame_funcs
    robot.getFrames = getFrames_c(robot)
    robot.getFramesf = getFramesf_c(robot)

    robot.getFramesf(rand(robot.num_dof))
    num_joint_pts = 0
    for i = 1:robot.num_chains
        num_joint_pts += length(robot.arms[i].static_out_pts)
    end
    robot.total_num_joint_pts = num_joint_pts
    j = zeros(robot.total_num_joint_pts*3, robot.num_dof)
    robot.linear_jacobian = j

    frame_subchain_indices = []
    for i = 1:num_chains
        push!(frame_subchain_indices, [])
        arm = arms[i]
        subchain_indices = robot.subchain_indices[i]
        idx = 1
        for j = 1:length(arm.joint_types)
            if !(arm.joint_types[j] == "fixed")
                push!(frame_subchain_indices[i], subchain_indices[idx])
                idx += 1
            else
                push!(frame_subchain_indices[i], -1)
            end
        end
        push!(frame_subchain_indices[i], -1)
    end

    robot.frame_subchain_indices = frame_subchain_indices

    frame_subchain_indices_concat = Array{Int64, 1}()
    for i = 1:length(frame_subchain_indices)
        for j = 1:length(frame_subchain_indices[i])
            push!(frame_subchain_indices_concat, frame_subchain_indices[i][j])
        end
    end

    robot.frame_subchain_indices_concat = frame_subchain_indices_concat

    x_frame_indices = Array{Array{Array{Int64,1},1},1}()
    for i = 1:num_dof
        push!(x_frame_indices, Array{Array{Int64,1},1}())
        for j = 1:num_chains
            for k = 1:length(frame_subchain_indices[j])
                if frame_subchain_indices[j][k] == i
                    push!(x_frame_indices[i], [j, k])
                end
            end
        end
    end

    robot.x_frame_indices = x_frame_indices

    x_subchain_indices = Array{Array{Array{Int64,1},1},1}()
    for i = 1:num_dof
        push!(x_subchain_indices, Array{Array{Int64,1},1}())
        for j = 1:num_chains
            for k = 1:length(robot.subchain_indices[j])
                if robot.subchain_indices[j][k] == i
                    push!(x_subchain_indices[i], [j, k])
                end
            end
        end
    end

    robot.x_subchain_indices = x_subchain_indices

    x_frame_chain_inclusion = Array{Array{Int64,1},1}()
    for i = 1:length(x_frame_indices)
        push!(x_frame_chain_inclusion, Array{Int64,1}())
        curr = x_frame_indices[i]
        for j = 1:length(curr)
            push!(x_frame_chain_inclusion[i], curr[j][1])
        end
    end

    robot.x_frame_chain_inclusion = x_frame_chain_inclusion

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
    for i in 1:num_chains
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
    for i in 1:robot.num_chains
        robot.get_frame_funcs[i](out_subchains[i])
    end
end

function getFramesf(x, robot)
    robot.split_state_into_subchains_c(x)
    out_subchains = robot.out_subchains
    for i in 1:robot.num_chains
        robot.arms[i].getFramesf(x)
    end
end

function getFrames_c(robot)
    f = x->getFrames(x, robot)
    return f
end

function getFramesf_c(robot)
    f = x->getFramesf(x, robot)
    return f
end

function get_linear_jacobian(robot, x)
    robot.getFramesf(x)
    robot.split_state_into_subchains_c(x)
    out_subchains = robot.out_subchains
    frame_subchain_indices = robot.frame_subchain_indices
    subchain_indices = robot.subchain_indices

    for i = 1:robot.num_dof # columns of jacobian
        for j = 1:robot.num_chains # loop through chains
            if j in robot.x_frame_chain_inclusion[i] # if joint i even affects chain j
                joint_frame_idx = robot.x_frame_indices[i][1][2] # TODO: need to change that hard coded 1 at some point, will work for now though!
                joint_subchain_idx = robot.x_subchain_indices[i][1][2] # TODO: need to change that hard coded 1 at some point, will work for now though!
                joint_position = robot.arms[j].static_out_pts[joint_frame_idx]
                rotation_axis = robot.arms[j].axis_types[joint_subchain_idx]
                if rotation_axis == "z" || rotation_axis == "Z" || rotation_axis == "-z"
                    joint_axis = robot.arms[j].static_out_frames[joint_frame_idx][:,3]
                    # return [c -s 0.; s c 0.; 0. 0. 1.]
                elseif rotation_axis == "y" || rotation_axis == "Y" || rotation_axis == "-y"
                    # return [c 0. s; 0. 1. 0.; -s 0. c]
                    joint_axis = robot.arms[j].static_out_frames[joint_frame_idx][:,2]
                elseif rotation_axis == "x" || rotation_axis == "X" || rotation_axis == "-x"
                    # return [1. 0. 0.; 0. c -s; 0. s c]
                    joint_axis = robot.arms[j].static_out_frames[joint_frame_idx][:,1]
                end

                num_out_pts = length(robot.arms[j].static_out_pts)

                row_idx = 1
                if j > 1
                    for m = 2:1:j
                        row_idx += length(robot.arms[m].static_out_pts)*3
                    end
                end

                for k = 1:num_out_pts # loop through joint points in chains, provided they are affected by joint i
                    if joint_frame_idx < k # only need to change values if joint in chain precedes frame point
                        jvec = LinearAlgebra.cross(joint_axis, robot.arms[j].static_out_pts[k] - joint_position)
                        robot.linear_jacobian[ row_idx:row_idx+2, i ] = jvec
                    end
                    row_idx += 3
                end
            end
        end
    end
end

Base.@ccallable function julia_main(ARGS::Vector{String})::Cint
    axis_types = ["-z","y","y","y","z","y"]
    displacements = [[0.0, 0.13585, 0.0], [0.0, -0.1197, 0.425], [0.0, 0.0, 0.39225], [0, 0.093, 0], [0, 0, 0.09465], [0.0,0.0823,0.0]]
    disp_offset = [0., 0., 0.089159]
    rot_offsets = [[0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0]]
    bounds = [[0.,0.],[0.,0.],[0.,0.],[0.,0.],[0.,0.],[0.,0.]]
    velocity_limits = [0.,0.,0.,0.,0.,0.]

    # rot_offsets = [eulerTupleTo3x3(t) for t in rot_offsets]
    joint_types = ["revolute","revolute","revolute","revolute","revolute","revolute"]
    # ur5 = Arm(axis_types,displacements,displacements,disp_offset,rot_offsets, joint_types,0,0,false)
    ur5 = Arm(axis_types, displacements, disp_offset, rot_offsets,joint_types,false)
    arms = [ur5]
    joint_names = [ ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"] ]
    joint_ordering = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

    robot = Robot(arms, joint_names, joint_ordering,bounds,velocity_limits)
    robot.getFrames( [1.,1.,1.,1.,1.,1.1]  )
    println(robot.arms[1].out_pts)
    return 0
end





#=
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
