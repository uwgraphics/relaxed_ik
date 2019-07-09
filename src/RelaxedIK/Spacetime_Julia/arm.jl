
import LinearAlgebra, Rotations, StaticArrays
using BenchmarkTools

mutable struct Arm
    joint_names
    axis_types
    displacements
    original_displacements
    static_displacements
    disp_offset
    static_disp_offset
    rot_offsets
    joint_types
    do_rot_offsets
    out_pts
    out_frames
    getFrames
    getFramesf
    getFramesh
    homogeneous_matrices # deprecated
    rot_offset_present_arr
    rot_mat_tmp
    static_rot_mat_tmp
    fk_results # deprecated
    static_rot_offsets # list of SMatrix objects
    static_rot_matrices # result after 3x3 rotations.  All SMatrix
    static_out_pts
    static_out_frames
end

function Arm(joint_names, axis_types, displacements, disp_offset, rot_offsets, joint_types, do_rot_offsets)
    rot_offset_matrices = [eulerTupleTo3x3(t) for t in rot_offsets]
    rot_mat_tmp = Array{MArray{Tuple{3,3},Float64,2,9}, 1}()
    count = 1
    for i = 1:length(rot_offset_matrices)-1
        if joint_types[i] == "fixed"
            push!(rot_mat_tmp, MMatrix{3,3}(rot_offset_matrices[i]))
        else
            r = rot3(axis_types[count], sin(0.), cos(0.))
            push!(rot_mat_tmp, MMatrix{3,3}(r))
        end
    end

    push!(rot_mat_tmp, MMatrix{3,3}(rot_offset_matrices[end]))

    static_rot_offsets = Array{SArray{Tuple{3,3},Float64,2,9}, 1}()
    for i = 1:length(rot_offset_matrices)
        push!(static_rot_offsets, SMatrix{3,3}(rot_offset_matrices[i]))
    end
    static_rot_matrices = copy(static_rot_offsets)
    static_displacements = Array{SArray{Tuple{3},Float64,1,3}, 1}()
    for i in 1:length(displacements)
        push!(static_displacements, SVector( displacements[i][1], displacements[i][2], displacements[i][3])   )
    end
    original_displacements = copy(displacements)
    static_disp_offset = SVector( disp_offset[1], disp_offset[2], disp_offset[3] )
    id_mat = one(RotMatrix{3, Float64})

    # homogeneous_matrices = Array{MArray{Tuple{4,4},Float64,2,16}, 1}()
    # fk_results = Array{MArray{Tuple{4,4},Float64,2,16}, 1}()
    homogeneous_matrices = MArray{Tuple{4,4},Float64,2,16}[]
    fk_results =  MArray{Tuple{4,4},Float64,2,16}[]
    for i = 1:length(rot_offsets)
        push!(homogeneous_matrices, MMatrix{4,4}([1. 0. 0. 0.; 0. 1. 0. 0.; 0. 0. 1. 0.; 0. 0. 0. 1.]))
        push!(fk_results, MMatrix{4,4}([1. 0. 0. 0.; 0. 1. 0. 0.; 0. 0. 1. 0.; 0. 0. 0. 1.]))
    end

    rot_offset_present_arr = Array{Bool, 1}()
    for i = 1:length(rot_offset_matrices)
        if rot_offset_matrices[i] == [1. 0. 0.; 0. 1. 0.; 0. 0. 1.]
            push!(rot_offset_present_arr, false)
        else
            push!(rot_offset_present_arr, true)
        end
    end

    # rot_mat_tmp = MMatrix{3,3}([1. 0. 0.; 0. 1. 0.; 0. 0. 1.])
    static_rot_mat_tmp = SMatrix{3,3}([1. 0. 0.; 0. 1. 0.; 0. 0. 1.])

    static_out_frames = Array{SArray{Tuple{3,3},Float64,2,9}, 1}()
    static_out_pts = Array{SArray{Tuple{3},Float64,1,3}, 1}()
    for i = 1:length(static_rot_matrices)
        push!(static_out_frames, SMatrix{3,3}(rand(3,3)))
        push!(static_out_pts, SVector{3}( rand(3) ) )
    end

    arm = Arm(joint_names, axis_types, displacements, original_displacements, static_displacements,
        disp_offset, static_disp_offset, rot_offset_matrices, joint_types, do_rot_offsets, 0, 0, 0, 0, 0,
        homogeneous_matrices, rot_offset_present_arr, rot_mat_tmp, static_rot_mat_tmp, fk_results,
        static_rot_offsets, static_rot_matrices, static_out_pts, static_out_frames)

    out_pts, out_frames = getEmptyFrames(arm)
    arm.out_pts = out_pts
    arm.out_frames = out_frames
    arm.getFrames = getFrames_closure(arm)
    arm.getFramesf = getFramesf_closure(arm)
    arm.getFramesh = getFramesh_closure(arm)

    # arm.out_frames = out_frames

    init_homogeneous_matrices!(arm)
    arm.fk_results = copy(arm.homogeneous_matrices)

    return arm
end

function rot3(axis, s, c)
    if axis == "Z" || axis == "z" || axis == "-z"
        return [c -s 0.; s c 0.; 0. 0. 1.]
    elseif axis == "Y" || axis == "y" || axis == "-y"
        return [c 0. s; 0. 1. 0.; -s 0. c]
    elseif axis == "X" || axis == "x" || axis == "-x"
        return [1. 0. 0.; 0. c -s; 0. s c]
    end
end

function rot3!(axis, s, c, out_mat)
    if axis == "Z" || axis == "z" || axis == "-z"
        # out_mat =  [c -s 0.; s c 0.; 0. 0. 1.]
        out_mat[1,1] = c
        out_mat[1,2] = -s
        out_mat[2,1] = s
        out_mat[2,2] = c
    elseif axis == "Y" || axis == "y" || axis == "-y"
        # out_mat =  [c 0. s; 0. 1. 0.; -s 0. c]
        out_mat[1,1] = c
        out_mat[1,3] = s
        out_mat[3,1] = -s
        out_mat[3,3] = c
    elseif axis == "X" || axis == "x" || axis == "-x"
        # out_mat =  [1. 0. 0.; 0. c -s; 0. s c]
        out_mat[2,2] = c
        out_mat[2,3] = -s
        out_mat[3,2] = s
        out_mat[3,3] = c
    end
end

function eulerTupleTo3x3(t)
    xm = rot3("X",sin(t[1]),cos(t[1]))
    ym = rot3("Y",sin(t[2]),cos(t[2]))
    zm = rot3("Z",sin(t[3]),cos(t[3]))

    zy = zm*ym
    # return RotMatrix{3}(zy*xm)
    return zy*xm
end

function getEmptyFrames(arm::Arm)
    return getFrames(arm, zeros(length(arm.axis_types)))
end

function getFrames(arm::Arm, state)
    pt = arm.disp_offset
    pts = []
    frames = []
    rot =  [[1. 0. 0.];[0. 1. 0.];[0. 0. 1.]]
    push!(pts, arm.disp_offset)
    push!(frames, [[1. 0. 0.];[0. 1. 0.];[0. 0. 1.]])

    axis_idx = 1
    for i in 1:length(arm.displacements)
        if arm.joint_types[i]=="revolute" || arm.joint_types[i]=="continuous"
            axis = arm.axis_types[axis_idx]
            if axis[1] == "-"
                s = sin(-state[axis_idx])
                c = cos(-state[axis_idx])
            else
                s = sin(state[axis_idx])
                c = cos(state[axis_idx])
            end
            axis_idx += 1
        end

        if arm.joint_types[i] == "prismatic"
            axis = arm.axes[axis_idx]
            if axis == 'x'
                arm.displacements[i][1] = arm.original_displacements[i][1] + state[axis_idx]
            elseif axis == 'y'
                arm.displacements[i][2] = arm.original_displacements[i][2] + state[axis_idx]
            elseif axis == 'z'
                arm.displacements[i][3] = arm.original_displacements[i][3] + state[axis_idx]
            end
            axis_idx += 1
        end


        if arm.joint_types[i] == "revolute" || arm.joint_types[i]=="continuous"
            rmat = rot3(axis,s,c)
            rot = rot * rmat
        end

        if arm.do_rot_offsets
            rot = rot * arm.rot_offsets[i]
        end

        pt = rot * arm.displacements[i] + pt
        push!(pts, pt)
        push!(frames, rot)
    end

    return pts, frames
end

function getFrames!(state, static_disp_offset, joint_types, axis_types, displacements, original_displacements, static_displacements, do_rot_offsets, rot_offsets, pts, frames)
    pt = static_disp_offset
    # rot = one(RotMatrix{3, Float64})
    rot =  rot_offsets[1]
    #push!(pts, pt)
    pts[1] = pt
    # push!(frames, rot)
    frames[1] = rot

    axis_idx = 1

    for i in 1:length(displacements)
        if joint_types[i]=="revolute" || joint_types[i]=="continuous"
            axis = axis_types[axis_idx]
            if axis[1] == "-"
                if axis == "Z" || axis == "z" || axis == "-z"
                    r = RotZ(-state[axis_idx])
                elseif axis == "Y" || axis == "y" || axis == "-y"
                    r = RotY(-state[axis_idx])
                elseif axis == "X" || axis == "x" || axis == "-x"
                    r = RotX(-state[axis_idx])
                end
            else
                if axis == "Z" || axis == "z" || axis == "-z"
                    r = RotZ(state[axis_idx])
                elseif axis == "Y" || axis == "y" || axis == "-y"
                    r = RotY(state[axis_idx])
                elseif axis == "X" || axis == "x" || axis == "-x"
                    r = RotX(state[axis_idx])
                end
            end
            axis_idx += 1
        end

        if joint_types[i] == "prismatic"
            axis = axis_types[axis_idx]
            if axis == 'x'
                displacements[i][1] = original_displacements[i][1] + state[axis_idx]
            elseif axis == 'y'
                displacements[i][2] = original_displacements[i][2] + state[axis_idx]
            elseif axis == 'z'
                displacements[i][3] = original_displacements[i][3] + state[axis_idx]
            end
            axis_idx += 1
        end

        if joint_types[i] == "revolute" || joint_types[i]=="continuous"
            rot = rot * r
        end

        pt = rot * static_displacements[i] + pt

        if do_rot_offsets
            rot = rot * rot_offsets[i+1]
        end

        #push!(pts, pt)
        pts[i+1] = pt
        # push!(frames, rot)
        frames[i+1] = rot
    end
end

function getFrames_closure(arm)
    f = x->getFrames!(x, arm.static_disp_offset, arm.joint_types, arm.axis_types,
    arm.displacements, arm.original_displacements, arm.static_displacements, arm.do_rot_offsets, arm.rot_offsets, arm.out_pts, arm.out_frames)
    return f
end

function getFramesf_closure(arm)
    # static_disp_offset = deepcopy(arm.static_disp_offset)
    #joint_types = deepcopy(arm.joint_types)
    #axis_types = deepcopy(arm.axis_types)
    #displacements = deepcopy(arm.displacements)
    #original_displacements = deepcopy(arm.original_displacements)
    #static_displacements = deepcopy(arm.static_displacements)
    #rot_offset_present_arr = deepcopy(arm.rot_offset_present_arr)
    #static_rot_offsets = deepcopy(arm.static_rot_offsets)
    #static_rot_matrices = deepcopy(arm.static_rot_matrices)
    #rot_mat_tmp = deepcopy(arm.rot_mat_tmp)
    f = x->getFrames_f!(x, arm.static_disp_offset, arm.joint_types, arm.axis_types, arm.displacements,
        arm.original_displacements, arm.static_displacements, arm.rot_offset_present_arr, arm.static_rot_offsets,
        arm.static_rot_matrices, arm.rot_mat_tmp, arm.static_rot_mat_tmp, arm.static_out_pts, arm.static_out_frames)
    return f
end

function getFramesh_closure(arm)
    f = x->getFrames_h!(arm, x)
    return f
end

function getFrames_f!(state, static_disp_offset, joint_types, axis_types, displacements,
    original_displacements, static_displacements, rot_offset_present_arr, static_rot_offsets,
    static_rot_matrices, rot_mat_tmp, static_rot_mat_tmp, out_pts, out_frames)

    update_out_frames!(state, static_disp_offset, joint_types, axis_types, displacements,
        original_displacements, static_displacements, rot_offset_present_arr, static_rot_offsets,
        static_rot_matrices, rot_mat_tmp, static_rot_mat_tmp, out_pts, out_frames)
end

function update_out_frames!(state, static_disp_offset, joint_types, axis_types, displacements,
    original_displacements, static_displacements, rot_offset_present_arr, static_rot_offsets,
    static_rot_matrices, rot_mat_tmp, static_rot_mat_tmp, out_pts, out_frames)

    count = 1
    for i = 1:length(joint_types)
        if joint_types[i] == "revolute" || joint_types[i] == "continuous" || joint_types[i] == "prismatic"
            axis = axis_types[count]
            if axis[1] == "-"
                a = axis_types[count]
                sc = state[count]
                s = sin(-sc)
                c = cos(-sc)
                rtm = rot_mat_tmp[i]
                rot3!(a, s, c, rtm)
                # rot3!(axis_types[count], sin(-state[count]), cos(-state[count]), rot_mat_tmp[i])
            else
                a = axis_types[count]
                sc = state[count]
                s = sin(sc)
                c = cos(sc)
                rtm = rot_mat_tmp[i]
                rot3!(a, s, c, rtm)
            end
            # println(rot_mat_tmp)
            if rot_offset_present_arr[i]
                static_rot_matrices[i] = static_rot_offsets[i] * SMatrix{3,3}(rtm)
            else
                static_rot_matrices[i] = static_rot_offsets[i] * SMatrix{3,3}(rtm)
            end
            count += 1
        end
    end
    out_frames[1] = static_rot_matrices[1]
    out_pts[1] = static_disp_offset
    for i = 2:length(static_rot_matrices)
        out_pts[i] = (out_frames[i-1] * static_displacements[i-1]) + out_pts[i-1]
        out_frames[i] = out_frames[i-1]*static_rot_matrices[i]
    end
end

function getFrames_h!(arm::Arm, state)
    update_homogenous_matrices!(arm, state)
    arm.fk_results[1] = arm.homogeneous_matrices[1]
    for i = 2:length(arm.homogeneous_matrices)
        arm.fk_results[i] = arm.fk_results[i-1] * arm.homogeneous_matrices[i]
    end

    for i = 1:length(arm.fk_results)
        arm.out_pts[i] = arm.fk_results[i][1:3,4]
        arm.out_frames[i] = arm.fk_results[i][1:3,1:3]
    end
end

function update_homogenous_matrices!(arm, state)
    count = 1
    for i = 1:length(arm.homogeneous_matrices)
        if i >= length(arm.joint_types)
            break
        else
            if arm.joint_types[i] == "revolute" || arm.joint_types[i] == "continuous" || arm.joint_types[i] == "prismatic"
                update_homogenous_matrix!(arm, i, count, state[count])
                count += 1
            end
        end
    end
end

function update_homogenous_matrix!(arm, idx, state_idx, xi)
    if arm.joint_types[idx] == "revolute" || arm.joint_types[idx] == "continuous"
        axis = arm.axis_types[state_idx]
        if axis[1] == "-"
            if axis == "Z" || axis == "z" || axis == "-z"
                arm.rot_mat_tmp = RotZ(-xi)
            elseif axis == "Y" || axis == "y" || axis == "-y"
                arm.rot_mat_tmp = RotY(-xi)
            elseif axis == "X" || axis == "x" || axis == "-x"
                arm.rot_mat_tmp = RotX(-xi)
            end
        else
            if axis == "Z" || axis == "z" || axis == "-z"
                arm.rot_mat_tmp = RotZ(xi)
            elseif axis == "Y" || axis == "y" || axis == "-y"
                arm.rot_mat_tmp = RotY(xi)
            elseif axis == "X" || axis == "x" || axis == "-x"
                arm.rot_mat_tmp = RotX(xi)
            end
        end

        if arm.rot_offset_present_arr[idx]
            arm.homogeneous_matrices[idx][1:3,1:3] = arm.rot_offsets[idx] * arm.rot_mat_tmp
        else
            arm.homogeneous_matrices[idx][1:3,1:3] = arm.rot_mat_tmp
        end

    elseif arm.joint_types[idx] == "prismatic"
    end
end

function init_homogeneous_matrices!(arm)
    for i = 1:length(arm.homogeneous_matrices)
        init_homogeneous_matrix!(arm, i)
    end
end

function init_homogeneous_matrix!(arm, idx)
    arm.homogeneous_matrices[idx][1:3,1:3] = arm.rot_offsets[idx]
    if idx == 1
        arm.homogeneous_matrices[idx][1:3,4] = arm.disp_offset
    else
        arm.homogeneous_matrices[idx][1:3,4] = arm.displacements[idx-1]
    end
end

#=
axis_types = ["-z","y","y","y","z","y"]
displacements = [[0.0, 0.13585, 0.0], [0.0, -0.1197, 0.425], [0.0, 0.0, 0.39225], [0, 0.093, 0], [0, 0, 0.09465], [0.0,0.0823,0.0]]
disp_offset = [0., 0., 0.089159]
rot_offsets = [[0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0]]
# rot_offsets = [eulerTupleTo3x3(t) for t in rot_offsets]
joint_types = ["revolute","revolute","revolute","revolute","revolute","revolute"]
# ur5 = Arm(axis_types,displacements,displacements,disp_offset,rot_offsets, joint_types,0,0,false)
ur5 = Arm(axis_types, displacements, disp_offset, rot_offsets,joint_types, false)
println(eulerTupleTo3x3([0.,0.,0.]))
return 0





func = getFrames_closure(ur5)

using BenchmarkTools

# @btime func([1.,1.,1.,1.,1.,1.])

using ForwardDiff, BenchmarkTools, Calculus

function test_func(x)
    ur5.getFrames(x)
    ee_pt = ur5.out_pts[end]
    return norm(ee_pt)
end
=#
