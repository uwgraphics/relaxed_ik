
import LinearAlgebra, Rotations, StaticArrays

mutable struct Arm
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
end

function Arm(axis_types, displacements, disp_offset, rot_offsets, joint_types,do_rot_offsets)
    rot_offset_matrices = [eulerTupleTo3x3(t) for t in rot_offsets]
    static_displacements = []
    for i in 1:length(displacements)
        push!(static_displacements, SVector(  displacements[i][1], displacements[i][2],displacements[i][3])   )
    end
    original_displacements = copy(displacements)
    static_disp_offset = SVector( disp_offset[1], disp_offset[2], disp_offset[3] )
    id_mat = one(RotMatrix{3, Float64})

    arm = Arm(axis_types, displacements, original_displacements, static_displacements, disp_offset, static_disp_offset, rot_offset_matrices, joint_types, do_rot_offsets, 0, 0, 0)

    out_pts, out_frames = getEmptyFrames(arm)
    arm.out_pts = out_pts
    arm.out_frames = out_frames
    arm.getFrames = getFrames_closure(arm)

    return arm
end

function rot3(axis, s, c)
    if axis == "Z" || axis == "z" || axis == "-z"
        return [[c -s 0.];[s c 0.]; [0. 0. 1.]]
    elseif axis == "Y" || axis == "y" || axis == "-y"
        return [[c 0. s]; [0. 1. 0.]; [-s 0. c]]
    elseif axis == "X" || axis == "x" || axis == "-x"
        return [[1. 0. 0.]; [0. c -s]; [0. s c]]
    end
end

function eulerTupleTo3x3(t)
    xm = rot3("X",sin(t[1]),cos(t[1]))
    ym = rot3("Y",sin(t[2]),cos(t[2]))
    zm = rot3("Z",sin(t[3]),cos(t[3]))

    zy = zm*ym
    return RotMatrix{3}(zy*xm)
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
            axis = self.axes[axis_idx]
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
