include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_vars.jl")
include("RelaxedIK/GROOVE_Julia/groove.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_objective.jl")
include("RelaxedIK/Spacetime_Julia/arm.jl")
using Rotations
using StaticArrays
using LinearAlgebra


function snoop1()
    eulerTupleTo3x3(rand(3))
end

function snoop2()
    axis_types = ["-z","y","y","y","z","y"]
    displacements = [[0.0, 0.13585, 0.0], [0.0, -0.1197, 0.425], [0.0, 0.0, 0.39225], [0, 0.093, 0], [0, 0, 0.09465], [0.0,0.0823,0.0]]
    disp_offset = [0., 0., 0.089159]
    rot_offsets = [[0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0]]
    joint_types = ["revolute","revolute","revolute","revolute","revolute","revolute"]
    ur5 = Arm(axis_types, displacements, disp_offset, rot_offsets,joint_types, false)
end






snoop1()
snoop2()
