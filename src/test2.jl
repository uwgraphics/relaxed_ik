
include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_vars.jl")
include("RelaxedIK/GROOVE_Julia/groove.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_objective.jl")
include("RelaxedIK/Spacetime_Julia/arm.jl")
using Rotations
using StaticArrays
using LinearAlgebra


function body()
    a = [1.,1.,1.,1.,1.,1.,1.,1.,1.]
    b = RotMatrix{3}(a)
    println(b)
    path_to_src = Base.source_dir()
    # path_to_src = path_to_src * "/.."
    relaxedIK = get_standard(path_to_src, "ur5_info.yaml")
    for i = 1:1000
        println(solve(relaxedIK, [[1.,0.,0.]], [Quat(1.,0.,0.,0.)]))
    end
end


Base.@ccallable function julia_main(ARGS::Vector{String})::Cint
    body()
    return 0
end


body()
