#!/usr/bin/env julia

using YAML
using BenchmarkTools

path_to_src = Base.source_dir()

include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_vars.jl")
include("RelaxedIK/GROOVE_Julia/groove.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_objective.jl")


relaxedIK = get_standard(path_to_src, "ur5_info.yaml")

vars = relaxedIK.relaxedIK_vars

# x = [0.475115, 0.410492, 0.630609, 0.491862, 0.291932, 0.996426, 0.116537, 0.545429, 0.211642, 0.575973, 0.0784703, 0.468633, 0.61519, 0.0431053, 0.625812]
x = [1.,1.,1.,1.,0.,0.]

# println(position_obj(x, vars))
c = x->rotation_obj(x, vars)
∇ = x->ForwardDiff.gradient(c, x)

@btime ∇(x)
# println(∇(x))

# idx = [1,2,3,5]
# subchain = vars.robot.subchain_indices

# println(x[subchain[2]])
