#!/usr/bin/env julia

using YAML


path_to_src = Base.source_dir()

f = open(path_to_src * "/RelaxedIK/Config/info_files/ur5_info.yaml")

y = YAML.load(f)

# println(y)

# include("RelaxedIK/GROOVE_Julia/objective.jl")
# include("RelaxedIK/Spacetime_Julia/arm.jl")
# include("RelaxedIK/Spacetime_Julia/robot.jl")

include("RelaxedIK/GROOVE_Julia/groove.jl")
include("RelaxedIK/GROOVE_Julia/vars.jl")
using BenchmarkTools

function o1(x, vars)
    return x[1]^vars.weight_priors[1]
end

function o2(x, vars)
    return x[1]^vars.weight_priors[2]
end

function ineq_con1(x, vars)
    return x[1]
end

function eq_con1(x, vars)
    return x[1] + 2
end

objectives = [o1, o2]
grad_types = ["finite_diff", "forward_ad"]
weight_priors = [2.0, 1.0]
inequality_constraints = []
ineq_grad_types = []
equality_constraints = []
eq_grad_types = []
bounds = []
vars = Vars([-1.0], objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types, bounds)

g = get_groove(vars, "cobyla")

@btime solve(g)
println(solve(g))
