#!/usr/bin/env julia

using YAML
using BenchmarkTools

path_to_src = Base.source_dir()

include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_vars.jl")
include("RelaxedIK/GROOVE_Julia/groove.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_objective.jl")


relaxedIK = get_standard(path_to_src, "hubo_info.yaml")

vars = relaxedIK.relaxedIK_vars

println(path_to_src)

# x = [0.475115, 0.410492, 0.630609, 0.491862, 0.291932, 0.996426, 0.116537, 0.545429, 0.211642, 0.575973, 0.0784703, 0.468633, 0.61519, 0.0431053, 0.625812]
# x = [0.,0.,1.,1.,0.,0.]
# @btime groove_solve(relaxedIK.groove)
# println(groove_solve(relaxedIK.groove))

# println(position_obj(x, vars))
# c = x->position_obj(x, vars)
# ∇ = x->ForwardDiff.gradient(c, x)

# @btime ∇(x)
# println(∇(x))

# @btime update_relaxedIK_vars!(vars, rand(6))


goal_positions = [SVector(0.0,0.0,0.00001), SVector(0.1,0.0,0.1)]
goal_quats = [rand(Quat), rand(Quat)]

# println(rand(Quat))
#@btime solve(relaxedIK, goal_positions, goal_quats)
# println(solve(relaxedIK, goal_positions, goal_quats))
# println(solve_local(relaxedIK, goal_positions, goal_quats))
# vars = relaxedIK.relaxedIK_vars

#=
function test(relaxedIK)
    vars = relaxedIK.relaxedIK_vars
    goal_positions = vars.init_ee_positions
    goal_disps = [ SVector(0.1,0.0,0.1),  SVector(0.0,0.0,0.0001)]
    goal_quats = [Quat(1.,0.,0.,0.), Quat(1.,0.,0.,0.)]

    println(typeof(goal_positions))
    println(typeof(vars.init_ee_positions))

    println(typeof(goal_quats))
    println(typeof(vars.init_ee_quats))

    for i=1:100
        # goal_positions[1] += SVector(0.0,0.0,0.001)
        for i = 1:vars.robot.num_chains
            goal_positions[i] += goal_disps[i]
        end
        vars.goal_positions = goal_positions

        xopt = groove_solve(relaxedIK.groove,  prev_state=[])
        update_relaxedIK_vars!(relaxedIK.relaxedIK_vars, xopt)

        println(xopt)
    end

end


test(relaxedIK)
=#




# println(groove_solve(relaxedIK.groove, prev_state=[]))
