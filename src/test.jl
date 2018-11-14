#!/usr/bin/env julia

using YAML
using BenchmarkTools

path_to_src = Base.source_dir()

# f = open(path_to_src * "/RelaxedIK/Config/info_files/ur5_info.yaml")

# y = YAML.load(f)

# println(y)

# include("RelaxedIK/GROOVE_Julia/objective.jl")
# include("RelaxedIK/Spacetime_Julia/arm.jl")
# include("RelaxedIK/Spacetime_Julia/robot.jl")

include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_vars.jl")
include("RelaxedIK/GROOVE_Julia/groove.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_objective.jl")

relaxedIK = get_standard(path_to_src, "ur5_info.yaml")
# println(relaxedIK.relaxedIK_vars.vars.init_state)

# @btime relaxedIK.relaxedIK_vars.vars.∇s[2](rand(6))
# show(relaxedIK.relaxedIK_vars.vars.∇s[4](rand(15)))
# @btime relaxedIK.relaxedIK_vars.vars.objective_closures[3](rand(6))
# show(position_obj([3.1277, -0.0398739, -2.0773, -1.03981, -1.58653, -1.57102], relaxedIK.relaxedIK_vars))
# @btime solve(relaxedIK.groove)
# @btime solve(relaxedIK.groove)

# show(groove_solve(relaxedIK.groove))

# println(relaxedIK.groove)
# show(solve(relaxedIK, [[0.,0.,-0.2]], [Quat(1.,0.,0.,0.)]))

@btime groove_solve(relaxedIK.groove)

#=
for i=1:1000
    println(solve(relaxedIK.groove))
    #xopt = solve(relaxedIK, [[0.,0.,0.0]], [Quat(1.,0.,0.,0.)])
    # update_relaxedIK_vars!(relaxedIK.relaxedIK_vars, xopt)
end
=#
