using NLopt
using LinearAlgebra
include("../GROOVE_Julia/vars.jl")
include("../GROOVE_Julia/groove.jl")
# include("../GROOVE_Julia/gradient.jl")


mutable struct Autoparam_vars
    vars
    goal_val
    bad_vals
    horrible_vals
end

function Autoparam_vars(init_state, goal_val, bad_vals, horrible_vals, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types, bounds)
    vars = Vars(init_state, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types, bounds)
    av = Autoparam_vars(vars, goal_val, bad_vals, horrible_vals)
    populate_vars!(vars, av)
    return av
end

function groove_loss(x_val, t, c, f)
    return (-2.718281828459^((-(x_val - t)^2.0) / (2.0 * c^2.0)) ) + f * (x_val - t)^2.0
end

function optimize_params(goal_val, bad_vals, horrible_vals)
    av = Autoparam_vars([1.0, 1.0], goal_val, bad_vals, horrible_vals, [bad_vals_obj, horrible_vals_obj], ["forward_ad", "forward_ad"], [1000.0, 1.0], [], [], [], [], [])
    groove = get_groove(av.vars, "mma", max_iter=1000)
    println(groove_solve(groove))
end

function pull_to_goal_obj(x, vars)
    val = groove_loss(vars.goal_val, x[1], x[2], x[3])
    return abs(val + 1)
end

function bad_vals_obj(x, vars)
    val = groove_loss(vars.bad_vals[1], vars.goal_val, x[1], x[2])
    return val^2.0
end

function horrible_vals_obj(x, vars)
    val = groove_loss(vars.horrible_vals[1], vars.goal_val, x[1], x[2])
    return (val-10.0)^2.0
end

# breaking_point is the split point where the gaussian stops and the quadratic takes over
function get_c_value(breaking_point, t)
    return sqrt( -(breaking_point-t)^2 / (2.0*log(0.01)) )
end

function get_f_value(breaking_point, t)
    return 1.0 / ( (breaking_point-t))^2
end

# println(get_c_value(5.0, 0.0))
# println(get_f_value(5.0, 0.0))
# init_state, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types, bounds
# vars = Vars([0.0,0.0], [], [], [], [], [], [], [], [])

#val = 1.1
#t = 0.0
#println(get_c_value(val, t))
#println(get_f_value(val, t))
# optimize_params(0.0, [val], [20*val])

# (-2.718281828459^((-(x_val - t)^2.0) / (2.0 * c^2.0)) )
# f * (x_val - t)^2.0
