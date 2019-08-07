using NLopt
include("objective.jl")

mutable struct Groove
    vars
    opt
end

function get_groove(vars, solver_name; max_iter=12, max_time = 0.05)
    #=
    solver name options:
    "slsqp", "mma", "ccsaq", "bobyqa", "cobyla"
    =#
    if length(vars.equality_constraints) > 0 && (solver_name == "mma" || solver_name == "ccsaq")
        throw(ArgumentError("equality constraints not compatible with solver choice.  Consider slsqp or cobyla."))
    end

    if (length(vars.inequality_constraints) > 0 || length(vars.equality_constraints) > 0) && (solver_name == "bobyqa")
        throw(ArgumentError("constraints not compatible with solver choice.  Consider slsqp or cobyla."))
    end

    if (length(vars.bounds) < 0) && (solver_name == "bobyqa")
        throw(ArgumentError("bobyqa solver requires bounds.  Exiting."))
    end

    num_dof = length(vars.init_state)

    if solver_name == "slsqp"
        opt = Opt(:LD_SLSQP, num_dof)
    elseif solver_name == "mma"
        opt = Opt(:LD_MMA, num_dof)
    elseif solver_name == "ccsaq"
        opt = Opt(:LD_CCSAQ, num_dof)
    elseif solver_name == "bobyqa"
        opt = Opt(:LN_BOBYQA, num_dof)
    elseif solver_name == "cobyla"
        opt = Opt(:LN_COBYLA, num_dof)
    else
        throw(ArgumentError("Not a valid solver name in groove.  Exiting."))
    end

    objective_function_closure = (x,g) -> obj_master(x,g,vars)

    min_objective!(opt, objective_function_closure)

    for i = 1:length(vars.ineq_constraint_closures)
        inequality_constraint!(opt, vars.ineq_constraint_closures[i], 1e-8)
    end

    for i = 1:length(vars.eq_constraint_closures)
        equality_constraint!(opt, vars.eq_constraint_closures[i], 1e-8)
    end

    lower_bounds = []
    upper_bounds = []
    for i = 1:length(vars.bounds)
        push!(lower_bounds, vars.bounds[i][1])
        push!(upper_bounds, vars.bounds[i][2])
    end

    lower_bounds = Array{Float64}(lower_bounds)
    upper_bounds = Array{Float64}(upper_bounds)

    if length(lower_bounds) > 0
        lower_bounds!(opt, lower_bounds)
    end

    if length(upper_bounds) > 0
        upper_bounds!(opt, upper_bounds)
    end

    xtol_abs!(opt, 0.0001)
    xtol_rel!(opt, 0.0001)
    maxeval!(opt, max_iter)
    if max_time > 0.0
        maxtime!(opt, max_time)
    end

    return Groove(vars, opt)
end

function groove_solve(groove; prev_state = nothing, ftol_abs=0.0, max_time=0.0, max_iter = 0)
    if prev_state == nothing
        initSol = groove.vars.xopt
    else
        initSol = prev_state
    end

    #=
    if max_time > 0.0
        maxtime!(groove.opt, max_time)
    end

    if ftol_abs > 0.0
        ftol_abs!(groove.opt, ftol_abs)
    end

    if max_iter > 0
        maxeval!(groove.opt, max_iter)
    end
    =#
    if max_iter > 0
        maxeval!(groove.opt, max_iter)
    end


    #xtol_rel!(groove.opt, 0.0001)
    # println(xtol_rel(groove.opt))

    (minf, minx, ret) = optimize(groove.opt, initSol)
    # println(groove.opt.numevals)
    # println(ret)
    # return minx, minf, groove.opt.numevals
    # println(groove.opt.numevals)
    return minx
end
