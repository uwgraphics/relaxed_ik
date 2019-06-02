include("gradient.jl")
include("constraint.jl")
include("objective.jl")

mutable struct Vars
    init_state
    objectives
    objective_closures
    grad_types
    ∇s
    weight_priors
    inequality_constraints
    ineq_constraint_closures
    ineq_grad_types
    equality_constraints
    eq_constraint_closures
    eq_grad_types
    bounds
    xopt
    prev_state
    prev_state2
    prev_state3
    dummy_val
end


function Vars(init_state, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types, bounds)
    xopt = copy(init_state) + 0.0000000001*ones(length(init_state))
    prev_state = copy(init_state) + 0.0000000001*ones(length(init_state))
    prev_state2 = copy(init_state) + 0.0000000001*ones(length(init_state))
    prev_state3 = copy(init_state) + 0.0000000001*ones(length(init_state))

    length_checek = length(objectives) == length(grad_types) == length(weight_priors)
    if length_checek == false
        throw(ArgumentError("ERROR: length of objectives, grad_types, and weight_priors must be equal in Vars"))
    end

    length_checek = length(inequality_constraints) == length(ineq_grad_types)
    if length_checek == false
        throw(ArgumentError("ERROR: length of inequality_constraints and ineq_grad_types must be equal in Vars"))
    end

    length_checek = length(equality_constraints) == length(eq_grad_types)
    if length_checek == false
        throw(ArgumentError("ERROR: length of equality_constraints and eq_grad_types must be equal in Vars"))
    end

    v = Vars(init_state, objectives, [], grad_types, [], weight_priors, inequality_constraints, [], ineq_grad_types, equality_constraints, [], eq_grad_types, bounds, xopt, prev_state, prev_state2, prev_state3, 0.0)

    populate_vars!(v, v)

    return v
end

function populate_vars!(vars, target_vars)
    populate_objective_closures!(vars, target_vars)
    populate_∇s!(vars)
    populate_constraint_closures!(vars, target_vars)
end

function populate_objective_closures!(vars, target_vars)
    vars.objective_closures = []
    objectives = vars.objectives
    num_objectives = length(objectives)
    for i=1:num_objectives
        push!(vars.objective_closures, get_obj_closure(objectives[i], target_vars))
    end
end

function populate_∇s!(vars)
    vars.∇s = []
    for i=1:length(vars.objective_closures)
        push!(vars.∇s, get_∇(vars.objective_closures[i], vars.grad_types[i]))
    end
end

function populate_constraint_closures!(vars, target_vars)
    vars.ineq_constraint_closures = []
    vars.eq_constraint_closures = []

    for i=1:length(vars.inequality_constraints)
        push!(vars.ineq_constraint_closures, get_constraint_closure(vars.inequality_constraints[i], vars.ineq_grad_types[i], target_vars))
    end

    for i=1:length(vars.equality_constraints)
        push!(vars.eq_constraint_closures, get_constraint_closure(vars.equality_constraints[i], vars.eq_grad_types[i], target_vars))
    end
end

function update!(vars, xopt)
    vars.prev_state3 = vars.prev_state2
    vars.prev_state2 = vars.prev_state
    vars.prev_state = vars.xopt
    vars.xopt = xopt
end
