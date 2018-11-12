include("objective.jl")

mutable struct Vars
    init_state
    objectives
    objective_closures
    grad_types
    ∇s
    weight_priors
    inequality_constraints
    equality_constraints
    bounds
    xopt
    prev_state
    prev_state2
    prev_state3
end

function Vars(init_state, objectives, grad_types, weight_priors, inequality_constraints, equality_constraints, bounds)
    xopt = copy(init_state)
    prev_state = copy(init_state)
    prev_state2 = copy(init_state)
    prev_state3 = copy(init_state)

    length_checek = length(objectives) == length(grad_types) == length(weight_priors)
    if length_checek == false
        throw(ArgumentError("ERROR: length of objectives, grad_types, and weight_priors must be equal in Vars"))
    end

    return Vars(init_state, objectives, [], grad_types, [], weight_priors, inequality_constraints, equality_constraints, bounds, xopt, prev_state, prev_state2, prev_state3)
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

function update!(xopt, vars)
    vars.prev_state3 = vars.prev_state2
    vars.prev_state2 = vars.prev_state
    vars.prev_state = vars.xopt
    vars.xopt = xopt
end
