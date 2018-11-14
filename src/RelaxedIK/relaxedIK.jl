include("GROOVE_RelaxedIK_Julia/relaxedIK_objective.jl")
include("GROOVE_RelaxedIK_Julia/relaxedIK_vars.jl")
include("GROOVE_Julia/groove.jl")
include("GROOVE_RelaxedIK_Julia/relaxedIK_objective.jl")

mutable struct RelaxedIK
    relaxedIK_vars
    groove
end

function RelaxedIK(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types; position_mode = "relative", rotation_mode = "relative", solver_name="slsqp")
    relaxedIK_vars = RelaxedIK_vars(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types, position_mode = position_mode, rotation_mode = rotation_mode)
    groove = get_groove(relaxedIK_vars.vars, solver_name)
    return RelaxedIK(relaxedIK_vars, groove)
end


function get_standard(path_to_src, info_file_name; solver_name = "slsqp")
    objectives = [position_obj, rotation_obj, min_jt_vel_obj, min_jt_accel_obj]
    grad_types = ["forward_ad", "forward_ad", "forward_ad", "forward_ad"]
    weight_priors = [50., 49., 1.,1.]
    inequality_constraints = []
    ineq_grad_types = []
    equality_constraints = []
    eq_grad_types = []
    return RelaxedIK(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types)
end

function solve(relaxedIK, goal_positions, goal_quats; prev_state = [])
end
