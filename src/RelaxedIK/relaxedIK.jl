
include("GROOVE_RelaxedIK_Julia/relaxedIK_objective.jl")
include("GROOVE_RelaxedIK_Julia/relaxedIK_vars.jl")
include("GROOVE_Julia/groove.jl")
include("GROOVE_RelaxedIK_Julia/relaxedIK_objective.jl")
include("GROOVE_Autocam_Julia/autocam_objective.jl")
include("GROOVE_Autocam_Julia/autocam_vars.jl")
include("Utils_Julia/transformations.jl")
include("Utils_Julia/ema_filter.jl")

mutable struct RelaxedIK
    relaxedIK_vars
    groove
    ema_filter
end

function RelaxedIK(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types; position_mode = "relative", rotation_mode = "relative", solver_name="slsqp", preconfigured=false)
    relaxedIK_vars = RelaxedIK_vars(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types, position_mode = position_mode, rotation_mode = rotation_mode, preconfigured=preconfigured)
    groove = get_groove(relaxedIK_vars.vars, solver_name)
    ema_filter = EMA_filter(relaxedIK_vars.vars.init_state)
    return RelaxedIK(relaxedIK_vars, groove, ema_filter)
end


function get_standard(path_to_src, info_file_name; solver_name = "slsqp", preconfigured=false)
    objectives = [position_obj_1, rotation_obj_1, min_jt_vel_obj, min_jt_accel_obj, min_jt_jerk_obj, collision_nn_obj]
    grad_types = ["forward_ad", "forward_ad", "forward_ad", "forward_ad", "forward_ad", "forward_ad"]
    weight_priors = [50.0, 40.0, 5.0 ,1.0, 1.0, 3.0]
    inequality_constraints = []
    ineq_grad_types = []
    equality_constraints = []
    eq_grad_types = []
    return RelaxedIK(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types, solver_name = solver_name, preconfigured=preconfigured)
end

function get_bimanual(path_to_src, info_file_name; solver_name = "slsqp", preconfigured=false)
    objectives = [position_obj_1, rotation_obj_1, position_obj_2, rotation_obj_2, min_jt_vel_obj, min_jt_accel_obj, min_jt_jerk_obj, bimanual_line_seg_collision_avoid_obj]
    grad_types = ["forward_ad", "forward_ad", "forward_ad", "forward_ad", "forward_ad", "forward_ad", "forward_ad", "forward_ad"]
    weight_priors = [50.0, 49.0, 50.0, 49.0, 10.0 ,9.0, 8.0, 2.0]
    inequality_constraints = []
    ineq_grad_types = []
    equality_constraints = []
    eq_grad_types = []
    return RelaxedIK(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types, solver_name = solver_name, preconfigured=preconfigured)
end

function get_autocam1(path_to_src, info_file_name; solver_name = "slsqp", preconfigured=false)
    objectives = [position_obj_1, rotation_obj_1, min_jt_vel_obj, min_jt_accel_obj, min_jt_jerk_obj, lookat_obj_1, camera_upright_obj_1, camera_occlusion_avoid_obj_1, avoid_environment_occlusions_obj_1, camera_dis_obj_1, collision_nn_obj]
    grad_types = ["forward_ad", "forward_ad", "forward_ad", "forward_ad", "forward_ad", "forward_ad", "forward_ad", "forward_ad", "forward_ad", "forward_ad", "forward_ad"]
    weight_priors = [15.0, 14.5, 4.0, 4.0, 0.01, 10.0, 10.0, 0.9, 8.0, 0.1, 2.0]
    inequality_constraints = []
    ineq_grad_types = []
    equality_constraints = []
    eq_grad_types = []
    rik = RelaxedIK(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types, solver_name = solver_name, preconfigured=preconfigured)
    rik.relaxedIK_vars.additional_vars = Autocam_vars()
    return rik
end


function get_autocam_visual_exploration_mode1(path_to_src, info_file_name; solver_name = "slsqp", preconfigured=false)
    objectives = [position_obj_1, rotation_obj_1, min_jt_vel_obj, min_jt_accel_obj, min_jt_jerk_obj, look_at_visual_target_obj1, camera_upright_obj_1, collision_nn_obj]
    grad_types = ["forward_ad", "forward_ad", "forward_ad", "forward_ad", "forward_ad", "forward_ad", "forward_ad", "forward_ad"]
    weight_priors = [15.0, 14.5, 2.0, 4.0, 0.01, 15.0, 10.0, 1.0]
    inequality_constraints = []
    ineq_grad_types = []
    equality_constraints = []
    eq_grad_types = []
    rik = RelaxedIK(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types, solver_name = solver_name, preconfigured=preconfigured)
    rik.relaxedIK_vars.additional_vars = Autocam_vars()
    return rik
end

function get_finite_diff_version(path_to_src, info_file_name; solver_name = "slsqp", preconfigured=false)
    objectives = [position_obj_1, rotation_obj_1, min_jt_vel_obj, min_jt_accel_obj, min_jt_jerk_obj, collision_nn_obj]
    grad_types = ["finite_diff", "finite_diff", "finite_diff", "finite_diff", "finite_diff", "finite_diff"]
    weight_priors = [50., 40.0, 1.0 ,1.0, 1.0, 0.4]
    inequality_constraints = []
    ineq_grad_types = []
    equality_constraints = []
    eq_grad_types = []
    return RelaxedIK(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types, solver_name = solver_name, preconfigured=preconfigured)
end


function solve(relaxedIK, goal_positions, goal_quats; prev_state = [])
    vars = relaxedIK.relaxedIK_vars

    if vars.position_mode == "relative"
        vars.goal_positions = copy(vars.init_ee_positions)
        for i = 1:vars.robot.num_chains
            vars.goal_positions[i] += goal_positions[i]
            # push!(vars.goal_positions, vars.init_ee_positions[i] + goal_positions[i])
        end
        # println(vars.goal_positions)
        # vars.goal_positions = vars.goal_positions_relative
    else
        vars.goal_positions = goal_positions
    end


    if vars.rotation_mode == "relative"
        for i = 1:vars.robot.num_chains
            vars.goal_quats[i] = goal_quats[i] * copy(vars.init_ee_quats[i])
        end
    else
        vars.goal_quats = goal_quats
    end

    xopt = groove_solve(relaxedIK.groove, prev_state=prev_state)
    xopt = filter_signal(relaxedIK.ema_filter, xopt)
    update_relaxedIK_vars!(relaxedIK.relaxedIK_vars, xopt)

    return xopt
end
