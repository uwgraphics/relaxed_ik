
include("RelaxedIK/Utils_Julia/ik_task.jl")
include("RelaxedIK/Utils_Julia/solver_output.jl")
include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_vars.jl")
include("RelaxedIK/GROOVE_Julia/groove.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_objective.jl")

################################################################################
robot_name = "ur5"
info_name = "ur5_info.yaml"
solver_name = "relaxed_ik"
task_name = "huboCircle"
################################################################################

path_to_src = Base.source_dir()

pos_goals, quat_goals = get_ik_task(path_to_src, task_name, scaling_factor=2.0)
solver_output = Solver_Output(path_to_src, solver_name, robot_name, task_name)

relaxedIK = get_standard(path_to_src, "ur5_info.yaml")

function run_trial(relaxedIK, pos_goals, quat_goals, solver_output)
    vars = relaxedIK.relaxedIK_vars

    for i=1:length(pos_goals)
        vars.goal_positions = copy(vars.init_ee_positions)
        vars.goal_positions[1] += pos_goals[i]
        vars.goal_quats[1] = quat_goals[i]

        # vars.goal_quats = copy(vars.init_ee_quats)
        vars.goal_quats[1] = Quat(quat_goals[i] * copy(vars.init_ee_quats[1]))
        # vars.goal_quats[1] = Quat(0.496594, 0.495679, 0.504295, -0.503371)

        xopt = groove_solve(relaxedIK.groove,  prev_state=[])
        update_relaxedIK_vars!(relaxedIK.relaxedIK_vars, xopt)

        add_line(solver_output, xopt, pos_goals[i], quat_goals[i])

        println(xopt)
    end

end

run_trial(relaxedIK, pos_goals, quat_goals, solver_output)
close_solver_output(solver_output)

# vars = relaxedIK.relaxedIK_vars
#display(Quat(1.0,0.0,0.0,0.0) * vars.init_ee_quats[1])
