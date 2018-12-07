#!/usr/bin/env julia


using Statistics
include("RelaxedIK/Utils_Julia/ik_task.jl")
include("RelaxedIK/Utils_Julia/solver_output.jl")
include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_vars.jl")
include("RelaxedIK/GROOVE_Julia/groove.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_objective.jl")
include("RelaxedIK/Utils_Julia/testbed_utils.jl")
include("RelaxedIK/Utils_Julia/ema_filter.jl")


################################################################################
robot_names = ["ur5", "sawyer", "iiwa", "hubo8"]
info_names = ["ur5_info.yaml", "sawyer_info.yaml", "iiwa_info.yaml", "hubo8_info.yaml"]
task_names = [ ["circle", "square","rotations"], ["circle", "square", "rotations"], ["circle", "square", "rotations"],
    ["huboCircle", "huboSquare", "rotations"] ]
scaling_factors = [ [2.0, 2.0, 1.0],  [2.0, 2.0, 1.0], [2.0, 2.0, 1.0], [1.0, 1.0, 1.0] ]
################################################################################

# relaxedIK = get_standard(path_to_src, "sawyer_info.yaml")
# relaxedIK.relaxedIK_vars.robot.getFrames([0.,0.,0.,0.,0.,0.,0.])
# relaxedIK = get_standard(path_to_src, "ur5_info.yaml")
# relaxedIK.relaxedIK_vars.robot.getFrames([0.,0.,0.,0.,0.,0.])



function run_trial(path_to_src, robot_names, info_names, solver_name, task_names, scaling_factors)
# should only be for ONE solver configuration!

    pos_errors = []
    rot_errors = []
    vels = []
    accs = []
    jerks = []
    times = []

    for r=1:length(robot_names)
        if solver_name == "relaxed_ik"
            relaxedIK = get_standard(path_to_src, info_names[r])
        elseif solver_name == "finite_diff"
            relaxedIK = get_finite_diff_version(path_to_src, info_names[r])
        end

        # num_dof = vars.robot.num_dof
        # vars.robot.getFrames(rand(num_dof))
        # vars.robot.arms[1].out_pts
        # relaxedIK = get_standard(path_to_src, info_names[r])
        vars = relaxedIK.relaxedIK_vars

        for t=1:length(task_names[r])

            pos_goals, quat_goals = get_ik_task(path_to_src, task_names[r][t], scaling_factor=scaling_factors[r][t])
            solver_output = Solver_Output(path_to_src, solver_name, robot_names[r], task_names[r][t])
            f = EMA_filter(vars.vars.init_state)

            for i=1:length(pos_goals)
                vars.goal_positions = copy(vars.init_ee_positions)
                vars.goal_positions[1] += pos_goals[i]
                vars.goal_quats[1] = quat_goals[i]

                # vars.goal_quats = copy(vars.init_ee_quats)
                vars.goal_quats[1] = Quat(quat_goals[i] * copy(vars.init_ee_quats[1]))
                # vars.goal_quats[1] = Quat(0.496594, 0.495679, 0.504295, -0.503371)

                start = time()
                xopt = groove_solve(relaxedIK.groove,  prev_state=[])
                stop = time()
                xopt_f = filter_signal(f, xopt)

                update_relaxedIK_vars!(relaxedIK.relaxedIK_vars, xopt)

                add_line(solver_output, xopt, pos_goals[i], quat_goals[i])

                metrics =  get_metrics(relaxedIK, xopt, vars.goal_positions[1], vars.goal_quats[1])
                push!(pos_errors, metrics[1])
                push!(rot_errors, metrics[2])
                push!(vels, metrics[3])
                push!(accs, metrics[4])
                push!(jerks, metrics[5])
                push!(times, stop - start)

                println("robot: $(robot_names[r]), task: $(task_names[r][t]), solver name: $solver_name, idx: $i, xopt: $xopt")

            end

            close_solver_output(solver_output)

        end

    end

    ret1 = [Statistics.mean(pos_errors), Statistics.mean(rot_errors), Statistics.mean(vels), Statistics.mean(accs), Statistics.mean(jerks), Statistics.mean(times)]
    ret2 = [Statistics.std(pos_errors), Statistics.std(rot_errors), Statistics.std(vels), Statistics.std(accs), Statistics.std(jerks), Statistics.std(times)]
    return ret1, ret2
end

path_to_src = Base.source_dir()

a = run_trial(path_to_src, robot_names, info_names, "relaxed_ik", task_names, scaling_factors)
# b = run_trial(path_to_src, robot_names, info_names, "finite_diff", task_names, scaling_factors)


println(a)
# println(b)


# vars = relaxedIK.relaxedIK_vars
#display(Quat(1.0,0.0,0.0,0.0) * vars.init_ee_quats[1])
