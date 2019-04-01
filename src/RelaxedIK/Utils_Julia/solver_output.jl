
mutable struct Solver_Output
    out_file
end

function Solver_Output(path_to_src, solver_name, robot_name, task_name)
    f = open(path_to_src * "/RelaxedIK/FileIO/solver_outputs/" * solver_name * "/" * robot_name * "/" * task_name, "w")
    return Solver_Output(f)
end

function add_line(solver_output, xopt, pos_goal, quat_goal)
    time = "0.0"
    xopt_str = ""
    for i=1:length(xopt)
        xopt_str = xopt_str * "$(xopt[i])"
        if i != length(xopt)
            xopt_str = xopt_str * ","
        end
    end
    pos_str = "$(pos_goal[1]),$(pos_goal[2]),$(pos_goal[3])"
    quat_str = "$(quat_goal[1]),$(quat_goal[2]),$(quat_goal[3]),$(quat_goal[4])"
    write(solver_output.out_file, "$time;$xopt_str;$pos_str;$quat_str\n")
end

function close_solver_output(solver_output)
    close(solver_output.out_file)
end
