

include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/Utils_Julia/transformations.jl")
using BenchmarkTools

function get_goals(path_to_src)
    in_file = open(path_to_src * "/RelaxedIK/Config/goals")
    goal_pos = []
    goal_quats = []

    goal_number = readline(in_file)

    line = readline(in_file)
    while line != ""
        line_split = split(line, ";")
        pos_split = split(line_split[1], ",")
        quat_split = split(line_split[2], ",")

        push!(goal_pos, [parse(Float64, pos_split[1]), parse(Float64, pos_split[2]), parse(Float64, pos_split[3])] )

        push!(goal_quats, Quat(parse(Float64, quat_split[1]), parse(Float64, quat_split[2]), parse(Float64, quat_split[3]), parse(Float64, quat_split[4])) )

        line = readline(in_file)
    end
    close(in_file)

    if length(goal_pos) == 0
        return Nothing, Nothing, Nothing
    end

    # println(goal_pos)
    return goal_pos, goal_quats, goal_number
end

function write_solution(path_to_src, x, goal_number)
    out_file = open(path_to_src * "/RelaxedIK/Config/solution", "w")
    write(out_file, goal_number * ',')
    for i = 1:length(x)
        write(out_file, string(x[i]) * ",")
    end
    close(out_file)
end

path_to_src = Base.source_dir()
loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
loaded_robot = readline(loaded_robot_file)

relaxedIK = get_standard(path_to_src, loaded_robot)

close(loaded_robot_file)

function run_relaxedIK(path_to_src)

    prev_pos_goals, prev_quat_goals, prev_goal_number = get_goals(path_to_src)

    while true

        pos_goals, quat_goals, goal_number = get_goals(path_to_src)

        if pos_goals == Nothing
            pos_goals = prev_pos_goals
            quat_goals = prev_quat_goals
            goal_number = prev_goal_number
        else
            prev_pos_goals = pos_goals
            prev_quat_goals = quat_goals
        end

        relaxedIK.relaxedIK_vars.goal_positions = pos_goals
        relaxedIK.relaxedIK_vars.goal_quats = quat_goals

        # xopt = groove_solve(relaxedIK.groove)
        xopt = solve(relaxedIK, pos_goals, quat_goals)

        write_solution(path_to_src, xopt, goal_number)

        println(pos_goals)
        # println(xopt)
    end

end


run_relaxedIK(path_to_src)
