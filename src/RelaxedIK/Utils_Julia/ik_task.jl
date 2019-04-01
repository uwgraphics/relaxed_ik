
include("transformations.jl")
using StaticArrays



function get_ik_task(path_to_src, task_name; scaling_factor=1.0)
    pos_goals = []
    quat_goals = []

    f = open(path_to_src * "/RelaxedIK/FileIO/task_recordings/" * task_name, "r")

    line = readline(f)
    while line != ""
        line_split = split(line, ";")

        pos_str = line_split[2]
        quat_str = line_split[3]

        pos_arr = split(pos_str, ",")
        quat_arr = split(quat_str, ",")

        pos = scaling_factor * SVector( parse(Float64, pos_arr[1]), parse(Float64, pos_arr[2]), parse(Float64, pos_arr[3])   )

        #quat = Quat(0.9999999,0.00000001,0.00000001,0.00000001)
        quat = Quat(   parse(Float64, quat_arr[1]), parse(Float64, quat_arr[2]), parse(Float64, quat_arr[3]), parse(Float64, quat_arr[4])   )

        push!(pos_goals, pos)
        push!(quat_goals, quat)

        line = readline(f)
    end

    return pos_goals, quat_goals
end
