
include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/Utils_Julia/transformations.jl")

path_to_src = Base.source_dir()
loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
loaded_robot = readline(loaded_robot_file)

relaxedIK = get_standard(path_to_src, loaded_robot)

# relaxedIK_closure = (goal_positions, goal_quats, prev_state) -> solve(relaxedIK, goal_positions, goal_quats, prev_state = prev_state)

function test(goal_positions, goal_quats, num_chains)
    gq = []
    for i=1:num_chains
        push!(gq, Quat(goal_quats[4*(i-1) + 1], goal_quats[4*(i-1) + 2], goal_quats[4*(i-1) + 3], goal_quats[4*(i-1) + 4]))
    end
    return solve(relaxedIK, goal_positions, goal_quats)
end

final = (gp, gq) -> test(gp, gq, relaxedIK.relaxedIK_vars.robot.num_chains)
