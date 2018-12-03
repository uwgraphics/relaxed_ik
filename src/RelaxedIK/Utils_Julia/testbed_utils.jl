include("transformations.jl")
using LinearAlgebra

function position_error(pos, pos_goal)
    return norm(pos - pos_goal)
end

function rotation_error(quat, quat_goal)
    # note that the units in the output are pure radians, NOT radians/2!!
    option1 = 2*norm(quaternion_disp(quat, quat_goal))
    option2 = 2*norm(quaternion_disp(Quat(-quat.w, -quat.x, -quat.y, -quat.z), quat_goal))

    return min(option1, option2)
end

function get_metrics(relaxedIK, xopt, pos_goal, quat_goal)
    vars = relaxedIK.relaxedIK_vars
    vars.robot.getFrames(xopt)

    ee_pos = vars.robot.arms[1].out_pts[end]
    ee_quat = Quat(vars.robot.arms[1].out_frames[end])

    pos_error = position_error(ee_pos, pos_goal)
    quat_error = rotation_error(ee_quat, quat_goal)

    vel = norm(xopt - vars.vars.prev_state)
    acc = norm((vars.vars.prev_state - vars.vars.prev_state2) - (xopt - vars.vars.prev_state))
    jerk = norm( ( (xopt - vars.vars.prev_state) - (vars.vars.prev_state - vars.vars.prev_state2) ) - ( (vars.vars.prev_state - vars.vars.prev_state2) - (vars.vars.prev_state2 - vars.vars.prev_state3) ) )

    return pos_error, quat_error, vel, acc, jerk
end
