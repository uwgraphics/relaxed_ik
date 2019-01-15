using LinearAlgebra

function get_camera_goal_location(x, vars, camera_arm_idx; Δ=0.5)
    vars.robot.arms[camera_arm_idx].getFrames(x[vars.robot.subchain_indices[camera_arm_idx]])

    eeMat = vars.robot.arms[camera_arm_idx].out_frames[end]
    camera_pt = vars.robot.arms[camera_arm_idx].out_pts[end]
    # for jaco7...
    up = eeMat[:,1]
    right = -eeMat[:,2]
    forward = -eeMat[:,3]

    search_direction = vars.additional_vars.search_direction
    search_direction_in_ee_frame = search_direction[1]*up + search_direction[2]*right + search_direction[3]*forward

    goal_position = vars.additional_vars.previous_camera_location + Δ*search_direction_in_ee_frame
    return goal_position
end
