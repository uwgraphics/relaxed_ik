using LinearAlgebra


function get_camera_goal_location(x, vars, camera_arm_idx; Δ=0.5)
    vars.robot.arms[camera_arm_idx].getFrames(x[vars.robot.subchain_indices[camera_arm_idx]])

    eeMat = vars.robot.arms[camera_arm_idx].out_frames[end]
    camera_pt = vars.robot.arms[camera_arm_idx].out_pts[end]
    # for jaco7...
    up = eeMat[:,1]
    right = -eeMat[:,2]
    back = eeMat[:,3]

    search_direction = vars.additional_vars.search_direction
    search_direction_in_ee_frame = search_direction[1]*up + search_direction[2]*right - search_direction[3]*back

    goal_position = vars.additional_vars.previous_camera_location + Δ*search_direction_in_ee_frame
    return goal_position
end

function get_camera_goal_location(vars, camera_arm_idx; α=0.1, β=0.1, max_dis = 3.0)
    vars.robot.arms[camera_arm_idx].getFrames(vars.vars.xopt[vars.robot.subchain_indices[camera_arm_idx]])

    eeMat = vars.robot.arms[camera_arm_idx].out_frames[end]
    camera_pt = vars.additional_vars.previous_camera_location
    # for jaco7...
    up = eeMat[:,1]
    right = -eeMat[:,2]
    back = eeMat[:,3]

    search_direction = vars.additional_vars.search_direction
    search_direction_in_ee_frame = search_direction[1]*up + search_direction[2]*right

    target_pt = camera_pt + α*(search_direction_in_ee_frame)
    movement_dir_n = LinearAlgebra.normalize(target_pt - camera_pt)

    new_camera_goal_location = camera_pt + β*(movement_dir_n)

    if LinearAlgebra.norm(new_camera_goal_location - camera_pt) > max_dix
        new_camera_goal_location = camera_pt + max_dis * LinearAlgebra.normalize(new_camera_goal_location - camera_pt)
    end

    return new_camera_goal_location, target_pt
end
