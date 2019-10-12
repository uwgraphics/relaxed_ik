
# objectives should be written with respect to x and vars.  Closure will be made later automatically.

using LinearAlgebra
using StaticArrays
using Rotations
include("../Utils_Julia/transformations.jl")
include("../Utils_Julia/joint_utils.jl")
include("../Utils_Julia/geometry_utils.jl")
include("../Utils_Julia/nn_utils.jl")

function groove_loss(x_val, t, d, c, f, g)
    return (-2.718281828459^((-(x_val - t)^d) / (2.0 * c^2)) ) + f * (x_val - t)^g
end

function groove_loss_derivative(x_val, t, d, c, f, g)
    return -2.718281828459^((-(x_val - t)^d) / (2.0 * c^2)) * ( (-d*(x_val-t) ) / (2. * c^2) ) + g*f*(x_val-t)
end

function position_obj_1(x, vars)
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    x_val = norm(vars.robot.arms[1].out_pts[end] - vars.goal_positions[1])

    # return groove_loss(x_val, 0.,2.,.1,10.,2.)
    # 0.23065358014379128
    # 0.10204081632653063
    # return groove_loss(x_val, 0., 2, 0.23065358014379128, 0.10204081632653063, 2)
    return groove_loss(x_val, 0., 2, .1, 10., 2)
end


function position_obj_2(x, vars)
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])
    x_val = norm(vars.robot.arms[2].out_pts[end] - vars.goal_positions[2])

    return groove_loss(x_val, 0., 2, .1, 10., 2)
    # return groove_loss(x_val, 0., 2.0, 2.02637, 100.789, 2.0)
end

function position_obj_3(x, vars)
    vars.robot.arms[3].getFrames(x[vars.robot.subchain_indices[3]])
    x_val = norm(vars.robot.arms[3].out_pts[end] - vars.goal_positions[3])

    return groove_loss(x_val, 0., 2, .1, 10., 2)
    # return groove_loss(x_val, 0., 2.0, 2.02637, 100.789, 2.0)
end

function position_obj_4(x, vars)
    vars.robot.arms[4].getFrames(x[vars.robot.subchain_indices[4]])
    x_val = norm(vars.robot.arms[4].out_pts[end] - vars.goal_positions[4])

    return groove_loss(x_val, 0., 2, .1, 10., 2)
    # return groove_loss(x_val, 0., 2.0, 2.02637, 100.789, 2.0)
end

function position_obj_5(x, vars)
    vars.robot.arms[5].getFrames(x[vars.robot.subchain_indices[5]])
    x_val = norm(vars.robot.arms[5].out_pts[end] - vars.goal_positions[5])

    return groove_loss(x_val, 0., 2, .1, 10., 2)
    # return groove_loss(x_val, 0., 2.0, 2.02637, 100.789, 2.0)
end

function rotation_obj_1(x, vars)
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    eeMat = vars.robot.arms[1].out_frames[end]

    goal_quat = vars.goal_quats[1]
    ee_quat = Quat(eeMat)

    ee_quat2 = Quat(-ee_quat.w, -ee_quat.x, -ee_quat.y, -ee_quat.z)

    disp = norm(quaternion_disp(goal_quat, ee_quat))
    disp2 = norm(quaternion_disp(goal_quat, ee_quat2))

    x_val = min(disp, disp2)

    # return groove_loss(x_val, 0.,2.,.1,10.,2.)
    # return groove_loss(x_val, 0., 2, 0.23065358014379128, 0.10204081632653063, 2)
    return groove_loss(x_val, 0., 2, .1, 10., 2)
end

function rotation_obj_2(x, vars)
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])
    eeMat = vars.robot.arms[2].out_frames[end]

    goal_quat = vars.goal_quats[2]
    ee_quat = Quat(eeMat)

    ee_quat2 = Quat(-ee_quat.w, -ee_quat.x, -ee_quat.y, -ee_quat.z)

    disp = norm(quaternion_disp(goal_quat, ee_quat))
    disp2 = norm(quaternion_disp(goal_quat, ee_quat2))

    x_val = min(disp, disp2)

    return groove_loss(x_val, 0., 2, .1, 10., 2)
    # return groove_loss(x_val, 0., 2., -0.00910925, 32.1129, 2.)
end

function rotation_obj_3(x, vars)
    vars.robot.arms[3].getFrames(x[vars.robot.subchain_indices[3]])
    eeMat = vars.robot.arms[3].out_frames[end]

    goal_quat = vars.goal_quats[3]
    ee_quat = Quat(eeMat)

    ee_quat2 = Quat(-ee_quat.w, -ee_quat.x, -ee_quat.y, -ee_quat.z)

    disp = norm(quaternion_disp(goal_quat, ee_quat))
    disp2 = norm(quaternion_disp(goal_quat, ee_quat2))

    x_val = min(disp, disp2)

    return groove_loss(x_val, 0., 2, .1, 10., 2)
    # return groove_loss(x_val, 0., 2., -0.00910925, 32.1129, 2.)
end

function rotation_obj_4(x, vars)
    vars.robot.arms[4].getFrames(x[vars.robot.subchain_indices[4]])
    eeMat = vars.robot.arms[4].out_frames[end]

    goal_quat = vars.goal_quats[4]
    ee_quat = Quat(eeMat)

    ee_quat2 = Quat(-ee_quat.w, -ee_quat.x, -ee_quat.y, -ee_quat.z)

    disp = norm(quaternion_disp(goal_quat, ee_quat))
    disp2 = norm(quaternion_disp(goal_quat, ee_quat2))

    x_val = min(disp, disp2)

    return groove_loss(x_val, 0., 2, .1, 10., 2)
    # return groove_loss(x_val, 0., 2., -0.00910925, 32.1129, 2.)
end

function rotation_obj_5(x, vars)
    vars.robot.arms[5].getFrames(x[vars.robot.subchain_indices[5]])
    eeMat = vars.robot.arms[5].out_frames[end]

    goal_quat = vars.goal_quats[5]
    ee_quat = Quat(eeMat)

    ee_quat2 = Quat(-ee_quat.w, -ee_quat.x, -ee_quat.y, -ee_quat.z)

    disp = norm(quaternion_disp(goal_quat, ee_quat))
    disp2 = norm(quaternion_disp(goal_quat, ee_quat2))

    x_val = min(disp, disp2)

    return groove_loss(x_val, 0., 2, .1, 10., 2)
    # return groove_loss(x_val, 0., 2., -0.00910925, 32.1129, 2.)
end

function joint_goal_obj(x, vars)
    goal, ret_k = interpolate_to_joint_limits(vars.vars.xopt, vars.joint_goal, t=0.033, joint_velocity_limits=vars.robot.velocity_limits)
    x_val = euclidean(x, goal)
    return groove_loss(x_val, 0., 2, .1, 10., 2)
end

function min_jt_vel_obj(x, vars)
    # return groove_loss(norm(x - vars.vars.xopt), 0.0, 2.0, 0.1, 10.0, 2.0)
    return groove_loss(norm(x - vars.vars.xopt), 0.0, 2, 0.1, 10.0, 2)
end

function min_jt_accel_obj(x, vars)
    # return groove_loss(norm((vars.vars.xopt - vars.vars.prev_state) - (x - vars.vars.xopt)), 0.0, 2.0, 0.1, 10.0, 2.0)
    return groove_loss(norm((vars.vars.xopt - vars.vars.prev_state) - (x - vars.vars.xopt)),  0.0, 2, .1, 10.0, 2)
end

function min_jt_jerk_obj(x, vars)
    # return groove_loss( norm( ( (x - vars.vars.xopt) - (vars.vars.xopt - vars.vars.prev_state) ) - ( (vars.vars.xopt - vars.vars.prev_state) - (vars.vars.prev_state - vars.vars.prev_state2) ) ),  0.0, 2.0, 0.1, 10.0, 2.0   )
    return groove_loss( norm( ( (x - vars.vars.xopt) - (vars.vars.xopt - vars.vars.prev_state) ) - ( (vars.vars.xopt - vars.vars.prev_state) - (vars.vars.prev_state - vars.vars.prev_state2) ) ),  0.0, 2, .1, 10.0, 2  )
end

function joint_limit_obj(x, vars)
    sum = 0.0
    penalty_cutoff = 0.85
    a = 0.05 / (penalty_cutoff^50.)
    # penalty = 1.0
    # d = 8
    joint_limits = vars.vars.bounds
    for i = 1:vars.robot.num_dof
        l = joint_limits[i][1]
        u = joint_limits[i][2]
        # mid = (u + l) / 2.0
        # a = penalty / (u - mid)^d
        # sum += a*(x[i] - mid)^d
        r = (x[i] - l) / (u - l)
        n = 2.0 * (r - 0.5)
        sum += a*n^50.
    end

    x_val = sum
    # return groove_loss(  x_val, 0.0, 2.0, 2.3, 0.003, 2.0 )
    return groove_loss(  x_val, 0.0, 2, 0.3295051144911304, 0.1, 2)
end

function collision_nn_obj(x, vars)
    state_to_joint_pts_inplace(x, vars)
    # state = state_to_joint_pts_withreturn(x, vars)
    # return groove_loss(  vars.nn_model( vars.joint_pts ) , 0.0, 2.0, 0.07, 100.0, 2.0 )
    # 0.2010929597597385, 0.5241930016229932, 1.1853951273805203
    # 0.2010929597597385, 0.5241930016229932, 1.1853951273805203
    # return groove_loss(  vars.nn_model( vars.joint_pts ) , 0.2010929597597385, 2, 0.52419, 1.1853951273805203, 2 )
    return groove_loss(  vars.nn_model3( vars.joint_pts ), vars.nn_t3, 2, vars.nn_c3, vars.nn_f3, 2 )
end

function joint_vel_limit_obj(x, vars)
    joint_vel_slider = vars.joint_vel_slider

    fixed_frequency = 125.
    velocity_limits = vars.robot.velocity_limits
    num_joints = length(velocity_limits)
    prev_config = vars.vars.xopt
    sum = 0.0

    for i = 1:num_joints
        diff = abs(x[i] - prev_config[i])
        allowable_diff = joint_vel_slider * (velocity_limits[i] / fixed_frequency)
        turning_point = allowable_diff
        a = 0.01 / (turning_point)^100.
        sum += a*diff^100.
    end

    return sum
end

function bimanual_line_seg_collision_avoid_obj(x, vars)
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])

    out_pts1 = vars.robot.arms[1].out_pts
    out_pts2 = vars.robot.arms[2].out_pts

    x_val = 0.0
    for i = 1:length(out_pts1)-1
        # x_val += (-2.718281828459^((-(out_pts1[i][3])^2) / (2.0 * c^2.0)) )
        # x_val += (-2.718281828459^((-(out_pts1[i+1][3])^2) / (2.0 * c^2.0)) )
        # for j = 1:length(out_pts2)-1
        c = 0.12
        dis1 = dis_between_line_segments( out_pts1[i], out_pts1[i+1], out_pts2[1], out_pts2[2] )
        dis2 = dis_between_line_segments( out_pts1[i], out_pts1[i+1], out_pts2[2], out_pts2[3] )
        dis3 = dis_between_line_segments( out_pts1[i], out_pts1[i+1], out_pts2[3], out_pts2[4] )
        dis4 = dis_between_line_segments( out_pts1[i], out_pts1[i+1], out_pts2[4], out_pts2[5] )
        dis5 = dis_between_line_segments( out_pts1[i], out_pts1[i+1], out_pts2[5], out_pts2[6] )
        dis6 = dis_between_line_segments( out_pts1[i], out_pts1[i+1], out_pts2[6], out_pts2[7] )

        x_val += (2.718281828459^((-(dis1)^2) / (2.0 * c^2.0)) )
        x_val += (2.718281828459^((-(dis2)^2) / (2.0 * c^2.0)) )
        x_val += (2.718281828459^((-(dis3)^2) / (2.0 * c^2.0)) )
        x_val += (2.718281828459^((-(dis4)^2) / (2.0 * c^2.0)) )
        x_val += (2.718281828459^((-(dis5)^2) / (2.0 * c^2.0)) )
        x_val += (2.718281828459^((-(dis6)^2) / (2.0 * c^2.0)) )

            # the next two are just to avoid the plane on the ground
            # x_val += (-2.718281828459^((-(out_pts2[j][3])^2) / (2.0 * c^2.0)) )
            # x_val += (-2.718281828459^((-(out_pts2[j+1][3])^2) / (2.0 * c^2.0)) )
        # end
    end

    return groove_loss(x_val, 0.,2.,.2, .4, 2.)
end
