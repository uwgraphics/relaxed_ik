
# objectives should be written with respect to x and vars.  Closure will be made later automatically.

using LinearAlgebra
using StaticArrays
using Rotations
include("../Utils_Julia/transformations.jl")
include("../Utils_Julia/geometry_utils.jl")

function groove_loss(x_val, t, d, c, f, g)
    return (-2.718281828459^((-(x_val - t)^d) / (2.0 * c^2.0)) ) + f * (x_val - t)^g
end

function position_obj_1(x, vars)
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    x_val = norm(vars.robot.arms[1].out_pts[end] - vars.goal_positions[1])

    return groove_loss(x_val, 0.,2.,.1,10.,2.)
end

function position_obj_2(x, vars)
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])
    x_val = norm(vars.robot.arms[2].out_pts[end] - vars.goal_positions[2])

    return groove_loss(x_val, 0.,2.,.1,10.,2.)
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

    return groove_loss(x_val, 0.,2.,.1,10.,2.)

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

    return groove_loss(x_val, 0.,2.,.1,10.,2.)
end

function min_jt_vel_obj(x, vars)
    return groove_loss(norm(x - vars.vars.xopt), 0.0, 2.0, 0.1, 10.0, 2.0)
end

function min_jt_accel_obj(x, vars)
    return groove_loss(norm((vars.vars.xopt - vars.vars.prev_state) - (x - vars.vars.xopt)), 0.0, 2.0, 0.1, 10.0, 2.0)
end

function min_jt_jerk_obj(x, vars)
    return groove_loss( norm( ( (x - vars.vars.xopt) - (vars.vars.xopt - vars.vars.prev_state) ) - ( (vars.vars.xopt - vars.vars.prev_state) - (vars.vars.prev_state - vars.vars.prev_state2) ) ),  0.0, 2.0, 0.1, 10.0, 2.0   )
end


function collision_nn_obj(x, vars)
    return groove_loss(  vars.collision_nn(x), 0.0, 2.0, 0.2, 0.3, 2.0 )
end

function lookat_obj_1(x, vars)
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])

    eeMat = vars.robot.arms[2].out_frames[end]
    camera_pt = vars.robot.arms[2].out_pts[end]
    manipulation_pt = vars.robot.arms[1].out_pts[end]
    forward = eeMat[:,3]

    x_val = pt_dis_to_line_seg(manipulation_pt, camera_pt, camera_pt - 100.0*forward)
    return groove_loss(x_val, 0.,2.,.1,10.,2.)
end

function lookat_obj_2(x, vars)
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])

    eeMat = vars.robot.arms[1].out_frames[end]
    camera_pt = vars.robot.arms[1].out_pts[end]
    manipulation_pt = vars.robot.arms[2].out_pts[end]
    forward = eeMat[:,3]

    x_val = pt_dis_to_line_seg(manipulation_pt, camera_pt, camera_pt - 100.0*forward)
    return groove_loss(x_val, 0.,2.,.1,10.,2.)
end

function camera_dis_obj_1(x, vars)
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])

    camera_pt = vars.robot.arms[2].out_pts[end]
    manipulation_pt = vars.robot.arms[1].out_pts[end]

    goal_dis = 0.6
    x_val = (LinearAlgebra.norm(camera_pt - manipulation_pt) - goal_dis)^2
    return groove_loss(x_val, 0.,2.,.1,10.,2.)
end


function camera_dis_obj_2(x, vars)
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])

    camera_pt = vars.robot.arms[1].out_pts[end]
    manipulation_pt = vars.robot.arms[2].out_pts[end]

    goal_dis = 0.6
    x_val = (LinearAlgebra.norm(camera_pt - manipulation_pt) - goal_dis)^2
    return groove_loss(x_val, 0.,2.,.1,10.,2.)
end

function camera_upright_obj_1(x, vars)
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])

    eeMat = vars.robot.arms[2].out_frames[end]
    side = eeMat[:,2]

    x_val = LinearAlgebra.dot(side, [0.,0.,1.])^2
    return groove_loss(x_val, 0.,2.,.1,10.,2.)
end

function camera_upright_obj_2(x, vars)
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[2]])

    eeMat = vars.robot.arms[1].out_frames[end]
    side = eeMat[:,2]

    x_val = LinearAlgebra.dot(side, [0.,0.,1.])^2
    return groove_loss(x_val, 0.,2.,.1,10.,2.)
end


function camera_occlusion_avoid_obj_1(x, vars)
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])

    eeMat = vars.robot.arms[2].out_frames[end]
    camera_pt = vars.robot.arms[2].out_pts[end]
    forward = eeMat[:,3]

    x_val = 0.0
    out_pts = vars.robot.arms[1].out_pts
    num_pts = length(out_pts)
    for i = 1:num_pts-1
        dis = dis_between_line_segments(camera_pt, camera_pt - 100.0*forward, out_pts[i], out_pts[i+1])
        c = 0.07
        x_val += (2.718281828459^((-(dis)^2) / (2.0 * c^2.0)) )
    end

    # x_val = 0.0
    return groove_loss(x_val, 0.,2.,.35, .4,2.)
end

function camera_occlusion_avoid_obj_2(x, vars)
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])

    eeMat = vars.robot.arms[1].out_frames[end]
    camera_pt = vars.robot.arms[1].out_pts[end]
    forward = eeMat[:,3]

    x_val = 0.0
    out_pts = vars.robot.arms[2].out_pts
    num_pts = length(out_pts)
    for i = 1:num_pts-1
        dis = dis_between_line_segments(camera_pt, camera_pt - 100.0*forward, out_pts[i], out_pts[i+1])
        c = 0.06
        x_val += (2.718281828459^((-(dis)^2) / (2.0 * c^2.0)) )
    end

    # x_val = 0.0
    return groove_loss(x_val, 0.,2.,.35, .4,2.)
end


function bimanual_line_seg_collision_avoid_obj(x, vars)
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    vars.robot.arms[2].getFrames(x[vars.robot.subchain_indices[2]])

    out_pts1 = vars.robot.arms[1].out_pts
    out_pts2 = vars.robot.arms[2].out_pts

    x_val = 0.0
    for i = 1:length(out_pts1)-1
        for j = 1:length(out_pts2)-1
            dis = dis_between_line_segments(out_pts1[i], out_pts1[i+1], out_pts2[i], out_pts2[i+1])
            c = 0.1
            x_val += (2.718281828459^((-(dis)^2) / (2.0 * c^2.0)) )
        end
    end

    return groove_loss(x_val, 0.,2.,.35, .4,2.)
end
