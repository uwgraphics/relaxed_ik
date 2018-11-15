
# objectives should be written with respect to x and vars.  Closure will be made later automatically.

using LinearAlgebra
using StaticArrays
using Rotations
include("../Utils_Julia/transformations.jl")

function groove_loss(x_val, t, d, c, f, g)
    return (-2.718281828459^((-(x_val - t)^d) / (2.0 * c^2.0)) ) + f * (x_val - t)^g
end

function position_obj(x, vars)
    x_val = 0.0
    for i=1:vars.robot.num_chains
        vars.robot.arms[i].getFrames(x[relaxedIK.relaxedIK_vars.robot.subchain_indices[i]])
        x_val += norm(vars.robot.arms[i].out_pts[end] - vars.goal_positions[i])
    end
    return x_val
end

function rotation_obj(x, vars)
    x_val = 0.0
    for i=1:vars.robot.num_chains
        vars.robot.arms[i].getFrames(x[relaxedIK.relaxedIK_vars.robot.subchain_indices[i]])
        eeMat = vars.robot.arms[i].out_frames[end]

        goal_quat = vars.goal_quats[i]
        ee_quat = Quat(eeMat)

        ee_quat2 = Quat(-ee_quat.w, -ee_quat.x, -ee_quat.y, -ee_quat.z)

        disp = norm(quaternion_disp(goal_quat, ee_quat))
        disp2 = norm(quaternion_disp(goal_quat, ee_quat2))

        x_val += min(disp, disp2)
    end

    return groove_loss(x_val, 0.,2.,.1,10.,2.)
end

function min_jt_vel_obj(x, vars)
    return groove_loss(norm(x - vars.vars.xopt), 0.0, 2.0, 0.1, 10.0, 2.0)
end

function min_jt_accel_obj(x, vars)
    return groove_loss(norm((vars.vars.xopt - vars.vars.prev_state) - (x - vars.vars.xopt)), 0.0, 2.0, 0.1, 10.0, 2.0)
end

function min_jt_jerk_obj(x, vars)
    return groove_loss( norm(x - vars.vars.xopt - vars.vars.xopt - vars.vars.prev_state - vars.vars.xopt - vars.vars.prev_state - vars.vars.prev_state - vars.vars.prev_state2), 0.0, 2.0, 0.1, 10.0, 2.0)
end
