
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
    idx = relaxedIK.relaxedIK_vars.robot.subchain_indices
    # vars.robot.getFrames(x)
    # subchain = vars.robot.split_state_into_subchains_c(x)
    for i=1:vars.robot.num_chains
        #subchain = zeros(length(relaxedIK.relaxedIK_vars.robot.subchain_indices[i]))
        #for j=1:length(relaxedIK.relaxedIK_vars.robot.subchain_indices[i])
        #    subchain[j] = x[relaxedIK.relaxedIK_vars.robot.subchain_indices[i][j]]
        #end
        #=
        if i == 1
            vars.robot.arms[i].getFrames( [x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8]] )
        else
            vars.robot.arms[i].getFrames( [x[1], x[9], x[10], x[11], x[12], x[13], x[14], x[15]] )
        end
        =#
        # vars.robot.arms[i].getFrames(subchain)
        vars.robot.arms[i].getFrames( [ x[1], x[2], x[3], x[4], x[5], x[6] ] )
        x_val += norm(vars.robot.arms[i].out_pts[end] - vars.goal_positions[i])
        # x_val += norm(vars.robot.arms[i].out_pts[end] - [1.,0.,0.])
    end
    return x_val
end

function rotation_obj(x, vars)
    x_val = 0.0
    for i=1:vars.robot.num_chains
        #subchain = zeros(length(relaxedIK.relaxedIK_vars.robot.subchain_indices[i]))
        #for j=1:length(relaxedIK.relaxedIK_vars.robot.subchain_indices[i])
        #    subchain[j] = x[relaxedIK.relaxedIK_vars.robot.subchain_indices[i][j]]
        #end
        #=
        if i == 1
            vars.robot.arms[i].getFrames( [x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8]] )
        else
            vars.robot.arms[i].getFrames( [x[1], x[9], x[10], x[11], x[12], x[13], x[14], x[15]] )
        end
        =#
        vars.robot.arms[i].getFrames([ x[1], x[2], x[3], x[4], x[5], x[6] ] )
        # vars.robot.arms[i].getFrames(subchain)
        eeMat = vars.robot.arms[i].out_frames[end]

        goal_quat = Quat(1,0,0,0)
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
