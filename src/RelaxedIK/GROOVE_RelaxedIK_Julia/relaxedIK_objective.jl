
# objectives should be written with respect to x and vars.  Closure will be made later automatically.

using LinearAlgebra
using StaticArrays
using Rotations
include("../Utils_Julia/transformations.jl")

function groove_loss(x_val, t, d, c, f, g)
    return (-2.718281828459^((-(x_val - t)^d) / (2.0 * c^2.0)) ) + f * (x_val - t)^g
end

function position_obj_1(x, vars)
    # x_val = 0.0
    # for i=1:vars.robot.num_chains
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    x_val = norm(vars.robot.arms[1].out_pts[end] - vars.goal_positions[1])
        # println(norm(vars.robot.arms[i].out_pts[end] - vars.goal_positions[i]))
    # end

    return groove_loss(x_val, 0.,2.,.1,10.,2.)
    # return x_val^2
end

function rotation_obj_1(x, vars)
    # x_val = 0.0
    # for i=1:vars.robot.num_chains
    vars.robot.arms[1].getFrames(x[vars.robot.subchain_indices[1]])
    eeMat = vars.robot.arms[1].out_frames[end]

    goal_quat = vars.goal_quats[1]
    ee_quat = Quat(eeMat)

    ee_quat2 = Quat(-ee_quat.w, -ee_quat.x, -ee_quat.y, -ee_quat.z)

    disp = norm(quaternion_disp(goal_quat, ee_quat))
    disp2 = norm(quaternion_disp(goal_quat, ee_quat2))

    x_val = min(disp, disp2)
    # end

    return groove_loss(x_val, 0.,2.,.1,10.,2.)
    # return x_val^2

end

function min_jt_vel_obj(x, vars)
    # return norm(x - vars.vars.xopt)^2
    return groove_loss(norm(x - vars.vars.xopt), 0.0, 2.0, 0.1, 10.0, 2.0)
end

function min_jt_accel_obj(x, vars)
    # return norm((vars.vars.xopt - vars.vars.prev_state) - (x - vars.vars.xopt))^2
    return groove_loss(norm((vars.vars.xopt - vars.vars.prev_state) - (x - vars.vars.xopt)), 0.0, 2.0, 0.1, 10.0, 2.0)
end

function min_jt_jerk_obj(x, vars)
    #=
    prev_state_3 = vars.vars.prev_state2
    prev_state_2 = vars.vars.prev_state
    prev_state = vars.vars.xopt

    v3 = (prev_state_2 - prev_state_3)
    v2 = (prev_state - prev_state_2)
    v1 = (x - prev_state)

    a2 = (v2 - v3)
    a1 = (v1 - v2)

    j = a1 - a2

    x_val = norm(j)
    return groove_loss( x_val,  0.0, 2.0, 0.1, 10.0, 2.0   )
    =#

    # return norm( ( (x - vars.vars.xopt) - (vars.vars.xopt - vars.vars.prev_state) ) - ( (vars.vars.xopt - vars.vars.prev_state) - (vars.vars.prev_state - vars.vars.prev_state2) ) )^2
    return groove_loss( norm( ( (x - vars.vars.xopt) - (vars.vars.xopt - vars.vars.prev_state) ) - ( (vars.vars.xopt - vars.vars.prev_state) - (vars.vars.prev_state - vars.vars.prev_state2) ) ),  0.0, 2.0, 0.1, 10.0, 2.0   )

    # return groove_loss( norm(x - vars.vars.xopt - vars.vars.xopt - vars.vars.prev_state - vars.vars.xopt - vars.vars.prev_state - vars.vars.prev_state - vars.vars.prev_state2), 0.0, 2.0, 0.1, 10.0, 2.0)
end

function collision_nn_obj(x, vars)
    # return vars.collision_nn(x)^2
    return groove_loss(  vars.collision_nn(x), 0.0, 2.0, 1.85, 0.004, 2.0)
end
