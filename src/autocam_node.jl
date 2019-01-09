#!/usr/bin/env julia

include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_vars.jl")
using YAML
using RobotOS
using Rotations
using BenchmarkTools
using ForwardDiff
using Knet
@rosimport relaxed_ik.msg : EEPoseGoals, JointAngles
@rosimport geometry_msgs.msg: Point, Quaternion, Pose
@rosimport std_msgs.msg: Float64MultiArray, Bool


rostypegen()
using .relaxed_ik.msg
using .geometry_msgs.msg
using .std_msgs.msg


eepg = Nothing
function eePoseGoals_cb(data::EEPoseGoals)
    global eepg
    eepg = data
end

search_direction = Nothing
function search_direction_cb(data::Float64MultiArray)
    global search_direction
    search_direction = data.data
end

too_close = false
function too_close_cb(data::BoolMsg)
    global too_close
    too_close = data.data
end

# function loop()
path_to_src = Base.source_dir()
println(path_to_src)
loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
loaded_robot = readline(loaded_robot_file)
close(loaded_robot_file)

relaxedIK = get_autocam1(path_to_src, loaded_robot)

num_chains = relaxedIK.relaxedIK_vars.robot.num_chains

println("loaded robot: $loaded_robot")

init_node("autocam_node")

Subscriber{EEPoseGoals}("/relaxed_ik/ee_pose_goals", eePoseGoals_cb, queue_size=3)
Subscriber{Float64MultiArray}("/autocam/search_direction", search_direction_cb, queue_size=3)
Subscriber{BoolMsg}("/autocam/too_close", too_close_cb, queue_size=3)
angles_pub = Publisher("/relaxed_ik/joint_angle_solutions", JointAngles, queue_size = 3)

sleep(0.4)

eepg = EEPoseGoals()
pose = Pose()
pose.position.x = 0.0
pose.position.y = 0.0
pose.position.z = 0.0
pose.orientation.w = 1.0
pose.orientation.x = 0.0
pose.orientation.y = 0.0
pose.orientation.z = 0.0
for i = 1:num_chains
    push!(eepg.ee_poses, pose)
end

too_close = false
search_direction = [0.,0.]
relaxedIK.relaxedIK_vars.robot.getFrames(relaxedIK.relaxedIK_vars.vars.init_state)
relaxedIK.relaxedIK_vars.additional_vars.previous_camera_location = relaxedIK.relaxedIK_vars.robot.arms[1].out_pts[end]

loop_rate = Rate(1000)
while ! is_shutdown()
    relaxedIK.relaxedIK_vars.additional_vars.search_direction = search_direction
    relaxedIK.relaxedIK_vars.additional_vars.too_close = too_close
    # while true
    # if length(eepg.ee_poses) == 0
    #    println("waiting for pose goals.......")
    # else
    pose_goals = eepg.ee_poses

    pos_goals = []
    quat_goals = []

    for i = 1:num_chains
        p = pose_goals[i]

        pos_x = p.position.x
        pos_y = p.position.y
        pos_z = p.position.z

        quat_w = p.orientation.w
        quat_x = p.orientation.x
        quat_y = p.orientation.y
        quat_z = p.orientation.z

        push!(pos_goals, [pos_x, pos_y, pos_z])
        push!(quat_goals, Quat(quat_w, quat_x, quat_y, quat_z))
    end


    xopt = solve(relaxedIK, pos_goals, quat_goals)
    ja = JointAngles()
    for i = 1:length(xopt)
        push!(ja.angles.data, xopt[i])
    end
    publish(angles_pub, ja)

    println(xopt)
    relaxedIK.relaxedIK_vars.robot.getFrames(xopt)
    relaxedIK.relaxedIK_vars.additional_vars.previous_camera_location = relaxedIK.relaxedIK_vars.robot.arms[1].out_pts[end]

    # println(relaxedIK.relaxedIK_vars.robot.arms[1].out_frames[end])

    rossleep(loop_rate)
end
# end


# loop()
