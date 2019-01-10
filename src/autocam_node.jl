#!/usr/bin/env julia

println("welcome.")
using YAML
using RobotOS
# using Rotations
# using BenchmarkTools
# using ForwardDiff
# using Knet
println("importing ros stuff.")
include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_vars.jl")
include("RelaxedIK/Utils_Julia/ros_utils.jl")
include("RelaxedIK/Utils_Julia/autocam_utils.jl")
@rosimport relaxed_ik.msg : EEPoseGoals, JointAngles
@rosimport geometry_msgs.msg: Point, Quaternion, Pose
@rosimport std_msgs.msg: Float64MultiArray, Bool
@rosimport visualization_msgs.msg: Marker

rostypegen()
using .relaxed_ik.msg
using .geometry_msgs.msg
using .std_msgs.msg
using .visualization_msgs.msg

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

fp = open(path_to_src * "/RelaxedIK/Config/info_files/" * loaded_robot)
y = YAML.load(fp)
close(fp)

fixed_frame = y["fixed_frame"]

relaxedIK = get_autocam1(path_to_src, loaded_robot)
println("relaxedIK initialized.")

num_chains = relaxedIK.relaxedIK_vars.robot.num_chains

println("loaded robot: $loaded_robot")

init_node("autocam_node")

println("node initialized.")

Subscriber{EEPoseGoals}("/relaxed_ik/ee_pose_goals", eePoseGoals_cb, queue_size=3)
Subscriber{Float64MultiArray}("/autocam/search_direction", search_direction_cb, queue_size=3)
Subscriber{BoolMsg}("/autocam/too_close", too_close_cb, queue_size=3)
angles_pub = Publisher("/relaxed_ik/joint_angle_solutions", JointAngles, queue_size = 3)
marker_pub = Publisher("/visualization_marker", Marker, queue_size = 3)

println("subscribers and publishers initialized.")

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
search_direction = [1.,0.]
relaxedIK.relaxedIK_vars.robot.getFrames(relaxedIK.relaxedIK_vars.vars.init_state)
relaxedIK.relaxedIK_vars.additional_vars.previous_camera_location = relaxedIK.relaxedIK_vars.robot.arms[2].out_pts[end]

loop_rate = Rate(1000)
println("ready to enter loop.")
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
    relaxedIK.relaxedIK_vars.additional_vars.previous_camera_location = relaxedIK.relaxedIK_vars.robot.arms[2].out_pts[end]

    # println(relaxedIK.relaxedIK_vars.vars.objective_closures[end-1](xopt))
    goal_pt = get_camera_goal_location(xopt, relaxedIK.relaxedIK_vars, 2, relaxedIK.relaxedIK_vars.additional_vars.search_direction)
    draw_arrow_in_rviz(marker_pub, fixed_frame, relaxedIK.relaxedIK_vars.additional_vars.previous_camera_location, goal_pt, 0.03, 0.03, [0.,1.,0.,1.]; id=1)
    draw_sphere_in_rviz(marker_pub, fixed_frame, relaxedIK.relaxedIK_vars.additional_vars.previous_camera_location, [0.1,0.1,0.1], [1.,0.,0.,1.])

    rossleep(loop_rate)
end
# end

# loop()
