#!/usr/bin/env julia
using YAML
using RobotOS

include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_vars.jl")
include("RelaxedIK/Utils_Julia/ros_utils.jl")
include("RelaxedIK/Utils_Julia/autocam_utils.jl")
include("RelaxedIK/Utils_Julia/ema_filter.jl")
@rosimport relaxed_ik.msg : EEPoseGoals, JointAngles
@rosimport geometry_msgs.msg: Point, Quaternion, Pose
@rosimport std_msgs.msg: Float64MultiArray, Bool, Float32
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

quit = false
function quit_cb(data::BoolMsg)
    global quit
    quit = data.data
end

camera_motion_magnitude = 0.1
function camera_motion_magnitude_cb(data::Float32Msg)
    global camera_motion_magnitude
    camera_motion_magnitude = data.data
end

# function loop()
path_to_src = Base.source_dir()
loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
loaded_robot = readline(loaded_robot_file)
close(loaded_robot_file)

fp = open(path_to_src * "/RelaxedIK/Config/info_files/" * loaded_robot)
y = YAML.load(fp)
close(fp)

fixed_frame = y["fixed_frame"]

relaxedIK = get_autocam1(path_to_src, loaded_robot)
# ema_filter = EMA_filter(relaxedIK.relaxedIK_vars.vars.init_state)

num_chains = relaxedIK.relaxedIK_vars.robot.num_chains

println("loaded robot: $loaded_robot")

init_node("autocam_node")

Subscriber{EEPoseGoals}("/relaxed_ik/ee_pose_goals", eePoseGoals_cb, queue_size=3)
Subscriber{Float64MultiArray}("/autocam/search_direction", search_direction_cb, queue_size=3)
Subscriber{BoolMsg}("/autocam/too_close", too_close_cb, queue_size=3)
Subscriber{BoolMsg}("/relaxed_ik/quit", quit_cb, queue_size=3)
Subscriber{Float32Msg}("/autocam/motion_magnitude", camera_motion_magnitude_cb, queue_size=3)
angles_pub = Publisher("/relaxed_ik/joint_angle_solutions", JointAngles, queue_size = 3)
marker_pub = Publisher("/visualization_marker", Marker, queue_size = 3)

sleep(0.3)

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

println("ready to get first solution...")
loop_rate = Rate(1000)
while true
    global quit
    if quit == true
        println("quitting")
        quit = false
        return
    end
    relaxedIK.relaxedIK_vars.additional_vars.search_direction = search_direction
    relaxedIK.relaxedIK_vars.additional_vars.too_close = too_close

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

    # xopt_f = filter_signal(ema_filter, xopt)
    println(xopt)

    relaxedIK.relaxedIK_vars.robot.getFrames(xopt)
    relaxedIK.relaxedIK_vars.additional_vars.previous_camera_location = relaxedIK.relaxedIK_vars.robot.arms[2].out_pts[end]

    # println(relaxedIK.relaxedIK_vars.vars.objective_closures[end-1](xopt))
    camera_goal_pt = get_camera_goal_location(xopt, relaxedIK.relaxedIK_vars, 2; Δ=camera_motion_magnitude)
    relaxedIK.relaxedIK_vars.additional_vars.visual_target_position = camera_goal_pt
    draw_arrow_in_rviz(marker_pub, fixed_frame, relaxedIK.relaxedIK_vars.additional_vars.previous_camera_location, camera_goal_pt, 0.03, 0.03, [0.,1.,0.,1.]; id=1)
    # draw_sphere_in_rviz(marker_pub, fixed_frame, relaxedIK.relaxedIK_vars.additional_vars.previous_camera_location, [0.1,0.1,0.1], [1.,0.,0.,1.])

    rossleep(loop_rate)
end

# end

# loop()
