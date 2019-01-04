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

rostypegen()
using .relaxed_ik.msg
using .geometry_msgs.msg

eepg = Nothing
function eePoseGoals_cb(data::EEPoseGoals)
    global eepg
    eepg = data
end

# function loop()
path_to_src = Base.source_dir()
println(path_to_src)
loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
loaded_robot = readline(loaded_robot_file)
close(loaded_robot_file)

relaxedIK = get_bimanual(path_to_src, loaded_robot)

num_chains = relaxedIK.relaxedIK_vars.robot.num_chains

println("loaded robot: $loaded_robot")

init_node("relaxed_ik_node_jl")

Subscriber{EEPoseGoals}("/relaxed_ik/ee_pose_goals", eePoseGoals_cb, queue_size=3)
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

loop_rate = Rate(1000)
while ! is_shutdown()
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
    # end
    rossleep(loop_rate)
end
# end


# loop()
