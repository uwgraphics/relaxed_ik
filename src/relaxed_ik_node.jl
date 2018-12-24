#!/usr/bin/env julia

include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_vars.jl")
using YAML
using RobotOS
using Rotations
@rosimport relaxed_ik.msg : EEPoseGoals, JointAngles
@rosimport geometry_msgs.msg: Point, Quaternion

rostypegen()
using .relaxed_ik.msg
using .geometry_msgs.msg

eepg = Nothing
function eePoseGoals_cb(data::EEPoseGoals)
    global eepg
    eepg = data
end

path_to_src = Base.source_dir()
println(path_to_src)
loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
loaded_robot = readline(loaded_robot_file)

println("loaded robot: $loaded_robot")

relaxedIK = get_standard(path_to_src, loaded_robot)
num_chains = min(relaxedIK.relaxedIK_vars.robot.num_chains, 2)

init_node("relaxed_ik_node_jl")

Subscriber{EEPoseGoals}("/relaxed_ik/ee_pose_goals", eePoseGoals_cb, queue_size=3)
angles_pub = Publisher("/relaxed_ik/joint_angle_solutions", JointAngles, queue_size = 3)

sleep(0.4)

eepg = EEPoseGoals()

loop_rate = Rate(200)
while ! is_shutdown()
    if length(eepg.ee_poses) == 0
    else
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
        println(xopt)

    end
    rossleep(loop_rate)
end
