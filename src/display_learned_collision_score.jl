#!/usr/bin/env julia

using RobotOS

@rosimport visualization_msgs.msg: Marker
@rosimport sensor_msgs.msg: JointState
@rosimport std_msgs.msg: Bool
rostypegen()
using .visualization_msgs.msg
using .sensor_msgs.msg
using .std_msgs.msg
include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/Utils_Julia/nn_utils.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_vars.jl")

joint_state = ""
function joint_state_cb(data::JointState)
    global joint_state
    joint_state = data
end

function get_positions_from_joint_state(joint_state, joint_ordering)
    js = joint_state.position
    names = joint_state.name
    positions = []
    for o in joint_ordering
        for i = 1:length(names)
            if o == names[i]
                push!(positions, js[i])
            end
        end
    end
    return positions
end

init_node("display_collision_info")
marker_pub = Publisher("/visualization_marker", Marker, queue_size = 3)
Subscriber{JointState}("/joint_states", joint_state_cb)

sleep(1.0)

path_to_src = Base.source_dir()
println(path_to_src)
loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
loaded_robot = readline(loaded_robot_file)
close(loaded_robot_file)

relaxedIK = get_standard(path_to_src, loaded_robot)
y = info_file_name_to_yaml_block(path_to_src, loaded_robot)
joint_ordering = y["joint_ordering"]



loop_rate = Rate(100)
while ! is_shutdown()
    if ! (joint_state == "")
        positions = get_positions_from_joint_state(joint_state, joint_ordering)
        state = state_to_joint_pts_withreturn(positions, relaxedIK.relaxedIK_vars)
        println(relaxedIK.relaxedIK_vars.nn_model(state) )
    end
    rossleep(loop_rate)
end
