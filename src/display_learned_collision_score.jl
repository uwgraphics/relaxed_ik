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


total_iters = 1.0
false_positive_count = 0.0
false_negative_count = 0.0
true_positive_count = 0.0
true_negative_count = 0.0

loop_rate = Rate(100)
while ! is_shutdown()
    if ! (joint_state == "")
        positions = get_positions_from_joint_state(joint_state, joint_ordering)
        state = state_to_joint_pts_withreturn(positions, relaxedIK.relaxedIK_vars)
        model_score = relaxedIK.relaxedIK_vars.nn_model3(state)
        incollision_m = in_collision(relaxedIK, positions)
        incollision_g = in_collision_groundtruth(relaxedIK, positions)

        if incollision_m == incollision_g
            if incollision_g == true
                global true_positive_count
                true_positive_count += 1.0
            else
                global true_negative_count
                true_negative_count += 1.0
            end
        else
            if incollision_g == true
                global false_positive_count
                false_positive_count += 1.0
            else
                global false_negative_count
                false_negative_count += 1.0
            end
        end

        global total_iters
        false_negative_perc = false_negative_count/total_iters
        false_positive_perc = false_positive_count/total_iters
        accuracy = (true_positive_count + true_negative_count)/total_iters
        # println("false negatives: $false_negative_count *** $false_negative_perc , false positives: $false_positive_count *** $false_positive_perc , true negatives: $true_negative_count , true positives: $true_positive_count, accuracy: $accuracy , num iters: $total_iters")
        println(incollision_m)

        total_iters += 1.0
    end
    rossleep(loop_rate)
end
