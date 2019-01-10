using RobotOS
include("RelaxedIK/Utils_Julia/ros_utils.jl")

@rosimport relaxed_ik.msg : EEPoseGoals, JointAngles
@rosimport geometry_msgs.msg: Point, Quaternion, Pose
@rosimport std_msgs.msg: Float64MultiArray, Bool
@rosimport visualization_msgs.msg: Marker

rostypegen()
using .relaxed_ik.msg
using .geometry_msgs.msg
using .std_msgs.msg
using .visualization_msgs.msg

init_node("test")

m = Marker()

pub = Publisher("/visualization_marker", Marker, queue_size = 3)


while ! is_shutdown()
    draw_sphere_in_rviz(pub, "world", [10.,2.,2.], [.1,.1,.1], [1.,0.,0.,1.]; id=4)
    draw_arrow_in_rviz(pub, "world", [0., 0., 0.], [10.,2.,2.], 0.1, 0.4, [0.,1.,0.,1.])
end
