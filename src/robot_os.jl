#!/usr/bin/env julia
using RobotOS
@rosimport geometry_msgs.msg: PoseStamped
@rosimport relaxed_ik.msg : EEPoseGoals, JointAngles
@rosimport std_msgs.msg : Float32

rostypegen()
using .geometry_msgs.msg
using .relaxed_ik.msg
using .std_msgs.msg

init_node("julia_test")
t = Publisher("/test_topic", JointAngles, queue_size = 1)

sleep(1)
ja = JointAngles()
push!(ja.angles, 1.0)
# push!(ja.angles, 2.0)
# push!(ja.angles, 3.0)
# println(ja)
# a = Float32Msg(1.0)
# ja.angles = [1.,1.,1.]
# ps.header.stamp = RobotOS.now()

for i = 1:100
    ja = JointAngles()
    ja.header.seq = i
    convert(Array{Any, 1}, ja.angles)
    println(ja.angles)
    a = Float32Msg(1.0)
    push!(ja.angles, a)
    publish(t, ja)
    println(ja)
end


a = Array{Float32Msg, 1}()
println(a)
push!(a, Float32Msg(1.0))
push!(a, Float32Msg(2.0))
println(a)
