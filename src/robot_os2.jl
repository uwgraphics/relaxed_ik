

using RobotOS
@rosimport std_msgs.msg : Float32

rostypegen()

using .std_msgs.msg

a = Float32Msg()
function cb(data::Float32Msg)
    global a
    a = data.data
    println("000000000000000000000000000000000000000")
end


init_node("robot_os2")
sub = Subscriber{Float32Msg}("float", cb, queue_size=10)

sleep(2.0)

loop_rate = Rate(5.0)
while ! is_shutdown()
    println(a)
    rossleep(loop_rate)
end

spin()
