

ENV["PYTHON"] = "/usr/bin/python"

#!/usr/bin/env julia

using RobotOS
@rosimport geometry_msgs.msg: Point, Pose2D
rostypegen()
#using geometry_msgs.msg
println(varinfo())

import geometry_msgs.msg
