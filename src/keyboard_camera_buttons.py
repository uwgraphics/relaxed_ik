#!/usr/bin/python

import readchar
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose
from std_msgs.msg import Float64MultiArray, Float32, Bool, Int8

rospy.init_node('keyboard_camera_buttons')

viewpoint_dir_pub = rospy.Publisher('/autocam/search_direction/manual',Float64MultiArray,queue_size=5)
camera_motion_magnitude_pub = rospy.Publisher('/autocam/motion_magnitude',Float32,queue_size=5)
goal_dis_pub = rospy.Publisher('/autocam/goal_dis',Float32,queue_size=5)
quit_pub = rospy.Publisher('/relaxed_ik/quit',Bool,queue_size=5)
camera_mode_pub = rospy.Publisher('/autocam/camera_mode', Int8, queue_size=3)
outer_cone_max_pub = rospy.Publisher('/autocam/outer_cone_max', Float32, queue_size=3)


mag_stride = 0.005
curr_magnitude = 0.1
max_magnitude = 4.0

goal_dis = 0.6
goal_dis_stride = 0.005
min_dis = 0.3
max_dis = 1.7
camera_mode = 0

outer_cone_max = 0.8

rate = rospy.Rate(1000)
while not rospy.is_shutdown():
    key = readchar.readkey()

    if key == 'l':
        dir = Float64MultiArray()
        dir.data = [0.,1.,0.]
        viewpoint_dir_pub.publish(dir)
    elif key == 'i':
        dir = Float64MultiArray()
        dir.data = [1.,0.,0.]
        viewpoint_dir_pub.publish(dir)
    elif key == 'm':
        dir = Float64MultiArray()
        dir.data = [-1.,0.,0.]
        viewpoint_dir_pub.publish(dir)
    elif key == 'j':
        dir = Float64MultiArray()
        dir.data = [0.,-1.,0.]
        viewpoint_dir_pub.publish(dir)
    elif key == '[':
        dir = Float64MultiArray()
        dir.data = [0.,0.,1.]
        viewpoint_dir_pub.publish(dir)
    elif key == ']':
        dir = Float64MultiArray()
        dir.data = [0.,0.,-1.]
        viewpoint_dir_pub.publish(dir)
    elif key == ',':
        curr_magnitude -= mag_stride
        curr_magnitude = max(curr_magnitude, 0.0000001)
    elif key == '.':
        curr_magnitude += mag_stride
        curr_magnitude = min(curr_magnitude, 3.0)
    elif key == 'q':
        q = Bool()
        q.data = True
        quit_pub.publish(q)
    elif key == '/':
        camera_mode = 1 - camera_mode
    elif key == 'b':
        outer_cone_max += 0.01
        outer_cone_max = min(outer_cone_max, 1.3)
    elif key == 'v':
        outer_cone_max -= 0.01
        outer_cone_max = max(outer_cone_max, 0.4)
    elif key == 'h':
        goal_dis += goal_dis_stride
        goal_dis = min(goal_dis, 1.2)
    elif key == 'g':
        goal_dis -= goal_dis_stride
        goal_dis = max(goal_dis, 0.4)
    elif key == 'c':
        rospy.signal_shutdown()

    q = Bool()
    q.data = False
    quit_pub.publish(q)

    f = Float32()
    f.data = curr_magnitude
    camera_motion_magnitude_pub.publish(f)

    d = Float32()
    d.data = goal_dis
    goal_dis_pub.publish(d)

    cm = Int8()
    cm.data = camera_mode
    camera_mode_pub.publish(cm)

    ocm = Float32()
    ocm.data = outer_cone_max
    outer_cone_max_pub.publish(ocm)
