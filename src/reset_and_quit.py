#!/usr/bin/python

import readchar
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose
from std_msgs.msg import Float64MultiArray, Float32, Bool, Int8

rospy.init_node('reset_and_quit')

quit_pub = rospy.Publisher('/relaxed_ik/quit',Bool,queue_size=1)
reset_pub = rospy.Publisher('relaxed_ik/reset', Bool, queue_size=1)

rate = rospy.Rate(100)
while not rospy.is_shutdown():
    quit_msg = Bool()
    quit_msg.data = False
    quit_pub.publish(quit_msg)

    key = readchar.readkey()

    if key == 'q':
        quit_msg = Bool()
        quit_msg.data = True
        quit_pub.publish(quit_msg)


    if key == 'r':
        reset_msg = Bool()
        reset_msg.data = True
        reset_pub.publish(reset_msg)

    elif key == 'c':
        rospy.signal_shutdown()

    rate.sleep()

