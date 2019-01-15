#!/usr/bin/python

import rospy
from std_msgs.msg import Bool

if __name__ == '__main__':
    rospy.init_node('quit')

    quit_pub = rospy.Publisher('/relaxed_ik/quit', Bool, queue_size=1)

    b = Bool()
    b.data = True

    for i in xrange(10000000):
        quit_pub.publish(b)


