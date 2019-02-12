#! /usr/bin/env python
'''
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 7/1/18

PLEASE DO NOT CHANGE CODE IN THIS FILE.  IF TRYING TO SET UP RELAXEDIK, PLEASE REFER TO start_here.py INSTEAD
AND FOLLOW THE STEP-BY-STEP INSTRUCTIONS THERE.  Thanks!
'''


######################################################################################################

import rospy
import os
from RelaxedIK.relaxedIK import RelaxedIK
from relaxed_ik.msg import EEPoseGoals, JointAngles
from std_msgs.msg import Float64
from RelaxedIK.Utils.colors import bcolors
from RelaxedIK.relaxedIK import get_relaxedIK_from_info_file

eepg = None
def eePoseGoals_cb(data):
    global eepg
    eepg = data

if __name__ == '__main__':
    rospy.init_node('relaxed_ik_node')
    angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions',JointAngles,queue_size=3)
    rospy.Subscriber('/relaxed_ik/ee_pose_goals', EEPoseGoals, eePoseGoals_cb)
    rospy.sleep(0.3)

    path_to_src = os.path.dirname(__file__)

    relaxedIK = get_relaxedIK_from_info_file(path_to_src)
    num_chains = relaxedIK.vars.robot.numChains

    while eepg == None: continue

    rate = rospy.Rate(150)
    while not rospy.is_shutdown():
        pos_goals = []
        quat_goals = []
        pose_goals = eepg.ee_poses
        header = eepg.header

        for i in xrange(num_chains):
            p = pose_goals[i]
            pos_x = p.position.x
            pos_y = p.position.y
            pos_z = p.position.z

            quat_w = p.orientation.w
            quat_x = p.orientation.x
            quat_y = p.orientation.y
            quat_z = p.orientation.z

            pos_goals.append([pos_x, pos_y, pos_z])
            quat_goals.append([quat_w, quat_x, quat_y, quat_z])

        xopt = relaxedIK.solve(pos_goals, quat_goals)
        ja = JointAngles()
        ja.header = header
        for x in xopt:
            ja.angles.data.append(x)

        angles_pub.publish(ja)
        print xopt

        rate.sleep()
