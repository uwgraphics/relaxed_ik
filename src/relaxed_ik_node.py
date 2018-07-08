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
from RelaxedIK.relaxedIK import RelaxedIK
from relaxed_ik.msg import EEPoseGoals, JointAngles
from std_msgs.msg import Float32
from RelaxedIK.Utils.colors import bcolors

eepg = None
def eePoseGoals_cb(data):
    global eepg
    eepg = data

if __name__ == '__main__':
    rospy.init_node('relaxed_ik_node')
    angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions',JointAngles,queue_size=3)
    rospy.Subscriber('/relaxed_ik/ee_pose_goals', EEPoseGoals, eePoseGoals_cb)
    rospy.sleep(0.3)

    config_file_name = rospy.get_param('config_file_name', default='relaxedIK.config')
    relaxedIK = RelaxedIK.init_from_config(config_file_name)
    num_chains = relaxedIK.vars.robot.numChains

    while eepg == None: continue

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        pose_goals = eepg.ee_poses
        header = eepg.header
        num_poses = len(pose_goals)
        if not num_poses == num_chains:
            print bcolors.FAIL + 'ERROR: Number of pose goals ({}) ' \
                                 'not equal to the number of kinematic chains ({}).  Exiting relaxed_ik_node'.format(num_poses, num_chains)
            rospy.signal_shutdown()

        pos_goals = []
        quat_goals = []

        for p in pose_goals:
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
            ja.angles.append(Float32(x))

        angles_pub.publish(ja)

        rate.sleep()





