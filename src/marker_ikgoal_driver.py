#!/usr/bin/python

import rospy
import os
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from RelaxedIK.relaxedIK import get_relaxedIK_from_info_file
from RelaxedIK.Utils.interactive_marker_utils import InteractiveMarkerFeedbackUtil, InteractiveMarkerUtil, InteractiveMarkerServerUtil
from relaxed_ik.msg import EEPoseGoals
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose
import RelaxedIK.Utils.transformations as T

rospy.init_node('marker_ikgoal_driver')

path_to_src = os.path.dirname(__file__)

relaxedIK = get_relaxedIK_from_info_file(path_to_src)
num_chains = relaxedIK.vars.robot.numChains

init_ee_positions =  relaxedIK.vars.init_ee_positions
init_ee_quats =  relaxedIK.vars.init_ee_quats

server = InteractiveMarkerServer("simple_marker")
ee_pose_goal_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=3)

rospy.sleep(0.2)

int_markers = []

for i in xrange(num_chains):
    int_marker = InteractiveMarkerUtil(init_pos=init_ee_positions[i], init_quat=init_ee_quats[i])
    int_marker.add_6dof_controls()
    int_markers.append(int_marker)

    server.insert(int_marker.interactive_marker, int_marker.feedback_util.feedback_handler)

server.applyChanges()


rate = rospy.Rate(40)
while not rospy.is_shutdown():
    eepg = EEPoseGoals()

    for i in xrange(num_chains):
        if not int_markers[i].feedback_util.active:
            pose = Pose()
            pose.position.x = 0.0
            pose.position.y = 0.0
            pose.position.z = 0.0

            pose.orientation.w = 1.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
        else:
            pose = Pose()

            pose.position.x = int_markers[i].feedback_util.feedback.pose.position.x - init_ee_positions[i][0]
            pose.position.y = int_markers[i].feedback_util.feedback.pose.position.y - init_ee_positions[i][1]
            pose.position.z = int_markers[i].feedback_util.feedback.pose.position.z - init_ee_positions[i][2]

            pose.orientation.w = int_markers[i].feedback_util.feedback.pose.orientation.w
            pose.orientation.x = int_markers[i].feedback_util.feedback.pose.orientation.x
            pose.orientation.y = int_markers[i].feedback_util.feedback.pose.orientation.y
            pose.orientation.z = int_markers[i].feedback_util.feedback.pose.orientation.z

        eepg.ee_poses.append(pose)

    ee_pose_goal_pub.publish(eepg)

    rate.sleep()
