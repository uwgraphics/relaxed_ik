#! /usr/bin/env python
'''
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 5/10/18
'''
######################################################################################################

from start_here import urdf_file_name, joint_names, joint_ordering, ee_fixed_joints, starting_config, \
    joint_state_define, collision_file_name, fixed_frame, config_file_name
from RelaxedIK.relaxedIK import RelaxedIK
from relaxed_ik.msg import EEPoseGoals
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import rospy
import roslaunch
import os
import tf
import math


if __name__ == '__main__':
    # Don't change this code####################################################################################
    rospy.init_node('sample_node')

    ####################################################################################################################
    relaxedIK = RelaxedIK.init_from_config(config_file_name)
    ####################################################################################################################

    urdf_file = open(relaxedIK.vars.urdf_path, 'r')
    urdf_string = urdf_file.read()
    rospy.set_param('robot_description', urdf_string)
    js_pub = rospy.Publisher('joint_states',JointState,queue_size=5)
    ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=3)
    tf_pub = tf.TransformBroadcaster()

    rospy.sleep(0.3)

    # Don't change this code ###########################################################################################
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch_path = os.path.dirname(__file__) + '/../launch/robot_state_pub.launch'
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
    launch.start()
    ####################################################################################################################

    rospy.sleep(1.0)

    counter = 0.0
    stride = 0.08
    idx = 0
    while not rospy.is_shutdown():
        c = math.cos(counter)
        s = 0.2
        num_ee = relaxedIK.vars.robot.numChains
        goal_pos = []
        goal_quat = []
        goal_pos.append([0,s*c,0])
        for i in range(num_ee):
            goal_quat.append([1,0,0,0])
            if i == 0:
                continue
            else:
                goal_pos.append([0,0,0])

        xopt = relaxedIK.solve(goal_pos, goal_quat)

        js = joint_state_define(xopt)
        if js == None:
            js = JointState()
            js.name = joint_ordering
            for x in xopt:
                js.position.append(x)
        now = rospy.Time.now()
        js.header.stamp.secs = now.secs
        js.header.stamp.nsecs = now.nsecs
        js_pub.publish(js)

        ee_pose_goals = EEPoseGoals()
        ee_pose_goals.header.seq = idx
        for i in range(num_ee):
            p = Pose()
            curr_goal_pos = goal_pos[i]
            curr_goal_quat = goal_quat[i]
            p.position.x = curr_goal_pos[0]
            p.position.y = curr_goal_pos[1]
            p.position.z = curr_goal_pos[2]

            p.orientation.w = curr_goal_quat[0]
            p.orientation.x = curr_goal_quat[1]
            p.orientation.y = curr_goal_quat[2]
            p.orientation.z = curr_goal_quat[3]
            ee_pose_goals.ee_poses.append(p)

        ee_pose_goals_pub.publish(ee_pose_goals)

        tf_pub.sendTransform((0, 0, 0),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             'common_world',
                             fixed_frame)

        idx += 1
        counter += stride