#! /usr/bin/env python

'''
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 5/10/18
'''
######################################################################################################

from start_here import urdf_file_name, joint_names, joint_ordering, ee_fixed_joints, starting_config, \
    joint_state_define, collision_file_name, fixed_frame
from RelaxedIK.GROOVE_RelaxedIK.relaxedIK_vars import RelaxedIK_vars
from relaxed_ik.msg import EEPoseGoals
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from RelaxedIK.Utils.yaml_utils import get_relaxedIK_yaml_obj
import rospy
import roslaunch
import os
import tf
import math
import os
from RelaxedIK.relaxedIK import get_relaxedIK_from_info_file
from relaxed_ik.msg import EEPoseGoals
# from RelaxedIK.Julia_Bridge.relaxedIK_julia import RelaxedIK_Julia



eepg = None
def eePoseGoals_cb(data):
    global eepg
    eepg = data


if __name__ == '__main__':
    # Don't change this code####################################################################################

    rospy.init_node('sample_node')

    path_to_src = os.path.dirname(__file__)

    # rik = RelaxedIK_Julia(path_to_src)
    rik = get_relaxedIK_from_info_file(path_to_src)
    # rik.reset(rik_p.vars.robot.numChains)

    y = get_relaxedIK_yaml_obj(path_to_src)
    if not y == None:
        urdf_file_name = y['urdf_file_name']
        joint_names = y['joint_names']
        joint_ordering = y['joint_ordering']
        ee_fixed_joints = y['ee_fixed_joints']
        starting_config = y['starting_config']
        collision_file_name = y['collision_file_name']
        fixed_frame = y['fixed_frame']
        joint_state_define_file_name = y['joint_state_define_func_file']
        joint_state_define_file = open(path_to_src + '/RelaxedIK/Config/joint_state_define_functions/' + joint_state_define_file_name, 'r')
        func = joint_state_define_file.read()
        exec(func)
        print urdf_file_name

    num_chains = len(ee_fixed_joints)

    urdf_file = open(path_to_src + '/RelaxedIK/urdfs/' + urdf_file_name, 'r')
    urdf_string = urdf_file.read()
    rospy.set_param('robot_description', urdf_string)
    js_pub = rospy.Publisher('joint_states',JointState,queue_size=5)
    ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=3)
    tf_pub = tf.TransformBroadcaster()

    rospy.Subscriber('/relaxed_ik/ee_pose_goals', EEPoseGoals, eePoseGoals_cb)

    rospy.sleep(0.5)

    # Don't change this code ###########################################################################################
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch_path = os.path.dirname(__file__) + '/../launch/robot_state_pub.launch'
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
    launch.start()
    ####################################################################################################################

    rospy.sleep(1.0)

    while eepg == None: continue

    counter = 0.0
    stride = 0.01
    idx = 0

    rate = rospy.Rate(10000)
    while not rospy.is_shutdown():
        pose_goals = eepg.ee_poses
        header = eepg.header
        num_ee = num_chains
        num_poses = len(pose_goals)
        # if not num_poses == num_chains:
        #     print bcolors.FAIL + 'ERROR: Number of pose goals ({}) ' \
        #                          'not equal to the number of kinematic chains ({}).  Exiting relaxed_ik_node'.format(num_poses, num_chains)
        #     rospy.signal_shutdown()

        pos_goals = []
        quat_goals = []

        for i in xrange(num_ee):
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

        xopt = rik.solve(pos_goals, quat_goals)


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
            curr_goal_pos = pos_goals[i]
            curr_goal_quat = quat_goals[i]
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

        rate.sleep()