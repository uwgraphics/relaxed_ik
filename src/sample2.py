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
from RelaxedIK.GROOVE_RelaxedIK.relaxedIK_vars import RelaxedIK_vars
from sensor_msgs.msg import JointState
import RelaxedIK.Spacetime.Arm_ext as Arm_ext
import rospy
import roslaunch
import os
import tf
import math


if __name__ == '__main__':
    # Don't change this code####################################################################################
    rospy.init_node('sample_node')

    urdf_file = open(os.path.dirname(__file__) + '/RelaxedIK/urdfs/' + urdf_file_name, 'r')
    urdf_string = urdf_file.read()
    rospy.set_param('robot_description', urdf_string)
    js_pub = rospy.Publisher('joint_states',JointState,queue_size=5)
    tf_pub = tf.TransformBroadcaster()

    rospy.sleep(0.3)

    ####################################################################################################################
    relaxedIK = RelaxedIK.init_from_config(config_file_name)
    ####################################################################################################################

    state = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    for i in xrange(1):
        state[1] = i
        state[2] = i
        state[3] = i
        state[5] = i
        state[6] = i
        state[7] = i

        a = relaxedIK.vars.robot
        print a.getFrames(state)[0][0]


    # Don't change this code ###########################################################################################
    # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    # roslaunch.configure_logging(uuid)
    # launch_path = os.path.dirname(__file__) + '/../launch/robot_state_pub.launch'
    # launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
    # launch.start()
    ####################################################################################################################