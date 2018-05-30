#! /usr/bin/env python
'''
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 5/9/18

PLEASE DO NOT CHANGE CODE IN THIS FILE.  IF TRYING TO SET UP RELAXEDIK, PLEASE REFER TO start_here.py INSTEAD
AND FOLLOW THE STEP-BY-STEP INSTRUCTIONS THERE.  Thanks!
'''
######################################################################################################

from start_here import urdf_file_name, joint_names, joint_ordering, ee_fixed_joints, starting_config, \
    joint_state_define, collision_file_name, fixed_frame
from RelaxedIK.relaxedIK import RelaxedIK
from RelaxedIK.GROOVE_RelaxedIK.relaxedIK_vars import RelaxedIK_vars
from sensor_msgs.msg import JointState
import rospy
import roslaunch
import os
import tf
import math


if __name__ == '__main__':
    # Don't change this code####################################################################################
    rospy.init_node('configuration')

    vars = RelaxedIK_vars('relaxedIK',os.path.dirname(__file__) + '/RelaxedIK/urdfs/' + urdf_file_name,joint_names,ee_fixed_joints,
                          joint_ordering,init_state=starting_config, collision_file=collision_file_name, config_override=True)

