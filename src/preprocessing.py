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
from RelaxedIK.Utils.yaml_utils import get_relaxedIK_yaml_obj


if __name__ == '__main__':
    # Don't change this code####################################################################################
    rospy.init_node('preprocessing')

    path_to_src = os.path.dirname(__file__)

    y = get_relaxedIK_yaml_obj(path_to_src)
    if not y == None:
        urdf_file_name = y['urdf_file_name']
        joint_names = y['joint_names']
        joint_ordering = y['joint_ordering']
        ee_fixed_joints = y['ee_fixed_joints']
        starting_config = y['starting_config']
        collision_file_name = y['collision_file_name']
        collision_nn_file = y['collision_nn_file']
        fixed_frame = y['fixed_frame']
        joint_state_define_file_name = y['joint_state_define_func_file']
        joint_state_define_file = open(path_to_src + '/RelaxedIK/Config/joint_state_define_functions/' + joint_state_define_file_name, 'r')
        func = joint_state_define_file.read()
        exec(func)


    vars = RelaxedIK_vars('relaxedIK',os.path.dirname(__file__) + '/RelaxedIK/urdfs/' + urdf_file_name,joint_names,ee_fixed_joints, joint_ordering,init_state=starting_config, collision_file=collision_file_name, config_override=True, path_to_src=path_to_src, collision_nn_file=collision_nn_file)

