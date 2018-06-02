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
import RelaxedIK.Spacetime.boost.Arm_ext as Arm_ext
from RelaxedIK.GROOVE_RelaxedIK.relaxedIK_objective import *
import rospy
import roslaunch
import os
import tf
import math
import numpy as np
import time


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

    # relaxedIK.solve([[0,0,0], [0,0,0]],[[1,0,0,0],[1,0,0,0]])

    obj = Collision_Avoidance_nn()
    vars = relaxedIK.vars
    vars.c_boost = True

    num_trials = 1000000

    rand_state = np.random.uniform(-3.0, 3.0, 15)

    start = time.clock()
    for i in xrange(num_trials):
        val = obj.__call__(np.array(rand_state), vars)
        # objective_master_relaxedIK(rand_state)
    print val
    end = time.clock()

    print end - start
    print (end - start) / num_trials

    vars.c_boost = False
    print

    start = time.clock()
    for i in xrange(num_trials):
        val = obj.__call__(rand_state, vars)
        # objective_master_relaxedIK(rand_state)
    print val
    end = time.clock()

    print end - start
    print (end - start) / num_trials

