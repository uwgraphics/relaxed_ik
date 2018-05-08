from RelaxedIK.GROOVE_RelaxedIK.relaxedIK_vars import RelaxedIK_vars
from RelaxedIK.relaxedIK import RelaxedIK
import numpy as np
import rospy
from broadcaster import *


rospy.init_node('test')

joint_names_r = ['WAIST', 'RIGHT_SHOULDER_PITCH', 'RIGHT_SHOULDER_ROLL', 'RIGHT_SHOULDER_YAW', 'RIGHT_ELBOW', 'RIGHT_WRIST_YAW',
               'RIGHT_WRIST_PITCH', 'RIGHT_WRIST_YAW_2']
joint_names_l = ['WAIST', 'LEFT_SHOULDER_PITCH', 'LEFT_SHOULDER_ROLL', 'LEFT_SHOULDER_YAW', 'LEFT_ELBOW', 'LEFT_WRIST_YAW',
               'LEFT_WRIST_PITCH', 'LEFT_WRIST_YAW_2']

joint_order = ['WAIST',
               'RIGHT_SHOULDER_PITCH', 'RIGHT_SHOULDER_ROLL', 'RIGHT_SHOULDER_YAW', 'RIGHT_ELBOW', 'RIGHT_WRIST_YAW',
               'RIGHT_WRIST_PITCH', 'RIGHT_WRIST_YAW_2',
               ]

joint_names = [joint_names_r]
fixed_joint_names = ['RIGHT_HAND']
init_state = [0,0,0,0,0,0,0,0]

vars = RelaxedIK_vars('hubo','RelaxedIK/urdfs/hubo_description.urdf',joint_names,fixed_joint_names, joint_order, pre_config=True, init_state=init_state)

relaxedIK = RelaxedIK(vars, optimization_package='scipy',solver_name='slsqp')

val = 0.0
while not rospy.is_shutdown():
    x_val = 0.1*math.sin(val)
    xopt =  relaxedIK.solve( [[x_val,2.0*x_val,x_val+0.2]], [[1,0,0,0]], verbose_output=False, maxtime=1.0, max_iter=10, unconstrained=True)
    joint_state_publish(xopt, 'hubo_upper_body')
    val += 0.05
    print xopt

# print relaxedIK.solve([0,0,0],[1,0,0,0])

