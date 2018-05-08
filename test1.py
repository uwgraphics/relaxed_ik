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
               'LEFT_SHOULDER_PITCH', 'LEFT_SHOULDER_ROLL', 'LEFT_SHOULDER_YAW', 'LEFT_ELBOW', 'LEFT_WRIST_YAW',
               'LEFT_WRIST_PITCH', 'LEFT_WRIST_YAW_2'
               ]

joint_names = [joint_names_r, joint_names_l]
fixed_joint_names = ['RIGHT_HAND', 'LEFT_HAND']
init_state = [0.0, 0.78852112, -0.14649155, -0.17695991, -1.86374666, -0.09144152, -0.46006443, -0.1494651, 0.78852112,
              0.14649155, 0.17695991, -1.86374666, 0.09144152, -0.46006443, 0.1494651]

vars = RelaxedIK_vars('hubo','RelaxedIK/urdfs/hubo_description.urdf',joint_names,fixed_joint_names, joint_order,
                      pre_config=False, init_state=init_state, collision_file='collision_hubo_bimanual.yaml')

relaxedIK = RelaxedIK(vars, optimization_package='scipy',solver_name='slsqp')

cg = relaxedIK.vars.collision_graph
cu = cg.c

val = 0.0
while not rospy.is_shutdown():
    x_val = 0.4*math.sin(val)
    xopt =  relaxedIK.solve( [[x_val,0.0,x_val], [x_val,0,0]], [[1,0,0,0], [1,0,0,0]], verbose_output=False, maxtime=1.0, max_iter=20, unconstrained=False)
    # xopt =  relaxedIK.solve( [[0,0,0], [0,0,0]], [[1,0,0,0], [1,0,0,0]], verbose_output=False, maxtime=1.0, max_iter=20, unconstrained=False)
    joint_state_publish(xopt, 'hubo_bimanual')

    frames = relaxedIK.vars.robot.getFrames(xopt)
    cu.update_all_transforms(frames)
    cg.get_collision_score(frames)
    cu.draw_all()

    val += 0.2
    print xopt

# print relaxedIK.solve([0,0,0],[1,0,0,0])

