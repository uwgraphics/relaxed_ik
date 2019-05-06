#! /usr/bin/env python
'''
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 11/4/18

PLEASE DO NOT CHANGE CODE IN THIS FILE.  IF TRYING TO SET UP RELAXEDIK, PLEASE REFER TO start_here.py INSTEAD
AND FOLLOW THE STEP-BY-STEP INSTRUCTIONS THERE.  Thanks!
'''
######################################################################################################

import os
import rospy
from RelaxedIK.Utils.colors import bcolors
from RelaxedIK.GROOVE_RelaxedIK.relaxedIK_vars import RelaxedIK_vars
from start_here import info_file_name, urdf_file_name, fixed_frame, joint_names, joint_ordering, ee_fixed_joints, starting_config, \
    collision_file_name, joint_state_define
import inspect

rospy.init_node('generate_info_file')

path_to_src = os.path.dirname(__file__)

if info_file_name == '':
    print bcolors.FAIL + 'info_file_name is a required field in start_here.py.  Please fill that in and run again.' + bcolors.ENDC
    exit(-1)

out_file = open(path_to_src + '/RelaxedIK/Config/info_files/' + info_file_name, 'w')

if urdf_file_name == '':
    print bcolors.FAIL + 'urdf_file_name is a required field in start_here.py.  Please fill that in and run again.' + bcolors.ENDC
    exit(-1)
else:
    out_file.write('urdf_file_name: \"{}\"\n'.format(urdf_file_name))

if fixed_frame == '':
    print bcolors.FAIL + 'fixed_frame is a required field in start_here.py.  Please fill that in and run again.' + bcolors.ENDC
    exit(-1)
else:
    out_file.write('fixed_frame: \"{}\"\n'.format(fixed_frame))

if len(joint_names) == 0:
    print bcolors.FAIL + 'joint_names is a required field in start_here.py.  Please fill that in and run again.' + bcolors.ENDC
    exit(-1)
else:
    num_chains = len(joint_names)
    chains_str = ''
    for i in range(num_chains):
        chains_str += '[ '
        for j in range(len(joint_names[i])):
            chains_str += '\"' + joint_names[i][j] + '\"'
            if not j == len(joint_names[i]) -1:
                chains_str += ', '
        chains_str += ' ]'
        if not i == num_chains - 1:
            chains_str += ', '
    out_file.write('joint_names: [ {} ]\n'.format(chains_str))


if len(joint_ordering) == 0:
    print bcolors.FAIL + 'joint_ordering is a required field in start_here.py.  Please fill that in and run again.' + bcolors.ENDC
    exit(-1)
else:
    ordering_str = '[ '
    for i in range(len(joint_ordering)):
        ordering_str += '\"{}\"'.format(joint_ordering[i])
        if not i == len(joint_ordering) -1:
            ordering_str += ', '
    ordering_str += ' ]'
    out_file.write('joint_ordering: {}\n'.format(ordering_str))

if len(ee_fixed_joints) == 0:
    print bcolors.FAIL + 'ee_fixed_joints is a required field in start_here.py.  Please fill that in and run again.' + bcolors.ENDC
    exit(-1)
else:
    ee_str = '[ '
    for i in range(len(ee_fixed_joints)):
        ee_str += '\"{}\"'.format(ee_fixed_joints[i])
        if not i == len(ee_fixed_joints) -1:
            ee_str += ', '
    ee_str += ' ]'
    out_file.write('ee_fixed_joints: {}\n'.format(ee_str))


if len(starting_config) == 0:
    print bcolors.FAIL + 'starting_config is a required field in start_here.py.  Please fill that in and run again.' + bcolors.ENDC
    exit(-1)
else:
    st_str = '[ '
    for i in range(len(starting_config)):
        st_str += '{}'.format(str(starting_config[i]))
        if not i == len(starting_config) -1:
            st_str += ', '
    st_str += ' ]'
    out_file.write('starting_config: {}\n'.format(st_str))



if collision_file_name == '':
    print bcolors.FAIL + 'collision_file_name is a required field in start_here.py.  Please fill that in and run again.' + bcolors.ENDC
    exit(-1)
else:
    out_file.write('collision_file_name: \"{}\"\n'.format(collision_file_name))

robot_name_split = urdf_file_name.split('.')
robot_name = robot_name_split[0]

out_file.write('collision_nn_file: \"{}\"\n'.format(robot_name + '_nn'))

out_file.write('path_to_src: \"{}\"\n'.format(path_to_src))


'''
thread_dofs_str = '[ '
for i in xrange(len(thread_dofs)):
    thread_dofs_str += ' [ '
    for j in xrange(len(thread_dofs[i])):
        thread_dofs_str += str(thread_dofs[i][j])
        if not j >= len(thread_dofs[i]) - 1:
            thread_dofs_str += ', '
    thread_dofs_str += ' ] '
    if not i >= len(thread_dofs) - 1:
        thread_dofs_str += ', '
thread_dofs_str += ' ] '

out_file.write('thread_dofs: {}\n'.format(thread_dofs_str))
'''


# AUTO############################################################################################################################################
##################################################################################################################################################

vars = RelaxedIK_vars('', path_to_src + '/RelaxedIK/urdfs/' + urdf_file_name, joint_names, ee_fixed_joints, joint_ordering, pre_config=True, init_state=starting_config, path_to_src=path_to_src)


robot = vars.robot
num_chains = robot.numChains



out_file.write('axis_types: [ ')
for i in range(num_chains):
    out_file.write('[ ')
    chain_len = len(robot.arms[i].axes)
    for j in range(chain_len):
        out_file.write('\"{}\"'.format(robot.arms[i].axes[j]))
        if not j == chain_len - 1:
            out_file.write(', ')
    out_file.write(' ]')
    if not i == num_chains - 1:
        out_file.write(', ')
out_file.write(' ]\n')

out_file.write('velocity_limits: [ ')
vels = robot.velocity_limits
for i in range(len(vels)):
    out_file.write('{}'.format(vels[i]))
    if not i == len(vels) - 1:
        out_file.write(', ')
out_file.write(' ]\n')

out_file.write('joint_limits: [ ')
joint_limits = robot.bounds
for i in range(len(vels)):
    out_file.write('[{},{}]'.format(joint_limits[i][0], joint_limits[i][1]))
    if not i == len(vels) - 1:
        out_file.write(', ')
out_file.write(' ]\n')


out_file.write('displacements: [ ')
for i in range(num_chains):
    out_file.write('[ ')
    chain_len = len(robot.arms[i].displacements)
    for j in range(chain_len):
        d = robot.arms[i].displacements[j]
        out_file.write('[{},{},{}]'.format(d[0], d[1], d[2]))
        if not j == chain_len - 1:
            out_file.write(', ')
    out_file.write(' ]')
    if not i == num_chains - 1:
        out_file.write(', ')
out_file.write(' ]\n')



out_file.write('disp_offsets: [ ')
for i in range(num_chains):
    d = robot.arms[i].dispOffset
    out_file.write('[{},{},{}]'.format(d[0], d[1], d[2]))
    if not i == num_chains - 1:
        out_file.write(', ')
out_file.write(' ]\n')


out_file.write('rot_offsets: [ ')
for i in range(num_chains):
    out_file.write('[ ')
    chain_len = len(robot.arms[i].original_rotOffsets)
    for j in range(chain_len):
        d = robot.arms[i].original_rotOffsets[j]
        out_file.write('[{},{},{}]'.format(d[0], d[1], d[2]))
        if not j == chain_len - 1:
            out_file.write(', ')
    out_file.write(' ]')
    if not i == num_chains - 1:
        out_file.write(', ')
out_file.write(' ]\n')


out_file.write('joint_types: [ ')
for i in range(num_chains):
    out_file.write('[ ')
    chain_len = len(robot.arms[i].joint_types)
    for j in range(chain_len):
        out_file.write('\"{}\"'.format(robot.arms[i].joint_types[j]))
        if not j == chain_len - 1:
            out_file.write(', ')
    out_file.write(' ]')
    if not i == num_chains - 1:
        out_file.write(', ')
out_file.write(' ]\n')


joint_state_define_func_file_name = robot_name + '_joint_state_define'
fp = path_to_src + '/RelaxedIK/Config/joint_state_define_functions/' + joint_state_define_func_file_name
js_file = open(fp, 'w')
js_file.write(inspect.getsource(joint_state_define))
out_file.write('joint_state_define_func_file: \"{}\"\n'.format(joint_state_define_func_file_name))

out_file.close()

print bcolors.OKGREEN + 'info file {} successfully created!'.format(info_file_name) + bcolors.ENDC

'''
in_file = open(path_to_src + '/RelaxedIK/Config/info_files/' + info_file_name, 'r')

import yaml

y = yaml.load(in_file)
'''
