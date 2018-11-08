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
from RelaxedIK.Utils.colors import bcolors
from start_here import info_file_name, urdf_file_name, fixed_frame, joint_names

path_to_src = os.path.dirname(__file__)

if info_file_name == '':
    print bcolors.FAIL + 'info_file_name is a required field in start_here.py.  Please fill that in and run again.' + bcolors.ENDC
    exit(-1)

out_file = open(path_to_src + '/RelaxedIK/Config/info_files/' + info_file_name, 'w')
out_file.write('path_to_src: \"{}\"\n'.format(path_to_src))

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
    out_file.write('joint_names: [ {} ]'.format(chains_str))

out_file.close()





in_file = open(path_to_src + '/RelaxedIK/Config/info_files/' + info_file_name, 'r')

import yaml

y = yaml.load(in_file)





