#! /usr/bin/env python

import julia
import os
from start_here import urdf_file_name, joint_names, joint_ordering, ee_fixed_joints, starting_config, \
    joint_state_define, collision_file_name, fixed_frame
# from julia import Julia

path_to_src = os.path.dirname(__file__)

j = julia.Julia()

fn = j.include(path_to_src + '/relaxedIK_wrapper.jl')

goal_positions = [[0.0,0.0,0.0]]
goal_quats = [ [1.0,0.0,0.0,0.0] ]

import time

for i in xrange(1000):
    idx = i*0.001
    goal_positions = [[idx, 0.0, 0.0]]
    start = time.time()
    fn(goal_positions, goal_quats)
    end = time.time()
    print end - start



