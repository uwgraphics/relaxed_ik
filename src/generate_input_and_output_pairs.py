#! /usr/bin/env python
'''
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 10/30/19

PLEASE DO NOT CHANGE CODE IN THIS FILE.  IF TRYING TO SET UP RELAXEDIK, PLEASE REFER TO start_here.py INSTEAD
AND FOLLOW THE STEP-BY-STEP INSTRUCTIONS THERE.  Thanks!
'''

######################################################################################################

import rospy
import os
from RelaxedIK.Utils.colors import bcolors
from RelaxedIK.relaxedIK import get_relaxedIK_from_info_file, get_relaxedIK_yaml_obj
import numpy.random as r
import numpy as np
import pickle


def frames_to_jt_pt_vec(all_frames):
    out_vec = []
    for frames in all_frames:
        jt_pts = frames[0]
        for j in jt_pts:
            out_vec.append(j[0])
            out_vec.append(j[1])
            out_vec.append(j[2])

    return out_vec

def rand_vec(relaxedIK):
    bounds = relaxedIK.vars.bounds
    vec = []
    for b in bounds:
        b1 = b[0]
        b2 = b[1]
        rand = r.uniform(low=b1, high=b2, size=(1))[0]
        vec.append(rand)

    return np.array(vec)

def get_input_output_pair(relaxedIK):
    rv = rand_vec(relaxedIK)
    frames = relaxedIK.vars.robot.getFrames(rv)
    jt_pt_vec = frames_to_jt_pt_vec(frames)
    collision_score = relaxedIK.vars.collision_graph.get_collision_score(frames)
    condition_score = relaxedIK.vars.robot.getMatrixConditioningMeasure(rv)
    yoshiwaka_score = relaxedIK.vars.robot.getYoshikawaMeasure(rv)
    return rv, jt_pt_vec, collision_score, condition_score, yoshiwaka_score

def get_collision_score(relaxedIK, state):
    frames = relaxedIK.vars.robot.getFrames(state)
    return relaxedIK.vars.collision_graph.get_collision_score(frames)

def list_of_values_to_string(list):
    out_str = '[ '
    for i in xrange(len(list) - 1):
        out_str += str(list[i])
        out_str += ', '
    out_str += str(list[-1])
    out_str += ' ]'
    return out_str

def list_of_list_of_values_to_string(list):
    out_str = '[ '
    if type(list[0]) == float or type(list[0]) == np.float64:
        return list_of_values_to_string(list)
    else:
        for i in xrange(len(list) - 1):
            out_str += list_of_list_of_values_to_string(list[i]) + ', '
        out_str += list_of_list_of_values_to_string(list[-1])

    out_str += ' ]'
    return out_str

def get_highest_file_number(files):
    if len(files) == 0:
        return -1

    highest = 0
    for f in files:
        s = f.split('.')
        n = int(s[0])
        if n > highest:
            highest = n

    return highest

total_number_of_examples = 200000
lines_per_file = 1000
if __name__ == '__main__':
    rospy.init_node('inupt_and_output_pairs_node')
    rospy.sleep(0.3)

    path_to_src = os.path.dirname(__file__)

    relaxedIK = get_relaxedIK_from_info_file(path_to_src)
    y = get_relaxedIK_yaml_obj(path_to_src)
    num_chains = relaxedIK.vars.robot.numChains

    robot_name = y['urdf_file_name'].split('.')[0]

    top_dir = path_to_src + '/RelaxedIK/Config/collision_inputs_and_outputs'
    dirs = os.walk(top_dir).next()

    dir_found = False
    for d in dirs[1]:
        if d == robot_name:
            dir_found = True

    if not dir_found:
        os.mkdir(top_dir + '/' + robot_name)

    top_dir = path_to_src + '/RelaxedIK/Config/collision_inputs_and_outputs/' + robot_name

    files = []
    # r=root, d=directories, f = files
    for rr, d, f in os.walk(top_dir):
        for file in f:
            files.append(file)

    highest_file = get_highest_file_number(files)
    file_idx = highest_file + 1

    total_example_count = 0

    while total_example_count < total_number_of_examples:
        out_file = open(top_dir + '/{}.pkl'.format(file_idx), 'w')
        states = []
        jt_pts = []
        collision_scores = []
        condition_scores = []
        yoshiwaka_scores = []
        for j in xrange(lines_per_file):
            print 'file {}, line {}'.format(file_idx, j)
            tup = get_input_output_pair(relaxedIK)
            states.append(tup[0])
            jt_pts.append(tup[1])
            collision_scores.append(tup[2])
            condition_scores.append(tup[3])
            yoshiwaka_scores.append(tup[4])
            total_example_count += 1

        d = (states, jt_pts, collision_scores, condition_scores, yoshiwaka_scores)
        pickle.dump(d, out_file)
        # out_file.write('states: {}\n'.format(list_of_list_of_values_to_string(states)))
        # out_file.write('jt_pts: {}\n'.format(list_of_list_of_values_to_string(jt_pts)))
        # out_file.write('collision_scores: {}'.format(str(collision_scores)))
        file_idx += 1



