#! /usr/bin/env python
'''
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 11/2/19

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
from RelaxedIK.Utils.yaml_utils import list_of_list_of_values_to_string, list_of_values_to_string, list_of_values_to_csv_string, list_of_list_of_values_to_csv_string
import copy
import math
from sklearn import svm
import sys, string, os
import time
from RelaxedIK.Utils.file_utils import *
import yaml
import pickle
from sklearn.neural_network import MLPClassifier, MLPRegressor
from sklearn.externals import joblib

def relu(x):
    return max(0.0, x)

def relu_prime(x):
    if x <= 0.0:
        return 0.0
    else:
        return 1.0

def relu_jacobian(x):
    m = np.zeros((len(x), len(x)))
    for i in xrange(len(x)):
        m[i,i] = relu_prime(x[i])
    return m

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

def get_input_output_pair(relaxedIK):
    rv = rand_vec(relaxedIK)
    frames = relaxedIK.vars.robot.getFrames(rv)
    jt_pt_vec = frames_to_jt_pt_vec(frames)
    collision_score = relaxedIK.vars.collision_graph.get_collision_score(frames)
    return rv, jt_pt_vec, collision_score

def get_collision_score(relaxedIK, state):
    frames = relaxedIK.vars.robot.getFrames(state)
    return relaxedIK.vars.collision_graph.get_collision_score(frames)

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

def get_random_normal(mean, sigma, dim):
    return np.random.normal(mean, sigma, dim)

def unspin(state, relaxedIK):
    bounds = relaxedIK.vars.bounds
    new_state = copy.deepcopy(state)
    for i in xrange(len(state)):
        while new_state[i] < bounds[i][0]:
            new_state[i] += 2.0 * math.pi
        while new_state[i] > bounds[i][1]:
            new_state[i] -= 2.0 * math.pi
    return new_state

def get_split_point(list, split_percent=0.05):
    list = np.array(list)
    list.sort()
    idx = np.ceil(len(list) * split_percent)
    return list[int(idx)]

def interpolate_singularity_score(val, split_point):
    if val == 0.0:
        return 50.0
    else:
        v = 1.0/(split_point**2 * 2.0)
        return min(1.0 / v * val**2, 50.0)

class PreprocessorEngine:
    def __init__(self, path_to_src, mode='nn', info_file_name=''):
        self.path_to_src = path_to_src
        self.mode = mode
        if info_file_name == '':
            relaxedIK = get_relaxedIK_from_info_file(path_to_src)
        else:
            relaxedIK = get_relaxedIK_from_info_file(path_to_src, info_file_name)
        self.relaxedIK = relaxedIK
        y = get_relaxedIK_yaml_obj(path_to_src)
        self.y = y
        self.robot_name = y['urdf_file_name'].split('.')[0]

        self.states = []
        self.jt_pts = []
        self.collision_scores = []
        self.in_collision = []
        self.condition_scores = []
        self.yoshikawa_scores = []
        self.singularity_scores = []
        self.in_singularity = []

    def load_input_and_output_pairs(self, num_samples=40000):
        dir = self.path_to_src + '/RelaxedIK/Config/collision_inputs_and_outputs/' + self.robot_name
        files = get_all_files_in_dir(dir)
        highest_number = get_highest_file_number(files)
        chosen_idxs = []

        sample_idx = 0
        while sample_idx < num_samples:
            print 'sample {} of {}'.format(sample_idx, num_samples)
            file_idx = np.random.randint(0, highest_number)
            try_idx = 0
            while file_idx in chosen_idxs and try_idx < 100:
                file_idx = np.random.randint(0, highest_number)
                try_idx += 1
            chosen_idxs.append(file_idx)

            pk = pickle.load(open(dir + '/{}.pkl'.format(file_idx)))
            for i in xrange(len(pk[0])):
                self.states.append(pk[0][i])
                self.jt_pts.append(pk[1][i])
                self.collision_scores.append(pk[2][i])
                if pk[2][i] >= 5.0:
                    self.in_collision.append(-1.)
                else:
                    self.in_collision.append(1.)
                self.condition_scores.append(pk[3][i])
                self.yoshikawa_scores.append(pk[4][i])
                sample_idx += 1

        split_point = get_split_point(self.yoshikawa_scores)
        for i in xrange(len(self.yoshikawa_scores)):
            self.singularity_scores.append(interpolate_singularity_score(self.yoshikawa_scores[i], split_point))
            if self.yoshikawa_scores[i] < split_point:
                self.in_singularity.append(-1.)
            else:
                self.in_singularity.append(1.)

        print bcolors.OKGREEN + 'samples all loaded.' + bcolors.ENDC

    def generate_input_and_output_pairs(self, num_samples=40000):
        for i in xrange(num_samples):
            print 'sample {} of {}'.format(i, num_samples)
            rv, jt_pt_vec, collision_score = get_input_output_pair(self.relaxedIK)
            self.states.append(rv)
            self.jt_pts.append(jt_pt_vec)
            self.collision_scores.append(collision_score)
            if collision_score >= 5.0:
                self.in_collision.append(-1.)
            else:
                self.in_collision.append(1.)

    def train_nn(self, layer_width = 15, num_layers = 5, input_samples=200000, init_load=True):
        if init_load:
            self.load_input_and_output_pairs(input_samples)
        hls = []
        for i in xrange(num_layers):
            hls.append(layer_width)
        hls = tuple(hls)
        clf = MLPRegressor(solver='adam', alpha=1,
                                hidden_layer_sizes=hls, max_iter=300000, verbose=True,
                                learning_rate='adaptive')

        clf.fit(self.states, self.collision_scores)

        self.clf = clf
        t1 = self.find_optimal_split_point(2000)
        print t1
        self.dump_yaml(t1[0])
        self.dump_clf()

        return clf

    def train_jointpoint_nn(self, layer_width = 15, num_layers = 5, input_samples=200000, init_load=True):
        if init_load:
            self.load_input_and_output_pairs(input_samples)
        hls = []
        for i in xrange(num_layers):
            hls.append(layer_width)
        hls = tuple(hls)
        clf = MLPRegressor(solver='adam', alpha=1,
                                hidden_layer_sizes=hls, max_iter=300000, verbose=True,
                                learning_rate='adaptive')

        clf.fit(self.jt_pts, self.collision_scores)

        self.clf = clf
        t1 = self.find_optimal_split_point_jointpoint(2000)
        print t1
        self.dump_yaml(split_point=t1[0], suffix='_jointpoint')
        self.dump_clf(suffix='_jointpoint')

        return clf

    def train_singularity_nn(self, layer_width = 10, num_layers = 5, input_samples=200000):
        self.load_input_and_output_pairs(input_samples)
        hls = []
        for i in xrange(num_layers):
            hls.append(layer_width)
        hls = tuple(hls)
        clf = MLPRegressor(solver='adam', alpha=1,
                           hidden_layer_sizes=hls, max_iter=300000, verbose=True,
                           learning_rate='adaptive')

        clf.fit(self.states, self.singularity_scores)

        self.clf = clf
        # t1 = self.find_optimal_split_point(2000)
        # print t1
        # self.dump_yaml(t1[0])
        # self.dump_clf()

        return clf

    def evaluate(self, num_samples=1000, split_point = 5.0):
        false_positive_count = 0.
        false_negative_count = 0.
        true_positive_count = 0.
        true_negative_count = 0.
        total_count = 0.

        false_positive_magnitudes = []
        false_negative_magnitudes = []

        for i in xrange(num_samples):
            rvec = rand_vec(self.relaxedIK)
            pred = self.clf.predict([rvec])
            ground_truth = get_collision_score(self.relaxedIK, rvec)
            if pred >= split_point and ground_truth >= 5.0:
                true_positive_count += 1.0
            elif pred < split_point and ground_truth < 5.0:
                true_negative_count += 1.0
            elif pred >= split_point and ground_truth < 5.0:
                false_negative_count += 1.0
                false_negative_magnitudes.append(abs(pred - ground_truth))
            else:
                false_positive_count += 1.0
                false_positive_magnitudes.append(abs(pred - ground_truth))

            total_count += 1.0

        # mean_false_positive_magnitude = np.array(false_positive_magnitudes).mean()
        # mean_false_negative_magnitude = np.array(false_negative_magnitudes).mean()
        # max_false_positive_magnitude = np.array(false_positive_magnitudes).max()
        # max_false_negative_magnitude = np.array(false_negative_magnitudes).max()

        return (false_positive_count / total_count, false_negative_count / total_count, true_positive_count / total_count, true_negative_count / total_count)

    def find_optimal_split_point(self, num_samples=1000):
        predictions = []
        ground_truths = []
        for i in xrange(num_samples):
            rvec = rand_vec(self.relaxedIK)
            pred = self.clf.predict([rvec])
            ground_truth = get_collision_score(self.relaxedIK, rvec)
            predictions.append(pred)
            ground_truths.append(ground_truth)

        best_split = 5.0
        best_split_score = 100000000000.
        split_point = 8.0
        best_false_positive_count = 0.0
        best_false_negative_count = 0.0

        while split_point > 0.0:
            false_positive_count = 0.
            false_negative_count = 0.
            total_count = 0.
            for i in xrange(num_samples):
                if predictions[i] >= split_point and ground_truths[i] < 5.0:
                    false_negative_count += 1.0
                elif predictions[i] < split_point and ground_truths[i] >= 5.0:
                    false_positive_count += 1.0
                total_count += 1.0

            split_score = 2.0*(false_positive_count / total_count) + (false_negative_count / total_count)
            if split_score < best_split_score:
                best_split_score = split_score
                best_split = split_point
                best_false_negative_count = false_negative_count
                best_false_positive_count = false_positive_count

            split_point -= 0.01

        return best_split, best_false_positive_count/ total_count, best_false_negative_count / total_count

    def find_optimal_split_point_jointpoint(self, num_samples=1000):
        predictions = []
        ground_truths = []
        for i in xrange(num_samples):
            rvec = rand_vec(self.relaxedIK)
            frames = self.relaxedIK.vars.robot.getFrames(rvec)
            jt_pt_vec = frames_to_jt_pt_vec(frames)
            pred = self.clf.predict([jt_pt_vec])
            ground_truth = get_collision_score(self.relaxedIK, rvec)
            predictions.append(pred)
            ground_truths.append(ground_truth)

        best_split = 5.0
        best_split_score = 100000000000.
        split_point = 8.0
        best_false_positive_count = 0.0
        best_false_negative_count = 0.0

        while split_point > 0.0:
            false_positive_count = 0.
            false_negative_count = 0.
            total_count = 0.
            for i in xrange(num_samples):
                if predictions[i] >= split_point and ground_truths[i] < 5.0:
                    false_negative_count += 1.0
                elif predictions[i] < split_point and ground_truths[i] >= 5.0:
                    false_positive_count += 1.0
                total_count += 1.0

            split_score = 3.0*(false_positive_count / total_count) + (false_negative_count / total_count)
            if split_score < best_split_score:
                best_split_score = split_score
                best_split = split_point
                best_false_negative_count = false_negative_count
                best_false_positive_count = false_positive_count

            split_point -= 0.01

        return best_split, best_false_positive_count/ total_count, best_false_negative_count / total_count

    def dump_clf(self, suffix=''):
        top_dir = self.path_to_src + '/RelaxedIK/Config/collision_nn_rust'
        f1 = open(top_dir + '/' + self.y['collision_nn_file'] + suffix, 'w')
        joblib.dump(self.clf, f1)

    def load_clf(self, suffix=''):
        top_dir = self.path_to_src + '/RelaxedIK/Config/collision_nn_rust'
        try:
            f1 = open(top_dir + '/' + self.y['collision_nn_file'] + suffix, 'r')
            self.clf = joblib.load(f1)
            return True
        except:
            return False

    def dump_yaml(self, split_point = 5.0, suffix=''):
        top_dir = self.path_to_src + '/RelaxedIK/Config/collision_nn_rust'
        f1 = open(top_dir + '/' + self.y['collision_nn_file'] + suffix + '.yaml', 'w')
        out_str = ''
        out_str += 'coefs: '
        out_str +=  list_of_list_of_values_to_string(self.clf.coefs_)
        out_str += '\n'
        out_str += 'intercepts: '
        out_str += list_of_list_of_values_to_string(self.clf.intercepts_)
        out_str += '\n'
        out_str += 'split_point: {}'.format(split_point)
        f1.write(out_str)
        f1.close()

    def load_yaml(self, suffix=''):
        top_dir = self.path_to_src + '/RelaxedIK/Config/collision_nn_rust'
        f1 = open(top_dir + '/' + self.y['collision_nn_file'] + suffix + '.yaml', 'r')
        y = yaml.load(f1)
        self.coefs = y['coefs']
        self.intercepts = y['intercepts']

    def manual_predict(self, x):
        xout = np.array(x)
        for i in xrange(len(self.coefs)):
            xout =  np.dot(xout, np.array(self.coefs[i])) + np.array(self.intercepts[i])
            for j in xrange(len(xout)):
                xout[j] = relu(xout[j])
        return xout

    def manual_predict2(self, x):
        pass

    def gradient(self, x):
        xout = np.array(x)
        grad = []
        for i in xrange(len(self.coefs)):
            xout =  np.dot(xout, np.array(self.coefs[i])) + np.array(self.intercepts[i])
            for j in xrange(len(xout)):
                xout[j] = relu(xout[j])
            dr = relu_jacobian(xout)
            dc = np.array(self.coefs[i]).T
            if len(grad) == 0:
                grad = np.dot(dr, dc)
            else:
                grad = np.dot(np.dot(dr, dc), grad)

        return grad[0]

    def finite_diff(self, x, h=0.00000001):
        grad = np.array(x)
        f_0 = self.manual_predict(x)
        for i in xrange(len(grad)):
            x_h = copy.deepcopy(x)
            x_h[i] += h
            f_h = self.manual_predict(x_h)
            grad[i] = (-f_0 + f_h) / h
        return grad

if __name__== '__main__':
    rospy.init_node('inupt_and_output_pairs_node')
    rospy.sleep(0.3)

    path_to_src = os.path.dirname(__file__)
    p = PreprocessorEngine(path_to_src)
    p.train_nn()
    p.train_jointpoint_nn(init_load=False)






