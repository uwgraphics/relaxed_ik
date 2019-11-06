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
                sample_idx += 1
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

    def train_nn(self):
        clf = MLPRegressor(solver='adam', alpha=1,
                                hidden_layer_sizes=(15,15,15,15,15), max_iter=300000, verbose=True,
                                learning_rate='adaptive')
        self.clf = clf

        self.clf.fit(self.states, self.collision_scores)

        top_dir = path_to_src + '/RelaxedIK/Config/collision_nn_rust'
        f1 = open(top_dir + '/' + self.y['collision_nn_file'], 'w')
        joblib.dump(self.clf, f1)

    def evaluate(self, num_samples=1000):
        for i in xrange(num_samples):
            rvec = rand_vec(self.relaxedIK)
            pred = self.clf.predict([rvec])
            ground_truth = get_collision_score(self.relaxedIK, rvec)
            print '{}, {}'.format(pred, ground_truth)
            print

    def load_clf(self):
        top_dir = path_to_src + '/RelaxedIK/Config/collision_nn_rust'
        try:
            f1 = open(top_dir + '/' + self.y['collision_nn_file'], 'r')
            self.clf = joblib.load(f1)
            return True
        except:
            return False

    def dump_yaml(self):
        top_dir = path_to_src + '/RelaxedIK/Config/collision_nn_rust'
        f1 = open(top_dir + '/' + self.y['collision_nn_file'] + '.yaml', 'w')
        out_str = ''
        out_str += 'coefs: '
        out_str +=  list_of_list_of_values_to_string(self.clf.coefs_)
        out_str += '\n'
        out_str += 'intercepts: '
        out_str += list_of_list_of_values_to_string(self.clf.intercepts_)
        f1.write(out_str)
        f1.close()

    def load_yaml(self):
        top_dir = path_to_src + '/RelaxedIK/Config/collision_nn_rust'
        f1 = open(top_dir + '/' + self.y['collision_nn_file'] + '.yaml', 'r')
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
    p.load_input_and_output_pairs(100000)
    p.train_nn()
    # p.load_input_and_output_pairs(100000)
    # p.train_nn()
    # p.evaluate()
    # p.load_clf()
    # p.load_yaml()
    p.dump_yaml()
    #print p.clf.get_params
    # p.load_yaml()
    # x = [0.0,0.0,0.0,0.,0.,0.0, 0.0]

    # print p.clf.predict([x])
    # print p.gradient(x)
    # print p.manual_predict(x)
    # print p.gradient(x)

    # idx = 0
    # print p.coefs[idx]
    # print np.array(p.coefs[idx]).shape
    # p.load_clf()
    # p.dump_yaml()
    # p.load_input_and_output_pairs(100000)
    # p.train_nn()
    # p.load_clf()
    # p.evaluate()
    # p.dump_yaml()

    '''
    rvec = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.000262779999999907, 0.0, 0.0, 0.0, 0.0, 0.0]
    pred = p.clf.predict([rvec])
    ground_truth = get_collision_score(p.relaxedIK, rvec)

    print pred
    print ground_truth
    '''




