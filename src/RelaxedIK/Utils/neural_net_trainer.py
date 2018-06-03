import numpy as np
from collision_graph import Collision_Graph
from sklearn.neural_network import MLPClassifier, MLPRegressor
from sklearn.externals import joblib
import pickle

import time
import numpy.random as r

def frames_to_jt_pt_vec(all_frames):
    out_vec = []
    for frames in all_frames:
        jt_pts = frames[0]
        for j in jt_pts:
            out_vec.append(j[0])
            out_vec.append(j[1])
            out_vec.append(j[2])

    return out_vec

def rand_vec(bounds):
    vec = []
    for b in bounds:
        b1 = b[0]
        b2 = b[1]
        rand = r.uniform(low=b1, high=b2, size=(1))[0]
        vec.append(rand)

    return np.array(vec)

class Collision_NN_Trainer:
    def __init__(self, collision_graph, num_samples=500000):
        self.num_samples = num_samples
        self.cg = collision_graph
        self.inputs = []
        self.outputs = []
        self.robot = self.cg.robot
        self.bounds = self.cg.robot.bounds

        for i in xrange(num_samples):
            rvec = rand_vec(self.bounds)
            frames = self.robot.getFrames(rvec)
            score = self.cg.get_collision_score(frames)
            input = frames_to_jt_pt_vec(frames)
            self.inputs.append(input)
            self.outputs.append(score)
            print str(i) + ' of ' + str(num_samples) + ' samples' + ': ' + str(score)


        inputs_and_outputs = [self.inputs, self.outputs]
        # file_name = self.robot.__name__ + '_' + str(time.time()) + '.pkl'
        # joblib.dump(inputs_and_outputs, file_name)


        self.clf = MLPRegressor(solver='adam', alpha=1,
                           hidden_layer_sizes=(70, 70, 70, 70, 70, 70), max_iter=3000, verbose=True,
                           learning_rate='adaptive')

        self.clf.fit(self.inputs, self.outputs)

        '''
        self.clf = Sequential()
        self.clf.add(Dense(units=70, activation='relu', input_dim=len(self.inputs[0])))
        self.clf.add(Dense(units=70, activation='relu'))
        self.clf.add(Dense(units=70, activation='relu'))
        self.clf.add(Dense(units=70, activation='relu'))
        self.clf.add(Dense(units=70, activation='relu'))
        self.clf.add(Dense(units=70, activation='relu'))
        self.clf.add(Dense(units=1, activation='relu'))

        self.clf.compile(optimizer='adam',loss='mean_squared_error')
        self.clf.fit(np.array(self.inputs), np.array(self.outputs))
        '''

        self.output_comparisons()

    def output_comparisons(self, num_samples=100):
        print 'output comparisons...'
        for i in xrange(num_samples):
            rand = rand_vec(self.robot.bounds)
            frames = self.robot.getFrames(rand)
            jt_pt_vec = frames_to_jt_pt_vec(frames)
            predicted = self.clf.predict([jt_pt_vec])
            print predicted
            print self.cg.get_collision_score(frames)
            print