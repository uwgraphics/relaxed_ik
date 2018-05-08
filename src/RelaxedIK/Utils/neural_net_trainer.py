import numpy as np
from collision_graph import Collision_Graph
from sklearn.neural_network import MLPClassifier, MLPRegressor
from sklearn.externals import joblib
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

    return vec

class Collision_NN_Trainer:
    def __init__(self, collision_graph, num_samples=200000):
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
            print str(i) + ': ' + str(score)


        inputs_and_outputs = [self.inputs, self.outputs]
        file_name = self.robot.__name__ + '_' + str(time.time()) + '.pkl'
        # joblib.dump(inputs_and_outputs, file_name)

        self.clf = MLPRegressor(solver='adam', alpha=1,
                           hidden_layer_sizes=(70, 70, 70, 70, 70, 70), max_iter=3000, verbose=True,
                           learning_rate='adaptive')

        self.clf.fit(self.inputs, self.outputs)

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

if __name__ == '__main__':
    from GROOVE_RelaxedIK.relaxedIK_vars import RelaxedIK_vars
    from GROOVE_RelaxedIK.relaxedIK_objective import objective_master_relaxedIK
    from relaxedIK import RelaxedIK
    import rospy
    import Utils.collision_graph as cg

    training = False
    testing = True

    rospy.init_node('nn_test')

    urdf_string = '../urdfs/hubo_description.urdf'
    init_state = [0, 0.78852112, -0.14649155, -0.17695991, -1.86374666, -0.09144152, -0.46006443, -0.1494651]
    joint_names = (
        'WAIST', 'RIGHT_SHOULDER_PITCH', 'RIGHT_SHOULDER_ROLL', 'RIGHT_SHOULDER_YAW', 'RIGHT_ELBOW', 'RIGHT_WRIST_YAW',
        'RIGHT_WRIST_PITCH', 'RIGHT_WRIST_YAW_2')
    vars = RelaxedIK_vars('rik', objective_master_relaxedIK, urdf_string, joint_names, 'RIGHT_HAND',
                          init_state=init_state, position_mode='absolute')
    relaxedIK = RelaxedIK(vars, optimization_package='scipy', solver_name='slsqp')

    orig_ee_pos = vars.arm.getFrames(init_state)[0][-1]

    c = cg.Collision_Graph('../Config/collision_hubo.yaml', relaxedIK.vars.arm, link_exclusion_list=[0])
    ####################################################################################################################

    urdf_string = '../urdfs/sawyer.urdf'
    init_state = [0.00385192, -0.57307964, 0.3837615, 2.25103946, -2.92990265, 1.7088147, 2.84393652]
    joint_names = ('right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6')
    vars = RelaxedIK_vars('rik', objective_master_relaxedIK, urdf_string, joint_names, 'right_hand',
                          init_state=init_state, position_mode='absolute')
    relaxedIK = RelaxedIK(vars, optimization_package='scipy', solver_name='slsqp')

    orig_ee_pos = vars.arm.getFrames(init_state)[0][-1]

    c = cg.Collision_Graph('../Config/collision_sawyer.yaml', relaxedIK.vars.arm, link_exclusion_list=[])


    if training:

        nn = Collision_NN_Trainer(c, 100000)

    if testing:
        clf = joblib.load('sawyer_nn.pkl')

        distribution_arr = []

        num_samples = 100

        for i in xrange(num_samples):
            # print i
            rand = rand_vec(relaxedIK.vars.arm.joint_limits)
            frames = relaxedIK.vars.arm.getFrames(rand)
            jt_pt_vec = frames_to_jt_pt_vec(frames)
            predicted = clf.predict([jt_pt_vec])
            print predicted
            print c.get_collision_score(frames)
            print
            distribution_arr.append(predicted)
