from GROOVE.groove import get_groove
import Utils.transformations as T
import math as M
import numpy as np
import numpy.random as r
from RelaxedIK.Utils.filter import EMA_filter

def rand_vec(bounds):
    vec = []
    for b in bounds:
        b1 = b[0]
        b2 = b[1]
        rand = r.uniform(low=b1, high=b2, size=(1))[0]
        vec.append(rand)

    return vec


class RelaxedIK(object):
    def __init__(self, vars, optimization_package='scipy', solver_name='slsqp'):
        self.vars = vars
        self.optimization_package = optimization_package
        self.solver_name = solver_name
        self.groove = get_groove(vars, optimization_package,solver_name)
        self.filter = EMA_filter(self.vars.init_state,a=0.5)


    @classmethod
    def init_from_config(self, config_name):
        import os
        from sklearn.externals import joblib
        from RelaxedIK.GROOVE_RelaxedIK.relaxedIK_vars import RelaxedIK_vars
        dirname = os.path.dirname(__file__)
        path = os.path.join(dirname, 'Config/{}'.format(config_name))
        file = open(path,'r')
        self.config_data = joblib.load(file)

        self.robot_name = self.config_data[0]
        self.collision_nn = self.config_data[1]
        self.init_state = self.config_data[2]
        self.full_joint_lists = self.config_data[3]
        self.fixed_ee_joints = self.config_data[4]
        self.joint_order = self.config_data[5]
        self.urdf_path = self.config_data[6]
        self.collision_file = self.config_data[7]

        vars = RelaxedIK_vars(self.robot_name,self.urdf_path,self.full_joint_lists,self.fixed_ee_joints,self.joint_order,
                              config_file_name=config_name, collision_file=self.collision_file,init_state=self.init_state)
        return RelaxedIK(vars)


    def solve(self, goal_positions, goal_quats, prev_state=None, vel_objectives_on=True, unconstrained=True, verbose_output=False, max_iter=11, maxtime=.05, rand_start=False):

        if self.vars.rotation_mode == 'relative':
            self.vars.goal_quats = []
            for i, q in enumerate(goal_quats):
                self.vars.goal_quats.append(T.quaternion_multiply(q, self.vars.init_ee_quats[i]))
        elif self.vars.rotation_mode == 'absolute':
            self.vars.goal_quats = []
            for i, q in enumerate(goal_quats):
                self.vars.goal_quats.append(q)

        self.vars.vel_objectives_on = vel_objectives_on
        self.vars.unconstrained = unconstrained

        # flip goal quat if necessary
        for i, j in enumerate(goal_quats):
            disp = np.linalg.norm(T.quaternion_disp(self.vars.prev_goal_quats[i], self.vars.goal_quats[i]))
            q = self.vars.goal_quats[i]
            if disp > M.pi / 2.0:
                self.vars.goal_quats[i] = [-q[0], -q[1], -q[2], -q[3]]

        if self.vars.position_mode == 'relative':
            self.vars.goal_positions = []
            for i, p in enumerate(goal_positions):
                self.vars.goal_positions.append(np.array(p) + self.vars.init_ee_positions[i])
        elif self.vars.position_mode == 'absolute':
            self.vars.goal_positions = []
            for i, p in enumerate(goal_positions):
                self.vars.goal_positions.append(np.array(p))

        if prev_state == None:
            initSol = self.vars.xopt
        else:
            initSol = prev_state

        if rand_start:
            initSol = rand_vec(self.vars.arm.joint_limits)
            self.reset(initSol)

        ################################################################################################################
        if self.optimization_package == 'scipy':
            xopt = self.groove.solve(prev_state=initSol,max_iter=max_iter,verbose_output=verbose_output)
        elif self.optimization_package == 'nlopt':
            xopt = self.groove.solve(prev_state=initSol,maxtime=maxtime,verbose_output=verbose_output)
        else:
            raise Exception('Invalid optimization package in relaxedIK.  Valid inputs are [scipy] and [nlopt].  Exiting.')
        ################################################################################################################

        xopt = self.filter.filter(xopt)
        self.vars.relaxedIK_vars_update(xopt)

        return xopt


    def reset(self, reset_state):
        self.vars.xopt = reset_state
        self.vars.prev_state = reset_state
        self.vars.prev_state2 = reset_state
        self.vars.prev_state3 = reset_state
        self.filter = EMA_filter(reset_state, a=0.5)

        self.vars.init_ee_pos = self.vars.arm.getFrames(reset_state)[0][-1]
        self.vars.init_ee_quat = T.quaternion_from_matrix(self.vars.arm.getFrames(reset_state)[1][-1])
        self.vars.ee_pos = self.vars.init_ee_pos
        self.vars.prev_ee_pos3 = self.vars.init_ee_pos
        self.vars.prev_ee_pos2 = self.vars.init_ee_pos
        self.vars.prev_ee_pos = self.vars.init_ee_pos

        self.vars.goal_pos = [0, 0, 0]
        self.vars.prev_goal_pos3 = self.vars.goal_pos
        self.vars.prev_goal_pos2 = self.vars.goal_pos
        self.vars.prev_goal_pos = self.vars.goal_pos
        self.vars.goal_quat = [1, 0, 0, 0]
        self.vars.prev_goal_quat3 = self.vars.goal_quat
        self.vars.prev_goal_quat2 = self.vars.goal_quat
        self.vars.prev_goal_quat = self.vars.goal_quat
