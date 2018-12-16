from GROOVE.groove import get_groove
import Utils.transformations as T
import math as M
import numpy as np
import numpy.random as r
from RelaxedIK.Utils.filter import EMA_filter
from relaxedIK_subchain import RelaxedIK_subchain
from GROOVE_RelaxedIK_Multithread.relaxedIK_mt_manager import Multithread_Manager
from GROOVE_RelaxedIK_Multithread.relaxedIK_mt_vars import RelaxedIK_mt_vars
from GROOVE_RelaxedIK_Multithread.relaxedIK_mt_utils import glue_subchains
import threading

class RelaxedIK_multithread(object):
    def __init__(self, subchain_indices, relaxedIK_full, optimization_package='scipy', solver_name='slsqp'):
        self.subchain_indices = subchain_indices
        self.relaxedIK_full = relaxedIK_full
        self.optimization_package = optimization_package
        self.solver_name = solver_name
        self.filter = EMA_filter(self.relaxedIK_full.vars.init_state,a=0.5)

        self.num_subchains = len(subchain_indices)
        self.relaxedIK_subchains = []
        for i in xrange(self.num_subchains):
            mt_manager = Multithread_Manager(subchain_indices, relaxedIK_full)
            mt_vars = RelaxedIK_mt_vars(relaxedIK_full, mt_manager, i)  ######################### if there were going to be different variants of solver per subchain, this is where it would be added!!!!!!  (e.g., spearate objectives for camera)
            relaxedIK_subchain = RelaxedIK_subchain(mt_vars)
            self.relaxedIK_subchains.append(relaxedIK_subchain)

        for r in self.relaxedIK_subchains:
            threading.Thread(target=r.run).start()


    def solve(self, goal_positions, goal_quats):

        for r in self.relaxedIK_subchains:
            if r.vars.rotation_mode == 'relative':
                r.vars.goal_quats = []
                for i, q in enumerate(goal_quats):
                    r.vars.goal_quats.append(T.quaternion_multiply(q, r.vars.init_ee_quats[i]))
            elif r.vars.rotation_mode == 'absolute':
                r.vars.goal_quats = []
                for i, q in enumerate(goal_quats):
                    r.vars.goal_quats.append(q)

            # flip goal quat if necessary
            for i, j in enumerate(goal_quats):
                disp = np.linalg.norm(T.quaternion_disp(r.vars.prev_goal_quats[i], r.vars.goal_quats[i]))
                q = r.vars.goal_quats[i]
                if disp > M.pi / 2.0:
                    r.vars.goal_quats[i] = [-q[0], -q[1], -q[2], -q[3]]

            if r.vars.position_mode == 'relative':
                r.vars.goal_positions = []
                for i, p in enumerate(goal_positions):
                    r.vars.goal_positions.append(np.array(p) + r.vars.init_ee_positions[i])
            elif r.vars.position_mode == 'absolute':
                r.vars.goal_positions = []
                for i, p in enumerate(goal_positions):
                    r.vars.goal_positions.append(np.array(p))

            r.vars.prev_goal_quats = goal_quats
            r.vars.prev_goal_positions = goal_positions

        xopt = glue_subchains(self.relaxedIK_subchains[0].vars.mt_manager.subchain_indices, self.relaxedIK_subchains[0].vars.mt_manager.subchains, self.relaxedIK_full.vars.robot.numDOF)
        xopt = self.filter.filter(xopt)

        return xopt






