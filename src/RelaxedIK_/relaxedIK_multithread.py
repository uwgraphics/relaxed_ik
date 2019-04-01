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
import multiprocessing as mp
import rospy



class RelaxedIK_multithread(object):
    def __init__(self, subchain_indices, relaxedIK_full, optimization_package='scipy', solver_name='slsqp'):
        self.subchain_indices = subchain_indices
        self.relaxedIK_full = relaxedIK_full
        self.optimization_package = optimization_package
        self.solver_name = solver_name
        self.filter = EMA_filter(self.relaxedIK_full.vars.init_state,a=0.5)

        self.num_subchains = len(subchain_indices)
        self.relaxedIK_subchains = []
        self.mt_manager = Multithread_Manager(subchain_indices, relaxedIK_full)

        for i in xrange(self.num_subchains):
            mt_vars = RelaxedIK_mt_vars(relaxedIK_full, self.mt_manager, i)  ######################### if there were going to be different variants of solver per subchain, this is where it would be added!!!!!!  (e.g., spearate objectives for camera)
            relaxedIK_subchain = RelaxedIK_subchain(mt_vars, optimization_package=optimization_package, solver_name=solver_name)
            self.relaxedIK_subchains.append(relaxedIK_subchain)


        self.threads = []
        for r in self.relaxedIK_subchains:
            self.threads.append(threading.Thread(target=r.run))

        for t in self.threads:
            t.start()

        # r= self.relaxedIK_subchains[1]
        # threading.Thread(target=r.run).start()

        # r= self.relaxedIK_subchains[1]
        # mp.Process(target=r.run).start()


    def solve(self, goal_positions, goal_quats):
        r = self.relaxedIK_subchains[0]

        pos_goals = []
        quat_goals = []

        # for r in self.relaxedIK_subchains:
        if r.vars.rotation_mode == 'relative':
            # r.vars.goal_quats = []
            for i, q in enumerate(goal_quats):
                quat_goals.append(T.quaternion_multiply(q, r.vars.init_ee_quats[i]))
        elif r.vars.rotation_mode == 'absolute':
            # r.vars.goal_quats = []
            for i, q in enumerate(goal_quats):
                quat_goals.append(q)

        # flip goal quat if necessary
        for i, j in enumerate(goal_quats):
            disp = np.linalg.norm(T.quaternion_disp(r.vars.prev_goal_quats[i], r.vars.goal_quats[i]))
            q = r.vars.goal_quats[i]
            if disp > M.pi / 2.0:
                quat_goals[i] = [-q[0], -q[1], -q[2], -q[3]]

        if r.vars.position_mode == 'relative':
            # r.vars.goal_positions = []
            for i, p in enumerate(goal_positions):
                pos_goals.append(np.array(p) + r.vars.init_ee_positions[i])
        elif r.vars.position_mode == 'absolute':
            # r.vars.goal_positions = []
            for i, p in enumerate(goal_positions):
                pos_goals.append(np.array(p))

        # print r.vars.goal_positions
        for r in self.relaxedIK_subchains:
            r.vars.goal_positions = pos_goals
            r.vars.goal_quats = quat_goals

        # rospy.sleep(1.0)

        # for t in self.threads:
        #    t.join()

        # threads = []
        #for r in self.relaxedIK_subchains:
        #   t = threading.Thread(target=r.run)
        #   threads.append(t)
        #   t.start()

        #for t in threads:
        #   t.join()


        curr_target_idx = self.mt_manager.solution_count + 1
        move_on = False
        while not move_on:
            local_move_on = True
            for s in self.relaxedIK_subchains:
                if not curr_target_idx == s.solution_count:
                    local_move_on = False
            move_on = local_move_on


        self.mt_manager.subchains = self.mt_manager.subchains_write
        xopt = glue_subchains(self.mt_manager.subchain_indices, self.mt_manager.subchains, self.relaxedIK_full.vars.robot.numDOF)
        # xopt = self.filter.filter(xopt)
        self.mt_manager.locked_x = xopt

        self.mt_manager.solution_count += 1

        for r in self.relaxedIK_subchains:
            r.vars.prev_goal_quats = quat_goals
            r.vars.prev_goal_positions = pos_goals

        return xopt






