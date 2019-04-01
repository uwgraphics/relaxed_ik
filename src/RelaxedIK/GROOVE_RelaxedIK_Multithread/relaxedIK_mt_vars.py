from ..GROOVE.GROOVE_Utils.vars import Vars
from relaxedIK_mt_objective import objective_master_relaxedIK_mt
from ..GROOVE_RelaxedIK.relaxedIK_objective import Min_Jt_Vel_Obj, Min_Jt_Accel_Obj, Min_Jt_Jerk_Obj, Orientation_MultiEE_Obj, Position_MultiEE_Obj, Collision_Avoidance_nn
from ..GROOVE_RelaxedIK.relaxedIK_weight_function import Identity_Weight
from ..GROOVE_RelaxedIK_Multithread.relaxedIK_mt_utils import get_subchains_from_indices, glue_subchains
import numpy as np

class RelaxedIK_mt_vars(Vars):
    def __init__( self, relaxedIK_full,
                 mt_manager,
                 subchain_idx,
                 name='',
                 objective_function = objective_master_relaxedIK_mt,
                 init_state = '',
                 objectives=(Position_MultiEE_Obj(), Orientation_MultiEE_Obj(), Min_Jt_Vel_Obj(), Min_Jt_Accel_Obj(), Min_Jt_Jerk_Obj(), Collision_Avoidance_nn()),
                 weight_funcs=(Identity_Weight(), Identity_Weight(), Identity_Weight(), Identity_Weight(), Identity_Weight(), Identity_Weight()),
                 weight_priors=(50.0, 40.0, 10., 1., 1., 2.),
                 constraints=(),
                 bounds=(),
                 position_mode='relative',
                 rotation_mode='relative'
                 ):

        self.relaxedIK_full = relaxedIK_full
        self.mt_manager = mt_manager
        self.subchain_idx = subchain_idx
        self.num_chains = relaxedIK_full.vars.num_chains
        self.full_x = relaxedIK_full.vars.init_state
        self.multithread = True
        self.unconstrained = True
        self.collision_nn = self.relaxedIK_full.vars.collision_nn

        full_bounds = self.relaxedIK_full.vars.bounds
        subchain_indices = self.mt_manager.subchain_indices


        if len(bounds) == 0:
            bounds = []
            for i in subchain_indices[self.subchain_idx]:
                bounds.append(full_bounds[i])
            bounds = tuple(bounds)

        self.goal_positions = []
        for i in range(self.num_chains):
            self.goal_positions.append(np.array([0,0,0]))
        self.goal_quats = []
        for i in range(self.num_chains):
            self.goal_quats.append([1,0,0,0])

        self.goal_positions = self.goal_positions
        self.prev_goal_quats = self.goal_quats

        if init_state == '':
            self.init_state = mt_manager.subchains[subchain_idx]
        else:
            self.init_state = init_state

        self.c_boost = False

        self.init_ee_positions = self.relaxedIK_full.vars.robot.get_ee_positions(self.relaxedIK_full.vars.init_state)
        self.init_ee_quats = self.relaxedIK_full.vars.robot.get_ee_rotations(self.relaxedIK_full.vars.init_state)


        self.position_mode = position_mode
        self.rotation_mode = rotation_mode

        Vars.__init__(self,name, objective_function, self.init_state,objectives,weight_funcs,weight_priors,constraints,bounds)
