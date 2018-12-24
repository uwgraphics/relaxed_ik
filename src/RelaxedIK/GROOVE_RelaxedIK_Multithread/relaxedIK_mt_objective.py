from ..GROOVE.GROOVE_Utils.objective import Objective, objective_master
from relaxedIK_mt_utils import glue_subchains, get_subchains_from_indices, inject_subchain, inject_state
import math
import numpy as np

def objective_master_relaxedIK_mt(x, vars):
    good_input = True
    for i in x:
       if math.isnan(i):
            # print 'yes'
            # x = vars.x_prev + np.random.uniform(-0.0001, 0.0001, len(x))
            good_input = False
            break

    subchain_idx = vars.subchain_idx
    # count = vars.mt_manager.solution_count
    # full_x = inject_subchain(x, subchain_idx, vars.mt_manager.subchain_indices, vars.mt_manager.subchains, vars.relaxedIK_full.vars.robot.numDOF)
    full_x = inject_state(vars.mt_manager.locked_x, x, vars.mt_manager.subchain_indices[subchain_idx])
    vars.frames = vars.relaxedIK_full.vars.robot.getFrames(full_x)

    vars.full_x = full_x
    if good_input:
        vars.x_prev = x
    return objective_master(x, vars)