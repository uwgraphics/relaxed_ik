from ..GROOVE.GROOVE_Utils.objective import Objective, objective_master
from relaxedIK_mt_utils import glue_subchains, get_subchains_from_indices, inject_subchain

def objective_master_relaxedIK_mt(x, vars):
    subchain_idx = vars.subchain_idx
    full_x = inject_subchain(x, subchain_idx, vars.mt_manager.subchain_indices, vars.mt_manager.subchains, vars.relaxedIK_full.vars.robot.numDOF)
    vars.frames = vars.relaxedIK_full.vars.robot.getFrames(full_x)
    return objective_master(x, vars)