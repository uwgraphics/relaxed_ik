from ..GROOVE.GROOVE_Utils.constraint import Constraint


class Singularity_Avoidance_Constraint(Constraint):
    def __init__(self, *args): pass
    def constraintType(self): return 'ineq'
    def name(self): return 'singularity_avoidance'
    def func(self, x, *args):
        # vars = get_groove_global_vars()
        mean = vars.yoshikawa_mean
        std = vars.yoshikawa_std
        min = mean - 2.6*std

        yoshikawa_score = vars.arm.getYoshikawaMeasure(x)

        return [yoshikawa_score - min]

class Joint_Velocity_Constraint(Constraint):
    def __init__(self, *args):
        self.joint_idx = args[0]
        self.velocity_scale = args[1]
    def constraintType(self): return 'ineq'
    def name(self): return 'joint_velocity_constraint_{}'.format(self.joint_idx)
    def func(self, x, *args):
        # vars = get_groove_global_vars()
        avg = vars.avg_solution_time
        avg = 0.02
        # print avg
        diff = abs(x[self.joint_idx] - vars.xopt[self.joint_idx])
        vel_limit = vars.robot.velocity_limits[self.joint_idx] * self.velocity_scale
        vel_limit_per_update = vel_limit * avg
        return vel_limit_per_update - diff
        # return .1*x[0]






