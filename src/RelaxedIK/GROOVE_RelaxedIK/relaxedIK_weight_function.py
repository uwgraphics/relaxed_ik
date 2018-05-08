from ..GROOVE.GROOVE_Utils.weight_function import Weight_Function, Identity_Weight

class Position_Weight(Weight_Function):
    def name(self): return 'Position_weight'
    def __call__(self, vars):
        dof = vars.arm.numDOF
        max_val = dof * 10.0
        return max((max_val - vars.joint_limit_obj_value) / max_val, 0.5)


