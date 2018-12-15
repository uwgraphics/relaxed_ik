


class RelaxedIK_MT_Central_Vars:
    def __init__(self, relaxedIK_full):
        self.relaxedIK_full = relaxedIK_full
        self.full_state = self.relaxedIK_full.vars.init_state
