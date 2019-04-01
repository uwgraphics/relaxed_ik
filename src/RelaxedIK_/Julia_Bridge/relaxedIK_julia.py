
class RelaxedIK_Julia:
    def __init__(self, path_to_src):
        self.path_to_src = path_to_src
        self.curr_send_idx = 0
        self.prev_solution = []

    def get_solution(self):
        f = open(self.path_to_src + '/RelaxedIK/Config/solution', 'r')
        line = f.readline()

        if line == '':
            return None

        f.close()
        line_split = line.split(',')

        # recv_idx = int(line_split[0])
        # if self.curr_send_idx == recv_idx:
        num_dof = len(line_split) - 2
        x = num_dof * [0.0]
        try:
            for i in xrange(1, num_dof+1):
                x[i-1] = float(line_split[i])
            self.prev_solution = x
            return x
        except:
            return self.prev_solution
        # else:
        #     return None


    def send_goals(self, pos_goals, quat_goals):
        f = open(self.path_to_src + '/RelaxedIK/Config/goals', 'w')
        f.write('{}\n'.format(self.curr_send_idx))
        num_goals = len(pos_goals)
        for i in xrange(num_goals):
            f.write('{},{},{};{},{},{},{}\n'.format(pos_goals[i][0], pos_goals[i][1], pos_goals[i][2], quat_goals[i][0],
                                                    quat_goals[i][1], quat_goals[i][2], quat_goals[i][3]))
        f.close()

    def solve(self, pos_goals, quat_goals):
        self.curr_send_idx += 1
        self.send_goals(pos_goals, quat_goals)

        xopt = self.get_solution()
        while xopt == None:
            xopt = self.get_solution()

        return xopt

    def reset(self, num_chains):
        pos_goals = []
        quat_goals = []
        for i in xrange(num_chains):
            pos_goals.append([0.,0.,0.])
            quat_goals.append([1.,0.,0.,0.])
        self.send_goals(pos_goals, quat_goals)




