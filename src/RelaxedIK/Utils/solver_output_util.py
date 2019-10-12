
class SolverOutputUtil:
    def __init__(self, path_to_src, solver, robot, task,overwrite=True):
        self.robot = robot
        self.task = task
        self.solver = solver

        from os import listdir
        from os.path import isfile, join

        dir = path_to_src + '/RelaxedIK/FileIO/solver_outputs/' + solver + '/' + robot

        onlyfiles = [f for f in listdir(dir) if isfile(join(dir, f))]

        if not overwrite:
            highest_matching_number = 0
            for f in onlyfiles:
                print f
                str_arr = f.split('_')
                print len(str_arr)
                if task == str_arr[0] and not len(str_arr) == 1:
                    suffix_num = int(str_arr[1])
                    if suffix_num > highest_matching_number:
                        highest_matching_number = suffix_num

            suffix_num = str(highest_matching_number + 1)
            fp = dir + '/' + task + '_' + suffix_num
        else:
            fp = dir + '/' + task

        self.file = open(fp,'w')


    def close(self):
        self.file.close()


    def add_line(self, time_stamp, xopt, pos_goal, quat_goal):
        xopt_str = ','.join(map(str, xopt))
        line = '{};{};{},{},{};{},{},{},{}\n'.format(time_stamp, xopt_str,
                                                pos_goal[0],pos_goal[1],pos_goal[2],
                                                quat_goal[0],quat_goal[1],quat_goal[2],quat_goal[3])

        self.file.write(line)

if __name__ == '__main__':
    su = SolverOutputUtil('relaxed_ik', 'ur5', 'test')
    xopt = [0,1,2,3,4,5,6]
    su.add_line(0.0,xopt,[0,0,0],[1,0,0,0])
