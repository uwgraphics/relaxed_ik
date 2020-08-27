from RelaxedIK.relaxedIK import *
import os
import RelaxedIK.Utils.transformations as T

path_to_src = os.path.dirname(__file__)
relaxedIK = get_relaxedIK_from_info_file(path_to_src, 'ur5_info.yaml')
# x = [-0.36253979180999973, -1.1395895999999999, 1.1623892805000007, 0.6443846, 0.7099999388999998, 1.8286839999999995, 1.7957343587400003, 1.0291857521399996, 1.7371355999999998, -1.54000871703, -1.0539368, 1.1133804351600003, -1.0842, 2.9091147939]
x = 6 * [1.0]
print relaxedIK.vars.robot.getFrames(x)[0][1][6]

# print T.quaternion_from_matrix(relaxedIK.vars.robot.getFrames(x)[1][1][-1])