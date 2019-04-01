import boost.Arm_ext as Arm_ext
from arm import UR5, Mico, IIWA7, Hubo_R
import numpy as np
import time

a = Hubo_R()
axes = a.axes

state = [0, 0.78852112, -0.14649155, -0.17695991, -1.86374666, -0.09144152, -0.46006443, -0.1494651]


ac = Arm_ext.Arm(a.axes, a.displacements, a.rotOffsets, a.dispOffset, 'name')

# print a.axes

test_size = 10000
# print a.dispOffset

start = time.clock()
for i in xrange(test_size):
    frames = a.getFrames(state)
    ee_pos = frames[0][-1]
    # print frames[0][0]
end = time.clock()

print (end - start) / test_size

start = time.clock()
for i in xrange(test_size):
    frames = ac.getFrames(state)
    ee_pos_c = frames[0][-1]
    # print frames[0][0]
end = time.clock()

print (end - start) / test_size

print 'l1: ' + str(np.linalg.norm(ee_pos_c - ee_pos, ord=1))
print 'l2: ' + str(np.linalg.norm(ee_pos_c - ee_pos))
print 'linf: ' + str(np.linalg.norm(ee_pos_c - ee_pos, ord=np.inf))


