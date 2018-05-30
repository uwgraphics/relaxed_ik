import Arm_ext
from arm import UR5, Mico, IIWA7, Hubo_R
import numpy as np
import time

a = Hubo_R()
axes = a.axes

state = [0.1,0.3,0.5,0.1,0.1,0.2,0.1]

ac = Arm_ext.Arm(a.axes, a.displacements, a.rotOffsets, a.dispOffset)
# print ac.numDOF
# print ac.displacements
# print ac.dispOffset
# print ac.numDOF

# mat = np.array(ac.getFrames(state)[1][1])

test_size = 100000

start = time.time()
for i in xrange(test_size):
    ac.getFrames(state)
end = time.time()

print end - start

start = time.clock()
for i in xrange(test_size):
    a.getFrames(state)
end = time.clock()

print end - start
# pts = ac.getFrames(state)[0]
# print mat
# print np.array(pts[0])

# print ac.displacements

