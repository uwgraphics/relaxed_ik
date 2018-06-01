import Arm_ext
from arm import UR5, Mico, IIWA7, Hubo_R
import numpy as np
import time

a = Hubo_R()
axes = a.axes

state = [0, 0.78852112, -0.14649155, -0.17695991, -1.86374666, -0.09144152, -0.46006443, -0.1494651]

ac = Arm_ext.Arm(a.axes, a.displacements, a.rotOffsets, a.dispOffset, 'name')

print a.axes

test_size = 1
# print a.dispOffset

start = time.clock()
for i in xrange(test_size):
    frames = a.getFrames(state)
    print frames[0][0]
end = time.clock()

print end - start

start = time.clock()
for i in xrange(test_size):
    frames = ac.getFrames(state)
    print frames[0][0]
end = time.clock()

print end - start

