import Arm_ext
from arm import UR5, Mico, IIWA7, Hubo_R
import numpy as np
import time

a = Hubo_R()
axes = a.axes

state = [0.0,0.3,0.5,0.1,0.1,0.2,0.1]

ac = Arm_ext.Arm(a.axes, a.displacements, a.rotOffsets, a.dispOffset, 'name')

test_size = 1000
# print a.dispOffset

start = time.clock()
for i in xrange(test_size):
    frames = a.getFrames(state)
end = time.clock()

print end - start

start = time.clock()
for i in xrange(test_size):
    frames = ac.getFrames(state)
end = time.clock()

print end - start

