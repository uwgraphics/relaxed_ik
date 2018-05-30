import Arm_ext
from arm import UR5
import numpy as np


a = UR5()
axes = a.axes

ac = Arm_ext.Arm(a.axes, a.displacements, a.rotOffsets, a.dispOffset)
# print ac.numDOF
# print ac.displacements
# print ac.dispOffset
mat= np.array(ac.getFrames([1,2,3,4,5,6])[1][0])
pts = ac.getFrames([1,1,1,1,1,1])[0]
print mat
print np.array(pts[0])

#print ac.dispOffset

