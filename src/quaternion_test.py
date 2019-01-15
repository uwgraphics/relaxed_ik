import RelaxedIK.Utils.transformations as T
import numpy as np

q1 = T.random_quaternion()

m = T.quaternion_matrix(q1)[:3,:3]
# m = np.identity(3)
# print m

axis, angle = T.quaternion_to_axisAngle(q1)

print axis

print np.dot(m, axis)

print q1
print T.rotate_quaternion_representation(q1, m)