import numpy as np
import RelaxedIK.Utils.transformations as T

R = T.random_rotation_matrix()
print R
arr = np.array([[1,2,3],[4,5,6],[7,8,9]])
new_mat = np.array(arr, dtype=np.float64, copy=False)[:4, :4]
print T.quaternion_from_matrix(R, isprecise=True)
print T.quaternion_from_matrix(R, isprecise=False)


quaternion = [1,1,1,0]
q = np.array(quaternion, dtype=np.float64, copy=True)
print np.negative(q[1:])
print q[1:]

rand= [-0.65206257,  0.3991842,  -0.51618717,  0.38602744]
rand2 = [-0.7590526,   0.60430621,  0.24175231, -0.01445611]

print rand2
print T.quaternion_disp(rand, rand2)