import math
import numpy
from _transformations import *

# added by Danny Rakita
def quaternion_log(quaternion):
    """
    Returns the log map vec3 of quaternion

    quaternion list is [w x y z]
    :param quaternion:
    :return:
    """
    v = numpy.array([quaternion[1], quaternion[2], quaternion[3]])
    if abs(quaternion[0] < 1.0):
        a = 1
        try:
            a = math.acos(quaternion[0])
        except: pass
        sina = math.sin(a)
        if abs(sina >= 0.005):
            c = a/sina
            v[0] *= c
            v[1] *= c
            v[2] *= c

    return v


# added by Danny Rakita
def quaternion_exp(vec3):
    """
    Returns the exponentiated quaternion from rotation vector vec3
    :param vec3:
    :return: quaternion in format [w x y z]
    """
    q = numpy.array([1.0, vec3[0], vec3[1], vec3[2]])
    a = numpy.linalg.norm(q)
    sina = math.sin(a)
    if abs(sina) >= 0.005:
        c = sina/a
        q[1] *= c
        q[2] *= c
        q[3] *= c

    q[0] = math.cos(a)

    return q

# added by Danny Rakita
def quaternion_disp(q, qPrime):
    """
    finds rotation vector displacement between two quaternions (from q1 to q2)
    :param q1: input quaternions in form [w x y z]
    :param q2:
    :return:
    """
    inv = quaternion_inverse(q)
    m = quaternion_multiply(inv, qPrime)
    return quaternion_log(m)

def quaternion_dispQ(q,qPrime):
    inv = quaternion_inverse(q)
    return quaternion_multiply(inv, qPrime)


if __name__ == '__main__':
    q1 = random_quaternion()
    q2 = random_quaternion()
    print quaternion_disp(q1,q2)

