import numpy as np
# from numba import jit

def interpolate_to_joint_limits(from_q, to_q, t=0.1, numDOF=6, joint_limits=None):
    '''
    interpolates from state from_q to sate to_q going as fast as the joints can in time t
    :param from_q:
    :param to_q:
    :param time:
    :param joint_limits:
    :return:
    '''
    ret_q = []
    ret_k = []

    if joint_limits == None:
        limits = numDOF*[6.0]
    else:
        limits = joint_limits

    for i in xrange(numDOF):
        disp = to_q[i] - from_q[i]
        disp_norm = abs(disp)

        # how far can joint go in the amount of time?
        max_disp = limits[i]*t

        if disp_norm > max_disp:
            k = max_disp/disp_norm
        else:
            k = 1.0

        ret_k.append(k)
        interp = from_q[i] + k*(disp)
        ret_q.append(interp)

    return ret_q, ret_k

def check_legal_velocity(from_q, to_q, t=0.1, numDOF=6, joint_limits=None):
    q,k = interpolate_to_joint_limits(from_q,to_q,t,numDOF,joint_limits)
    if not k == numDOF*[1.0]:
        return False
    else:
        return True

if __name__ == '__main__':
    q,k= interpolate_to_joint_limits([-.1,.3,0],[-.2,.3,.1],numDOF=3, joint_limits=[3.15,6.0,3.15])
    print check_legal_velocity([-.1,.3,0],[-.1,.4,.07],numDOF=3, t=0.02,joint_limits=[3.15,6.0,3.15])

