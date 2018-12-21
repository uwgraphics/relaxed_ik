from RelaxedIK.relaxedIK_multithread import RelaxedIK_multithread
import sys


if __name__ == '__main__':
    import rospy
    from RelaxedIK.relaxedIK import get_relaxedIK_from_info_file

    rospy.init_node('test')

    path_to_src = "/home/rakita/catkin_ws/src/relaxed_ik/src/"

    relaxedIK = get_relaxedIK_from_info_file(path_to_src, 'ur5_info.yaml')
    relaxedIK.vars.unconstrained = True

    rik_mt = RelaxedIK_multithread(  [ [0,1], [2,3], [4, 5] ], relaxedIK, solver_name='slsqp')

    # print rik_mt.relaxedIK_subchains[1].groove.solve()
    # print rik_mt.relaxedIK_subchains[1].vars.objectives

    idx = 0.0
    rate = rospy.Rate(100)
    for i in xrange(1000):
        print rik_mt.solve([[idx,0,0.0]], [[1.,0,0,0]])
        idx += 0.0001
        rate.sleep()