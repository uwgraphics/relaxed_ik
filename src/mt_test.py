from RelaxedIK.relaxedIK_multithread import RelaxedIK_multithread



if __name__ == '__main__':
    import rospy
    from RelaxedIK.relaxedIK import get_relaxedIK_from_info_file

    rospy.init_node('test')

    path_to_src = "/home/rakita/catkin_ws/src/relaxed_ik/src/"

    relaxedIK = get_relaxedIK_from_info_file(path_to_src, 'jaco7_info.yaml')

    rik_mt = RelaxedIK_multithread([[0,1,2,3,4], [5], [6]], relaxedIK)

    # print rik_mt.relaxedIK_subchains[1].groove.solve()
    # print rik_mt.relaxedIK_subchains[1].vars.objectives

    print rik_mt.solve([[0.0,0.0,0.0]], [[1.,0,0,0]])

    rospy.signal_shutdown()
