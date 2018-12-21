from RelaxedIK.relaxedIK_multithread import RelaxedIK_multithread
import numpy as np
import threading

def obj(x, vars, idx):
    frames = vars.robot.getFrames(x)
    print np.linalg.norm(frames[0][0][-1])
    print idx


if __name__ == '__main__':
    import rospy
    from RelaxedIK.relaxedIK import get_relaxedIK_from_info_file

    rospy.init_node('test')

    path_to_src = "/home/rakita/catkin_ws/src/relaxed_ik/src/"

    relaxedIK = get_relaxedIK_from_info_file(path_to_src, 'jaco7_info.yaml')

    obj_closure1 = lambda : obj(np.random.uniform(low=-6, high=6, size=7), relaxedIK.vars, 1)
    obj_closure2 = lambda : obj(np.random.uniform(low=-6, high=6, size=7), relaxedIK.vars, 2)
    obj_closure3 = lambda : obj(np.random.uniform(low=-6, high=6, size=7), relaxedIK.vars, 3)
    obj_closure4 = lambda : obj(np.random.uniform(low=-6, high=6, size=7), relaxedIK.vars, 4)
    obj_closure5 = lambda : obj(np.random.uniform(low=-6, high=6, size=7), relaxedIK.vars, 5)

    # rand = np.random.uniform(low=-6, high=6, size=7)

    # obj_closure()
    # obj_closure()

    for i in xrange(100):
        threading.Thread(target=obj_closure1).start()
        threading.Thread(target=obj_closure2).start()
        threading.Thread(target=obj_closure3).start()
        threading.Thread(target=obj_closure4).start()
        threading.Thread(target=obj_closure5).start()

    # print rand
    # print obj_closure(rand)
