
from relaxedIK_mt_utils import get_subchains_from_indices, glue_subchains
import rospy
from relaxed_ik.msg import JointAngles
from std_msgs.msg import Float32

class Multithread_Manager:
    def __init__(self, subchain_indices, relaxedIK_full):
        self.subchain_indices = subchain_indices
        self.relaxedIK_full = relaxedIK_full
        self.num_subchains = len(subchain_indices)

        self.subchains = get_subchains_from_indices(subchain_indices, relaxedIK_full.vars.init_state)
        self.full_state = relaxedIK_full.vars.init_state

        for i in xrange(self.num_subchains):
            rospy.Subscriber('/subchain_{}'.format(i), JointAngles, self.subchain_cb)


    def subchain_cb(self, data):
        subchain_id = int(data.header.frame_id)

        for i, s in enumerate(self.subchains[subchain_id]):
            self.subchains[subchain_id][i] = float(data.angles[i].data)




if __name__ == '__main__':
    rospy.init_node('test')

    ja = JointAngles()
    ja.header.seq = 1
    ja.angles = [ Float32(4.0), Float32(2.0) ]

    a = [0,1,2,3]

    idx = ja.header.seq

    from RelaxedIK.relaxedIK import get_relaxedIK_from_info_file

    path_to_src = "/home/rakita/catkin_ws/src/relaxed_ik/src/"

    relaxedIK = get_relaxedIK_from_info_file(path_to_src, 'jaco7_info.yaml')

    subchain_indicies = [[0,1], [2,3], [4,5,6]]


    mm = Multithread_Manager(subchain_indicies, relaxedIK)


    print mm.num_subchains

    pub0 = rospy.Publisher('subchain_0', JointAngles, queue_size=3)
    pub1 = rospy.Publisher('subchain_1', JointAngles, queue_size=3)

    idx = 0.0
    while not rospy.is_shutdown():
        ja0 = JointAngles()
        ja0.header.frame_id = '0'
        ja0.angles = [Float32(idx), Float32(1.0)]


        ja1 = JointAngles()
        ja1.header.frame_id = '1'
        ja1.angles = [Float32(2.0), Float32(3.0)]

        pub0.publish(ja0)
        pub1.publish(ja1)

        idx += 0.01

        print mm.subchains







