
from relaxedIK_mt_utils import get_subchains_from_indices, glue_subchains
import rospy
from relaxed_ik.msg import JointAngles
from std_msgs.msg import Float64

class Multithread_Manager:
    def __init__(self, subchain_indices, relaxedIK_full):
        self.subchain_indices = subchain_indices
        self.relaxedIK_full = relaxedIK_full
        self.num_subchains = len(subchain_indices)

        self.subchains = get_subchains_from_indices(subchain_indices, relaxedIK_full.vars.init_state)
        self.subchains_write = get_subchains_from_indices(subchain_indices, relaxedIK_full.vars.init_state)
        self.locked_x = relaxedIK_full.vars.init_state

        self.solution_count = 0

        self.subchain_msgs = [   ]
        for i in xrange(self.num_subchains):
            self.subchain_msgs.append(JointAngles())
            for j, s in enumerate(self.subchains[i]):
                self.subchain_msgs[i].angles.append(Float64(self.subchains[i][j]))

        self.full_state = relaxedIK_full.vars.init_state

        # for i in xrange(self.num_subchains):
        #     rospy.Subscriber('/subchain_{}'.format(i), JointAngles, self.subchain_cb)


    def subchain_cb(self, data):
        subchain_id = int(data.header.frame_id)
        self.subchain_msgs[subchain_id] = data

        # for i, s in enumerate(self.subchains[subchain_id]):
        #     self.subchains[subchain_id][i] = float(data.angles[i].data)

    def get_curr_subchain_by_idx(self, idx):
        correct_length = len(self.subchain_indices[idx])
        curr_subchain_msg_angles = self.subchain_msgs[idx].angles
        subchain = []
        for j in xrange(len(curr_subchain_msg_angles)):
            subchain.append(float(curr_subchain_msg_angles[j].data))
        if not len(subchain) == correct_length:
            print 'warning: subchain was not correct length.  Retrying'
            # return self.get_curr_subchain_by_idx(idx)
            return None

        return subchain


    def get_all_subchains(self):
        subchains = []
        for i in xrange(self.num_subchains):
            subchain = self.get_curr_subchain_by_idx(i)
            # for j in xrange(len(self.subchain_msgs[i].angles)):
            #     subchain.append(float(self.subchain_msgs[i].angles[j].data))

            subchains.append(subchain)

        return subchains



if __name__ == '__main__':
    rospy.init_node('test')

    ja = JointAngles()
    ja.header.seq = 1
    ja.angles = [ Float64(4.0), Float64(2.0) ]

    a = [0,1,2,3]

    idx = ja.header.seq

    from RelaxedIK.relaxedIK import get_relaxedIK_from_info_file

    path_to_src = "/home/rakita/catkin_ws/src/relaxed_ik/src/"

    relaxedIK = get_relaxedIK_from_info_file(path_to_src, 'jaco7_info.yaml')

    subchain_indicies = [[0,1], [2,3], [4,5,6]]


    mm = Multithread_Manager(subchain_indicies, relaxedIK)

    print mm.get_all_subchains()


    print mm.num_subchains

    pub0 = rospy.Publisher('subchain_0', JointAngles, queue_size=3)
    pub1 = rospy.Publisher('subchain_1', JointAngles, queue_size=3)

    idx = 0.0
    while not rospy.is_shutdown():
        ja0 = JointAngles()
        ja0.header.frame_id = '0'
        ja0.angles = [Float64(idx), Float64(1.0)]


        ja1 = JointAngles()
        ja1.header.frame_id = '1'
        ja1.angles = [Float64(2.0), Float64(3.0)]

        pub0.publish(ja0)
        pub1.publish(ja1)

        idx += 0.01

        print mm.get_all_subchains()








