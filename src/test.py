from RelaxedIK.relaxedIK import *
import os
import rospy
from RelaxedIK.Julia_Bridge.relaxedIK_julia import RelaxedIK_Julia

rospy.init_node('a')

path_to_src = os.path.dirname(__file__)

# rik = get_relaxedIK_from_info_file(path_to_src)

rik = RelaxedIK_Julia(path_to_src)

for i in xrange(10000):
    print rik.solve([[1.,0.,0.]], [[1.,0.,0.,0.]])




