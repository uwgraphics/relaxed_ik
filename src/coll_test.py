from RelaxedIK.relaxedIK import RelaxedIK
from RelaxedIK.GROOVE_RelaxedIK.relaxedIK_vars import RelaxedIK_vars
from start_here import urdf_file_name, joint_names, joint_ordering, ee_fixed_joints, starting_config, \
    joint_state_define, collision_file_name, fixed_frame
import os
import rospy

rospy.init_node('coll_test')

path_to_src = os.path.dirname(__file__)
vars = RelaxedIK_vars('relaxedIK' ,path_to_src + '/RelaxedIK/urdfs/' + 'ur5.urdf' ,joint_names
                      ,ee_fixed_joints,
                      joint_ordering ,init_state=starting_config, collision_file=collision_file_name, config_override=False, path_to_src=path_to_src, pre_config=True)

def get_score(x):
    frames = vars.robot.getFrames(x)
    return vars.collision_graph.get_collision_score(frames)




