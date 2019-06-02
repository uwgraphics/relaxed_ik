from RelaxedIK.relaxedIK import RelaxedIK
from RelaxedIK.GROOVE_RelaxedIK.relaxedIK_vars import RelaxedIK_vars
# from start_here import urdf_file_name, joint_names, joint_ordering, ee_fixed_joints, starting_config, \
#    joint_state_define, collision_file_name, fixed_frame
import os
import rospy
import yaml
import numpy.random as r


class CollisionVars:
    def __init__(self, path_to_src):
        # rospy.init_node('coll_test')
        # print "got here!"
        # path_to_src = os.path.dirname(__file__)
        # print path_to_src
        # print path_to_src + '/RelaxedIK/Config/loaded_robot'
        loaded_robot_file = open(path_to_src + '/RelaxedIK/Config/loaded_robot', 'r')
        loaded_robot = loaded_robot_file.readline()
        y = yaml.load(open(path_to_src + '/RelaxedIK/Config/info_files/' + loaded_robot))
        urdf = y['urdf_file_name']
        joint_names = y['joint_names']
        ee_fixed_joints = y['ee_fixed_joints']
        joint_ordering = y['joint_ordering']
        starting_config = y['starting_config']
        collision_file_name = y['collision_file_name']

        self.vars = RelaxedIK_vars('relaxedIK', path_to_src + '/RelaxedIK/urdfs/' + urdf, joint_names, ee_fixed_joints,
                               joint_ordering, init_state=starting_config, collision_file=collision_file_name,
                               config_override=False, path_to_src=path_to_src, pre_config=True)


def get_score(x, CollisionVars):
    frames = CollisionVars.vars.robot.getFrames(x)
    return CollisionVars.vars.collision_graph.get_collision_score(frames)
