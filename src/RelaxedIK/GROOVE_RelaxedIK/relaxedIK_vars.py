from ..GROOVE.GROOVE_Utils.vars import Vars
from ..GROOVE.GROOVE_Utils.weight_function import Weight_Function,Identity_Weight
from ..GROOVE_RelaxedIK.relaxedIK_weight_function import Position_Weight
from ..Spacetime.robot import Robot
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from ..Utils.urdf_load import *
from ..GROOVE_RelaxedIK.relaxedIK_constraint import Singularity_Avoidance_Constraint, Joint_Velocity_Constraint
from ..GROOVE_RelaxedIK.relaxedIK_objective import *
from ..Utils.collision_graph import Collision_Graph
from ..Utils.config_engine import Config_Engine
# import rospy
import os
from sklearn.externals import joblib
from RelaxedIK.Utils.yaml_utils import get_relaxedIK_yaml_obj
import rospy




class RelaxedIK_vars(Vars):
    def __init__(self, name,
                 urdf_path,
                 full_joint_lists,
                 fixed_ee_joints,
                 joint_order,
                 objective_function=objective_master_relaxedIK,
                 full_arms=[],
                 init_state=6*[0],
                 rotation_mode = 'relative',  # could be 'absolute' or 'relative'
                 position_mode = 'relative',
                 objectives=(Position_MultiEE_Obj(), Orientation_MultiEE_Obj(), Min_Jt_Vel_Obj(),Min_Jt_Accel_Obj(),Min_Jt_Jerk_Obj(), Joint_Limit_Obj(), Collision_Avoidance_nn()),
                 weight_funcs=(Identity_Weight(), Identity_Weight(), Identity_Weight(),Identity_Weight(),Identity_Weight(), Identity_Weight(), Identity_Weight()),
                 weight_priors=(50.0,49.0,3.0,1.0,2.0,10.0,2.0),
                 constraints=(),
                 bounds=(),
                 collision_file='',
                 collision_nn_file='',
                 collision_link_exclusion_list=[],
                 config_file_name='',
                 path_to_src='',
                 pre_config=False,
                 config_override=False,
                 c_boost=False
                 ):

        Vars.__init__(self,name, objective_function, init_state,objectives,weight_funcs,weight_priors,constraints,bounds)
        # check inputs #####################################################################################################################
        if not (rotation_mode == 'relative' or rotation_mode == 'absolute'):
            print bcolors.FAIL + 'Invalid rotation_mode.  Must be <relative> or <absolute>.  Exiting.' + bcolors.ENDC
            raise ValueError('Invalid rotation_mode.')
        if not (position_mode == 'relative' or position_mode == 'absolute'):
            print bcolors.FAIL + 'Invalid position_mode.  Must be <relative> or <absolute>.  Exiting.' + bcolors.ENDC
            raise ValueError('Invalid position_mode.')
        num_objs = len(objectives)
        if not (num_objs == len(weight_funcs) == len(weight_priors)):
            print bcolors.FAIL + 'Invalid Inputs.  The number of objectives ({}) must be the same as the number' \
                                 'of weight functions ({}) and weight priors ({}).  Exiting.'.format(str(num_objs),
                                                                                                     str(len(
                                                                                                         weight_funcs)),
                                                                                                     str(len(
                                                                                                         weight_priors))) + bcolors.ENDC
            raise ValueError('Invalid function arguments.')
        ###################################################################################################################################


        self.full_joint_lists = full_joint_lists
        self.fixed_ee_joints = fixed_ee_joints
        self.joint_order = joint_order
        self.urdf_path = urdf_path
        self.collision_file = collision_file
        self.c_boost = c_boost
        self.num_chains = len(full_joint_lists)
        self.arms = []
        self.urdf_robots = []
        self.trees = []

        try:
            from boost import objectives_ext
        except:
            self.c_boost = False


        if full_arms == []:
            for i in xrange(self.num_chains):
                urdf_robot, arm, arm_c, tree = urdf_load(urdf_path, '', '', full_joint_lists[i], fixed_ee_joints[i])
                if self.c_boost:
                    self.arms.append(arm_c)
                else:
                    self.arms.append(arm)
                self.urdf_robots.append(urdf_robot)
                self.trees.append(tree)
        else:
            self.arms = full_arms
            self.urdf_robots = None
            self.trees = None

        # make robot
        self.robot = Robot(self.arms, full_joint_lists,joint_order)

        # self.urdf_robot = urdf_robot
        # self.arm = arm
        # self.tree = tree
        self.rotation_mode = rotation_mode
        self.position_mode = position_mode

        self.numDOF = self.robot.numDOF

        if bounds == ():
            self.bounds = self.robot.bounds
        else:
            self.bounds = bounds

        if not collision_file == '':
            cf = path_to_src + '/RelaxedIK/Config/collision_files/' + collision_file
            # cf = os.path.join(dirname, '../Config/' + collision_file)
            # cf = 'RelaxedIK/Config/' + collision_file
            self.collision_graph = Collision_Graph(cf, self.robot, collision_link_exclusion_list)

        if pre_config:
            '''
            response = raw_input(bcolors.FAIL + 'WARNING: Running in pre-config mode should only be used in simulation for diagnostics or generating a collision.yaml'
                                                'file.  Collisions will NOT be avoided in this mode, which could result in damage if run on a real robot.  Continue?  (y or n): ' + bcolors.ENDC)
            if not response == 'y':
                exit(1)
            '''

            objectives = list(objectives)
            weight_funcs = list(weight_funcs)
            weight_priors = list(weight_priors)
            del objectives[-1]
            del weight_funcs[-1]
            del weight_priors[-1]
            objectives = tuple(objectives)
            weight_funcs = tuple(weight_funcs)
            weight_priors = tuple(weight_priors)

        velocity_constraints = True
        velocity_scale = 1.0
        if velocity_constraints:
            for i in xrange(self.robot.numDOF):
                self.constraints += (Joint_Velocity_Constraint(i,velocity_scale),)

        if not self.numDOF == len(init_state):
            # self.init_state = self.numDOF * [0]
            # print bcolors.WARNING + 'WARNING: Length of init_state does not match number of robot DOFs.  Automatically ' \
            #                         'initializing init_state as {}.  This may cause errors.'.format(
            #    str(self.init_state)) + bcolors.ENDC
            print bcolors.WARNING + 'WARNING: Length of init_state does not match number of robot DOFs.  Is this what you intended?' + bcolors.ENDC

        Vars.__init__(self, name, objective_function, self.init_state,objectives,weight_funcs,weight_priors,constraints=self.constraints,bounds=self.bounds)

        self.goal_positions = []
        for i in range(self.num_chains):
            self.goal_positions.append(np.array([0,0,0]))
        self.prev_goal_positions3 = self.goal_positions
        self.prev_goal_positions2 = self.goal_positions
        self.prev_goal_positions = self.goal_positions
        self.goal_quats = []
        for i in range(self.num_chains):
            self.goal_quats.append([1,0,0,0])
        self.prev_goal_quats3 = self.goal_quats
        self.prev_goal_quats2 = self.goal_quats
        self.prev_goal_quats = self.goal_quats
        self.frames = self.robot.getFrames(self.init_state)
        self.joint_limit_obj_value = 0.0
        self.multithread = False

        self.init_ee_positions = self.robot.get_ee_positions(self.init_state)
        self.init_ee_quats = self.robot.get_ee_rotations(self.init_state)
        self.ee_positions = self.init_ee_positions
        self.prev_ee_positions3 = self.init_ee_positions
        self.prev_ee_positions2 = self.init_ee_positions
        self.prev_ee_positions = self.init_ee_positions
        self.first = True
        self.unconstrained = False
        self.marker_pub = rospy.Publisher('visualization_marker',Marker,queue_size=5)

        self.solveID = 1.0
        self.avg_solution_time = 0.0
        self.total_run_time = 0.0
        # self.start_time = rospy.get_time()
        self.solverCounter = 0

        if not pre_config:
            # y = get_relaxedIK_yaml_obj(path_to_src)
            # collision_nn_file = y['collision_nn_file']
            self.ce = Config_Engine(self.collision_graph, self, path_to_src,collision_nn_file,config_fn=config_file_name, override=config_override)
            self.collision_nn = self.ce.collision_nn

    def update(self, xopt, f_obj, publish_objectives=True,publish_constraints=True, publish_weight_funcs=True):
        Vars.update(self, xopt, f_obj, publish_objectives=publish_objectives,publish_constraints=publish_constraints, publish_weight_funcs=publish_weight_funcs)


    def relaxedIK_vars_update(self, xopt):
        self.prev_ee_positions3 = self.prev_ee_positions2
        self.prev_ee_positions2 = self.prev_ee_positions
        self.prev_ee_positions = self.ee_positions
        self.ee_positions = self.robot.get_ee_positions(xopt)
        self.prev_goal_quats3 = self.prev_goal_quats2
        self.prev_goal_quats2 = self.prev_goal_quats
        self.prev_goal_quats = self.goal_quats
        self.prev_goal_positions3 = self.prev_goal_positions2
        self.prev_goal_positions2 = self.prev_goal_positions
        self.prev_goal_positions =  self.goal_positions
        self.solveID += 1.0
        # self.total_run_time = rospy.get_time() - self.start_time
        self.avg_solution_time = self.total_run_time / self.solveID









