import numpy as np

from abc import ABCMeta, abstractmethod
import math
from ..Utils import transformations as T
import tf.broadcaster
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

keyboard_pos_goal = [0,0,0]
keyboard_quat_goal = [1,0,0,0]
def keyboard_goal_cb(data):
    global keyboard_pos_goal, keyboard_quat_goal
    keyboard_pos_goal[0] = data.pose.position.x
    keyboard_pos_goal[1] = data.pose.position.y
    keyboard_pos_goal[2] = data.pose.position.z
    keyboard_quat_goal[0] = data.pose.orientation.w
    keyboard_quat_goal[1] = data.pose.orientation.x
    keyboard_quat_goal[2] = data.pose.orientation.y
    keyboard_quat_goal[3] = data.pose.orientation.z


class IK_Task:
    @abstractmethod
    def __init__(self, *args): pass

    @abstractmethod
    def name(self): pass

    @abstractmethod
    def get_goal(self, time, task_recorder, display_goal=True, scale_factor=1.0, orig_ee_pos=None, orig_ee_quat=None): return None

########################################################################################################################

tf_broadcaster = tf.broadcaster.TransformBroadcaster()
marker_pub = rospy.Publisher('visualization_marker',Marker,queue_size=10)
def display_ik_goal(pos_goal, quat_goal):
    global tf_broadcaster, marker_pub
    quat_mat = T.quaternion_matrix(quat_goal)
    quat_goal = T.quaternion_from_matrix(quat_mat)
    tf_broadcaster.sendTransform(tuple(pos_goal), (quat_goal[1],quat_goal[2],quat_goal[3],quat_goal[0]), rospy.Time.now(), 'ik_goal', 'common_world')
    marker = Marker()
    marker.header.frame_id = 'common_world'
    marker.id = 100
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.position.x = pos_goal[0]
    marker.pose.position.y = pos_goal[1]
    marker.pose.position.z = pos_goal[2]
    marker.scale.x = .05
    marker.scale.y = .05
    marker.scale.z = .05
    marker.color.a = 1
    marker.color.r = 0.0
    marker.color.g = 0.9
    marker.color.b = 0.2
    marker_pub.publish(marker)

def translate_and_scale_pos_goal(pos_goal, scale_factor, orig_ee_pos):
    p = np.array(pos_goal)
    o = np.array(orig_ee_pos)
    vec = (o + scale_factor*p)
    return vec

def get_relative_orientation(quat_goal, orig_ee_quat):
    return T.quaternion_multiply(quat_goal, orig_ee_quat)

class Keyboard_input(IK_Task):
    def __init__(self):
        rospy.Subscriber('keyboard_ik_goal',PoseStamped,keyboard_goal_cb)
    def name(self): return 'keyboardInput'
    def type(self): return 'keyboardInput'

    def get_goal(self, time, task_recorder, display_goal=True, scale_factor=1.0, orig_ee_pos=None, orig_ee_quat=None):
        global keyboard_pos_goal, keyboard_quat_goal
        if orig_ee_pos.all() == None:
            orig_ee_pos = [0,0,0]


        pos_goal = keyboard_pos_goal
        quat_goal = keyboard_quat_goal

        if not task_recorder == None:
            task_recorder.add_line(time, pos_goal, quat_goal)

        pos_goal = translate_and_scale_pos_goal(pos_goal, scale_factor, orig_ee_pos)


        if not orig_ee_quat == None:
            quat_goal = get_relative_orientation(quat_goal, orig_ee_quat)

        if display_goal:
            display_ik_goal(pos_goal, quat_goal)

        return pos_goal, quat_goal

class File_input(IK_Task):
    def __init__(self, file_name, loop=False):
        root = '/home/rakita/catkin_ws/src/relaxed_ik_rss/bin/FileIO/task_recordings/'
        fp = root + file_name
        self.file = open(fp, 'r')
        self.file_name = file_name
        self.times = []
        self.ik_pos_goals = []
        self.ik_quat_goals = []
        self.loop = loop
        self.task_name = self.file_name

        line = self.file.readline()
        while not line == '':
            line_arr = line.split(';')
            self.times.append(float(line_arr[0]))
            ik_pos_goal = [float(f) for f in line_arr[1].split(',')]
            self.ik_pos_goals.append(ik_pos_goal)
            ik_quat_goal = [float(f) for f in line_arr[2].split(',')]
            self.ik_quat_goals.append(ik_quat_goal)

            line = self.file.readline()

        self.num_goals = len(self.ik_pos_goals)
        self.idx = 0

    def name(self): return self.task_name
    def type(self): return 'fileInput'

    def get_goal(self, time, task_recorder, display_goal=True, scale_factor=1.0, orig_ee_pos=None, orig_ee_quat=None):
        if orig_ee_pos == None:
            orig_ee_pos = [0,0,0]

        if self.idx >= self.num_goals:
            if self.loop == True:
                self.idx = 0
            else:
                return None, None

        pos_goal = self.ik_pos_goals[self.idx]
        quat_goal = self.ik_quat_goals[self.idx]
        pos_goal = translate_and_scale_pos_goal(pos_goal, scale_factor, orig_ee_pos)

        if not orig_ee_quat == None:
            quat_goal = get_relative_orientation(quat_goal, orig_ee_quat)

        if display_goal:
            display_ik_goal(pos_goal, quat_goal)

        self.idx += 1

        return pos_goal, quat_goal



class Test_Task1(IK_Task):
    def __init__(self, max_time):
        self.max_time = max_time
    def name(self): return 'testTask1'

    def get_goal(self, time, task_recorder, display_goal=True, scale_factor=1.0, orig_ee_pos=None, orig_ee_quat=None):
        if orig_ee_pos == None:
            orig_ee_pos = [0,0,0]
        pos_goal = [0.0,0.2*math.sin(time),0.3*math.sin(time)]
        quat_goal = [1, 0, 0, 0]
        task_recorder.add_line(time, pos_goal, quat_goal)
        pos_goal = translate_and_scale_pos_goal(pos_goal, scale_factor, orig_ee_pos)

        if not orig_ee_quat == None:
            quat_goal = get_relative_orientation(quat_goal, orig_ee_quat)

        if display_goal:
            display_ik_goal(pos_goal, quat_goal)

        return pos_goal, quat_goal

class Test_SelfCollision(IK_Task):
    def __init__(self, max_time):
        self.max_time = max_time
    def name(self): return 'testSelfCollision'

    def get_goal(self, time, task_recorder, display_goal=True, scale_factor=1.0, orig_ee_pos=None, orig_ee_quat=None):
        if orig_ee_pos == None:
            orig_ee_pos = [0,0,0]
        pos_goal = [min(math.sin(time),0.0) ,0.1,-.5]
        quat_goal = [1,0,0,0]

        task_recorder.add_line(time, pos_goal, quat_goal)

        pos_goal = translate_and_scale_pos_goal(pos_goal, scale_factor, orig_ee_pos)

        if not orig_ee_quat == None:
            quat_goal = get_relative_orientation(quat_goal, orig_ee_quat)

        if display_goal:
            display_ik_goal(pos_goal, quat_goal)

        return pos_goal, quat_goal

class Up_and_down(IK_Task):
    def __init__(self, max_time):
        self.max_time = max_time
    def name(self): return 'UpAndDown'

    def get_goal(self, time, task_recorder, display_goal=True, scale_factor=1.0, orig_ee_pos=None, orig_ee_quat=None):
        if orig_ee_pos == None:
            orig_ee_pos = [0,0,0]
        pos_goal = [0.0,0.0, .5*math.sin(time)]
        quat_goal = [1,0,0,0]

        task_recorder.add_line(time, pos_goal, quat_goal)

        pos_goal = translate_and_scale_pos_goal(pos_goal, scale_factor, orig_ee_pos)

        if not orig_ee_quat == None:
            quat_goal = get_relative_orientation(quat_goal, orig_ee_quat)

        if display_goal:
            display_ik_goal(pos_goal, quat_goal)

        return pos_goal, quat_goal

class Test_Task_Reactor(IK_Task):
    def __init__(self, max_time):
        self.max_time = max_time
    def name(self): return 'testTaskReactor'

    def get_goal(self, time, task_recorder, display_goal=True, scale_factor=1.0, orig_ee_pos=None, orig_ee_quat=None):
        if orig_ee_pos == None:
            orig_ee_pos = [0,0,0]
        pos_goal = [0.1*math.sin(time),0.0,0.05*math.sin(time)]
        quat_goal = [1,0,0,0]

        task_recorder.add_line(time, pos_goal, quat_goal)

        pos_goal = translate_and_scale_pos_goal(pos_goal, scale_factor, orig_ee_pos)

        if not orig_ee_quat == None:
            quat_goal = get_relative_orientation(quat_goal, orig_ee_quat)

        if display_goal:
            display_ik_goal(pos_goal, quat_goal)

        return pos_goal, quat_goal

class Static_test(IK_Task):
    def __init__(self, max_time):
        self.max_time = max_time
    def name(self): return 'static'

    def get_goal(self, time, task_recorder, display_goal=True, scale_factor=1.0, orig_ee_pos=None, orig_ee_quat=None):
        if orig_ee_pos == None:
            orig_ee_pos = [0,0,0]
        pos_goal = [0,0,0]
        # pos_goal = translate_and_scale_pos_goal(pos_goal, scale_factor, orig_ee_pos)
        quat_goal = [1,0,0,0]

        task_recorder.add_line(time, pos_goal, quat_goal)

        if not orig_ee_quat == None:
            quat_goal = get_relative_orientation(quat_goal, orig_ee_quat)

        if display_goal:
            display_ik_goal(pos_goal, quat_goal)

        return pos_goal, quat_goal