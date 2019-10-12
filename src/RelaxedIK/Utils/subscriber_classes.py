from sensor_msgs import point_cloud2 as pc2
import rospy
from relaxed_ik.msg import EEPoseGoals, JointAngles
from std_msgs.msg import Int8, Float32, Float64
from geometry_msgs.msg import Pose, Vector3, Quaternion, PoseStamped, Vector3Stamped, QuaternionStamped
import copy


class sub_eeposegoals:
    def __init__(self, log_all=False, value_names='', init_time=-1.):
        self.topic = '/relaxed_ik/ee_pose_goals'
        self.log_all = log_all

        if value_names == '':
            self.value_names = ['ee_pose_goals_pos_goals', 'ee_pose_goals_quat_goals']
        else:
            self.value_names = value_names

        rospy.Subscriber(self.topic, EEPoseGoals, self.cb)
        self.pos_goals = []
        self.quat_goals = []
        self.all_values = [ [], [] ]
        self.all_values_times = []

        if init_time == -1:
            self.init_time = rospy.get_time()
        else:
            self.init_time = init_time


    def cb(self, data):
        self.pos_goals = []
        self.quat_goals = []

        pose_goals = data.ee_poses

        for i in xrange(len(pose_goals)):
            p = pose_goals[i]
            pos_x = p.position.x
            pos_y = p.position.y
            pos_z = p.position.z

            quat_w = p.orientation.w
            quat_x = p.orientation.x
            quat_y = p.orientation.y
            quat_z = p.orientation.z

            self.pos_goals.append([pos_x, pos_y, pos_z])
            self.quat_goals.append([quat_w, quat_x, quat_y, quat_z])

            if self.log_all:
                self.all_values[0].append([pos_x, pos_y, pos_z])
                self.all_values[1].append([quat_w, quat_x, quat_y, quat_z])
                self.all_values_times.append( rospy.get_time() - self.init_time )


class sub_jointanglesolutions:
    def __init__(self, log_all=False, value_names='', init_time=-1.):
        self.topic = '/relaxed_ik/joint_angle_solutions'
        self.log_all = log_all

        if value_names == '':
            self.value_names = ['joint_angle_solutions']
        else:
            self.value_names = value_names

        rospy.Subscriber(self.topic, JointAngles, self.cb)
        self.angles = []
        self.all_values = []
        self.all_values_times = []

        if init_time == -1:
            self.init_time = rospy.get_time()
        else:
            self.init_time = init_time


    def cb(self, data):
        self.angles = []
        for a in data.angles.data:
            self.angles.append(a)

        if self.log_all:
            self.all_values.append( copy.deepcopy(self.angles) )
            self.all_values_times.append(rospy.get_time() - self.init_time)


class sub_int8:
    def __init__(self, topic, log_all=False,value_names='', init_time=-1.):
        self.topic = topic
        self.log_all = log_all

        if value_names == '':
            self.value_names = ['int8']
        else:
            self.value_names = value_names

        rospy.Subscriber(self.topic, Int8, self.cb)
        self.value = -12345
        self.all_values = []
        self.all_values_times = []

        if init_time == -1:
            self.init_time = rospy.get_time()
        else:
            self.init_time = init_time


    def cb(self, data):
        self.value = data.data
        if self.log_all:
            self.all_values.append(data.data)
            self.all_values_times.append(rospy.get_time() - self.init_time)


class sub_float32:
    def __init__(self, topic, log_all=False, value_names='', init_time=-1.):
        self.topic = topic
        self.log_all = log_all

        if value_names == '':
            self.value_names = ['float32']
        else:
            self.value_names = value_names

        rospy.Subscriber(self.topic, Float32, self.cb)
        self.value = -12345.
        self.all_values = []
        self.all_values_times = []

        if init_time == -1:
            self.init_time = rospy.get_time()
        else:
            self.init_time = init_time

    def cb(self, data):
        self.value = data.data
        if self.log_all:
            self.all_values.append(data.data)
            self.all_values_times.append(rospy.get_time() - self.init_time)


class sub_vector3:
    def __init__(self, topic, stamped=True, log_all=False, value_names='', init_time=-1.):
        self.topic = topic
        self.log_all = log_all
        self.stamped = stamped

        if value_names == '':
            self.value_names = ['vector3']
        else:
            self.value_names = value_names

        if self.stamped:
            rospy.Subscriber(self.topic, Vector3Stamped, self.cb)
        else:
            rospy.Subscriber(self.topic, Vector3, self.cb)

        self.vector3 = []
        self.all_values = []
        self.all_values_times = []

        if init_time == -1:
            self.init_time = rospy.get_time()
        else:
            self.init_time = init_time

    def cb(self, data):
        self.vector3 = []

        if self.stamped:
            self.vector3.append(data.vector.x)
            self.vector3.append(data.vector.y)
            self.vector3.append(data.vector.z)
        else:
            self.vector3.append(data.x)
            self.vector3.append(data.y)
            self.vector3.append(data.z)


        if self.log_all:
            self.all_values.append(copy.deepcopy(self.vector3))
            self.all_values_times.append(rospy.get_time() - self.init_time)

class sub_quaternion:
    def __init__(self, topic, stamped=True, log_all=False, value_names='', init_time=-1.):
        self.topic = topic
        self.log_all = log_all
        self.stamped = stamped

        if value_names == '':
            self.value_names = ['quaternion']
        else:
            self.value_names = value_names

        if self.stamped:
            rospy.Subscriber(self.topic, QuaternionStamped, self.cb)
        else:
            rospy.Subscriber(self.topic, Quaternion, self.cb)

        self.quaternion = []
        self.values = []
        self.all_values_times = []

        if init_time == -1:
            self.init_time = rospy.get_time()
        else:
            self.init_time = init_time

    def cb(self, data):
        self.quaternion = []
        if self.stamped:
            self.quaternion.append(data.quaternion.w)
            self.quaternion.append(data.quaternion.x)
            self.quaternion.append(data.quaternion.y)
            self.quaternion.append(data.quaternion.z)
        else:
            self.quaternion.append(data.w)
            self.quaternion.append(data.x)
            self.quaternion.append(data.y)
            self.quaternion.append(data.z)

        if self.log_all:
            self.values.append(copy.deepcopy(self.quaternion))
            self.all_values_times.append(rospy.get_time() - self.init_time)




point_cloud2 = ''
def point_cloud2_cb(data):
    global point_cloud2
    point_cloud2 = []
    x = pc2.read_points(data)
    point_cloud2 = list(x)
# rospy.Subscriber('/point_cloud', PointCloud2, point_cloud2_cb)

pos = ''
quat = ''
def pose_cb(data):
    global pos, quat
    pos = []
    quat = []
    pos.append(data.position.x)
    pos.append(data.position.y)
    pos.append(data.position.z)
    quat.append(data.orientation.w)
    quat.append(data.orientation.x)
    quat.append(data.orientation.y)
    quat.append(data.orientation.z)
# rospy.Subscriber('/autocam/ee_pose/camera_arm', Pose, pose_cb)




