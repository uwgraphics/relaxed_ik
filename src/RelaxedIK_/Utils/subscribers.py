from sensor_msgs import point_cloud2 as pc2


pos_goals = ''
quat_goals = ''
def eePoseGoals_cb(data):
    global pos_goals, quat_goals
    pos_goals = []
    quat_goals = []
    eepg = data

    pose_goals = eepg.ee_poses

    for i in xrange(len(pose_goals)):
        p = pose_goals[i]
        pos_x = p.position.x
        pos_y = p.position.y
        pos_z = p.position.z

        quat_w = p.orientation.w
        quat_x = p.orientation.x
        quat_y = p.orientation.y
        quat_z = p.orientation.z

        pos_goals.append([pos_x, pos_y, pos_z])
        quat_goals.append([quat_w, quat_x, quat_y, quat_z])
# rospy.Subscriber('/relaxed_ik/ee_pose_goals', EEPoseGoals, eePoseGoals_cb)



point_cloud2 = ''
def point_cloud2_cb(data):
    global point_cloud2
    point_cloud2 = []
    x = pc2.read_points(data)
    point_cloud2 = list(x)
# rospy.Subscriber('/point_cloud', PointCloud2, point_cloud2_cb)



angles = ''
def joint_angles_cb(data):
    global angles
    angles = []
    for a in data.angles.data:
        angles.append(a)
# rospy.Subscriber('/relaxed_ik/joint_angle_solutions', JointAngles, joint_angle_solutions_cb)



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




vector3 = ''
def vector3_cb(data):
    global vector3
    vector3 = []
    vector3.append(data.vector.x)
    vector3.append(data.vector.y)
    vector3.append(data.vector.z)
# rospy.Subscriber('/vive_controller/position_r', Vector3, vector3_cb)



quaternion = ''
def quaternion_cb(data):
    global quaternion
    quaternion = []
    quaternion.append(data.quaternion.w)
    quaternion.append(data.quaternion.x)
    quaternion.append(data.quaternion.y)
    quaternion.append(data.quaternion.z)
# rospy.Subscriber('/vive_controller/quaternion_r', Quaternion, quaternion_cb)



int8 = 0
def int8_cb(data):
    global int8
    int8 = data.data
# rospy.Subscriber('vive_controller/clutch_down_r', Int8, int8_cb)



float32 = 0.0
def float32_cb(data):
    global float32
    float32 = data.data
# rospy.Subscriber(' ', Float32, float32_cb)



