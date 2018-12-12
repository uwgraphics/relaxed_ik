#! /usr/bin/env python
'''
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 5/8/18

PLEASE DO NOT CHANGE CODE IN THIS FILE.  IF TRYING TO SET UP RELAXEDIK, PLEASE REFER TO start_here.py INSTEAD
AND FOLLOW THE STEP-BY-STEP INSTRUCTIONS THERE.  Thanks!
'''
######################################################################################################


from start_here import urdf_file_name, joint_names, joint_ordering, ee_fixed_joints, starting_config, \
    joint_state_define, collision_file_name, fixed_frame
from RelaxedIK.GROOVE_RelaxedIK.relaxedIK_vars import RelaxedIK_vars
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from RelaxedIK.Utils.yaml_utils import get_relaxedIK_yaml_obj
import rospy
import roslaunch
import os
import tf


if __name__ == '__main__':
    rospy.init_node('collision_viewer')

    path_to_src = os.path.dirname(__file__)

    y = get_relaxedIK_yaml_obj(path_to_src)
    if not y == None:
        urdf_file_name = y['urdf_file_name']
        joint_names = y['joint_names']
        joint_ordering = y['joint_ordering']
        ee_fixed_joints = y['ee_fixed_joints']
        starting_config = y['starting_config']
        collision_file_name = y['collision_file_name']
        fixed_frame = y['fixed_frame']
        joint_state_define_file_name = y['joint_state_define_func_file']
        joint_state_define_file = open(path_to_src + '/RelaxedIK/Config/joint_state_define_functions/' + joint_state_define_file_name, 'r')
        func = joint_state_define_file.read()
        exec(func)

    urdf_file = open(path_to_src + '/RelaxedIK/urdfs/' + urdf_file_name, 'r')
    urdf_string = urdf_file.read()
    rospy.set_param('robot_description', urdf_string)
    js_pub = rospy.Publisher('joint_states',JointState,queue_size=5)
    marker_pub = rospy.Publisher('visualization_marker',Marker,queue_size=5)
    tf_pub = tf.TransformBroadcaster()

    rospy.sleep(0.2)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch_path = path_to_src + '/../launch/robot_state_pub.launch'
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
    launch.start()

    vars = RelaxedIK_vars('relaxedIK', path_to_src + '/RelaxedIK/urdfs/' + urdf_file_name,joint_names,ee_fixed_joints,
                          joint_ordering,init_state=starting_config, collision_file=collision_file_name, pre_config=True, path_to_src=path_to_src)

    sample_states = vars.collision_graph.sample_states
    idx = 0
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if idx >= len(sample_states): idx = 0

        state = sample_states[idx]
        cg = vars.collision_graph
        robot = vars.robot
        frames = robot.getFrames(state)
        cg.get_collision_score(frames)
        cg.c.draw_all()

        js = joint_state_define(state)
        if js == None:
            js = JointState()
            js.name = joint_ordering
            for x in state:
                js.position.append(x)
        now = rospy.Time.now()
        js.header.stamp.secs = now.secs
        js.header.stamp.nsecs = now.nsecs
        js_pub.publish(js)

        tf_pub.sendTransform((0,0,0),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             'common_world',
                             fixed_frame)

        marker = Marker()
        marker.type = marker.TEXT_VIEW_FACING
        marker.header.frame_id = 'common_world'
        marker.header.stamp.nsecs = now.nsecs
        marker.header.stamp.secs = now.secs
        marker.text = str(idx)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.pose.position.z = 0.3
        marker.pose.position.x = -0.3
        marker.color.a = 1.0
        marker_pub.publish(marker)

        idx += 1

        rate.sleep()