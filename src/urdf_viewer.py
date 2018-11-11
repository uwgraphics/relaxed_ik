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

from start_here import urdf_file_name, fixed_frame, joint_ordering
import rospy
import roslaunch
import tf
import os
import yaml
from RelaxedIK.Utils.yaml_utils import get_relaxedIK_yaml_obj
from sensor_msgs.msg import JointState

joint_state = None
def joint_state_cb(data):
    global  joint_state
    joint_state = data

if __name__ == '__main__':
    rospy.init_node('urdf_viewer')

    path_to_src = os.path.dirname(__file__)

    y = get_relaxedIK_yaml_obj(path_to_src)
    if not y == None:
        urdf_file_name = y['urdf_file_name']
        fixed_frame = y['fixed_frame']
        joint_ordering = y['joint_ordering']

    urdf_file = open(path_to_src + '/RelaxedIK/urdfs/' + urdf_file_name, 'r')
    urdf_string = urdf_file.read()
    rospy.set_param('robot_description', urdf_string)
    tf_pub = tf.TransformBroadcaster()
    rospy.Subscriber('joint_states', JointState, joint_state_cb)

    rospy.sleep(1.0)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch_path = path_to_src + '/../launch/joint_state_pub.launch'
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
    launch.start()

    prev_state = []

    rate = rospy.Rate(5.0)
    while not rospy.is_shutdown():
        tf_pub.sendTransform((0,0,0),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             'common_world',
                             fixed_frame)

        if not joint_state == None:
            js = joint_state.position
            if not prev_state == js:
                names = joint_state.name
                positions = []
                for o in joint_ordering:
                    for i,n in enumerate(names):
                        if o == n:
                            positions.append(js[i])
                print str(positions) + ','
                prev_state = js

    rate.sleep()