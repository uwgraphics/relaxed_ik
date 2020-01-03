#! /usr/bin/env python
'''
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 5/8/18

PLEASE DO NOT CHANGE CODE IN THIS FILE.  IF TRYING TO SET UP RELAXEDIK, PLEASE REFER TO start_here.py INSTEAD
AND FOLLOW THE STEP-BY-STEP INSTRUCTIONS THERE.  Thanks!
'''

from start_here import urdf_file_name, fixed_frame, joint_ordering, starting_config, joint_state_define
import rospy
import roslaunch
import tf
import os
from RelaxedIK.Utils.yaml_utils import get_relaxedIK_yaml_obj
from sensor_msgs.msg import JointState
from relaxed_ik.msg import JointAngles

ja_solution = ''
def ja_solution_cb(data):
    global ja_solution
    ja_solution = []
    for a in data.angles.data:
        ja_solution.append(a)

if __name__ == '__main__':
    rospy.init_node('rviz_viewer')

    path_to_src = os.path.dirname(__file__)

    y = get_relaxedIK_yaml_obj(path_to_src)
    if not y == None:
        urdf_file_name = y['urdf_file_name']
        fixed_frame = y['fixed_frame']
        joint_ordering = y['joint_ordering']
        starting_config = y['starting_config']
        joint_state_define_file_name = y['joint_state_define_func_file']
        joint_state_define_file = open(path_to_src + '/RelaxedIK/Config/joint_state_define_functions/' + joint_state_define_file_name, 'r')
        func = joint_state_define_file.read()
        exec(func)


    urdf_file = open(path_to_src + '/RelaxedIK/urdfs/' + urdf_file_name, 'r')
    urdf_string = urdf_file.read()
    rospy.set_param('robot_description', urdf_string)
    js_pub = rospy.Publisher('joint_states',JointState,queue_size=5)
    rospy.Subscriber('/relaxed_ik/joint_angle_solutions',JointAngles,ja_solution_cb)
    tf_pub = tf.TransformBroadcaster()

    rospy.sleep(0.5)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch_path = path_to_src + '/../launch/joint_state_pub_nojsp.launch'
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
    launch.start()

    prev_state = []

    rate = rospy.Rate(200.0)
    prev_sol = starting_config
    while not rospy.is_shutdown():
        tf_pub.sendTransform((0, 0, 0),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             'common_world',
                             fixed_frame)
        if len(ja_solution) == 0:
            xopt = starting_config
        else:
            xopt = ja_solution
            if not len(xopt) == len(starting_config):
                xopt = prev_sol
            else:
                prev_sol = xopt

        js = joint_state_define(xopt)
        if js == None:
            js = JointState()
            js.name = joint_ordering
            for x in xopt:
                js.position.append(x)
        now = rospy.Time.now()
        js.header.stamp.secs = now.secs
        js.header.stamp.nsecs = now.nsecs
        js_pub.publish(js)
	
    rate.sleep()
