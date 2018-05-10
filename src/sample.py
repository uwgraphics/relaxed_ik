#! /usr/bin/env python
'''
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 5/10/18
'''
######################################################################################################

from start_here import urdf_file_name, joint_names, joint_ordering, ee_fixed_joints, starting_config, \
    joint_state_define, collision_file_name, fixed_frame, config_file_name
from RelaxedIK.relaxedIK import RelaxedIK
from RelaxedIK.GROOVE_RelaxedIK.relaxedIK_vars import RelaxedIK_vars
from sensor_msgs.msg import JointState
import rospy
import roslaunch
import os
import tf
import math


if __name__ == '__main__':
    # Don't change this code####################################################################################
    rospy.init_node('sample_node')

    urdf_file = open(os.path.dirname(__file__) + '/urdfs/' + urdf_file_name, 'r')
    urdf_string = urdf_file.read()
    rospy.set_param('robot_description', urdf_string)
    js_pub = rospy.Publisher('joint_states',JointState,queue_size=5)
    tf_pub = tf.TransformBroadcaster()

    rospy.sleep(0.3)

    ####################################################################################################################



    # Only change this if you want to extend/ adapt the solver to have more functionality ##############################
    vars = RelaxedIK_vars('relaxedIK',os.path.dirname(__file__) + '/urdfs/' + urdf_file_name,joint_names,ee_fixed_joints,
                          joint_ordering,init_state=starting_config, collision_file=collision_file_name,
                          config_file_name=config_file_name)
    relaxedIK = RelaxedIK(vars)
    print relaxedIK
    ####################################################################################################################



    # Don't change this code ###########################################################################################
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch_path = os.path.dirname(__file__) + '/../launch/robot_state_pub.launch'
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
    launch.start()
    ####################################################################################################################


    rospy.sleep(1.0)

    counter = 0.0
    stride = 0.1
    while not rospy.is_shutdown():
        c = math.cos(counter)
        s = 0.1
        xopt = relaxedIK.solve([[s*c,0,0]],[[1,0,0,0]])

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

        tf_pub.sendTransform((0, 0, 0),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             fixed_frame,
                             'common_world')

        counter += stride