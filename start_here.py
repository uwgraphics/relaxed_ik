'''
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 5/8/18

Intro: Welcome to RelaxedIK! RelaxedIK is an inverse kinematics (IK) solver designed for robot platforms such that the conversion
between Cartesian end-effector pose goals (such as "move the robot's right arm up to position X, while maintaining an end-effector
orientation Y") to Joint-Space (i.e., the robot's rotation values for each joint degree-of-freedom at a particular time-point) is
done both ACCURATELY and FEASIBLY.  By this, we mean that RelaxedIK attempts to find the closest possible solution to the
desired end-effector pose goals without exhibiting negative effects such as self-collisions, environment collisions,
kinematic-singularities, or joint-space discontinuities.  Our solver has been used...

To get started, just follow the instructions found here.

If anything with the solver is not working as expected, or if you have any feedback, feel free to let us know (rakita@cs.wisc.edu)!
We are actively supporting and extending this code, so we are interested to hear about how the solver is being used and any positive
or negative experiences in using it.
'''

import rospy

##########
# Step 1: Please set the  #
##########



