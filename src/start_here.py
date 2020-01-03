#! /usr/bin/env python
'''
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 8/16/18

DEVELOPMENT BRANCH

Intro: Welcome to RelaxedIK! RelaxedIK is an inverse kinematics (IK) solver designed for robot platforms such that the conversion
between Cartesian end-effector pose goals (such as "move the robot's right arm end-effector to position X, while maintaining an end-effector
orientation Y") to Joint-Space (i.e., the robot's rotation values for each joint degree-of-freedom at a particular time-point) is
done both ACCURATELY and FEASIBLY.  By this, we mean that RelaxedIK attempts to find the closest possible solution to the
desired end-effector pose goals without exhibiting negative effects such as self-collisions, environment collisions,
kinematic-singularities, or joint-space discontinuities.

To get started, just follow the instructions found here.

If anything with the solver is not working as expected, or if you have any feedback, feel free to let us know! (rakita@cs.wisc.edu)
We are actively supporting and extending this code, so we are interested to hear about how the solver is being used and any positive
or negative experiences in using it.
'''
######################################################################################################


# Step-by-step guide starts here!


######################################################################################################
# Step 0: The RelaxedIK project should be included as a package within a catkin workspace
#   for more info on this step, see http://wiki.ros.org/catkin/workspaces
#   Make sure the package builds correctly (using catkin_make in the workspace root directory)
#   and source the workspace such that the package is visible to the ROS environment
#   (using source/devel setup.bash)
######################################################################################################


######################################################################################################
# Step 1a: Please add your robot urdf to the directory "urdfs", found in the project root directory.
#   Make sure that the robot corresponding to the urdf links to some visible robot description ros package,
#   otherwise the references described in the urdf will be meaningless.  For instance, if you are
#   setting up the ur5 robot with a urdf ur5.urdf, there should be some associated ros package such as
#   ur5_description built and sourced in your catkin workspace.
######################################################################################################


######################################################################################################
# Step 1b: Please set the following variable to the file name of your robot urdf.  For example, for the
#   ur5 robot urdf already in the urdfs folder, this variable would read 'ur5.urdf'
#   ex: urdf_file_name = 'ur5.urdf'
urdf_file_name = ''
######################################################################################################


######################################################################################################
# Step 1c: Please provide the fixed frame name.  This will be the root link name in the urdf
#   ex: fixed_frame  = 'base_link'
fixed_frame = ''
######################################################################################################

######################################################################################################
# Step 1d: At the end of this walk-through, there will be a central yaml file automatically generated that
#   will contain information about your robot setup.  Please provide a name for that file.
#   ex: info_file_name = 'ur5_info.yaml'
info_file_name = ''
######################################################################################################


######################################################################################################
# Step 2b: To test that your urdf is being read correctly, run the following command:
#   roslaunch relaxed_ik urdf_viewer.launch
#
#   you should see rviz start up, and your robot platform should be visible.  You can rotate the joints
#       in rviz by using the GUI pop-up
######################################################################################################



######################################################################################################
# Step 3a: Please provide the names of the joints for all chains.
#   This variable will be a list of lists, where each list specifies the names of joints in the particular
#   chain, adhering to the naming scheme in the urdf supplied in step 1b.
#   These lists can include fixed, prismatic, or revolute joints making up their respective kinematic chains.
#   example 1 shows what this would be for a multi-end effector robot, specifically to use the DRC-Hubo+ robot's
#   right and left arm with waist rotation (15 DOF total)
#   ex1: [ ['WAIST', 'RIGHT_SHOULDER_PITCH', 'RIGHT_SHOULDER_ROLL', 'RIGHT_SHOULDER_YAW', 'RIGHT_ELBOW', 'RIGHT_WRIST_YAW',
#               'RIGHT_WRIST_PITCH', 'RIGHT_WRIST_YAW_2'],
#         ['WAIST', 'LEFT_SHOULDER_PITCH', 'LEFT_SHOULDER_ROLL', 'LEFT_SHOULDER_YAW', 'LEFT_ELBOW', 'LEFT_WRIST_YAW',
#                'LEFT_WRIST_PITCH', 'LEFT_WRIST_YAW_2'] ]
#   example 2 shows what this would be for a single end-effector robot, specifically using the UR5 robot
#   ex2: [ ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'] ]
joint_names = [ [ ] ]
######################################################################################################


######################################################################################################
# Step 3b: Please provide the order that you want joints to appear in the final returned joint configurations,
#   using the names specified in step 3a.  ALL ACTUATED JOINTS specified in step 3a should appear somewhere in
#   this list.  In other words, every prismatic or revolute joint appearing in step 3a MUST be in the
#   joint_ordering list below.  However, NONE of the fixed joints listed in step 3a should appear in
#   the joint ordering list.
#   If the same joint appears in separate chains in step 3a, it should only appear once in
#   the list here.  If you are using a single chain robot, feel free to use the same joint list
#   that appears in step 3a.
#   example 1 shows one possible ordering for example 1 in Step 3a.  Notice how in this example, the 'WAIST'
#   joint only shows up once in the joint ordering list, even though it was a part of two separate subchains in
#   Step 3a.
#   NOTE: This is a single list NOT a list of lists like in Step 3a.
#   ex1: [ 'WAIST', 'RIGHT_SHOULDER_PITCH', 'RIGHT_SHOULDER_ROLL', 'RIGHT_SHOULDER_YAW', 'RIGHT_ELBOW', 'RIGHT_WRIST_YAW',
#               'RIGHT_WRIST_PITCH', 'RIGHT_WRIST_YAW_2','LEFT_SHOULDER_PITCH', 'LEFT_SHOULDER_ROLL', 'LEFT_SHOULDER_YAW',
#               'LEFT_ELBOW', 'LEFT_WRIST_YAW', 'LEFT_WRIST_PITCH', 'LEFT_WRIST_YAW_2' ]
joint_ordering =  [  ]
######################################################################################################


######################################################################################################
# Step 3c: Please provide the name(s) of all end-effector fixed joints.  These joints are usually labeled as
#   "fixed" in the urdf, and specify the exact "grasping point" of the robot's hand.  If it appears
#   that your urdf does not specify this ahead of time, please add it to the urdf.  Make sure to provide
#   one end-effector joint name per chain (i.e., each chain will have its own end-effector).  The order of
#   these joint names should correspond to the ordering of the chains specified in Step 3a.
#   For example 1, using the DRC-Hubo+ robot, we should specify two separate fixed joint names, one
#   for the right hand and one for the left hand
#   ex1: ee_fixed_joints = ['RIGHT_HAND', 'LEFT_HAND']
#   For example 2, using the UR5, this is a single chain robot, so it will only have a single end-effector joint
#   ex2: ee_fixed_joints = ['ee_fixed_joint']
ee_fixed_joints = [  ]
######################################################################################################


######################################################################################################
# Step 3d: Please provide a starting configuration for the robot.
#   The configuration should be a single list of values for each joint's rotation (in radians) adhering
#   to the joint order you specified in Step 3b
#   ex: starting_config = [ 3.12769839, -0.03987385, -2.07729916, -1.03981438, -1.58652782, -1.5710159 ]
starting_config = [ ]
######################################################################################################




######################################################################################################
# Step 4: Please provide a function that takes in a vector corresponding to a robot configuration (x) and
#   returns a sensor_msg.JointState message in ROS.  http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html
#   You do not have to include velocity or effort information, just the joint positions.
#   The point of having you manually provide a function that does this is to account for situations
#   where joints other than the ones included in Step 3b are part of a larger robot platform.  If you would
#   rather not manually provide a function to do this, just leave it how it is by default (returning None), and
#   we will provide a simple function that does this automatically.  But note, if you decide to do this, there may be
#   errors when viewing things in rviz as only the default joints listed in Step 3b will be visible, while
#   the other joints in the urdf are not published to.
#
# example function for the UR5:
#
# from sensor_msgs.msg import JointState
# def joint_state_define(x):
#    js = JointState()
#    js.name = joint_ordering
#    js.position = tuple(x)
#    return js
#
# example function for the DRC-Hubo+
#
# from sensor_msgs.msg import JointState
# def joint_state_define(x):
#   js = JointState()
#   js.name = ['LEFT_SHOULDER_PITCH', 'LEFT_SHOULDER_ROLL', 'LEFT_SHOULDER_YAW', 'LEFT_ELBOW', 'LEFT_WRIST_YAW',
#                      'LEFT_WRIST_PITCH', 'LWFT', 'LEFT_WRIST_YAW_2', 'LHAND_a1', 'LHAND_a2',
#                     'LHAND_a3', 'LHAND_b1', 'LHAND_b2', 'LHAND_b3', 'LHAND_c1',
#                     'LHAND_c2', 'LHAND_c3', 'RIGHT_SHOULDER_PITCH', 'RIGHT_SHOULDER_ROLL', 'RIGHT_SHOULDER_YAW',
#                     'RIGHT_ELBOW', 'RIGHT_WRIST_YAW', 'RIGHT_WRIST_PITCH', 'RWFT', 'RIGHT_WRIST_YAW_2',
#                     'RHAND_a1', 'RHAND_a2', 'RHAND_a3', 'RHAND_b1', 'RHAND_b2',
#                     'RHAND_b3', 'RHAND_c1', 'RHAND_c2', 'RHAND_c3', 'WAIST',
#                     'LEFT_HIP_YAW', 'LEFT_HIP_ROLL', 'LEFT_HIP_PITCH', 'LEFT_KNEE', 'LEFT_ANKLE_PITCH',
#                     'LEFT_ANKLE_ROLL', 'LAFT', 'LFUNI', 'LFWH', 'LEFT_WHEEL',
#                     'RIGHT_HIP_YAW', 'RIGHT_HIP_ROLL', 'RIGHT_HIP_PITCH', 'RIGHT_KNEE', 'RIGHT_ANKLE_PITCH',
#                     'RIGHT_ANKLE_ROLL', 'RAFT', 'RFUNI','RFWH', 'RIGHT_WHEEL',
#                     'NECK_PITCH_1', 'NECK_YAW', 'NECK_PITCH_2', 'NECK_ROLL']
#
#          js.position = 59*[0]
#          js.position[34] = x[0]
#          js.position[17] = x[1]
#          js.position[18] = x[2]
#          js.position[19] = x[3]
#          js.position[20] = x[4]
#          js.position[21] = x[5]
#          js.position[22] = x[6]
#          js.position[24] = x[7]
#          js.position[0] =  x[8]
#          js.position[1] =  x[9]
#          js.position[2] =  x[10]
#          js.position[3] =  x[11]
#          js.position[4] =  x[12]
#          js.position[5] =  x[13]
#          js.position[7] =  x[14]
#          js.position = tuple(js.position)
#          return js
#
#
# TODO: fill out this function, or leave it how it is for the default option
from sensor_msgs.msg import JointState
def joint_state_define(x):
	return None

######################################################################################################


######################################################################################################
# Step 5a: We will now set up collision information.  RelaxedIK avoids self-collisions by
#   first receiving a potential function for "how close" it is to a collision state,
#   then learning this potential function using a neural network such that it can be quickly run
#   in the context of a real-time optimization.  RelaxedIK uses fcl (flexible
#   collision library) to do distance checking between links and any environment objects.  Collision objects
#   will automatically envelope the robot's links based on its geometry defined in the urdf, but you
#   must supply a few other pieces of information for collision avoidance to work well.
#
#   To start, duplicate the collision_example.yaml file found in the RelaxedIK/Config/collision_files directory, and rename the duplicate.
#   The renamed file should retain its yaml file extension and should also be in the RelaxedIK/Config/collision_files
#   directory.
#
#   Now, we'll work in this new file and make some adjustments to tailor the collision information to your robot platform.
#   You can first set the desired radius for the robot's links.  The default option of 0.05 has been
#   found to work well in most cases, but feel free to change that however you like.
#
#   Next, the robot needs a set of collision-free "sample states" so that it can learn what is close to a collision state and what is not.
#   These sample states help the robot decide the difference between a configuration that is close to collision state and
#   a configuration where two links are natively and safely close together.  THIS STEP IS VERY IMPORTANT FOR THE NEURAL
#   NETWORK TO LEARN A GOOD COLLISION FUNCTION.  Good candidates for "sample states" are robot configurations that are
#   somewhat close to collisions states, but do not exhibit a collision.  If it seems like the robot is being too cautious after
#   training the neural network (i.e., it is staying too far away from collision states), include more sample states that are closer to
#   collision states without colliding.
#
#   Add these collision-free sample states as lists next to the samples_states field in your yaml file, as seen in
#   the collision_example.yaml file.  Feel free to use the urdf_viewer tool provided in the relaxed_ik package to pick out
#   collision-free sample states.  Joint states are displayed in the console window when using this tool to make
#   them easier to copy and paste into the collision yaml file.
#
#   To start this tool, use the command:
#       roslaunch relaxed_ik urdf_viewer.launch
#
#   The next (optional) fields in the collision file are the following:
#      training_states: [ ]
#      problem_states: [ ]
#
#   The training_states list should be filled with any states that you think will be useful in the neural network training process
#   NOTE: in constrast to the sample_states list filled above, the training_states states do NOT have to be in a collision free state!
#   Good states to include in the training_states list are those that are near the interface between collision regions, such that the robot
#   can learn through example the difference between near-collision and collision states
#
#   The problem_states list is similar to the training_states list (i.e., can feature either collision or non-collision states), but the
#   system will try to give these states extra attention in the learning process.  Good states to include in the problem_states list are those
#   that appear to have been learned incorrectly on previous runs of the neural network learning, and you want to try to fix it on another run
#   of the learning process.
#
#   Both the training_states and problem_states lists are optional parameters.  If you do not want to include states in these lists, leave them as they are in the
#	example file.
#
#   Again, feel free to use the urdf_viewer tool provided in the relaxed_ik package to select training_states and problem_states
#
#   The next fields in the yaml file (boxes, spheres, ellipsoids, capsules, cylinders) are used to specify additional
#   objects around the environment that the robot should avoid.
#   These fields all follow a similar pattern:
#   1. Provide a name for the object under "name"
#   2. Provide parameters for the respective shape
#       - for boxes: [x_scale, y_scale, z_scale]
#       - for spheres: radius    <---- note that this isn't a list, just a single value
#       - for ellipsoids: [x_scale, y_scale, z_scale]
#       - for capsules: [radius, height_along_z_axis]
#       - for cylinders: [radius, height_along_z_axis]
#   3. Provide the coordinate frame that this this collision object should be represented in.  If it is static with respect
#       to the environment, this should be set to 0.  If, however, this should be coupled with the robot's end effector, for example
#       this should be set to the joint index corresponding to the robot's end effector.
#   4. Provide the rotation (in [rx, ry, rz] euler angles) that the object should be rotated, with respect to the coordinate
#       frame specified in 3
#   5. Provide the translation (in [x,y,z]) that the object should be translated, with respect to the coordinate frame
#       specified in 3
#
#   REMEMBER: The robot's links are AUTOMATICALLY added, so no need to add these into the yaml file.  However, this
#       likely won't include the robot's end effector since its geometry is often not included in the urdf, so a common
#       strategy is to tightly envelope your end effector using an ellipsoid or box object in the yaml file
#       so collisions with the end-effector are avoided as best as possible.
#
#   Please provide the name of the collision file that you have been filling out in the RelaxedIK/Config directory:
#   ex: collision_file_name = 'collision.yaml'
collision_file_name = ''
###########################################################################################################



######################################################################################################
# Step 5b: Generate a robot info file using the following command:
#
#   roslaunch relaxed_ik generate_info_file.launch
######################################################################################################




######################################################################################################
# Step 5c: There should now be a robot info file corresponding to the robot you are currently setting up
#   in the RelaxedIK/Config/info_files directory.  From now on, the solver will get all information directly
#   from this file upon initialization
######################################################################################################




######################################################################################################
# Step 6: load the newly created info file by changing the info_file_name argument in the load_info_file
#   launch file and running the following command:
#
#      roslaunch relaxed_ik load_info_file.launch
#######################################################################################################




######################################################################################################
# Step 7: To see that your collision file was put together accurately, use the following command:
#   rosrun relaxed_ik urdf_viewer_with_collision_info.py
#
#   You will see an rviz scene with collision objects in their specified locations, including the
#   collision capsules on the robot's links.
#   Text will appear above the robot that will either say "No Collision" (in green) or "Collision" (in red)
#   Using the provided GUI pop-up, use the sliders to change the robot's configuration.  Verfiy that
#   the "No Collision" and "Collision" match up with your interpretation of what a collision
#   and non-collision state means for your robot platform.  If it does NOT align with what you have
#   in mind for collision and non-collision states, go back to step 5a and adjust your collision file.
#   This will usually consist of changing the states in the sample_states list.
######################################################################################################




######################################################################################################
# Step 8a: If this is your first time setting up the RelaxedIK solver for a particular robot platform,
#   the solver will need to go through a one-time pre-processing step.  In this process, our method
#   trains a neural network so that the robot can learn about its own geometry so that it avoids
#   collisions with itself and other items defined in your collision yaml file, as well as learns
#   to avoid kinematic singularities.
#
#   IMPORTANT: Make sure you are happy with the results in Step 5b before training the neural network!
#
#   To start this process, first run the following command:
#
#       rosrun relaxed_ik generate_input_and_output_pairs.py
#
#   As the name implies, this script will generate hundreds of thousands of input and output pairs to be
#   used for the learning process.  After this script finishes (usually takes between 10 - 20 minutes),
#   run the following command to initialize the learning process: 
#
#       roslaunch relaxed_ik preprocessing_rust.launch
#
#   The system will immediately start training the nueral network.
#   This process will take about 5 - 20 minutes, depending on the robot and number of degrees of freedom
######################################################################################################



######################################################################################################
# Step 8b (optional): If you would also like the option of running the python or julia variants of RelaxedIK,
#   you will have to run separate preprocessing steps to learn neural networks in those environments.
#   The python and julia versions of the solver run considerably slower than the default Rust version; however,
#   we still want to support these versions going forward in case people would still like to use them.
#
#   To train the python version, run the following command:
#
#       roslaunch relaxed_ik preprocessing_python.launch
#
#
#   To train the julia version, run the following command:
#
#   roslaunch relaxed_ik preprocessing_julia.launch
######################################################################################################



######################################################################################################
# Step 9a: Now that the solver has gone through its preprocessing, you can now use the relaxedIK
#   solver as a standalone ROS node.  To start the default Rust solver, please first ensure that the 
#   Rust code has been compiled first (explained in the preliminaries section of the ReadMe)! Next, load 
#   a desired info file using the command found in Step 7.
#
#   For the rust version of the solver (recommended), run the following command:
#       roslaunch relaxed_ik relaxed_ik_rust.launch
#
#   Note: The first time this command is run, it will take a few minutes to compile.  It will only take
#   this long the first time it's ever built though, after that it'll start immediately with this command.  
#
#   (optional)
#   If you would like to start up the python or julia versions of the solver, please use one of the following
#   commands:
#
#   For the Python version of the solver, run:
#       roslaunch relaxed_ik relaxed_ik_python.launch
#
#   For the Julia version of the solver, run:
#       roslaunch relaxed_ik relaxed_ik_julia.launch
#   (NOTE: The julia version takes a little while to do its JIT compilation, ~20-40 seconds in testing)
#
#   Using one of these commands, your relaxed_ik solver will initialize in its own node and will await
#   end-effector pose goal commands.  Refer to step 9b for instructions on publishing end-effector
#   pose goals and receiving solutions.
######################################################################################################


######################################################################################################
# Step 9b: To receive solutions from the relaxed_ik node launched in Step 9a, you first have to publish
#   end effector pose goals for each of the end effectors in the kinematic chain.  The relaxed_ik package
#   provides a custom message called EEPoseGoals which encapsulates all necessary pose goal information.
#
#   The EEPoseGoals message has the following fields:
#      std_msgs/Header header
#      geometry_msgs/Pose[] ee_poses
#
#   The header is a standard header that can include a time stamp, sequence number, or frame id.
#   The ee_poses field should contain the end effector poses for all end effectors specified in Step 3c.
#   These poses in ee_poses should consist of a position goal (x,y,z) and quaternion orientation goal (w,x,y,z)
#   for all end effectors.
#
#   IMPORTANT: All position goals and orientation goals are specified with respect to the INITIAL CONFIGURATION
#       specified in Step 3d.  For example, for a robot platform with two end effectors, ee_poses of
#       < pose1: Point:[0,0,0],Orientation:[1,0,0,0], pose2: Point:[0,0,0],Orientation:[1,0,0,0] >
#       will just return the initial configuration specified in Step 3d.
#
#   To get a solution from relaxed_ik, publish EEPoseGoals messages on the topic '/relaxed_ik/ee_pose_goals'
#   A solution (i.e., a vector of joint angles) will be published on the topic '/relaxed_ik/joint_angle_solutions'
#
#   Solutions will be of a message type called JointAngles, which is another custom message type
#   in the relaxed_ik package.
#
#   The JointAngles message has the following fields:
#       std_msgs/Header header
#       std_msgs/Float32[] angles
#
#   The header is a standard header that corresponds to the exact header from the input EEPoseGoals message
#   (the header sequence number can be used to get a correspondence between input pose goals and output joint solutions
#   in a stream of solutions)
#   The angles field contains the joint angle solutions as Float32 values, adhering to the naming order
#   provided in step 3b when the configuration file was created.
#######################################################################################################


######################################################################################################
# Step 10: To verify that your solver has been set up properly, we provide a control simulation environment
#   using keyboard inputs.  To start, ensure that a version of relaxedIK has been initialized, following
#   Steps 7 and 9a.  Next, start up the rviz viewer by running the following command:
#
#   roslaunch relaxed_ik rviz_viewer.launch
#
#   Once this command is run, you should see an rviz window come up, and your robot platform should be
#   stationary in the initial configuation you specified for the info file in step 3d.
#   NOTE: if you ever want to CHANGE this initial configuration, change it in the robot's info file, NOT in this
#   start_here.py file!!
#
#   Next, we provide a simple keyboard interface in order to pass end-effector pose goals to the solver.
#   This keyboard interface has been designed just for testing.  To start up the keyboard controller, use
#   the following command:
#
#   rosrun relaxed_ik keyboard_ikgoal_driver.py
#
#   The keyboard controller handles up to two chains in the robot platform (if you only have a single chain in your
#   system, the solver will know not to listen to the single end-effector goal).  To use the keyboard controller,
#   first ensure that the termainal window where the keyboard_ikgoal_driver script was run from has focus (i.e.,
#   make sure it's clicked), then use the following keystrokes:
#
#   c - kill the controller controller script
#   w - move chain 1 along +X
#   x - move chain 1 along -X
#   a - move chain 1 along +Y
#   d - move chain 1 along -Y
#   q - move chain 1 along +Z
#   z - move chain 1 along -Z
#   1 - rotate chain 1 around +X
#   2 - rotate chain 1 around -X
#   3 - rotate chain 1 around +Y
#   4 - rotate chain 1 around -Y
#   5 - rotate chain 1 around +Z
#   6 rotate chain 1 around -Z
#
#   i - move chain 2 along +X
#   m - move chain 2 along -X
#   j - move chain 2 along +Y
#   l - move chain 2 along -Y
#   u - move chain 2 along +Z
#   n - move chain 2 along -Z
#   = - rotate chain 2 around +X
#   - - rotate chain 2 around -X
#   0 - rotate chain 2 around +Y
#   9 - rotate chain 2 around -Y
#   8 - rotate chain 2 around +Z
#   7 - rotate chain 2 around -Z
######################################################################################################


# Step-by-step guide ends here!


########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
