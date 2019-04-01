__author__ = 'drakita'


from urdf_parser_py.urdf import URDF
from ..Spacetime.arm import *


# try:
#     from ..Spacetime.boost import Arm_ext
# except:
#     print 'ERROR when importing boost library extension.  Defaulting to python implementation (which will be slower).  ' \
#           'To get speed boost, please install and configure the boost python library: ' \
#           'https://www.boost.org/doc/libs/1_67_0/more/getting_started/unix-variants.html'
#     arm_c = False

from colors import *
import kdl_parser_py.urdf as pyurdf
import PyKDL as kdl

'''
NOTE:
These functions require kdl_parser_py and urdf_parser_py to be installed

commands to install these:
>> sudo apt-get install ros-[your ros distro]-urdfdom-py

>> sudo apt-get install ros-[your ros distro]-kdl-parser-py
>> sudo apt-get install ros-[your ros distro]-kdl-conversions

'''

def urdf_load(urdfString, startJoint, endJoint, full_joint_list, fixed_ee_joint = None, Debug=False):
    '''
    Takes in a urdf file and parses that into different object representations of the robot arm

    :param urdfString: (string) file path to the urdf file.  example: < 'mico.urdf' >
    :param startJoint: (string) name of the starting joint in the chain
    :param endJoint: (string) name of the final joint in the chain
        NOTE: this is the final ROTATING joint in the chain, for a fixed joint on the end effector, add this as the fixed joint!
    :param fixed_ee_joint (string) name of the fixed joint after end joint in the chain.  This is read in just to get a final
        displacement on the chain, i.e. usually just to add in an extra displacement offset for the end effector
    :return: returns the robot parsed object from urdf_parser, Mike's "arm" version of the robot arm, as well as the kdl tree
    '''

    if urdfString == '':
        urdf_robot = URDF.from_parameter_server()
        (ok, kdl_tree) = pyurdf.treeFromParam('/robot_description')
    else:
        urdf_robot = URDF.from_xml_file(urdfString)
        (ok, kdl_tree) = pyurdf.treeFromFile(urdfString)

    if not (startJoint == '' or endJoint == ''):
        chain = kdl_tree.getChain(startJoint, endJoint)
    if full_joint_list == ():
        arm, arm_c = convertToArm(urdf_robot, startJoint, endJoint, fixed_ee_joint, Debug=Debug)
    else:
        arm, arm_c = convertToArmJointList(urdf_robot, full_joint_list, fixed_ee_joint, Debug=Debug)

    if Debug:
        o = open('out', 'w')
        o.write(str(urdf_robot))

    return urdf_robot, arm, arm_c, kdl_tree

def convertToArmJointList(urdf_robot, full_joint_list, fixedJoint, Debug=False):
    if urdf_robot == None:
        raise ValueError('Incorrect Argument in convertToArm.  urdf_robot is None type.')

    joints = urdf_robot.joints

    name = urdf_robot.name
    axes = []
    offset = []
    displacements = []
    rotOffsets = []
    joint_limits = []
    velocity_limits = []
    joint_types = []
    firstPass = True

    for j in full_joint_list:
        for js in joints:
            if js.name == j:
                if firstPass:
                    joint_types.append(js.type)
                    offset = tuple(js.origin.xyz)
                    rotOffsets.append(tuple(js.origin.rpy))
                    if not js.type == 'fixed':
                        axes.append(toAxisLetter(js.axis))
                        joint_limits.append((js.limit.lower, js.limit.upper))
                        velocity_limits.append(js.limit.velocity)
                    firstPass = False
                else:
                    joint_types.append(js.type)
                    displacements.append(tuple(js.origin.xyz))
                    rotOffsets.append(tuple(js.origin.rpy))
                    if not js.type == 'fixed':
                        axes.append(toAxisLetter(js.axis))
                        joint_limits.append((js.limit.lower, js.limit.upper))
                        velocity_limits.append(js.limit.velocity)


        # add any additional joints in the chain listed after the end joint
    if not fixedJoint == None:
        currJoint = []
        for j in joints:
            if j.name == fixedJoint:
                currJoint = j
                displacements.append(tuple(currJoint.origin.xyz))
                rotOffsets.append(tuple(currJoint.origin.rpy))
        if currJoint == []:
            print bcolors.FAIL + 'fixed_ee_joint: {} not found!'.format(fixedJoint) + bcolors.ENDC
            raise Exception('Invalid fixed_ee_joint.  Exiting.')


    numDOF = len(axes)
    # rotOffsets = rotOffsets[0:numDOF]

    if Debug:
        outStr = 'name:\n {} \n axes:\n {} \n displacements:\n {} \n ' \
                 'rotOffsets:\n {} \n offset:\n {} offset'.format(name, tuple(axes), displacements, rotOffsets, offset)

        print outStr

    # if not arm_c:
    #     arm = Arm(tuple(axes), displacements, rotOffsets, offset, name)
    # else:
    #     arm = Arm_ext.Arm(list(axes), displacements, rotOffsets, offset, name)
    # arm.velocity_limits = velocity_limits
    # arm.joint_limits = joint_limits


    arm = Arm(tuple(axes), displacements, rotOffsets, offset, name)
    # arm_c = Arm_ext.Arm(list(axes), displacements, rotOffsets, offset, name)
    arm_c = None
    arm.velocity_limits = velocity_limits
    arm.joint_limits = joint_limits
    arm.joint_types = joint_types
    # arm_c.velocity_limits = velocity_limits
    # arm_c.joint_limits = joint_limits
    return arm, arm_c


def convertToArm(urdf_robot, startJoint, endJoint, fixedJoint, Debug=False):
    '''
    This function parses the (axes, offset, displacements, rotOffsets) in order to return an Arm
    from mg's spacetime code

    :param urdf_robot: urdf_robot object returned from URDF.from_xml_file
    :param startJoint: start joint string, e.g. <'shoulder_pan_joint'> for UR5
    :param endJoint:  end joint string, e.g. <'ee_fixed_joint'> for UR5
    :return: Arm object
    '''

    if urdf_robot == None:
        raise ValueError('Incorrect Argument in convertToArm.  urdf_robot is None type.')

    joints = urdf_robot.joints

    s = []
    e = []
    for j in joints:
        if j.name == startJoint:
            s = j
        if j.name == endJoint:
            e = j
    if s == []:
        print bcolors.FAIL + 'startJoint: {} not found in joint list!  Please check to make sure startJoint is a joint found in' \
                         'the URDF and is spelled correctly'.format(startJoint) + bcolors.ENDC
        raise ValueError('Invalid Value.  Exiting.')
    if e == []:
        print bcolors.FAIL + 'endJoint: {} not found in joint list!  Please check to make sure endJoint is a joint found in' \
                         'the URDF and is spelled correctly'.format(endJoint).format(startJoint) + bcolors.ENDC
        raise ValueError('Invalid Value.  Exiting.')

    name = urdf_robot.name
    axes = []
    displacements = []
    rotOffsets = []
    joint_limits = []
    velocity_limits = []

    currJoint = s
    axes.append(toAxisLetter(currJoint.axis))
    offset = tuple(currJoint.origin.xyz)
    rotOffsets.append(tuple(currJoint.origin.rpy))
    joint_limits.append((currJoint.limit.lower, currJoint.limit.upper))
    velocity_limits.append(currJoint.limit.velocity)

    currJoint = findNextJoint(joints, currJoint.child)

    while True:
        axes.append(toAxisLetter(currJoint.axis))
        displacements.append(tuple(currJoint.origin.xyz))
        rotOffsets.append(tuple(currJoint.origin.rpy))
        joint_limits.append((currJoint.limit.lower, currJoint.limit.upper))
        velocity_limits.append(currJoint.limit.velocity)
        if currJoint.name == endJoint:
            break
        currJoint = findNextJoint(joints, currJoint.child)

    # add any additional joints in the chain listed after the end joint
    if not fixedJoint == None:
        currJoint = []
        for j in joints:
            if j.name == fixedJoint:
                currJoint = j
                displacements.append(tuple(currJoint.origin.xyz))
        if currJoint == []:
            print bcolors.FAIL + 'fixed_ee_joint: {} not found!'.format(fixedJoint) + bcolors.ENDC
            raise Exception('Invalid fixed_ee_joint.  Exiting.')

    numDOF = len(axes)
    rotOffsets = rotOffsets[0:numDOF]

    if Debug:
        outStr = 'name:\n {} \n axes:\n {} \n displacements:\n {} \n ' \
                 'rotOffsets:\n {} \n offset:\n {} offset'.format(name, tuple(axes), displacements, rotOffsets, offset)

        print outStr


    arm = Arm(tuple(axes), displacements, rotOffsets, offset, name)
    # arm_c = Arm_ext.Arm(list(axes), displacements, rotOffsets, offset, name)
    arm_c = None
    arm.velocity_limits = velocity_limits
    arm.joint_limits = joint_limits
    # arm_c.velocity_limits = velocity_limits
    # arm_c.joint_limits = joint_limits
    return arm, arm_c

def toAxisLetter(ax):
    if ax == None:
        return ''
    ax_val = ''
    if ax[0] == 1:
        ax_val = 'x'
    elif ax[0] == -1:
        ax_val = '-x'
    elif ax[1] == 1:
        ax_val = 'y'
    elif  ax[1] == -1:
        ax_val = '-y'
    elif ax[2] == 1:
        ax_val = 'z'
    elif ax[2] == -1:
        ax_val = '-z'
    return ax_val

def findNextJoint(joints, child):
    for j in joints:
        if j.parent == child:
            return j
    print bcolors.FAIL + 'joint with matching parent link: {} not found!'.format(child) + bcolors.ENDC
    raise Exception('Invalid joint chain.  Exiting.')
