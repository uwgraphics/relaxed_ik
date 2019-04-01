__author__ = 'gleicher'

# stuff for doing articulated figures
import robot_function

import math
# import adInterface as AD
import numpy as N
from numbers import Number
import copy
# from numba import jitclass
import time


# sin = AD.MATH.sin
# cos = AD.MATH.cos
sin = math.sin
cos = math.cos


class TwoLink(robot_function.RobotFunction):
    """
    Simple 2D articulated arm. Uses agles. Assumes unit segment lengths.
    """
    def __init__(self):
        robot_function.RobotFunction.__init__(self, 2, 3, "Arm2D")
        self.noZ = True

    def constraint(self, **kwargs):
        return [],[]

    def __call__(self, state):
        return [ (0,0),
                 (cos(state[0]), sin(state[0]) ),
                 (cos(state[0]) + cos(state[0]+state[1]), sin(state[0])+ sin(state[0]+state[1]))
               ]

#####
# straightforward way to deal with the linear algebra
# not necessarily efficient
def rotMatrix(axis, s, c):
    if axis=="Z" or axis=="z":
        return N.array([[c,-s,0,0], [s,c,0,0], [0,0,1,0], [0,0,0,1]])
    elif axis=="Y" or axis=="y":
        return N.array([[c,0,s,0], [0,1,0,0], [-s,0,c,0], [0,0,0,1]])
    elif axis=="X" or axis=="x":
        return N.array([[1,0,0,0], [0,c,-s,0], [0,s,c,0], [0,0,0,1]])
    else:
        print "Unsupported Axis:", axis
        raise NotImplementedError

'''
def rotTransMatrix(axis, s, c, t):
    """
    build a rotate * translate matrix - MUCH faster for derivatives
    since we know there are a ton of zeros and can act accordingly
    :param axis: x y or z as a character
    :param s: sin of theta
    :param c: cos of theta
    :param t: translation (a 3 tuple)
    :return:
    """
    if axis=="Z" or axis=="z":
        return N.array([[c,-s,0,AD.fastLC2(c,t[0],s,-t[1])],
                        [s, c,0,AD.fastLC2(s,t[0],c, t[1])],
                        [0, 0,1,t[2]],
                        [0, 0,0,1]])
    elif axis=="Y" or axis=="y":
        return N.array([[c, 0,s,AD.fastLC2(c, t[0],s,t[2])],
                        [0, 1,0,t[1]],
                        [-s,0,c,AD.fastLC2(s,-t[0],c,t[2])],
                        [0,0,0,1]])
    elif axis=="X" or axis=="x":
        return N.array([[1,0, 0,t[0]],
                        [0,c,-s,AD.fastLC2(c,t[1],s,-t[2])],
                        [0,s, c,AD.fastLC2(s,t[1],c, t[2])],
                        [0,0,0,1]])
    else:
        print "Unsupported Axis:", axis
        raise NotImplementedError
'''

def rot3(axis, s, c):
    """
    build a rotate * translate matrix - MUCH faster for derivatives
    since we know there are a ton of zeros and can act accordingly
    :param axis: x y or z as a character
    :param s: sin of theta
    :param c: cos of theta
    :param t: translation (a 3 tuple)
    :return:
    """
    if axis=="Z" or axis=="z" or axis=='-z':
        return N.array([[c,-s,0.0], [s,c,0.0], [0.0,0.0,1.0] ])
    elif axis=="Y" or axis=="y" or axis == '-y':
        return N.array([[c,0.0,s], [0.0,1.0,0.0], [-s,0.0,c] ])
    elif axis=="X" or axis=="x" or axis == '-x':
        return N.array([[1.0,0.0,0.0], [0.0,c,-s], [0.0,s,c] ])
    else:
        print "Unsupported Axis:", axis
        raise NotImplementedError

def eulerTupleTo3x3(t):
    """
    given an XYZ tuple, return a rotation matrix

    the order is ZYX because angles are in the parent system

    note: this does it the slow way, but that's OK, since this is only used at robot setup
    :param t: a tuple (x,y,z)
    :return: a 3x3 matrix
    """
    xm = rot3('X',math.sin(t[0]),math.cos(t[0]))
    ym = rot3('Y',math.sin(t[1]),math.cos(t[1]))
    zm = rot3('Z',math.sin(t[2]),math.cos(t[2]))
#    xy = xm.dot(ym)
#    return xy.dot(zm)
    zy = zm.dot(ym)
    return zy.dot(xm)


def rotTransMatrixNOAD(axis, s, c, t):
    """
    build a rotate * translate matrix - MUCH faster for derivatives
    since we know there are a ton of zeros and can act accordingly
    :param axis: x y or z as a character
    :param s: sin of theta
    :param c: cos of theta
    :param t: translation (a 3 tuple)
    :return:
    """
    if axis=="Z" or axis=="z":
        return N.array([[c,-s,0, c*t[0] - s * t[1]],
                        [s, c,0, s*t[0] + c * t[1]],
                        [0, 0,1,t[2]],
                        [0, 0,0,1]])
    elif axis=="Y" or axis=="y":
        return N.array([[c, 0,s, c * t[0] + s * t[2]],
                        [0, 1,0,t[1]],
                        [-s,0,c, c * t[2] - s * -t[0]],
                        [0,0,0,1]])
    elif axis=="X" or axis=="x":
        return N.array([[1,0, 0,t[0]],
                        [0,c,-s, c*t[1] - s * t[2]],
                        [0,s, c, s*t[1] + c * t[2]],
                        [0,0,0,1]])
    else:
        print "Unsupported Axis:", axis
        raise NotImplementedError


def transMatrix(vector):
    m = N.eye(4)
    m[0:3,3] = vector[0:3]
    return m

def multV(matrix,vector):
    if len(vector)==3:
        return N.dot(matrix,(vector[0],vector[1],vector[2],1))[0:3]
    else:
        return N.dot(matrix,vector)

def translate(matrix,vector):
    matrix[0:3,3] += vector[0:3]

def getTrans(matrix):
    return matrix[0:3,3]

twopi = 2*math.pi
def despin(a):
    """
    this makes an angle be +/- pi
    """
    na = ((a+math.pi) % twopi) - math.pi
    return na

def despinSeries(array):
    """
    does the de-spin on an array. rather than having an absolute boundary, flips things such that
    its close to the prior. this begins at the first value and goes from there. it does not change
    the first value.
    it modifies the array in place
    :param array: should be able to take a keyvector
    :return: the number of elements changed
    """
    nchanged = 0
    for i in range(1,len(array)):
        cur  = array[i]
        prev = array[i-1]
        for j in range(len(cur)):
            d = cur[j]-prev[j]
            if d>3:
                cur[j] -= twopi
                nchanged += 1
            elif d<-3:
                cur[j] += twopi
                nchanged += 1
    return nchanged

# despin a series given as a vector - but we need to know the number vars and states
def despinArray(array,nvars):
    nstates = len(array)/nvars
    nchanged = 0
    for i in range(1,nstates):
        for j in range(nvars):
            idx = i*nvars + j
            d = array[idx] - array[(i-1)*nvars+j]
            if d>3:
                array[idx] -= twopi
                nchanged += 1
            elif d<-3:
                array[idx] += twopi
                nchanged += 1
    return nchanged


# naively assume that all variables are angles, and should be "de-spun"
def deSpinCB(dsv):
    for i in range(len(dsv)):
        dsv[i] = despin(dsv[i])

#
# if we model things with variables "s" and "c" compute the X and Y from them
# this should be just x = c and y = s, but we need to deal with normalization
def normSC(s,c):
    d2 = s*s+c*c
    if d2>.001:
        d = math.sqrt(d2)
    else:
        d = math.sqrt(d2)
    if d > .1:
        return s/d,c/d
    else:
        # things have vanished, so we basically we have no transform
        # we could just return the identity, but we want to fade to that
        # when d = .01, return s,1+c
        if d<.01:
            d=.01
        a = (d-.01) / .09  # .1-.01
        a1 = 1-a
        return a*s/d + a1*s, a1 * (1+c) + a * c/d

# we model robot arms as a series of 1-axis joints, each with a rotation between them
# for now, the axes have to be X,Y or Z - this will need to change at some point
#
# a 2 axis joint would be a zero displacement - change later
#
# rotations first (so the "base" rotation is at the origin) - this would be easy to fix
#
# warning: this does everything with general 4x4 matrices - and is probably
# way inefficient
#
# the rotational offsets take things as X,Y,Z Euler angles - which is the way that ROS does
# it, so we can try to be compatible with ROS
#
# while i hate that this is AD aware, it is so much in the critical path
# that its worth making it fast
class Arm(robot_function.RobotFunction):
    def __init__(self, axes=[], displacements=[], rotOffsets=None, dispOffset=(0,0,0), name="Arm",rep="angle"):
        # we allow to specify the axes as "Z" - in which case all joints are Z
        if axes=="Z" or axes=="z":
            self.axes = ["Z"] * len(displacements)
            noZ = True
        else:
            self.axes = axes
            noZ = False
        # try to handle different ways to store angles
        self.rep = rep
        if rep=="angle":
            self.varsPerJoint = 1
        elif rep=="sc":
            self.varsPerJoint = 2
        else:
            raise NameError("Bad Angle Type to Arm!")
        # the displacements should be a list of tuples, but sometimes its convenient
        # to just give the X axis
        self.displacements = [ ( [t,0,0] if isinstance(t,Number) else list(t) ) for t in displacements]
        # create a list of the rotational offsets
        if rotOffsets == None:
            self.rotOffsets = None
        else:
            self.original_rotOffsets = rotOffsets
            self.rotOffsets = [ eulerTupleTo3x3(t) if not(t is None) else None for t in rotOffsets]
        # now we're ready to initialize
        robot_function.RobotFunction.__init__(self, _nvars=len(self.axes) * self.varsPerJoint, _npoints=len(self.axes) + 1, _name=name)
        self.noZ = noZ
        if self.rep == "angle":
            # self.varnames = [ "J%d.%c" % (jn,ax) for jn,ax in enumerate(self.axes)]

            self.cleanupCallback = deSpinCB # not always what we want
            self.xUBounds = N.full(self.nvars, twopi)
            self.xLBounds = N.full(self.nvars, -twopi)
            self.default = N.full(self.nvars,.1)
        elif self.rep == "sc":
            self.varnames = []
            for jn,ax in enumerate(self.axes):
                self.varnames.append("J%d.%c.S" % (jn,ax))
                self.varnames.append("J%d.%c.C" % (jn,ax))
                self.default[jn*2  ] = sin(math.radians(15))
                self.default[jn*2+1] = cos(math.radians(15))
            self.cleanupCallback = None
            self.xUBounds = N.full(self.nvars, 2)
            self.xLBounds = N.full(self.nvars, -2)
        self.dispOffset = dispOffset

        # DR: initialze a Jacobian matrix
        self.numDOF =  len(axes)
        self.jacobianMat = N.zeros((6,self.numDOF))
        self.joint_limits = []
        self.velocity_limits = []
        self.original_displacements = copy.deepcopy(self.displacements)


    def cleanupMode(self, mode="array"):
        if self.rep=="angle":
            if mode=="array":
                self.cleanupCallback = lambda x: despinArray(x,self.nvars)
            elif mode=="perframe":
                self.cleanupCallback = deSpinCB
            elif mode==None:
                self.cleanupCallback = None
            else:
                raise NameError("Bad Cleanup Mode")
        else:
            self.cleanupCallback = None

    def constraint(self, state, **kwargs):
        if self.rep=="sc":
            return [state[i*2]*state[i*2]+state[i*2+1]*state[i*2+1]-1 for i in range(len(state)/2)],[]
        else:
            return [],[]

    def __call__(self, state):
        """
        given the state vector, return all the points
        this is really performance critical for automatic differentiaiton
        so try to figure out if we need a fast path
        :param state:
        :return:
        """
        try:
            if state.dtype == object:
                do_ad = True
            else:
                do_ad = False
        except:
            do_ad = True        # be conservative

        pt = N.array(self.dispOffset)
        pts = [ self.dispOffset ]
        rot = N.eye(3)
        for i,axis in enumerate(self.axes):
            if self.varsPerJoint == 1:
                if do_ad == False:
                    s = math.sin(state[i])
                    c = math.cos(state[i])
                else:
                    s = sin(state[i])
                    c = cos(state[i])
            else:
                s,c = normSC(state[i*2], state[i*2+1])
            # since we know that the rot matrix doesn't change, and that
            # this is an affine thing, we can do this a bit more quickly
            #lmat = N.dot(rotMatrix(axis,s,c) , transMatrix(self.displacements[i]))
            if self.rotOffsets:
                if not (self.rotOffsets[i] is None):
                    rot = rot.dot(self.rotOffsets[i])
            rmat = rot3(axis,s,c)
            rot = rot.dot(rmat)
            pt = rot.dot(self.displacements[i]) + pt
            pts.append( pt )
        return pts


    def getFrames(self,state):
        """
        given the state vector, return all the points
        this is really performance critical for automatic differentiaiton
        so try to figure out if we need a fast path
        :param state:
        :return:
        """
        try:
            if state.dtype == object:
                do_ad = True
            else:
                do_ad = False
        except:
            do_ad = True        # be conservative

        do_ad = False
        pt = N.array(self.dispOffset)
        pts = [ self.dispOffset ]
        rot = self.rotOffsets[0]
        frames = [ rot ]
        axis_idx = 0

        for i,disp in enumerate(self.displacements):
            if self.joint_types[i] == 'revolute' or self.joint_types[i] == 'continuous':
                axis = self.axes[axis_idx]
                if self.varsPerJoint == 1:
                    if do_ad == False:
                        if axis[0] == '-':
                            s = math.sin(-state[axis_idx])
                            c = math.cos(-state[axis_idx])
                        else:
                            s = math.sin(state[axis_idx])
                            c = math.cos(state[axis_idx])
                    else:
                        if axis[0] == '-':
                            s = sin(-state[axis_idx])
                            c = cos(-state[axis_idx])
                        else:
                            s = sin(state[axis_idx])
                            c = cos(state[axis_idx])
                else:
                    s,c = normSC(state[i*2], state[i*2+1])

                axis_idx += 1


            if self.joint_types[i] == 'prismatic':
                axis = self.axes[axis_idx]
                if axis == 'x':
                    self.displacements[i][0] = self.original_displacements[i][0] + state[axis_idx]
                elif axis == 'y':
                    self.displacements[i][1] = self.original_displacements[i][1] + state[axis_idx]
                elif axis == 'z':
                    self.displacements[i][2] = self.original_displacements[i][2] + state[axis_idx]
                else:
                    raise Exception('invalid prismatic joint.  Axis must be [1 0 0], [0 1 0], or [0 0 1]')

                axis_idx += 1
                    # since we know that the rot matrix doesn't change, and that
                # this is an affine thing, we can do this a bit more quickly
                #lmat = N.dot(rotMatrix(axis,s,c) , transMatrix(self.displacements[i]))

            if self.joint_types[i] == 'revolute' or self.joint_types[i] == 'continuous':
                rmat = rot3(axis,s,c)
                rot = rot.dot(rmat)

            pt = rot.dot(self.displacements[i]) + pt

            if self.rotOffsets:
                if not (self.rotOffsets[i+1] is None):
                    rot = rot.dot(self.rotOffsets[i+1])

            # print end - start
            pts.append( pt )
            frames.append(rot)
        return pts,frames


    def getJacobian(self, state):
        '''
        added by DR
        returns the 6xn Jacobian matrix corresponding to the robot given the provided state.
        This will be used to get a first IK solution used a damped least squares pseudoinverse solve.
        Currently only works with revolute joints.  But could easily be extended to prismatic joints if need be.
        :param state:
        :return:
        '''
        # do not initialize a new numpy matrix each time (this will be called a lot).
        # instead, just use a single numpy array of the class

        frames = self.getFrames(state)
        ptArr = frames[0]
        del ptArr[0]
        rotMatArr = frames[1]
        del rotMatArr[0] # delete the first element, it's always identity

        eePt = N.array(ptArr[-1])
        for i,a in enumerate(self.axes):
            disp = eePt - N.array(ptArr[i])
            if a == 'x':
                pAxis = N.array(rotMatArr[i][:,0])
            elif a == 'y':
                pAxis = N.array(rotMatArr[i][:,1])
            elif a == 'z':
                pAxis = N.array(rotMatArr[i][:,2])

            self.jacobianMat[0:3,i] = N.cross(pAxis,disp)
            # self.jacobianMat[0:3, i] = self.cross(pAxis, disp)
            self.jacobianMat[3:6,i] = pAxis


        return self.jacobianMat

    def cross(self, a, b):
        c = [a[1] * b[2] - a[2] * b[1],
             a[2] * b[0] - a[0] * b[2],
             a[0] * b[1] - a[1] * b[0]]

        return N.array(c)

    def getJacobian_fromFrames(self, frames):

        ptArr = frames[0]
        del ptArr[0]
        rotMatArr = frames[1]
        del rotMatArr[0]  # delete the first element, it's always identity



        eePt = N.array(ptArr[-1])

        for i, a in enumerate(self.axes):
            disp = eePt - N.array(ptArr[i])
            if a == 'x':
                pAxis = N.array(rotMatArr[i][:, 0])
            elif a == 'y':
                pAxis = N.array(rotMatArr[i][:, 1])
            elif a == 'z':
                pAxis = N.array(rotMatArr[i][:, 2])


            self.jacobianMat[0:3, i] = N.cross(pAxis, disp)
            # self.jacobianMat[0:3, i] = self.cross(pAxis, disp)
            self.jacobianMat[3:6, i] = pAxis



        return self.jacobianMat

    def getYoshikawaMeasure(self, state):
        '''
        added by DR
        returns the yoshikawa manipulability measure to be used as part of
        the singularity avoidance constraint
        :return:
        '''
        j = self.getJacobian(state)
        jT = N.transpose(j)
        jjT = N.dot(j,jT)
        det = abs(N.linalg.det(jjT))
        return math.sqrt(det)

    def getYoshikawaMeasure_fromFrames(self, frames):
        j = self.getJacobian_fromFrames(frames)
        jT = N.transpose(j)
        jjT = N.dot(j,jT)
        det = abs(N.linalg.det(jjT))
        return math.sqrt(det)


    def getMatrixConditioningMeasure(self, state):
        j = self.getJacobian(state)
        U,s,V = N.linalg.svd(j)
        # print s
        singVal_largest = s[0]
        singVal_smallest = s[-1]
        return  singVal_smallest / singVal_largest

    def getAxesIDs(self):
        ret = self.numDOF*[-1]
        for i,a in enumerate(self.axes):
            if a == 'x':
                ret[i] = 0
            elif a == 'y':
                ret[i] = 1
            else:
                ret[i] = 2

        return ret


# make first displacement from 0  (mico 0 0 .153)
# apply X Y Z rot offsets (z is pointing down)
# a pply joint (which is about Z)

# apply next displacement from point (mico 0 0 -.1 - move farther up)
# apply X Y Z

# doesn't have an end-effector (6 offsets) - need to add 7th offset
# the first joint's displacement is the global offset

class Old_Reactor(Arm):
    def __init__(self, *args, **kwargs):
        Arm.__init__(self,
                        axes=['Y','Z','Z','Z','X'],
                         displacements=[(0,125,0),(40,140,0),(142,0,0),(155,0,0),(0,0,0) ],
                         name="Reactor",
                        *args, **kwargs
                     )
        pass


# new reactor model based on URDF
reactor_joint_names = ['shoulder_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll']
reactor_joint_axes  = ['z','y','y','y','x']
reactor_displacements = [ (0, 0, 0.081) , (0, 0, 0.0265) ,
                          (-0.1445, 0, 0.0385) ,  # (0.03851,0,0.14434), #
                          (0.1535, 0, 0) , (0.071, 0, 0) ]
reactor_end = (0.0762, 0, 0)
reactor_disp_end = reactor_displacements[1:]
reactor_disp_end.append(reactor_end)
reactor_rotations = [ (0, 0, 0) , (0, math.radians(-270), 0) , (0, math.radians(-90), math.radians(180)) , (0, 0, 0) , (0, 0, 0) ]

reactor_neutral = N.array([0, 0, 0, 0, 0])
reactor_rest =  N.array([math.radians(x) for x in [0, -90, -90, -32, 0]])
reactor_ready = N.array([math.radians(x) for x in [0, -50, -35, -60, 0]])

# CB - for new 2 point path
reactor_ready2 = N.array([0.0, -0.7552, -0.0023, -0.8812, 0.0])
reactor_inter = N.array([0.0, -0.1924, -0.5707, 0.3783, 0.0])

class Reactor(Arm):
    def __init__(self,*args,**kwargs):
        Arm.__init__(self,reactor_joint_axes,
                     displacements=reactor_disp_end,
                     dispOffset=reactor_displacements[0],
                     name="Reactor",
                     rotOffsets=reactor_rotations,
                     *args, **kwargs)
        self.default = N.array(reactor_ready)

micoRotOffsets_old = [ [180, 0, 0] , [-90, -90, 0] , [180, 0, 180] , [0, -90, 0] , [0, 55, 180] , [0, 55, 180] ]
micoRotOffsetsRads_old = [ (math.radians(t[0]), math.radians(t[1]),math.radians(t[2])) for t in micoRotOffsets_old ]

micoRotOffsets = [ [180, 0, 0] , [-90, -90, 0] , [180, 0, 180] , [0, -90, 0] , [0, 60, 180] , [0, 60, 180] ]
micoRotOffsetsRads = [ (math.radians(t[0]), math.radians(t[1]),math.radians(t[2])) for t in micoRotOffsets ]

micoDisplacements_old = [ (0, 0, -0.1185) , (0.29, 0, 0) ,
                      (0.123, 0, -0.00845) , (0.0343, 0, -0.06588) , (0.0343, 0, -0.06588), (0,0,-.16) ]
micoDisplacements = [ (0, 0, -0.1185) , (0.29, 0, 0) ,
                      (0.123, 0, -0.00845) , (0.037, 0, -0.06408) , (0.037, 0, -0.06408), (0,0,-.16) ]

micoReady = [-1.4870590852717527, 3.0559304792307405, 1.371559253533616, -2.0360851903259105, 1.4648866713001893, 1.311377006061951]

class Mico(Arm):
    def __init__(self,*args,**kwargs):
        Arm.__init__(self,
                     axes = ['z'] * 6,
                     dispOffset= (0, 0, 0.1535) ,
                     displacements = micoDisplacements,
                     rotOffsets=micoRotOffsetsRads,
                     name = "Mico",
                     *args,**kwargs
                     )

jacoRotOffsets = [ [180, 0, 0] , [-90, -90, 0] , [180, 0, 180] , [0, -90, 0] , [0, 60, 180] , [0, 60, 180] ]
jacoRotOffsetsRads = [ (math.radians(t[0]), math.radians(t[1]),math.radians(t[2])) for t in jacoRotOffsets ]
jacoDisplacements = [ (0, 0, -0.1185) , (0.41, 0, 0) , (0.207, 0, -0.01125) , (0.0343, 0, -0.06588) , (0.0343, 0, -0.06588), (0,0,-.16) ]

jacoReady = [-1.4870590852717527, 3.0559304792307405, 1.371559253533616, -2.0360851903259105, 1.4648866713001893, 1.311377006061951]
jacoVelocities = [5,5,5,5,5,5]

class Jaco(Arm):
    def __init__(self, *args, **kwargs):
        Arm.__init__(self,
                     axes=['z'] * 6,
                     dispOffset=(0, 0, 0.1535),
                     displacements=jacoDisplacements,
                     rotOffsets=jacoRotOffsetsRads,
                     name="Jaco",
                     *args, **kwargs
                     )
        self.velocities = jacoVelocities


# UR5 Robot
# ur5RotOffsets = [ [0.0, 0.0, 0.0] , [0.0, 90.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 90.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] ]
# DR: changed rotational offset to zero
ur5RotOffsets = [[0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 0.0, 0.0]]
# DR: for ROS/ urScript
# ur5RotOffsets = [[0.0, 0.0, 180.0] , [0.0, -90.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, -90.0, 0.0] , [0.0, 0.0, 0.0] , [0.0, 90.0, 0.0]]
ur5RotOffsetsRads = [ (math.radians(t[0]), math.radians(t[1]),math.radians(t[2])) for t in ur5RotOffsets ]
# ur5Displacements = [ (0, 0.13585, 0) , (0, -0.1197, 0.42500) ,
#                       (0, 0, 0.39225) , (0, 0.093, 0) , (0, 0, 0.09465), (0,0.2423,0) ]
ur5Displacements = [(0.0, 0.13585, 0.0), (0.0, -0.1197, 0.425),
                    (0.0, 0.0, 0.39225), (0, 0.093, 0), (0, 0, 0.09465), (0.0,0.0823,0.0)]
ur5Ready = [0.0, -1.570796, 0.0, 0.0, 0.0, 0.0]

class UR5(Arm):
    def __init__(self,*args,**kwargs):
        Arm.__init__(self,
                     axes = ['z','y','y','y','z','y'],
                     dispOffset= (0, 0, 0.089159) ,
                     displacements = ur5Displacements,
                     rotOffsets=ur5RotOffsetsRads,
                     name = "UR5",
                     *args,**kwargs
                     )
        self.default = N.array(ur5Ready)

# Kuka iiwa7 robot
# added by DR
# 7 DOF simulation test robot for kines control
kukaRotOffsets = [ (0,0,0), (1.57079632679,0,3.14159265359), (1.57079632679,0,3.14159265359), (1.57079632679, 0, 0), (-1.57079632679, 3.14159265359, 0), (1.57079632679, 0, 0), (-1.57079632679, 3.14159265359, 0) ]
kukaDisplacements = [ (0, 0, 0.19), (0, 0.21, 0), (0, 0, 0.19), (0, 0.21, 0), (0, 0.06070, 0.19), (0, 0.081, 0.06070),(0, 0, 0.045)]
kukaAxes = ['z','z','z','z','z','z','z']

iiwa7Ready=[0,0,0,0,0,0,0]
class IIWA7(Arm):
    def __init__(self,*args,**kwargs):
        Arm.__init__(self,
                     axes=kukaAxes,
                     dispOffset=(0, 0, 0.15),
                     displacements=kukaDisplacements,
                     rotOffsets=kukaRotOffsets,
                     name="IIWA7",
                     *args,**kwargs
                     )
        self.default = N.array(iiwa7Ready)

hubo_r_dispOffset = (0.0, -0.242, 0.456)
hubo_r_displacements = [(0.0, 0.0, 0),(0.0, 0.0, 0.0),(0.035, 0, -0.35),(-0.035, 0.0, -0.34), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0326), (0.0, 0.0, -.25)]
hubo_r_axes = ['y','x','z','y','z', 'y', 'z']
hubo_r_init_state = 7*[0.0]
# hubo_r_rotOffsets = [(0.0,0.0,0.0), (-0.244346095279, 0.0, 0.0), (0.0,0.0,0.0), (0.0, -0.296705972839, 0.0), (0.0,0.0,0.0), (0.0,0.0,0.0), (0.0,0.0,0.0)]
hubo_r_rotOffsets = [(0.0,0.0,0.0), (0.0, 0.0, 0.0), (0.0,0.0,0.0), (0.0, 0.0, 0.0), (0.0,0.0,0.0), (0.0,0.0,0.0), (0.0,0.0,0.0)]
hubo_r_velocities = [4.254240051736178, 4.254240051736178, 7.644542123735163, 7.644542123735163, 8.901179185171081, 14.66076571675237, 14.66076571675237]

class Hubo_R(Arm):
    def __init__(self, *args, **kwargs):
        Arm.__init__(self,
                     axes = hubo_r_axes,
                     dispOffset=hubo_r_dispOffset,
                     displacements=hubo_r_displacements,
                     rotOffsets=hubo_r_rotOffsets,
                     name='hubo_r',
                     *args,**kwargs
                     )
        self.velocities = hubo_r_velocities


hubo_l_dispOffset = (0.0, 0.242, 0.456)
hubo_l_displacements = [(0.0, 0.0, 0),(0.0, 0.0, 0.0),(0.035, 0, -0.35),(-0.035, 0.0, -0.34), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0326), (0.0, 0.0, -.25)]
hubo_l_axes = ['y','x','z','y','z', 'y', 'z']
hubo_l_init_state = 7*[0.0]
hubo_l_rotOffsets = [(0.0,0.0,0.0), (-0.244346095279, 0.0, 0.0), (0.0,0.0,0.0), (0.0, -0.296705972839, 0.0), (0.0,0.0,0.0), (0.0,0.0,0.0), (0.0,0.0,0.0)]
hubo_l_velocities = [4.254240051736178, 4.254240051736178, 7.644542123735163, 7.644542123735163, 8.901179185171081, 14.66076571675237, 14.66076571675237]

class Hubo_L(Arm):
    def __init__(self, *args, **kwargs):
        Arm.__init__(self,
                     axes = hubo_l_axes,
                     dispOffset=hubo_l_dispOffset,
                     displacements=hubo_l_displacements,
                     rotOffsets=hubo_l_rotOffsets,
                     name='hubo_l',
                     *args,**kwargs
                     )
        self.velocities = hubo_l_velocities


hubo_ub_dispOffset = (0.0, 0.0, 0.0)
hubo_ub_displacements = [(0.0, -0.242, 0.456),(0.0, 0.0, 0),(0.0, 0.0, 0.0),(0.035, 0, -0.35),(-0.035, 0.0, -0.34), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0326), (0.0, 0.0, -.25)]
hubo_ub_axes = ('z','y','x','z','y','z', 'y', 'z')
hubo_ub_init_state = 8*[0.0]
hubo_ub_rotOffsets = [(0.0,0.0,0.0),(0.0,0.0,0.0), (0.0, 0.0, 0.0), (0.0,0.0,0.0), (0.0, 0.0, 0.0), (0.0,0.0,0.0), (0.0,0.0,0.0), (0.0,0.0,0.0)]
hubo_ub_velocities = [4.254240051736178, 4.254240051736178, 4.254240051736178, 7.644542123735163, 7.644542123735163, 8.901179185171081, 14.66076571675237, 14.66076571675237]

class Hubo_UpperBody(Arm):
    def __init__(self, *args, **kwargs):
        Arm.__init__(self,
                     axes = hubo_ub_axes,
                     dispOffset=hubo_ub_dispOffset,
                     displacements=hubo_ub_displacements,
                     rotOffsets=hubo_ub_rotOffsets,
                     name='hubo_ub',
                     *args,**kwargs
                     )
        self.velocities = hubo_ub_velocities
