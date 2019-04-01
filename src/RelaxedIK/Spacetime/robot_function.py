__author__ = 'gleicher'

"""
A second attempt at a spacetime trajectory optimizer, this time possibly a little
simple and not tied to FuncDesigner

because we don't know how the automatic differentiation will work, we have to
assume it is naive

a robot is basically a function that takes a state vector and produces a list of
3D positions (points)

the function produces ALL of the points on the robot - rather than the specific ones needed.
this is because we don't know about caching, so we might as well compute everything once
(no lazy evaluation)

because I don't know what the best kinds of variables are to have, i am making this
generic - it's not just for robot arms

ideally the function eval is built in a manner that allows for the gradient function
to be evaluated as well (by using AD objects or oovars)
"""

import numpy as N
from itertools import chain

class RobotFunction:
    """
    Abstract class for making a robot. The actual base class doesn't do much.
    """
    def __init__(self, _nvars, _npoints, _name=""):
        self.npoints = _npoints
        self.nvars = _nvars
        self.__name__ = "Robot" if _name=="" else "Robot:%s" % _name
        self.varnames = ["v"] * self.nvars
        self.noZ = False
        self.cleanupCallback = None
        #CB - added for ptipopt (default bounds are +- Inf)
        #MG - infinite bounds are problematic - just pick a big number
        self.xUBounds = N.full(self.nvars,float(1000))
        self.xLBounds = N.full(self.nvars,float(-1000))
        # this used to be a method, but now let's make it a variable that can be adjusted
        self.default = N.zeros(self.nvars)

    def __call__(self, state):
        """
        allows the robot to be treated as a function:
            state -> list(points)
        :param state: a state vector (numpy array or list)
        :return: a list of points
        """
        raise NotImplementedError

    def getFrames(self,state):
        """
        just call call - but returns a two things - the points (like call) and the frames
        :param state:
        :return: a list of points and a list of 3x3 matrices (in global coords)
        """
        pts = self(state)
        eye = N.eye(3)
        return pts, [eye for i in range(len(pts))]

    def constraint(self, **kwargs):
        """
        returns the evaluation of 2 sets of constraint functions (eq, ineq)
        the first list should be eq 0
        the second list should be geq 0
        note: like the evaluation function, this should be able to take ad (or oovar)
        objects
        note that we take points - which is exactly self(state) - to have compatibility
        with the constraint functions
        :param state:
        :param points:
        :return: pair of lists of values
        """
        raise NotImplementedError


##################################################################################
"""
Simplest possible test robot - a collection of 2D points
Useful for testing various things
"""
class Particle2DRobot(RobotFunction):
    def __init__(self, _npoints):
        RobotFunction.__init__(self, _nvars =_npoints * 2, _npoints=_npoints, _name="Particle2D")
        self.noZ = True
        self.varnames = list(chain.from_iterable([["x%d"%i,"y%d"%i] for i in range(self.npoints)]))

    def __call__(self, state):
        return [ (state[i*2], state[i*2+1], 0) for i in range(self.npoints) ]

    def constraint(self, **kwargs):
        return [],[]
