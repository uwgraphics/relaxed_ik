import numpy as np
import math as M
from abc import ABCMeta, abstractmethod
import rospy
from std_msgs.msg import Float32
# from numba import jit, jitclass

# specific variable classes for specific solvers should inherit from Vars and initialize this super class and override update
class Vars:
    def __init__(self, name, objective_function, init_state, objectives, weight_funcs, weight_priors, constraints=(), bounds=(), unconstrained=False):
        self.name = name
        self.objective_function = objective_function
        self.init_state = init_state
        self.objectives = objectives
        self.weight_funcs = weight_funcs
        self.weight_priors = weight_priors
        self.constraints = constraints
        self.bounds = bounds

        self.vel_objectives_on = True

        self.objective_publishers = []
        self.constraint_publishers = []
        self.weight_func_publishers = []
        self.f_obj_publisher = rospy.Publisher(self.name + '_f_obj', Float32, queue_size=5)
        self.__populate_publishers()

        self.xopt = init_state
        self.prev_state = init_state
        self.prev_state2 = init_state
        self.prev_state3 = init_state
        self.f_obj = 0.0

        self.unconstrained = unconstrained

        self.all_states = []

    def __populate_publishers(self):
        for o in self.objectives:
            self.objective_publishers.append(rospy.Publisher('/objective/' + o.name(), Float32, queue_size=5))

        for c in self.constraints:
            self.constraint_publishers.append(rospy.Publisher('/constraint/' + c.name(), Float32, queue_size=5))

        for wf in self.weight_funcs:
            self.weight_func_publishers.append(rospy.Publisher('/weight_function/' + wf.name(), Float32, queue_size=5))


    def update(self, xopt, f_obj, publish_objectives=False,publish_constraints=False, publish_weight_funcs=False):
        if publish_objectives:
            for i,o in enumerate(self.objective_publishers):
                data = self.objectives[i].__call__(xopt, self)
                data = self.weight_priors[i]*data
                o.publish(data)

        self.f_obj_publisher.publish(f_obj)

        if publish_weight_funcs:
            for i,w in enumerate(self.weight_func_publishers):
                if self.weight_funcs[i].name() == 'Identity_weight':
                    continue
                else:
                    data = self.weight_funcs[i].__call__(self)
                    w.publish(data)

        if publish_constraints:
            if self.unconstrained == False:
                for i,c in enumerate(self.constraints):
                    con_val = c.func(xopt)
                    self.constraint_publishers[i].publish(con_val)


        self.prev_state3 = self.prev_state2
        self.prev_state2 = self.prev_state
        self.prev_state = self.xopt
        self.xopt = xopt
        self.f_obj = f_obj

        self.all_states.append(xopt)

