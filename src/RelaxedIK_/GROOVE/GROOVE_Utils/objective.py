__author__ = 'drakita'


from abc import ABCMeta, abstractmethod
import scipy.optimize as O


def objective_master(x, vars):
    objectives = vars.objectives
    weight_funcs = vars.weight_funcs
    weight_priors = vars.weight_priors
    objective_sum = 0.0
    for i,o in enumerate(objectives):
        if o.isVelObj() and not vars.vel_objectives_on:
            continue
        else:
            weight_func = weight_funcs[i]
            term_weight = weight_priors[i]*weight_func(vars)
            objective_sum += term_weight*o(x,vars)

    return float(objective_sum)


def objective_master_nlopt(x, grad):
    numDOF = len(x)
    g = O.approx_fprime(x, vars.objective_function, numDOF * [0.001])
    if grad.size > 0:
        for i in xrange(numDOF):
            grad[i] = g[i]

    return vars.objective_function(x)


#################################################################################################

class Objective:
    __metaclass__ = ABCMeta

    def __init__(self, *args): pass

    @abstractmethod
    def isVelObj(self): return False

    @abstractmethod
    def name(self): pass

    @abstractmethod
    def __call__(self, x, vars): pass


class Test_Objective(Objective):
    def isVelObj(self): return False
    def name(self): return 'Test'
    def __call__(self, x, vars): return 1.0

class Test_Objective2(Objective):
    def isVelObj(self): return False
    def name(self): return 'Test_2'
    def __call__(self, x, vars): return 1.5

class Test_Objective3(Objective):
    def isVelObj(self): return False
    def name(self): return 'Test_3'
    def __call__(self, x, vars): return x[0]**2 + x[1]**2

'''
x_val = np.linalg.norm(v)
t = 0.0
d = 2.0
c = .08
f = 0.1
g = 2
return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2))) + f * (x_val - t) ** g
'''