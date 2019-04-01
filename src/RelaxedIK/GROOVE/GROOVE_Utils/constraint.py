__author__ = 'drakita'

from abc import ABCMeta, abstractmethod
import scipy.optimize as O

class Constraint:
    __metaclass__ = ABCMeta

    def __init__(self, *args): pass

    @abstractmethod
    def constraintType(self):
        'return eq for equality and ineq for inequality'
        return None

    @abstractmethod
    def name(self): pass

    @abstractmethod
    def func(self, x): pass

    def func_nlopt(self, x, grad):
        numDOF = len(x)
        g = O.approx_fprime(x, self.func, numDOF * [0.001])
        if grad.size > 0:
            for i in xrange(numDOF):
                grad[i] = -g[i]
        return -self.func(x)

class Test_Constraint(Constraint):
    def __init__(self, *args): pass
    def constraintType(self): return 'ineq'
    def name(self): return 'test1'
    def func(self,x,*args):
        # vars = get_groove_global_vars()
        return 1.0

class Test_Constraint_2(Constraint):
    def __init__(self, *args): pass
    def constraintType(self): return 'ineq'
    def name(self): return 'test2'
    def func(self,x,*args):
        # vars = get_groove_global_vars()
        f = x[1] - 0.5
        return x[1] - 0.5



