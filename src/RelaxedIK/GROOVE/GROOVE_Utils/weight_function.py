__author__ = 'drakita'

import numpy as np
from abc import ABCMeta, abstractmethod, abstractproperty

class Weight_Function:
    __metaclass__ = ABCMeta

    @abstractmethod
    def name(self): pass

    @abstractmethod
    def __call__(self, vars): pass

class Identity_Weight(Weight_Function):
    def name(self): return 'Identity_weight'
    def __call__(self, vars): return 1.0