__author__ = 'gleicher'

# note:
# using defaultdict IS faster than using dict in my experiments
# (if you use a regular dictionary, you need a try/except for the +=)
# (although only marginally so)
# the code is much much cleaner, so I am sticking with it

import ad
import ad.admath as MATH

from collections import defaultdict

ADF = ad.ADF
AVD = ad.ADV

def adnumber(val, name):
    return ad.adnumber(val, name)

def apply_chain_rule_noHess(ad_funcs, variables, lc_wrt_args, qc_wrt_args,
                      cp_wrt_args):
    """
    This function applies the first and second-order chain rule to calculate
    the derivatives with respect to original variables (i.e., objects created
    with the ``adnumber(...)`` constructor).

    For reference:
    - ``lc`` refers to "linear coefficients" or first-order terms
    - ``qc`` refers to "quadratic coefficients" or pure second-order terms
    - ``cp`` refers to "cross-product" second-order terms

    """
    grad = defaultdict(float)    
    for f,df in zip(ad_funcs, lc_wrt_args):
        try:
            for fvar,fval in f._lc.iteritems():
                grad[fvar] += df * fval
        except:
            pass
    return (grad, None, None)

withHessians = ad._apply_chain_rule

# set up for no hessians
ad._apply_chain_rule = apply_chain_rule_noHess

# compute the sum - quickly
def fsum(list):
    # compute the sum of values, but also keep any AD objects
    # in case we are computing a derivative - this may be a
    # little slower in the event of value computation, but better for
    # derivatives
    val = 0
    ads = []
    for v in list:
        try:
            val += v.x
            ads.append(v)
        except AttributeError:
            val += v

    if len(ads):
        grad = defaultdict(float)
        for f in ads:
            for fvar,fval in f._lc.iteritems():
                grad[fvar] += fval
        return ADF(val, grad, None, None)
    else:
        return val

# replace the slow parts
def ADF_Mult(self,val):
    try:
        x = self.x
        dx = self._lc
    except AttributeError:
        x = self
        dx = None
        if x==0.0: return 0.0
    try:
        y = val.x
        dy = val._lc
    except AttributeError:
        y = val
        dy = None
        if y==0.0: return 0.0

    if dx or dy:
        grad = defaultdict(float)
        if dx:
            for a,b in dx.iteritems():
                grad[a] = b * y
        if dy:
            for a,b in dy.iteritems():
                grad[a] += b * x
        return ADF(x*y, grad, None, None)
    else:
        return x*y

def ADF_Add(self,val):
    try:
        x = self.x
        dx = self._lc
    except AttributeError:
        x = self
        dx = None
    try:
        y = val.x
        dy = val._lc
    except AttributeError:
        y = val
        dy = None

    if dx and dy:
        grad = defaultdict(float)
        for a,b in dx.iteritems():
            grad[a] = b
        for a,b in dy.iteritems():
            grad[a] += b
        return ADF(x+y, grad, None, None)
    elif dx:
        return ADF(x+y, dx, None, None)
    elif dy:
        return ADF(x+y, dy, None, None)
    else:
        return x+y

def ADF_Sub(self,val):
    try:
        x = self.x
        dx = self._lc
    except AttributeError:
        x = self
        dx = None
    try:
        y = val.x
        dy = val._lc
    except AttributeError:
        y = val
        dy = None

    if dx and dy:
        grad = defaultdict(float)
        for a,b in dx.iteritems():
            grad[a] = b
        for a,b in dy.iteritems():
            grad[a] -= b
        return ADF(x-y, grad, None, None)
    elif dx:
        return ADF(x-y, dx, None, None)
    elif dy:
        return ADF(x-y, dy, None, None)
    else:
        return x-y

def fastLC2(a1,t1,a2,t2):
    """
    t1 * a1 + t2 * a2 - assumes that the t's are constant
    :param a1:
    :param t1:
    :param a2:
    :param t2:
    :return:
    """
    try:
        if t1 != 0.0:
            x = a1.x
            dx = a1._lc
        else:
            x = 0.0
            dx = None
    except AttributeError:
        x = a1
        dx = None
    try:
        if t2 != 0.0:
            y = a2.x
            dy = a2._lc
        else:
            y = 0.0
            dy = None
    except AttributeError:
        y = a2
        dy = None

    if dx or dy:
        grad = defaultdict(float)
        if dx:
            for a,b in dx.iteritems():
                grad[a] = b * t1
        if dy:
            for a,b in dy.iteritems():
                grad[a] += b * t2
        return ADF(t1*x + t2*y, grad, None, None)
    else:
        return t1*x + t2*y


ADF.__add__ = ADF_Add
ADF.__mul__ = ADF_Mult
ADF.__sub__ = ADF_Sub