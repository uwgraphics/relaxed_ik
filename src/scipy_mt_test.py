import scipy.optimize as o
import threading

class Vars:
    def __init__(self, init):
        self.init = init
        self.curr_x = self.init[0]
        self.curr_y = self.init[1]


def obj(x):
    return x[0]**2 + x[1]**2


def obj1(x, vars):
    return x**2 + vars.curr_y**2

def obj2(x, vars):
    return vars.curr_x**2 + x**2


if __name__ == '__main__':
    init = [3., 1000.]
    v = Vars(init)

    o1 = lambda x : obj1(x, v)
    o2 = lambda x : obj2(x, v)


    def t1():
        xopt = o.fmin_slsqp(o1, v.curr_x,iprint=0)[0]
        v.curr_x = xopt
        for i in xrange(100000000):
            y = 1+1
        print 't1: {}'.format(xopt)

    def t2():
        xopt = o.fmin_slsqp(o2, v.curr_y,iprint=0)[0]
        v.curr_y = xopt
        print 't2: {}'.format(xopt)


    for i in xrange(10):
        thread1 = threading.Thread(target=t1)
        thread2 = threading.Thread(target=t2)

        thread2.start()
        thread1.start()

        thread2.join()
        thread1.join()

        print




    print o.fmin_slsqp(obj,init, iprint=0)