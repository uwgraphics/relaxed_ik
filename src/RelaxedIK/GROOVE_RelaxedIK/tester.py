from boost import objectives_ext
import math
import time

test_size = 10000


x_val = 0.4
t = 0.0
d = 2.0
c = .1
f = 10
g = 2

start = time.clock()
for i in xrange(test_size):
    res = (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2))) + f * (x_val - t) ** g
end = time.clock()

print (end - start) / test_size


start = time.clock()
for i in xrange(test_size):
    objectives_ext.nloss(x_val, t, d, c, f, g)
end = time.clock()

print (end - start) / test_size