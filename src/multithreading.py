import threading
import multiprocessing as mp
import rospy
from std_msgs.msg import Int8
import numpy as np

class array_test:
    def __init__(self):
        self.arr = [1,2,3,4,5]


class threading_test:
    def __init__(self, idx, arr):
        self.arr = arr
        self.idx = idx
        self.pub = rospy.Publisher('pub' + str(idx), Int8, queue_size=3)

    def print_idx(self):
        while True:
            self.arr.arr = np.random.randint(0, 5, 5).tolist()
            self.pub.publish(Int8(self.arr.arr[self.idx]))

def cb(data):
    print data.data

if __name__ == '__main__':
    rospy.init_node('start')

    arr = array_test()

    t1 = threading_test(1, arr)
    t2 = threading_test(2, arr)
    t3 = threading_test(3, arr)

    rospy.Subscriber('pub3', Int8, callback=cb)

    thread1 = threading.Thread(target=t1.print_idx)
    thread2 = threading.Thread(target=t2.print_idx)
    thread3 = threading.Thread(target=t3.print_idx)

    # print np.random.randint(0,5,5).tolist()
    thread1.start()
    thread2.start()
    thread3.start()





