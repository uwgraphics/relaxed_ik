import threading
import rospy
from std_msgs.msg import Int8

class threading_test:
    def __init__(self, idx):
        self.idx = idx
        self.pub = rospy.Publisher('pub' + str(idx), Int8, queue_size=3)

    def print_idx(self):
        while True:
            self.pub.publish(Int8(self.idx))
            print self.idx




if __name__ == '__main__':
    rospy.init_node('start')

    t1 = threading_test(1)
    t2 = threading_test(2)
    t3 = threading_test(3)


    thread1 = threading.Thread(target=t1.print_idx)
    thread2 = threading.Thread(target=t2.print_idx)
    thread3 = threading.Thread(target=t3.print_idx)


    thread1.start()
    thread2.start()
    thread3.start()





