from GROOVE.groove import get_groove
from RelaxedIK.Utils.filter import EMA_filter
from relaxed_ik.msg import JointAngles
import rospy
from std_msgs.msg import Float32


class RelaxedIK_subchain(object):
    def __init__(self, vars, optimization_package='scipy', solver_name='slsqp'):
        self.vars = vars
        self.optimization_package = optimization_package
        self.solver_name = solver_name
        self.groove = get_groove(vars, optimization_package,solver_name)
        # self.filter = EMA_filter(self.vars.init_state,a=0.5)

        self.ja = JointAngles()
        self.ja.header.frame_id = str(self.vars.subchain_idx)

        self.solution_pub = rospy.Publisher('/subchain_{}'.format(vars.subchain_idx), JointAngles, queue_size=3)


    def run(self):
        while not rospy.is_shutdown():
            xopt = self.groove.solve()
            self.ja.angles = []
            for x in xopt:
                self.ja.angles.append(Float32(x))

            self.solution_pub.publish(self.ja)
