from GROOVE_Utils.objective import *
from GROOVE_Utils.vars import *
from GROOVE_Utils.constraint import *
from GROOVE_Utils.weight_function import *
from groove import get_groove

rospy.init_node('test_node')

objectives = (Test_Objective(),Test_Objective2(), Test_Objective3())
constraints = (Test_Constraint(),Test_Constraint_2())
weight_funcs = (Identity_Weight(),Identity_Weight(),Identity_Weight())
weight_priors = (1.0,1.0,1.0)
init_state = [0.0,1.0]

vars = Vars('test', objective_master, init_state, objectives, weight_funcs, weight_priors,constraints=constraints,unconstrained=False)
g = get_groove(vars, 'nlopt', 'mma')

while not rospy.is_shutdown():
    print g.solve()