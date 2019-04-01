from ..Utils.colors import *

from ..GROOVE.GROOVE_Utils.objective import Objective, objective_master
from ..Utils import tf_fast as Tf
from ..Utils.geometry_utils import *
from ..Utils.joint_utils import *

# try:
#     from boost import objectives_ext
# except:
#     print 'ERROR when importing boost library extension.  Defaulting to python implementation (which will be slower).  ' \
#           'To get speed boost, please install and configure the boost python library: ' \
#           'https://www.boost.org/doc/libs/1_67_0/more/getting_started/unix-variants.html'


def objective_master_relaxedIK(x, vars):
    # vars = get_groove_global_vars()
    vars.frames = vars.robot.getFrames(x)

    return objective_master(x, vars)


########################################################################################################################

class Position_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return False
    def name(self): return 'Position'

    def __call__(self, x, vars):
        # positions = vars.arm.getFrames(x)[0]
        positions = vars.frames[0]
        eePos = positions[-1]
        goal_pos = vars.goal_pos
        diff = (eePos - goal_pos)
        norm_ord = 2
        x_val = np.linalg.norm(diff, ord=norm_ord)
        t = 0.0
        d = 2.0
        c = .1
        f = 10
        g = 2
        return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g

class Position_MultiEE_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return False
    def name(self): return 'Position_MultiEE'

    def __call__(self, x, vars):
        if vars.c_boost:
            x_val = objectives_ext.position_multiEE_obj(vars.frames, vars.goal_positions, [1.0, 1.0])
        else:
            x_val_sum = 0.0


            for i, f in enumerate(vars.frames):
                positions = f[0]
                eePos = positions[-1]
                goal_pos = vars.goal_positions[i]
                diff = (eePos - goal_pos)
                norm_ord = 2
                x_val = np.linalg.norm(diff, ord=norm_ord)
                x_val_sum += x_val

            x_val = x_val_sum

        t = 0.0
        d = 2.0
        c = .1
        f = 10
        g = 2

        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g


class Orientation_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return False
    def name(self): return 'Orientation'

    def __call__(self, x, vars):
        frames = vars.frames[1]
        eeMat = frames[-1]

        goal_quat = vars.goal_quat
        new_mat = np.zeros((4, 4))
        new_mat[0:3, 0:3] = eeMat
        new_mat[3, 3] = 1

        ee_quat = Tf.quaternion_from_matrix(new_mat)

        q = ee_quat
        ee_quat2 = [-q[0],-q[1],-q[2],-q[3]]

        norm_ord = 2
        # start = time.time()
        disp = np.linalg.norm(Tf.quaternion_disp(goal_quat,ee_quat), ord=norm_ord)
        disp2 = np.linalg.norm(Tf.quaternion_disp(goal_quat,ee_quat2),ord=norm_ord)
        # after = time.time()
        # print after - start

        x_val = min(disp, disp2)
        # x_val = np.min(np.array([disp,disp2]))
        t = 0.0
        d = 2.0
        c = .1
        f = 10
        g = 2
        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g


class Orientation_MultiEE_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return False
    def name(self): return 'Orientation_MultiEE'

    def __call__(self, x, vars):
        if vars.c_boost:
            x_val = objectives_ext.orientation_multiEE_obj(vars.frames, vars.goal_quats, [1.0, 1.0])
        else:
            x_val_sum = 0.0

            for i, f in enumerate(vars.frames):
                eeMat = f[1][-1]

                goal_quat = vars.goal_quats[i]
                new_mat = np.zeros((4, 4))
                new_mat[0:3, 0:3] = eeMat
                new_mat[3, 3] = 1

                ee_quat = Tf.quaternion_from_matrix(new_mat)

                q = ee_quat
                ee_quat2 = [-q[0], -q[1], -q[2], -q[3]]

                norm_ord = 2
                disp = np.linalg.norm(Tf.quaternion_disp(goal_quat, ee_quat), ord=norm_ord)
                disp2 = np.linalg.norm(Tf.quaternion_disp(goal_quat, ee_quat2), ord=norm_ord)

                x_val = min(disp, disp2)
                x_val_sum += x_val

            x_val = x_val_sum

        t = 0.0
        d = 2.0
        c = .1
        f = 10
        g = 2
        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g



class Min_Jt_Vel_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return True
    def name(self): return 'Min_Jt_Vel'

    def __call__(self, x, vars):
        if vars.c_boost:
            x_val = objectives_ext.min_jt_vel_obj(x, vars.xopt)
        else:
            v = x - np.array(vars.xopt)
            x_val = np.linalg.norm(v)

        t = 0.0
        d = 2.0
        c = .1
        f = 10.0
        g = 2

        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g



class Min_EE_Vel_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return True
    def name(self): return 'Min_EE_Vel'

    def __call__(self, x, vars):
        jtPt = vars.frames[0][-1]
        x_val = np.linalg.norm(vars.ee_pos - jtPt)
        t = 0.0
        d = 2.0
        c = .1
        f = 10.0
        g = 2
        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g


class Min_Jt_Accel_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return True
    def name(self): return 'Min_Jt_Accel'

    def __call__(self, x, vars):
        if vars.c_boost:
            x_val = objectives_ext.min_jt_accel_obj(x, vars.xopt, vars.prev_state)
        else:
            prev_state_2 = np.array(vars.prev_state)
            prev_state = np.array(vars.xopt)

            v2 = prev_state - prev_state_2
            v1 = x - prev_state

            a = v2 - v1

            x_val = np.linalg.norm(a)

        t = 0.0
        d = 2.0
        c = .1
        f = 10.0
        g = 2
        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g

class Min_EE_Accel_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return True
    def name(self): return 'Min_EE_Accel'

    def __call__(self, x, vars):
        jtPt = vars.frames[0][-1]
        prev_jtPt_2 = np.array(vars.prev_ee_pos)
        prev_jtPt = np.array(vars.ee_pos)

        v2 = prev_jtPt - prev_jtPt_2
        v1 = jtPt - prev_jtPt

        a = v2 - v1

        x_val = np.linalg.norm(a)
        t = 0.0
        d = 2.0
        c = .2
        f = 0.0
        g = 2
        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g

class Min_Jt_Jerk_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return True
    def name(self): return 'Min_Jt_Jerk'

    def __call__(self, x, vars):
        if vars.c_boost:
            x_val = objectives_ext.min_jt_jerk_obj(x, vars.xopt, vars.prev_state, vars.prev_state2)
        else:
            prev_state_3 = np.array(vars.prev_state2)
            prev_state_2 = np.array(vars.prev_state)
            prev_state = np.array(vars.xopt)

            v3 = prev_state_2 - prev_state_3
            v2 = prev_state - prev_state_2
            v1 = x - prev_state

            a2 = v2 - v3
            a1 = v1 - v2

            j = a1 - a2

            x_val = np.linalg.norm(j)

        t = 0.0
        d = 2.0
        c = .2
        f = 0.0
        g = 2

        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g

class Min_EE_Jerk_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return True
    def name(self): return 'Min_EE_Jerk'

    def __call__(self, x, vars):
        jtPt = vars.frames[0][-1]
        prev_jtPt_3 = np.array(vars.prev_ee_pos2)
        prev_jtPt_2 = np.array(vars.prev_ee_pos)
        prev_jtPt = np.array(vars.ee_pos)

        v3 = prev_jtPt_2 - prev_jtPt_3
        v2 = prev_jtPt - prev_jtPt_2
        v1 = jtPt - prev_jtPt

        a2 = v2 - v3
        a1 = v1 - v2

        j = a1 - a2

        x_val = np.linalg.norm(j)
        t = 0.0
        d = 2.0
        c = .2
        f = 1.0
        g = 2
        return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2))) + f * (x_val - t) ** g


class Joint_Limit_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return False
    def name(self): return 'Joint_Limit'

    def __call__(self, x, vars):
        sum = 0.0
        penalty = 50.0
        d = 8
        joint_limits = vars.robot.bounds
        for i in xrange(vars.robot.numDOF):
            l = joint_limits[i][0]
            u = joint_limits[i][1]
            mid = (u + l) / 2.0
            a = penalty / (u - mid)**d
            sum += a*(x[i] - mid)**d

        vars.joint_limit_obj_value = sum

        x_val = sum
        t = 0
        d = 2
        c = 2.3
        f = .003
        g = 2
        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g


class Self_Collision_Avoidance_Obj(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return False
    def name(self): return 'Self_Collision_Avoidance'

    def __call__(self, x, vars):
        frames = vars.frames
        jt_pts = frames[0]

        x_val = vars.collision_graph.get_collision_score(frames)
        t = 0.0
        d = 2.0
        c = .08
        f = 1.0
        g = 2
        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g


class Collision_Avoidance_nn(Objective):
    def __init__(self, *args): pass
    def isVelObj(self): return False
    def name(self): return 'Collision_Avoidance_nn'

    def __call__(self, x, vars):
        frames = vars.frames
        out_vec = []
        for f in frames:
            jt_pts = f[0]
            for j in jt_pts:
                out_vec.append(j[0])
                out_vec.append(j[1])
                out_vec.append(j[2])

        val = vars.collision_nn.predict([out_vec])[0]

        # nn_stats = vars.nn_stats

        # x_val =  (val - nn_stats[0])/ nn_stats[1]
        x_val = val
        t = 0
        d = 2
        c = 1.85
        f = .004
        g = 2
        if vars.c_boost:
            return objectives_ext.nloss(x_val, t, d, c, f, g)
        else:
            return (-math.e ** ((-(x_val - t) ** d) / (2.0 * c ** 2)) ) + f * (x_val - t) ** g

        # return math.exp(x_val - 0.64) - 1


