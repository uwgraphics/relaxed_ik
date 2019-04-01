import objective as obj
import constraint as con
from colors import bcolors

class GrooveType:
    def __init__(self, vars):
        self.vars = vars
        self.constraint_dict = self.__construct_constraint_dict(self.vars.constraints)

        obj_closure = lambda x : self.vars.objective_function(x, vars)

        self.objective_function = obj_closure #### change this to a closure around the vars object


    def __construct_constraint_dict(self, constraints):
        constraint_dicts = []
        for c in constraints:
            d = {
                'type': c.constraintType(),
                'fun': c.func,
                'args': (self.vars,)
            }
            constraint_dicts.append(d)

        return tuple(constraint_dicts)


class GrooveType_scipy(GrooveType):
    def __init__(self, vars, solver_name):
        GrooveType.__init__(self, vars)
        self.solver_name = solver_name

    def solve(self, prev_state=(), max_iter=12, verbose_output=False):
        if prev_state == ():
            initSol = self.vars.xopt
        else:
            initSol = prev_state

        if self.vars.unconstrained:
        # if True:
            xopt_full = obj.O.minimize(self.objective_function, initSol,
                                   bounds=self.vars.bounds, args=(), method=self.solver_name,
                                   options={'maxiter': max_iter, 'disp': verbose_output})
        else:
            xopt_full = obj.O.minimize(self.objective_function, initSol, constraints=self.constraint_dict,
                                   bounds=self.vars.bounds, args=(), method=self.solver_name,
                                   options={'maxiter': max_iter, 'disp': verbose_output})


        xopt = xopt_full.x
        f_obj = xopt_full.fun

        if verbose_output:
            print bcolors.OKBLUE + str(xopt_full) + bcolors.ENDC + '\n'

        self.vars.update(xopt, f_obj)

        return xopt



class GrooveType_nlopt(GrooveType):
    def __init__(self, vars, solver_name):
        try:
            import nlopt as N
        except:
            raise Exception('Error: In order to use an NLopt solver, you must have NLopt installed.')

        GrooveType.__init__(self, vars)
        self.solver_name = solver_name

        if solver_name == 'slsqp':
            self.opt = N.opt(N.LD_SLSQP, len(self.vars.init_state))
        elif solver_name == 'ccsaq':
            self.opt = N.opt(N.LD_CCSAQ, len(self.vars.init_state))
        elif solver_name == 'mma':
            self.opt = N.opt(N.LD_MMA, len(self.vars.init_state))
        elif solver_name == 'bobyqa':
            self.opt = N.opt(N.LN_BOBYQA, len(self.vars.init_state))
        elif solver_name == 'cobyla':
            self.opt = N.opt(N.LN_COBYLA, len(self.vars.init_state))
        elif solver_name == 'lbfgs':
            self.opt = N.opt(N.LD_LBFGS, len(self.vars.init_state))
        elif solver_name == 'mlsl':
            self.opt = N.opt(N.GD_MLSL, len(self.vars.init_state))
        elif solver_name == 'direct':
            self.opt = N.opt(N.GN_DIRECT_L_RAND, len(self.vars.init_state))
        elif solver_name == 'newuoa':
            self.opt = N.opt(N.LN_NEWUOA_BOUND, len(self.vars.init_state))
        else:
            raise Exception('Invalid solver_name in subroutine [GrooveType_nlopt]!')

        self.opt.set_min_objective(obj.objective_master_nlopt)
        self.opt.set_xtol_rel(1e-4)
        # self.opt.set_maxtime(.025)
        if self.vars.bounds == ():
            self.opt.set_lower_bounds(len(self.vars.init_state) * [-50.0])
            self.opt.set_upper_bounds(len(self.vars.init_state) * [50.0])
        else:
            u = []
            l = []
            for b in self.vars.bounds:
                u.append(b[1])
                l.append(b[0])

            self.opt.set_lower_bounds(l)
            self.opt.set_upper_bounds(u)


    def solve(self, prev_state='', verbose_output=False, maxtime=None):
        if prev_state == '':
            initSol = self.vars.xopt
        else:
            initSol = prev_state

        if not maxtime == None:
            self.opt.set_maxtime(maxtime)

        if not self.vars.unconstrained:
            for c in self.vars.constraints:
                if c.constraintType() == 'ineq':
                    self.opt.add_inequality_constraint(c.func_nlopt, 0.1)
                elif c.constraintType() == 'eq':
                    self.opt.add_equality_constraint(c.func_nlopt, 0.1)
        else:
            self.opt.remove_inequality_constraints()
            self.opt.remove_equality_constraints()

        xopt = self.opt.optimize(initSol)
        f_obj = self.opt.last_optimum_value()

        if verbose_output:
            print bcolors.OKBLUE + str(xopt) + bcolors.ENDC + '\n'

        self.vars.update(xopt, f_obj)

        return xopt