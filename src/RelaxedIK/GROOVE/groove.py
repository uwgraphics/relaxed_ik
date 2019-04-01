from GROOVE_Utils.groove_type import GrooveType_scipy, GrooveType_nlopt

def get_groove(vars, optimization_package='scipy', solver_name='slsqp'):
    '''
    :param vars: vars object
    :param optimization_package: either 'scipy' or 'nlopt'
    :param solver_name:
        for scipy, options are 'methods' under this reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html
        for nlopt, options are 'slsqp', 'ccsaq', 'mma', 'bobyqa', 'lbfgs', 'mlsl', 'newuoa'
    '''
    if optimization_package == 'scipy':
        return GrooveType_scipy(vars, solver_name)
    elif optimization_package == 'nlopt':
        return GrooveType_nlopt(vars, solver_name)
    else:
        raise Exception('Invalid optimization_package input in subroutine [groove].  Valid options are scipy or nlopt.  Exiting.')