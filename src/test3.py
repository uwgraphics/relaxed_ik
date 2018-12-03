
from RelaxedIK.Julia_Bridge.relaxedIK_julia import RelaxedIK_Julia

if __name__ == '__main__':
    import os

    path_to_src = os.path.dirname(__file__)

    rj = RelaxedIK_Julia(path_to_src)

    while True:
        print rj.solve([[0.,0.,0.]], [[1.,0.,0.,0.]])