import numpy as np
import math

class EMA_filter:
    def __init__(self, init_state, a = 0.6, window_size = 25):
        if a > 1.0 or a < 0.0:
            raise Exception('a must be between 0 and 1 in EMA_filter')

        self.a = a
        self.window_size = window_size
        self.filtered_signal = []
        for i in xrange(window_size):
            self.filtered_signal.append(np.array(init_state))

    def filter(self, state):
        state_arr = np.array(state)

        weights = self.window_size*[0]
        filtered_state = np.zeros((len(state)))

        # sum = 0.0
        for i in xrange(self.window_size):
            weights[i] = self.a*(1.0 - self.a)**(i)
            if i == 0:
                filtered_state = weights[i]*state_arr
            else:
                try:
                    filtered_state = filtered_state + weights[i]*self.filtered_signal[-i]
                except:
                    return state

            # sum += weights[i]

        self.filtered_signal.append(filtered_state)
        return filtered_state



def bilateral_filter(array, window_size=10, r=15.0, s=15.0, c=1.0):
    array_size = len(array)
    filtered_array = []

    for i in xrange(array_size):
        window = range(i-window_size/2, (i+window_size/2)+1)

        # numerator = np.array([0.0,0.0,0.0])
        if type(array[0]) == float or type(array[0]) == int:
            numerator = 0.0
        else:
            z = np.zeros((1,len(array[0])))
            numerator = np.array(z)

        normalizer = 0.0

        for xi in window:
            if xi < 0: xi = 0
            elif xi >= array_size-1: xi = array_size-1
            numerator += array[xi]*gaussian( np.linalg.norm(array[xi] - array[i]), r )* gaussian( abs(xi - i), s )
            normalizer += gaussian( np.linalg.norm(array[xi] - array[i]), r,  c=c)* gaussian( abs(xi - i), s, c=c )

        vec = (1.0/ normalizer)*numerator
        filtered_array.append(vec)

    return filtered_array


def gaussian(x, sigma, mean=0, c= 1.0):
    return (c/(sigma*math.sqrt(2.0*math.pi)))*math.exp(-0.5*((x - mean)/sigma)**2)


if __name__ == '__main__':
    # ema = EMA_filter([1,2,3])
    # for i in range(20):
     #   print ema.filter([4,5,6])

    array = []
    for i in xrange(200):
        array.append(np.random.uniform(size=(3)))

    print array[40]
    print bilateral_filter(array)[40]