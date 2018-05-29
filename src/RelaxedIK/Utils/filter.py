import numpy as np

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

        sum = 0.0
        for i in xrange(self.window_size):
            weights[i] = self.a*(1.0 - self.a)**(i)
            if i == 0:
                filtered_state = weights[i]*state_arr
            else:
                try:
                    filtered_state = filtered_state + weights[i]*self.filtered_signal[-i]
                except:
                    return state

            sum += weights[i]

        self.filtered_signal.append(filtered_state)
        return filtered_state


if __name__ == '__main__':
    ema = EMA_filter([1,2,3])
    for i in range(20):
        print ema.filter([4,5,6])



