from keras.models import Sequential
from keras.layers import Dense
import keras
import numpy as np
import time
from sklearn.neural_network import MLPClassifier, MLPRegressor


import os
# os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"   # see issue #152
# os.environ["CUDA_VISIBLE_DEVICES"] = ""

inputs = np.random.uniform(0.0,1.0,(10000,100))
outputs = np.random.uniform(-1.4,-1.2,(10000,1))

print outputs

model = Sequential()

model.add(Dense(units=70, activation='relu', input_dim=100))
model.add(Dense(units=70, activation='relu'))
model.add(Dense(units=1, activation='relu'))

op = keras.optimizers.sgd(lr=0.1,nesterov=True)
model.compile(optimizer=op,loss='mean_squared_error')
model.fit(inputs,outputs, epochs=10000, batch_size=1)


rand = np.ones(shape=(1,100))
print rand.shape
start = time.clock()
print model.predict(rand,batch_size=10000)
end = time.clock()
print end - start

clf = MLPRegressor(solver='adam', alpha=1,
                        hidden_layer_sizes=(70, 70, 70, 70, 70, 70), max_iter=3000, verbose=True,
                        learning_rate='adaptive')

clf.fit(inputs, outputs)
for i in xrange(100):
    start = time.clock()
    clf.predict(rand)
    end = time.clock()
    print end - start

