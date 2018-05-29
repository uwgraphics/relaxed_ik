from keras.models import Sequential
from keras.layers import Dense
import numpy as np

model = Sequential()

model.add(Dense(units=64, activation='relu', input_dim=2))
model.add(Dense(units=1, activation='relu', input_dim=64))

# model.add(Dense(units=1, activation='softmax'))

model.compile(loss='categorical_crossentropy',
              optimizer='sgd',
              metrics=['accuracy'])

inputs = np.array([ [1,0], [0,1], [0,0], [1,1] ])
outputs = np.array([[0],[0],[1],[1]])

model.train_on_batch(inputs, outputs)

