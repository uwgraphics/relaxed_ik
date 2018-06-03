import os
#os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"   # see issue #152
#os.environ["CUDA_VISIBLE_DEVICES"] = ""

from keras.models import Sequential
from keras.layers import Dense
import keras
import numpy as np
import time
from sklearn.neural_network import MLPClassifier, MLPRegressor


model = Sequential()
model.add(Dense(32, activation='relu', input_dim=100))
model.add(Dense(32, activation='relu'))
model.add(Dense(32, activation='relu'))
model.add(Dense(32, activation='relu'))
model.add(Dense(32, activation='relu'))
model.add(Dense(32, activation='relu'))
model.add(Dense(10, activation='softmax'))
model.compile(optimizer='rmsprop',
              loss='categorical_crossentropy')

# Generate dummy data
import numpy as np
data = np.random.random((10000, 100))
labels = np.random.randint(10, size=(10000, 1))

print labels

# Convert labels to categorical one-hot encoding
one_hot_labels = keras.utils.to_categorical(labels, num_classes=10)

# Train the model, iterating on the data in batches of 32 samples
model.fit(data, one_hot_labels, epochs=10, batch_size=32)

for i in xrange(10000):
    start = time.clock()
    model.predict(np.random.random((1,100)))
    end = time.clock()
    print end - start