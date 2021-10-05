import numpy as np
import keras

# the four different states of the XOR gate
training_data = np.array([[0,0],[0,1],[1,0],[1,1]], "float32")

# start = keras.Input(shape=(2,))
# hidden1 = keras.layers.Dense(16, activation='relu')(start)
# stop = keras.layers.Dense(1, activation='sigmoid')(hidden1)
# model = keras.models.Model(inputs=start, outputs=stop)

model = keras.models.load_model('test.h5')

print(model.predict(training_data).round())