from __future__ import division
from sklearn import preprocessing
from sklearn.neural_network import MLPClassifier
from sklearn.externals import joblib
import numpy as np
import time

mlp = joblib.load("jetson_mlp_driver.pkl")
min_max_scaler = joblib.load("mlp_normalizer.pkl")
x_test = np.array([[-21.4358, -4.1706, 0, 0]])
initial_time = time.time()
print np.argmax(mlp.predict(min_max_scaler.transform(x_test)))
final_time = time.time()
print final_time - initial_time
