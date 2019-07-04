'''
Using RANSAC filter
'''

import numpy as np
import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation
from numpy import exp, abs, angle
import math
from sklearn.linear_model import RANSACRegressor

# /////////////////////////
# >>> Basic Setup
# /////////////////////////
# Setup parameters
FRAMERATE = 24    # Framerate during optical flow
FOCAL_LENGTH_M = 3.6 * (1000)  # original focal lenght: 3.6mm
QUEUE_SIZE = 10
# Init varible

# import the raw data
data = np.load('experiment_raw.npy')

# /////////////////////////
# >>> Import Data
# /////////////////////////
# data[frame][row, columns]
x = np.copy(data['x']) # 1119 frames, 46.625s
y = np.copy(data['y']) # 30 row, 41 columns
sad = np.copy(data['sad'])
print ("Size of Data: %d" %(len(data['x'])))


# /////////////////////////
# >>> Setting RANSAC Filter # use 40 frame for testing
# /////////////////////////
class LinearLeastSquares2D(object):

    def fit(self, data):
        data_mean = data.mean(axis=0)
        x0, y0 = data_mean
        if data.shape[0] > 2: # over determined
            u, v, w = np.linalg.svd(data-data_mean)
            vec = w[0]
            theta = math.atan2(vec[0], vec[1])
        elif data.shape[0] == 2: # well determined
            theta = math.atan2(data[1,0]-data[0,0], data[1,1]-data[0,1])
        theta = (theta + math.pi * 5 / 2) % (2*math.pi)
        d = x0*math.sin(theta) + y0*math.cos(theta)
        return d, theta

    def residuals(self, model, data):
        d, theta = model
        dfit = data[:,0]*math.sin(theta) + data[:,1]*math.cos(theta)
        return np.abs(d-dfit)

    def is_degenerate(self, sample):
        return False
def ransac(data, model_class, min_samples, threshold, max_trials=400):
    best_model = None
    best_inlier_num = 0
    best_inliers = None
    data_idx = np.arange(data.shape[0])     #retrun how many row in set
    for _ in range(max_trials):
        sample = data[np.random.randint(0, data.shape[0], 2)]
        if model_class.is_degenerate(sample):
            continue
        sample_model = model_class.fit(sample)
        sample_model_residua = model_class.residuals(sample_model, data)
        sample_model_inliers = data_idx[sample_model_residua<threshold]
        inlier_num = sample_model_inliers.shape[0]
        if inlier_num > best_inlier_num:
            best_inlier_num = inlier_num
            best_inliers = sample_model_inliers
    if best_inliers is not None:
        best_model = model_class.fit(data[best_inliers])
    return best_model, best_inliers

# Set up XY data to 1D array >>Testing on 25
for j in range (len(data)):
    x_queue = np.zeros(QUEUE_SIZE, dtype=np.float)
    y_queue = np.zeros(QUEUE_SIZE, dtype=np.float)
    x_queue[1:] = x_queue[:-1]
    y_queue[1:] = y_queue[:-1]
    x_queue[0] = (x[j].mean()) * QUEUE_SIZE
    y_queue[0] = (y[j].mean()) * QUEUE_SIZE
    _XY = x_queue.mean(), y_queue.mean()
    _x = x[j].ravel()
    _y = y[j].ravel()
    X = np.zeros([len(_x),2])
    Y = np.zeros([len(_y),2])
    XY = np.zeros([len(x),2])
    for i in range (len(_x)):
        X[i] = i, _x[i]
        Y[i] = i, _y[i]
    x_queue = np.zeros(QUEUE_SIZE, dtype=np.float)
    y_queue = np.zeros(QUEUE_SIZE, dtype=np.float)
    modelx, inliersx = ransac(X[(np.where(X[:,1] == 0)[0])], LinearLeastSquares2D(), (len(_x))/2, 3)
    modely, inliersy = ransac(Y[(np.where(Y[:,1] == 0)[0])], LinearLeastSquares2D(), (len(_y))/2, 3)
    print(X)
    print(X[:,1])
    print (len(np.where(X[:,1] == 0)[0]))
    plt.hist(X[inliersy,1], bins='auto')
    plt.hist(Y[inliersy,1], bins='auto')
    plt.title("Hist of Filtered XY")
    x_queue[1:] = x_queue[:-1]
    y_queue[1:] = y_queue[:-1]
    x_queue[0] = (X[inliersx,1].mean()) * QUEUE_SIZE
    y_queue[0] = (Y[inliersy,1].mean()) * QUEUE_SIZE
    XY = x_queue.mean(), y_queue.mean()
    print (j)
    print("- Direct mean: x= %f, y= %f" %(_XY))
    print("- Filter mean: x= %f, y= %f" %(XY))
    plt.show()
    plt.plot(x[j], 'bo')
    plt.plot(X[inliersy,1], 'ro')
    plt.title("Dot of X, old in blue new in red")
    plt.show()
    plt.plot(y[j], 'bo')
    plt.plot(Y[inliersy,1], 'ro')
    plt.title("Dot of Y, old in blue new in red")
    plt.show()
