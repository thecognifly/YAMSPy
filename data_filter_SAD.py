'''
Using SAD filter
'''
import numpy as np
import matplotlib
import math
from matplotlib import pyplot as plt
from matplotlib import animation
from numpy import exp, abs, angle
from sklearn import linear_model, datasets
from sklearn.linear_model import RANSACRegressor

# /////////////////////////
# >>> Import Data
# /////////////////////////
data = np.load('experiment_raw.npy')# import the raw data
# data[frame][row, columns]
x = np.copy(data['x']) # using 640x480 resolution
y = np.copy(data['y']) # 30 row, 41 columns
sad = np.copy(data['sad'])
print ("Size of Data: %d" %(len(data['x'])))

# /////////////////////////
# >>> Basic Setup
# /////////////////////////
FRAMERATE = 24    # Framerate during optical flow
FOCAL_LENGTH_M = 3.6 * (1000)  # original focal lenght: 3.6mm

# /////////////////////////
# >>> Define Mean of the data set
# /////////////////////////
def mean(x, y, QUEUE_SIZE = 10):
    x_queue = np.zeros(QUEUE_SIZE, dtype=np.float)
    y_queue = np.zeros(QUEUE_SIZE, dtype=np.float)
    x_queue[1:] = x_queue[:-1]
    y_queue[1:] = y_queue[:-1]
    x_queue[0] = (x.mean()) * QUEUE_SIZE
    y_queue[0] = (y.mean()) * QUEUE_SIZE
    return x_queue.mean(), y_queue.mean()

# /////////////////////////
# >>> Setting SAD Filter # Using the average sad value of each set
# /////////////////////////
def sad_filter (frame, k = 1.5): # This function only work on each frame. In real time, delect all the frame argument
    global x,  y, sad
    SAD_THRESHOLD = np.mean(sad[frame]) * k # smaller the sad value, the better of data. k is use to reduce the limit
    sad_filter = np.where(sad[frame]>SAD_THRESHOLD)
    x[frame][sad_filter] = 0
    y[frame][sad_filter] = 0

# /////////////////////////
# >>> Creating 1D array for x and y instead of 2D
# /////////////////////////
def twoD2oneD(frame): # This function only work on each frame. In real time, delect all the frame argument
    _x = x[frame].ravel()
    _y = y[frame].ravel()
    X = np.zeros([len(_x),1]) # blank numpy array of x-axis data in 1D
    Y = np.zeros([len(_y),1]) # blank numpy array of y-axis data in 1D
    for i in range (len(_x)): # Make the list contain (data)
        X[i] = _x[i]
        Y[i] = _y[i]
    return X, Y # X, Y 1D array

# /////////////////////////
# >>> Creating 1D array without zero
# /////////////////////////
def nonzero(frame): # This function only work on each frame. In real time, delect all the frame argument
    global X, Y # Receive the main SAD filtered X and Y data
    X_nonzero = np.zeros([len(np.where(X[:,0] != 0)[0])])
    Y_nonzero = np.zeros([len(np.where(Y[:,0] != 0)[0])])
    return X[(np.where(X[:,0] != 0)[0])],  Y[(np.where(Y[:,0] != 0)[0])], (len(X)-len(X_nonzero))*100/len(X), (len(Y)-len(Y_nonzero))*100/len(Y)

# /////////////////////////
# >>> Main
# /////////////////////////
for j in range (len(data)): # looping for reading each frame of data, j
    sad_filter(j, 1.8) # (frame, k) using 1.8 gain to lower the SAD limit. Default is 1.5
    X, Y = twoD2oneD(j) # (frame) X = 1D x data, Y = 1D y data
    X_nonzero, Y_nonzero ,x_zero_pt, y_zero_pt = nonzero(j)
    print ("%d - X Data is zero =  %f percent - Y Data is zero =  %f percent" % (j, x_zero_pt, y_zero_pt))

    bin = np.arange(-64, 65)
    plt.xlabel("Data")
    plt.ylabel("Number of Data")
    plt.hist (X_nonzero[:], bins = bin, color = 'red', label = 'X')
    plt.hist (Y_nonzero[:], bins = bin, color = 'blue', label = 'Y')
    plt.xticks(bin-.5, bin)
    plt.legend(loc='upper right')
    plt.title("Frame %d  x: %f  - y: %f" %(j, (100-x_zero_pt), (100-y_zero_pt)))
    plt.show()
