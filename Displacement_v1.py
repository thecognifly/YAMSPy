'''
Using SAD filter
'''
import numpy as np
import matplotlib
import math
import time
from matplotlib import pyplot as plt
from matplotlib import animation
from scipy import stats

# /////////////////////////
# >>> Import Data
# /////////////////////////
data = np.load('experiment_raw.npy')# import the raw data
_data = []
# data[frame][row, columns]
x = np.copy(data['x']) # using 640x480 resolution
y = np.copy(data['y']) # 30 row, 41 columns
sad = np.copy(data['sad']) # Sum of Absolute Difference >> hows the data reliable
print ("Size of Data: %d" %(len(data['x'])))

# /////////////////////////
# >>> Basic Parameter
# /////////////////////////
FRAMERATE = 24    # Framerate during optical flow
FOCAL_LENGTH_M = 3.6 * (1000)  # original focal lenght: 3.6mm

# /////////////////////////
# >>> Data Parameter
# /////////////////////////
displacementx = displacementy = 0 # Displacement of x and y

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
# >>> Define mode of the data set
# /////////////////////////
def mode(x, y): # If the size of the data set is zero, it will return 0
    if len(x) > 0 :
        xmode = stats.mode(x)
        xmode = (xmode[0][0][0]) + 1 # Present the data
    else:
        xmode = 0
    if len(y) > 0:
        ymode = stats.mode(y)
        ymode = (ymode[0][0][0]) + 1
    else:
        ymode = 0
    return xmode, ymode  # offset 1 unit

# /////////////////////////
# >>> Setting SAD Filter # Using the average sad value of each set
# /////////////////////////
def sad_filter (frame, k = 1.5): # This function only work on each frame. In real time, delect all the frame argument
    global x,  y, sad
    sad_threshold = np.mean(sad[frame]) * k # smaller the sad value, the better of data. k is use to reduce the limit
    sad_filter = np.where(sad[frame]>sad_threshold) # sad_filter is mean the data not reliable
    x[frame][sad_filter] = 0                        # therefore, set those data to zero.
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
    X_nonzero = np.zeros([len(np.where(X[:,0] != 0)[0])]) # left all the data not equal to zero
    Y_nonzero = np.zeros([len(np.where(Y[:,0] != 0)[0])])
    return (X[(np.where(X[:,0] != 0)[0])], # return x-y nonzero data and the percentage of zero data
            Y[(np.where(Y[:,0] != 0)[0])],
            (len(X)-len(X_nonzero))*100/len(X), (len(Y)-len(Y_nonzero))*100/len(Y))

# /////////////////////////
# >>> Main
# /////////////////////////
try:
    for j in range (len(data)): # looping for reading each frame of data, j
        sad_filter(j, 1.8) # (frame, k) using 1.8 gain to lower the SAD limit. Default is 1.5
        X, Y = twoD2oneD(j) # (frame) X = 1D x data, Y = 1D y data
        X_nonzero, Y_nonzero ,x_zero_pt, y_zero_pt = nonzero(j) # Reutrn 1D array of the data without any zero member
        x_mode, y_mode = mode(X_nonzero[:], Y_nonzero[:]) # Return the mode of the X_nonzero and Y_nonzero
        # print ("%d - X Data is zero =  %f percent - Y Data is zero =  %f percent" % (j, x_zero_pt, y_zero_pt))
        # print ("Mode X = %f, Mode Y = %f" % (x_mode, y_mode))
        _data.append((displacementx, displacementy, j))
        displacementx += x_mode # update the displacement x-y with summing. NOT IN SCALE !
        displacementy += y_mode
finally:
        np.save('experiment', _data) # Save the displacement data in .npy 
