'''
This Program using a filter to define the optical flow have or have not change in vertical motion
When an output given, mean the optical flow have a vertical motion.
By observate the histogram, can discover that the no. of vector divied by zero is similar
The similar ratio is depent on size of the data set and default 14% -> UP_DOWN_THRESHOLD
>> 07 July 2019
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
UP_DOWN_THRESHOLD = 0.14 # for filtering change in altitude motion

# /////////////////////////
# >>> Data Parameter
# /////////////////////////
displacementx = displacementy = 0 # Displacement of x and y
fin_x = fin_y = 0

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
# >>> Define mode and upper & lower quartile of the data set
# /////////////////////////
def mode_q1q3_median(x, y): # If the size of the data set is zero, it will return 0
    if len(x) > 0 :
        xmode = stats.mode(x)
        xmode = (xmode[0][0]) # Present the data
        x_q1 = x[int(len(x)/4)] # 25% of the data
        medianx =  x[int(len(x)/2)] # Median of data
        x_q3 = x[int(3*(len(x))/4)] # 75% of the data
    else:
        xmode = medianx = x_q1 = x_q3 = 0
    if len(y) > 0:
        ymode = stats.mode(y)
        ymode = (ymode[0][0])
        y_q1 = y[int(len(y)/4)] # 25% of the data
        mediany =  y[int(len(y)/2)] # Median of data
        y_q3 = y[int(3*(len(y))/4)] # 25% of the data
    else:
        ymode = mediany = y_q1 = y_q3 =0
    return xmode, ymode, x_q1, x_q3 ,y_q1, y_q3, medianx, mediany

# /////////////////////////
# >>> Sort the data set
# /////////////////////////
def sort(x, y): # Rearrange of order of the sample
    return np.sort(x[:,0]), np.sort(y[:,0]) # x[:,0] = all array with first elenment

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
# >>> Setting UP and Down Filter # by checking the speartion of >0 or <0
# /////////////////////////
def speartion_filter (x, y):
    global UP_DOWN_THRESHOLD # Check the speartion is smaller than 14% default of total data set
    X = Y = True
    if (len(x) > 0):
        if ((abs(len(np.where(x<0)[0])-len(np.where(x>0)[0]))/len(x)) < UP_DOWN_THRESHOLD):
            print (abs(len(np.where(x<0)[0])-len(np.where(x>0)[0]))/len(x))
            X = False
    if (len(y) > 0):
        if ((abs(len(np.where(y<0)[0])-len(np.where(y>0)[0]))/len(y)) < UP_DOWN_THRESHOLD):
            print (abs(len(np.where(y<0)[0])-len(np.where(y>0)[0]))/len(y))
            Y = False
    return X, Y

# /////////////////////////
# >>> Setting range Filter # check the q1 q3 is same as mode
# /////////////////////////
def range_filter(x, y, modex, modey, q1x, q3x, q1y, q3y):
    if (modex != q1x != q3x):
        a = np.where(x == q1x)[0][0]  # First arrays of q1 in list
        b = np.where(x == q3x)[0][-1] # Last array of q3 in list
        X = np.mean(x[a:b])
    else:
        X = modex # if it is the same, return the mode
    if (modey != q1y != q3y):
        a = np.where(y == q1y)[0][0]  # First arrays of q1 in list
        b = np.where(y == q3y)[0][-1] # Last array of q3 in list
        Y = np.mean(y[a:b])
    else:
        Y = modey # if it is the same, return the mode
    return X, Y

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
# >>> Calc the displacement
# /////////////////////////
def displacement(x,y):
    global displacementx, displacementy
    displacementx += x # by summing
    displacementy += y # by summing
    return  displacementx, displacementy

# /////////////////////////
# >>> Main
# /////////////////////////
try:
    for j in range (len(data)):
    # for j in range (927, 943): # looping for reading each frame of data, j
        sad_filter(j, 1.8) # (frame, k) using 1.8 gain to lower the SAD limit. Default is 1.5
        X, Y = twoD2oneD(j) # (frame) X = 1D x data, Y = 1D y data
        X_nonzero, Y_nonzero ,x_zero_pt, y_zero_pt = nonzero(j) # Reutrn 1D array of the data without any zero member
        X_nonzero, Y_nonzero = sort(X_nonzero, Y_nonzero) # find the q1 and q3 by array form
        x_mode, y_mode, x_q1, x_q3 ,y_q1, y_q3, x_med, y_med = mode_q1q3_median(X_nonzero[:], Y_nonzero[:]) # Return the mode of the X_nonzero and Y_nonzero
        x_h, y_h = speartion_filter(X_nonzero, Y_nonzero) # Return True of False
        fin_x, fin_y = range_filter(X_nonzero, Y_nonzero, x_mode, y_mode, x_q1, x_q3 ,y_q1, y_q3) # for finding average output
        # print ("%d - X Data is zero =  %f percent - Y Data is zero =  %f percent" % (j, x_zero_pt, y_zero_pt))
        if (x_h == False or y_h == False):
            print ("%d - Mode X = %f, med = %f, Q1 = %f, Q3 = %f -- %f" % (j, x_mode, x_med, x_q1, x_q3, x_zero_pt))
            print ("%d - Mode Y = %f, med = %f, Q1 = %f, Q3 = %f -- %f" % (j, y_mode, y_med, y_q1, y_q3, y_zero_pt))
            bin = np.arange(-64, 65)
            plt.xlabel("Data")
            plt.ylabel("Number of Data")
            # plt.hist (X_nonzero[:], bins = bin, color = 'red', label = 'X')
            plt.hist (Y_nonzero[:], bins = bin, color = 'blue', label = 'Y')
            plt.xticks(bin-.5, bin)
            plt.legend(loc='upper right')
            plt.title("Frame %d  x: %f  - y: %f" %(j, (100-x_zero_pt), (100-y_zero_pt)))
            plt.show()
        # print ("final X = %f" % fin_x)
        # print ("final Y = %f" % fin_y)



finally:
        # np.save('experiment2', _data) # Save the displacement data in .npy
        pass
