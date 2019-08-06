# !pip install filterpy
# https://filterpy.readthedocs.io/en/latest/
import time
import numpy as np
from multiprocessing import Process, Pipe

import filterpy
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from tof_node import ToF

control_ToF_pipe_write, control_ToF_pipe_read = Pipe()
TofFilter_freq = 30 #Hz
# dim_x is number of state
# dim_z is number of measurement
# dim_u is number of control input
# The output will become [displacement
#                           velocity  ]

# Setup the ToF library
tof = ToF(control_ToF_pipe_write, control_ToF_pipe_read)

# Sleep Time:
t = 1/TofFilter_freq
# Set up the filter
tof_filter = KalmanFilter(dim_x = 2, dim_z = 1, dim_u = 1)

# initial value from the sensor
# the cognifly have the initial heigth of 0.11m
tof_filter.x = np.array([[0.11], 
                        [0]]) 
# The Sensor Model
tof_filter.F = np.array([[1, t],
                        [0, 1]]) 
# Control Matrix
tof_filter.B = np.array([[0.5*(t**2)*(1)],
                        [t*(1)]]) 

# Measurement Matrix
tof_filter.H = np.array([[1, 0]])

# covariance matrix
tof_filter.P *= 0.01 

# Process covariance
tof_filter.Q *= 0.01

# Measurement covariance
# Noise of he sensor ~0.05m
tof_filter.R = np.array([[1]])

# >>> Read data
if control_ToF_pipe_read.poll():
        tof_filter.update([control_ToF_pipe_read.recv()])
else:
        # u is the acceration of the z-axis
        tof_filter.predict(u)
