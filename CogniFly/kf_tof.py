# pip install filterpy
# https://filterpy.readthedocs.io/en/latest/
import filterpy
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

# dim_x is number of state
# dim_z is number of measurement
# The output will become [displacement
#                           velocity  ]
tof_filter = KalmanFilter(dim_x=2, dim_z=1)

# t is the time different between each collected data
t = None


# initial value from the sensor
# Assume the 
tof_filter.x = None 

# The Sensor Model
tof_filter.F = np.array([[1, t],
                         [0, 1]]) 
# Control Matrix
# in cm/s
tof_filter.B = np.array([[0.5*(t**2)*(-98.1)],
                         [t*(-98.1)]]) 

# Measurement Matrix
tof_filter.H = np.array([[1, 0]])

# covariance matrix
tof_filter.P *= .1     

# Process covariance
tof_filter.Q *= 0.005 

# Measurement covariance
tof_filter.R = np.array([[0.7]])

def tof():
    tof_filter.predict()
    return (tof_filter.x)

def update(data, _t):
    global t
    if not tof_filter.x:
        tof_filter.x = np.array([[data], 
                                [0]]) 
    t = _t
    tof_filter.update([data])