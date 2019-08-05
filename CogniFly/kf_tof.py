# !pip install filterpy
# https://filterpy.readthedocs.io/en/latest/
import time
import numpy as np

import filterpy
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

# dim_x is number of state
# dim_z is number of measurement
# The output will become [displacement
#                           velocity  ]
tof_filter = KalmanFilter(dim_x=2, dim_z=1, dim_u = 1)

def init(freq ,data):
        # Sleep Time:
        global t
        t = 1/freq
        # initial value from the sensor
        tof_filter.x = np.array([[data], 
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
        tof_filter.P *= .1 

        # Process covariance
        tof_filter.Q *= 0

        # Measurement covariance
        tof_filter.R = np.array([[1]])

def tof(pipe_read, pipe_write):
        global t
        while True:
                tof_filter.predict()
                if not pipe_read.poll():
                        pipe_write.send(tof_filter.x)
                time.sleep(t)

def update(data):
        tof_filter.update([data])
