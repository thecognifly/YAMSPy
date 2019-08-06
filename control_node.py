import os
import time
import numpy as np

import filterpy
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from pid import PID

FREQ = 20
TofFilter_freq = 20
PERIOD = 1/FREQ
ABS_MAX_VALUE_ROLL = 150
ABS_MAX_VALUE_PITCH = 150
ABS_MAX_VALUE_THROTTLE = 150


Y_GAIN = 40
X_GAIN = 40
Z_GAIN = 75

def control_process(*args):

    control_optflow_pipe_read, control_cv_pipe_read, control_tof_pipe_read, control_imu_pipe_read, ext_control_pipe_write, ext_control_pipe_read, nice_level = args

    os.nice(nice_level)

    # >>Initialize KF
    dt = 1/TofFilter_freq
    # Set up the filter
    tof_filter = KalmanFilter(dim_x = 2, dim_z = 1, dim_u = 1)

    # Measurement Matrix
    tof_filter.H = np.array([[1, 0]])

    # The Sensor Model
    tof_filter.F = np.array([[1, dt],
                            [0, 1]]) 
    # tof_filter.F[0,1] = dt

    # Control Matrix
    tof_filter.B = np.array([[0.5*(dt**2)],
                            [dt]]) 
    # tof_filter.B[0] = 0.5*(dt**2)
    # tof_filter.B[1] = dt

    # Process covariance
    tof_filter.Q *= 0.001
    # Measurement covariance
    # Noise of he sensor ~0.01m (1cm)
    tof_filter.R = np.array([[0.01]])

    #throttle PID
    throttle_pd = PID(0.8, 0, 0.4)

    imu = [[0,0,0]]

    value_available = False
    altitude = None
    postition_hold = False
    prev_altitude = 0
    prev_time = time.time()
    while True:

        CMDS = {
                'throttle': 0,
                'roll':     0,
                'pitch':    0
        }

        if ext_control_pipe_write.poll(): # joystick loop tells when to save the current values
            postition_hold = ext_control_pipe_write.recv()
            if not postition_hold:
                prev_altitude = None
            
        if control_imu_pipe_read.poll():
            imu = control_imu_pipe_read.recv() # [[accX,accY,accZ], [gyroX,gyroY,gyroZ], [magX,magY,magZ]]

        if postition_hold:
            # Remember to reset integrator here too!
            prev_altitude = altitude
            postition_hold = False

            # initial value from the sensor
            # the cognifly have the initial heigth of 0.11m
            tof_filter.x = np.array([[altitude], 
                                    [0]]) 
            # covariance matrix
            tof_filter.P *= np.array([[0.1, 0],
                                      [0, 0.1]])
            continue

        if prev_altitude:
            # This is reading the ToF output
            dt = time.time()-prev_time
            tof_filter.F[0,1] = dt
            tof_filter.B[0] = 0.5*(dt**2)
            tof_filter.B[1] = dt            
            altitude_kf = tof_filter.predict(u = imu[0][2])
            altitude = altitude_kf[0][0]
            velocity = altitude_kf[0][1]

            error = altitude - prev_altitude
            # next_throttle = -Z_GAIN*error
            next_throttle = throttle_pd.calc(altitude, velocity=velocity)
            CMDS['throttle'] = next_throttle if abs(next_throttle) <= ABS_MAX_VALUE_THROTTLE else (-1 if next_throttle < 0 else 1)*ABS_MAX_VALUE_THROTTLE 
            value_available = True

        if control_tof_pipe_read.poll():
            tof_filter.update(control_tof_pipe_read.recv())


        # This is reading the opticalflow output (around 10Hz)
        if control_optflow_pipe_read.poll():
            x_motion, y_motion = control_optflow_pipe_read.recv()

            next_roll = -Y_GAIN*y_motion
            CMDS['roll'] = next_roll if abs(next_roll) <= ABS_MAX_VALUE_ROLL else (-1 if next_roll < 0 else 1)*ABS_MAX_VALUE_ROLL 

            next_pitch = -X_GAIN*x_motion
            CMDS['pitch'] = next_pitch if abs(next_pitch) <= ABS_MAX_VALUE_PITCH else (-1 if next_pitch < 0 else 1)*ABS_MAX_VALUE_PITCH 

            value_available = True

        # # This is just to check the speed... (around 2Hz)
        # if control_cv_pipe_read.poll():
        #     data_recv = control_cv_pipe_read.recv()
        #     if data_recv:
        #         (x_motion, y_motion), area = data_recv

        #         if y_motion:
        #             next_roll = -Y_GAIN*y_motion
        #             CMDS['roll'] = next_roll if abs(next_roll) <= ABS_MAX_VALUE_ROLL else (-1 if next_roll < 0 else 1)*ABS_MAX_VALUE_ROLL 

        #         if y_motion:
        #             next_pitch = -X_GAIN*x_motion
        #             CMDS['pitch'] = next_pitch if abs(next_pitch) <= ABS_MAX_VALUE_PITCH else (-1 if next_pitch < 0 else 1)*ABS_MAX_VALUE_PITCH 

        #         value_available = True


        if value_available and (not ext_control_pipe_read.poll()):
            ext_control_pipe_write.send(CMDS)
            value_available = False

        prev_time = time.time()
        time.sleep(PERIOD)
