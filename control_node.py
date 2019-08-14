import os
import time
import numpy as np

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from pid import PID

# Basic parameter
FREQ = 100
PERIOD = 1/FREQ
TOFFILTER_FREQ = 20
ABS_MAX_VALUE_ROLL = 50
ABS_MAX_VALUE_PITCH = 50
ABS_MAX_VALUE_THROTTLE = 100

# Take off altitude
TAKEOFF_ALTITUDE = 1 # m
TAKEOFF_THRUST = 400 # 11.6V -> 400
# 420 is too much for takeoff
TAKEOFF_LIST = np.zeros(20)
for t in range(len(TAKEOFF_LIST)):
    TAKEOFF_LIST[t] = ((1-(1/np.exp(t))))# act like a cap-charge shape
TAKEOFF_LIST = TAKEOFF_LIST.tolist()

#Pitch PD Gain 
PX_GAIN = 40
DX_GAIN = 40

#Roll PD Gain
PY_GAIN = 40
DY_GAIN = 40

#Altitude Gain
PZ_GAIN = 70 * 0.8
DZ_GAIN = 130 * 0.8

def control_process(*args):
    
    control_optflow_pipe_read, control_cv_pipe_read, control_tof_pipe_read, control_imu_pipe_read, ext_control_pipe_write, ext_control_pipe_read, nice_level = args
    
    os.nice(nice_level)
    
    dt = 1/TOFFILTER_FREQ
    '''
    ToF Filter
    '''
    # Set up the ToF filter
    tof_filter = KalmanFilter(dim_x = 2, dim_z = 2, dim_u = 1)
    # Measurement Matrix
    tof_filter.H = np.array([[1, 0], [0, 1]])
    # The Sensor Model
    tof_filter.F = np.array([[1, dt],
                            [0, 1]])   
    # Control Matrix
    tof_filter.B = np.array([[0],
                             [dt]]) 
    # Process covariance
    tof_filter.Q = np.diag([0.9, 0.4])
    # Measurement covariance
    # Noise of he sensor ~0.01m (1cm)
    tof_filter.R = np.diag([0.02**2, 0.05**2])
    
    '''
    XY Filter
    '''
    # Set up the XY filter
    KFXY = KalmanFilter(dim_x = 4, dim_z = 2, dim_u = 1)

    KFXY.x = np.array([ [0], #dx
                        [0], #dy
                        [0], #vx
                        [0]],dtype=float)#vy

    KFXY.F = np.diag([1., 1., 1., 1.])

    KFXY.P = np.diag([.9, .9, 1., 1.])

    KFXY.B = np.diag([1., 1., 1., 1.]) 

    KFXY.H = np.array([ [0, 0, 1., 0], 
                        [0, 0, 0, 1.]]) 

    KFXY.Q = .9
    KFXY.R = 0.12
    KFXY_z = np.array([ [0.], # Update value of the XY filter
                        [0.]],dtype=float)
    KFXY_u = np.array([ [0.], # Control input for XY filter
                        [0.],
                        [0.],
                        [0.]],dtype=float)

    # IMU value 
    imu = [[0,0,0][0,0,0][0,0,0]]

    '''
    PID
    '''
    #throttle PID
    throttle_pd = PID(PZ_GAIN, 0, DZ_GAIN)
    #roll PID
    roll_pd = PID(PY_GAIN, 0, DY_GAIN)
    pre_y = 0
    #pitch PID
    pitch_pd = PID(PX_GAIN, 0, DX_GAIN)
    pre_x = 0

    # init takeoff
    TAKEOFF = True
    
    prev_altitude_sensor = None
    altitude_sensor = None
    value_available = False
    altitude = None
    postition_hold = False
    init_altitude = 0
    prev_time = time.time()

    CMDS = {
            'throttle': 0,
            'roll':     0,
            'pitch':    0
    }
    while True:
        CMDS['throttle'] = 0
        CMDS['roll']     = 0
        CMDS['pitch']    = 0

        if ext_control_pipe_write.poll(): # joystick loop tells when to save the current values
            postition_hold = ext_control_pipe_write.recv()
            altitude_sensor = control_tof_pipe_read.recv()
            if not postition_hold:
                init_altitude = None                    
                # np.save("/home/pi/saved_data", save_values)
            
        if control_imu_pipe_read.poll():
            imu = control_imu_pipe_read.recv() # [[accX,accY,accZ], [gyroX,gyroY,gyroZ], [roll,pitch,yaw]]

        if postition_hold and altitude_sensor:
            # Remember to reset integrator here too!
            prev_altitude_sensor = init_altitude = altitude_sensor
            postition_hold = False

            # initial value from the sensor
            # the cognifly have the initial heigth of 0.11m
            tof_filter.x = np.array([[init_altitude], 
                                    [0]]) 
            # covariance matrix
            tof_filter.P = np.array([[0.1, 0],
                                     [0, 0.1]])
            continue

        # Altitude hold
        if init_altitude:
            # This is reading the ToF output
            dt = time.time()-prev_time
            tof_filter.F[0,1] = dt
            tof_filter.B[0] = 0.5*(dt**2)
            tof_filter.B[1] = dt
            tof_filter.predict(u = 0) #Just test for non -9.81*(0.99-imu[0][2])
            altitude = tof_filter.x[0,0]
            velocity = tof_filter.x[1,0]
            
            # Takeoff 
            if TAKEOFF:
                if len(TAKEOFF_LIST):
                    CMDS['throttle'] = TAKEOFF_LIST[0]*TAKEOFF_THRUST
                    # TAKEOFF_LIST = np.delete(TAKEOFF_LIST, 0)
                    value_available = True
                    TAKEOFF_LIST.pop(0)
                    cancel_gravity_value = CMDS['throttle']
                else:
                    # print (">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Reached")
                    init_altitude = altitude    
                    TAKEOFF = False
                        
            # print("altitude,velocity,sensor", altitude, velocity, altitude_sensor)
            # save_values.append([dt, altitude, velocity, altitude_sensor,-9.81*(0.99-imu[0][2])])
            
            error_altitude =  init_altitude - altitude
            next_throttle = throttle_pd.calc(error_altitude, velocity=-velocity) 
            if (not TAKEOFF):
                CMDS['throttle'] = next_throttle if abs(next_throttle) <= ABS_MAX_VALUE_THROTTLE else (-1 if next_throttle < 0 else 1)*ABS_MAX_VALUE_THROTTLE
                CMDS['throttle'] += cancel_gravity_value # Constant CG
                value_available = True 
                prev_altitude_sensor = altitude_sensor
            if control_tof_pipe_read.poll():
                if not init_altitude:
                    altitude_sensor = control_tof_pipe_read.recv()
                else:
                    altitude_sensor = control_tof_pipe_read.recv()
                    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>IMU", imu)
                    # altitude_sensor =  altitude_sensor * (np.cos(imu[2][0]*np.pi*1/180))* (np.cos(imu[2][1]*np.pi*1/180)) # turning the altitdue back to ground
                    tof_filter.update([altitude_sensor, (altitude_sensor-prev_altitude_sensor)/dt])
                # print (">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>", altitude_sensor, prev_altitude_sensor, dt, (altitude_sensor-prev_altitude_sensor)/dt)
                    
        # XY hold 
        # update the filter
        # if control_optflow_pipe_read.poll():
        #     KFXY_z[0,0], KFXY_z[1,0] = control_optflow_pipe_read.recv()
        #     KFXY.update(KFXY_z)

        # dt = time.time()-prev_time
        # KFXY.F[0,2] = (tof_filter.x[0,0]*dt)
        # KFXY.F[1,3] = (tof_filter.x[0,0]*dt)
        # KFXY.B[2,2] = dt
        # KFXY.B[3,3] = dt
        # KFXY_u[2,0] = imu[0][0] # ax
        # KFXY_u[3,0] = imu[0][1] # ay
        # KFXY.predict(u=KFXY_u) # [dx, dy, vx, vy]
    
        # next_roll = roll_pd.calc((KFXY.x[1,0] - pre_y), velocity=-KFXY.x[3,0]) # Y
        # next_pitch = pitch_pd.calc((KFXY.x[0,0] - pre_x), velocity=-KFXY.x[2,0]) # X
        # pre_x = KFXY.x[0,0]
        # pre_y = KFXY.x[1,0]
        # CMDS['roll'] = next_roll if abs(next_roll) <= ABS_MAX_VALUE_ROLL else (-1 if next_roll < 0 else 1)*ABS_MAX_VALUE_ROLL 
        # CMDS['pitch'] = -next_pitch if abs(next_pitch) <= ABS_MAX_VALUE_PITCH else (-1 if next_pitch < 0 else 1)*ABS_MAX_VALUE_PITCH 
        # value_available = True
        
        # This is reading the opticalflow output (around 10Hz)

        # save_values.append((tof_filter.x[0,0]*dt, dt, imu[0][0], imu[0][1],KFXY_z))

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
