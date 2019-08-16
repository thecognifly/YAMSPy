import os
import time
import numpy as np

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from pid import PID

'''Basic parameter'''
PERIOD = 1/100               # Sleeping time
ABS_MAX_VALUE_ROLL = 30      # PID Roll limit
ABS_MAX_VALUE_PITCH = 30     # PID Pitch limit
ABS_MAX_VALUE_THROTTLE = 100 # PID Throttle limit

'''Takeoff parameter'''
TAKEOFF_ALTITUDE = 0.2#m     # Take off altitude
TAKEOFF_THRUST = 360 #12.35V ->360  # 11.6V -> 400 #11.31 -> 410 # weight -> 340 # 420 is too much for takeoff
TAKEOFF_LIST = np.zeros(20)  # Creating the take off curve
for t in range(len(TAKEOFF_LIST)):
    TAKEOFF_LIST[t] = ((1-(1/np.exp(t))))# act like a cap-charge shape
TAKEOFF_LIST = TAKEOFF_LIST.tolist()

'''PID'''
#Pitch PD Gain 
PX_GAIN = 10
DX_GAIN = 0
#Roll PD Gain
PY_GAIN = 100
DY_GAIN = 0
#Altitude Gain
PZ_GAIN = 60
IZ_GAIN = 5
DZ_GAIN = 5 #130 * 0.15

def control_process(*args):
    
    control_optflow_pipe_read, control_cv_pipe_read, control_tof_pipe_read, control_imu_pipe_read, ext_control_pipe_write, ext_control_pipe_read, nice_level = args

    os.nice(nice_level)
    
    '''ToF Filter'''
    dt = 0.01                                                    # Just random assumation
    tof_filter = KalmanFilter(dim_x = 2, dim_z = 2, dim_u = 1)   # Set up the ToF filter
    tof_filter.F = np.array([[1, dt],                            # The Sensor Model
                             [0, 1]])
                
    tof_filter.P = np.diag([0.1, 0.1])                          # covariance matrix
    tof_filter.B = np.array([[0],                                # Control Matrix
                             [dt]]) 
    tof_filter.H = np.diag([1., 1.])                             # Measurement Matrix
    tof_filter.Q = np.diag([0.9, 0.4])                           # Process covariance
    tof_filter.R = np.diag([0.02**2, 0.05**2])                   # Measurement covariance  # Noise of he sensor ~0.01m (1cm)
   
    '''XY Filter'''
    KFXY = KalmanFilter(dim_x = 4, dim_z = 2, dim_u = 1)         # Set up the XY filter
    KFXY.x = np.array([ [0], #dx
                        [0], #dy
                        [0], #vx
                        [0]],#vy
                        dtype=float)

    KFXY.F = np.array([1., 1., 1., 1.])
    KFXY.P = np.diag([.9, .9, 1., 1.])
    KFXY.B = np.diag([1., 1., 1., 1.]) 
    KFXY.H = np.array([[0, 0, 1., 0], 
                       [0, 0, 0, 1.]]) 
    KFXY.Q *= 0.01
    KFXY.R *= 0.01
    KFXY_z = np.array([ [0.], # Update value of the XY filter
                        [0.]],dtype=float)
    KFXY_u = np.array([ [0.], # Control input for XY filter
                        [0.],
                        [0.],
                        [0.]],dtype=float)

    '''IMU value init''' 
    imu = [[0,0,0],[0,0,0],[0,0,0]]

    ''' PID Init '''
    throttle_pd = PID(PZ_GAIN, IZ_GAIN, DZ_GAIN)    #throttle PID
    roll_pd = PID(PY_GAIN, 0, DY_GAIN)        #roll PID
    pitch_pd = PID(PX_GAIN, 0, DX_GAIN)       #pitch PID
    init_x = 0
    init_y = 0

    '''init takeoff'''
    TAKEOFF = True

    
    prev_altitude_sensor = None
    altitude_sensor = None
    altitude = None
    altitude_corrected = None
    value_available = False
    postition_hold = False
    init_altitude = 0
    prev_time = time.time()

    '''CMDS init'''
    CMDS = {
            'throttle': 0,
            'roll':     0,
            'pitch':    0}

    while True:
        CMDS['throttle'] = 0
        CMDS['roll']     = 0
        CMDS['pitch']    = 0

        '''Read the joystick_node trigger the auto mode or not'''
        if ext_control_pipe_write.poll(): # joystick loop tells when to save the current values
            postition_hold = ext_control_pipe_write.recv()
            if not postition_hold:
                init_altitude = None                    
        
        '''Update the IMU value'''
        if control_imu_pipe_read.poll():
            imu, battery_voltage = control_imu_pipe_read.recv() # [[accX,accY,accZ], [gyroX,gyroY,gyroZ], [roll,pitch,yaw]]
            if TAKEOFF:
                TAKEOFF_THRUST = int(1015-53*(battery_voltage))
                # print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n",TAKEOFF_THRUST, battery_voltage)

        '''Update the ToF Kalman Filter with the ground value'''
        if postition_hold and altitude_sensor:
            # Remember to reset integrator here too!
            prev_altitude_sensor = init_altitude = altitude_sensor
            postition_hold = False
            # initial value from the sensor
            # the cognifly have the initial heigth of 0.11m
            tof_filter.x = np.array([[init_altitude], 
                                    [0]]) 
            continue

        '''Vertical Movement Control'''
        if init_altitude:
            # Update the ToF Filter
            dt = time.time()-prev_time
            tof_filter.F[0,1] = dt
            tof_filter.B[0] = 0.5*(dt**2)
            tof_filter.B[1] = dt
            tof_filter.predict(u = 0) #Just test for non -9.81*(0.99-imu[0][2])
            # Capture the Predicted value
            altitude = tof_filter.x[0,0]
            velocity = tof_filter.x[1,0]
            
            # '''Takeoff Setting''' 
            if TAKEOFF:
                if len(TAKEOFF_LIST):
                    CMDS['throttle'] = TAKEOFF_LIST[0] * TAKEOFF_THRUST
                    value_available = True
                    TAKEOFF_LIST.pop(0)
                    cancel_gravity_value = CMDS['throttle']
                else:
                    init_altitude = TAKEOFF_ALTITUDE 
                    velocity = 0
                    TAKEOFF = False

            # '''PID at Throttle'''
            if (not TAKEOFF):
                error_altitude =  init_altitude - altitude # altitude
                next_throttle = throttle_pd.calc(error_altitude, time = dt, velocity=-velocity) 
                CMDS['throttle'] = next_throttle if abs(next_throttle) <= ABS_MAX_VALUE_THROTTLE else (-1 if next_throttle < 0 else 1)*ABS_MAX_VALUE_THROTTLE
                CMDS['throttle'] += cancel_gravity_value # Constant CG
                value_available = True 
                prev_altitude_sensor = altitude_corrected
                
        '''Update the ToF value'''
        if control_tof_pipe_read.poll():
            if not init_altitude:
                altitude_sensor = control_tof_pipe_read.recv() # Flushing the old value 
            else:
                # turning the altitdue back to ground, reference as global coordinate
                altitude_sensor = control_tof_pipe_read.recv()
                # h * cos(roll) * cos(pitch)
                altitude_corrected =  altitude_sensor * (np.cos(imu[2][0]*np.pi*1/180)) * (np.cos(imu[2][1]*np.pi*1/180)) 
                altitude_corrected = int(altitude_corrected*100)
                altitude_corrected = altitude_corrected/100  # Truncate 2 d.p.
                tof_filter.update([altitude_corrected, (altitude_corrected-prev_altitude_sensor)/dt])
                    
        '''Update the XY Filter'''

        # if ((not TAKEOFF) and (abs(error_altitude) < 0.2)):
        if (not TAKEOFF):
            dt = time.time()-prev_time
            KFXY.F[0,2] = (altitude*dt)
            KFXY.F[1,3] = (altitude*dt)
            KFXY.B[2,2] = dt
            KFXY.B[3,3] = dt
            KFXY_u[2,0] = 0 #imu[0][0] # ax
            KFXY_u[3,0] = 0 #imu[0][1] # ay
            if control_optflow_pipe_read.poll():
                KFXY_z[0,0], KFXY_z[1,0] = control_optflow_pipe_read.recv()
                KFXY_z[0,0], KFXY_z[1,0] = control_optflow_pipe_read.recv() # it will block until a brand new value comes.
                KFXY.update(KFXY_z*(-altitude))# To real scale # X-Y reversed
            KFXY.predict(u=KFXY_u) # [dx, dy, vx, vy]
            error_roll = (init_y - KFXY.x[1,0])
            error_pitch =(init_x - KFXY.x[0,0])
            error_roll = (int(error_roll*100))/100      # Truncate to 2d.p.
            error_pitch = (int(error_pitch*100))/100    # Truncate to 2d.p.
            velocity_roll = KFXY.x[3,0]
            velocity_pitch = KFXY.x[2,0]
            velocity_roll = (int(velocity_roll*100))/100      # Truncate to 2d.p.
            velocity_pitch = (int(velocity_pitch*100))/100    # Truncate to 2d.p.
            next_roll = roll_pd.calc(error_roll, velocity=-velocity_roll) # Y
            next_pitch = pitch_pd.calc(error_pitch, velocity=-velocity_pitch) # X
            CMDS['roll'] = next_roll if abs(next_roll) <= ABS_MAX_VALUE_ROLL else (-1 if next_roll < 0 else 1)*ABS_MAX_VALUE_ROLL 
            print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n",error_roll, velocity_roll, next_roll)
            # CMDS['pitch'] = -next_pitch if abs(next_pitch) <= ABS_MAX_VALUE_PITCH else (-1 if next_pitch < 0 else 1)*ABS_MAX_VALUE_PITCH 
            # value_available = True

            # # This is just to check the speed... (around 2Hz)
            # if control_cv_pipe_read.poll():
            #     data_recv = control_cv_pipe_read.recv()
            #     if data_recv:
            #         (x_motion, y_motion), area = data_recv
            #         if y_motion:
            #             next_roll = -Y_GAIN*y_motion
            #             CMDS['roll'] = next_roll if abs(next_roll) <= ABS_MAX_VALUE_ROLL else (-1 if next_roll < 0 else 1)*ABS_MAX_VALUE_ROLL 
            #         if x_motion:
            #             next_pitch = -X_GAIN*x_motion
            #             CMDS['pitch'] = next_pitch if abs(next_pitch) <= ABS_MAX_VALUE_PITCH else (-1 if next_pitch < 0 else 1)*ABS_MAX_VALUE_PITCH 

            #         value_available = True


        '''Send out the CMDS values back to the joystick loop'''
        if value_available and (not ext_control_pipe_read.poll()):
            ext_control_pipe_write.send(CMDS)
            value_available = False

        prev_time = time.time()
        time.sleep(PERIOD)        
