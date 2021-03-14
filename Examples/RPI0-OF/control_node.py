import os
import time
import numpy as np

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from pid import PID

class control():

    def __init__(self):
        ''' Define all the parameter'''
        '''Basic parameter'''''
        self.PERIOD = 1/200               # Sleeping time
        self.ABS_MAX_VALUE_ROLL = 250      # PID Roll limit
        self.ABS_MAX_VALUE_PITCH = 250     # PID Pitch limit
        self.ABS_MAX_VALUE_THROTTLE = 300 # PID Throttle limit

        '''Takeoff parameter'''
        self.TAKEOFF_ALTITUDE = 0.7#m     # Take off altitude
        self.TAKEOFF_THRUST = 360 #12.35V ->360  # 11.6V -> 400 #11.31 -> 410 # weight -> 340 # 420 is too much for takeoff
        self.TAKEOFF_LIST = np.zeros(10)  # Creating the take off curve
        for t in range(len(self.TAKEOFF_LIST)):
            self.TAKEOFF_LIST[t] = ((1-(1/np.exp(t))))# act like a cap-charge shape
        self.TAKEOFF_LIST = self.TAKEOFF_LIST.tolist()

        '''init takeoff and landing'''
        self.TAKEOFF = True
        self.LANDING = False

        '''Data Record'''
        self.DATA = False
        self.data = []

        '''ToF sensor offset'''
        # CogniFly offset -> z: -40mm, y: +38mm
        # self.TOFOFFSET_Z = 0.04 #m
        # self.TOFOFFSET_Y = 0.038 #m
        self.TOFOFFSET_Z = -0.02 #m
        self.TOFOFFSET_Y = -0.045 #m

        self.TOFOFFSET_R = np.sqrt(self.TOFOFFSET_Z**2 + self.TOFOFFSET_Y**2)
        self.TOFOFFSET_ANGLE = np.tan(self.TOFOFFSET_Z/self.TOFOFFSET_Y)

        '''Alpha Filter'''
        self.a = 1

        '''Time'''
        self.IMU_TIME = 0 # IMU Timestamp
        self.OF_TIME  = 0 # Optical Flow Timestamp
        self.TOF_Time = 0 # Time of Flight Timestamp

        '''PID'''
        #Pitch PID G0
        self.PX_GAIN = 90
        self.IX_GAIN = 0.
        self.DX_GAIN = 103 
        #Roll PID Gain
        self.PY_GAIN = 90
        self.IY_GAIN = 0.
        self.DY_GAIN = 103
        #Altitude PID Gain
        # For 2S battery
        self.PZ_GAIN = 60
        self.IZ_GAIN = 0.19
        self.DZ_GAIN = 35
                
        '''IMU value init''' 
        self.imu = [[0,0,0],[0,0,0],[0,0,0]]

    def tof_filter_init(self):
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
        return tof_filter

    def xy_of_filter_init(self):
        '''XY Filter'''
        KFXY = KalmanFilter(dim_x = 4, dim_z = 2, dim_u = 1)         # Set up the XY filter
        KFXY.x = np.array([ [0], #dx(pitch)
                            [0], #dy(roll)
                            [0], #vx(pitch)
                            [0]],#vy(roll)
                            dtype=float)

        KFXY.F = np.diag([1., 1., 1., 1.])
        KFXY.P = np.diag([.9, .9, 1., 1.])
        KFXY.B = np.diag([1., 1., 1., 1.]) 
        KFXY.H = np.array([[0, 0, 1., 0], 
                        [0, 0, 0, 1.]]) 
        KFXY.Q *= 0.1**2
        KFXY.R *= 0.01**2
        KFXY_z = np.array([ [0.], # Update value of the XY filter
                            [0.]],dtype=float)
        KFXY_u = np.array([ [0.], # Control input for XY filter
                            [0.],
                            [0.],
                            [0.]],dtype=float)
        
        return KFXY, KFXY_z, KFXY_u # Filter, Filter_Sensor_varible, Control_input

    def value_limit(self, output, limit):
        '''Set the value not excite the limited value'''
        if abs(output) >= limit:
            if output < 0 :
                output = -limit
            else:
                output = limit
        return output 
    
    def truncate(self, data, dp = 2):
        ''' Truncate the value down to 2 dp as default'''
        return (int(data*(10**dp)))/(10**dp)

    def control_process(self, *args):
        '''This function will called from joystick_async as subprocess'''
        control_optflow_pipe_read, control_cv_pipe_read, control_tof_pipe_read, control_imu_pipe_read, ext_control_pipe_write, ext_control_pipe_read, nice_level = args

        os.nice(nice_level)
        
        # Init the ToF kalman filter
        tof_filter = self.tof_filter_init()
        KFXY, KFXY_z, KFXY_u = self.xy_of_filter_init()

        ''' PID Init '''
        throttle_pd = PID(self.PZ_GAIN, self.IZ_GAIN, self.DZ_GAIN)    #throttle PID
        roll_pd = PID(self.PY_GAIN, self.IY_GAIN, self.DY_GAIN)        #roll PID
        pitch_pd = PID(self.PX_GAIN, self.IX_GAIN, self.DX_GAIN)       #pitch PID
        prev_velocity_pitch = 0
        prev_velocity_roll = 0
        prev_error_roll = 0
        prev_error_pitch = 0

        '''Z-axis init'''       
        prev_altitude_sensor = None
        altitude_sensor = None
        altitude = None
        altitude_corrected = None
        value_available = False
        postition_hold = False
        init_altitude = 0

        '''CMDS init'''
        CMDS = {'throttle': 0,
                'roll':     0,
                'pitch':    0}
        prev_time = time.time()
        OF_DIS = of_dis = 0
        # For init
        error_altitude = error_roll = error_pitch = velocity_pitch = velocity_roll = 0

        '''Angular Speed'''
        pre_roll = 0
        pre_pitch = 0

        while True:
            CMDS['throttle'] = 0 
            CMDS['roll']     = 0
            # The betaflight config trim the pitch -10 for unbalance of the drone
            CMDS['pitch']    = 0
            # Let the OF Pipe run
            control_tof_pipe_read.send('a')
            '''Read the joystick_node trigger the auto mode or not'''
            if ext_control_pipe_write.poll(): # joystick loop tells when to save the current values
                postition_hold = ext_control_pipe_write.recv()
                if not postition_hold:
                    self.LANDING = True
                    self.DATA = True
                    init_altitude = None
            
            '''Update the IMU value'''
            if control_imu_pipe_read.poll():
                self.imu, battery_voltage, self.IMU_TIME = control_imu_pipe_read.recv() # [[accX,accY,accZ], [gyroX,gyroY,gyroZ], [roll,pitch,yaw]]
                #DEBUG USE
                imut=time.time()
                angu_time = prev_time-self.IMU_TIME
                if angu_time > 0.001:
                    angu_roll = (self.imu[2][0]-pre_roll)/angu_time
                    angu_pitch = (self.imu[2][1]-pre_pitch)/angu_time
                    pre_roll = self.imu[2][0]
                    pre_pitch = self.imu[2][1]
                else:
                    angu_roll = angu_pitch = 0
                if self.TAKEOFF:
                    # Tested voltage throttle relationship
                    TAKEOFF_THRUST = int(1015-60*(battery_voltage))

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
                dt = prev_time-self.TOF_Time
                # For init reading will very large, but normal case would not larger than 1s
                tof_filter.F[0,1] = dt if abs(dt<3) else 0
                tof_filter.B[0] = 0.5*(dt**2) if abs(dt<3) else 0
                tof_filter.B[1] = dt
                tof_filter.predict(u = 0) #Just test for non -9.81*(0.99-imu[0][2])
                # Capture the Predicted value
                altitude = tof_filter.x[0,0]
                velocity = tof_filter.x[1,0]
                
                # '''Takeoff Setting''' 
                if self.TAKEOFF:
                    if len(self.TAKEOFF_LIST):
                        CMDS['throttle'] = self.TAKEOFF_LIST[0] * TAKEOFF_THRUST
                        value_available = True
                        self.TAKEOFF_LIST.pop(0)
                        cancel_gravity_value = CMDS['throttle']
                    else:
                        # control_optflow_pipe_read.send('a')
                        init_altitude = self.TAKEOFF_ALTITUDE 
                        velocity = 0
                        self.TAKEOFF = False

                # '''PID at Throttle'''
                if (not self.TAKEOFF):
                    error_altitude =  init_altitude - altitude # altitude
                    next_throttle = throttle_pd.calc(error_altitude, time = dt, velocity=-velocity) 
                    # Set throttle by PID control
                    CMDS['throttle'] = self.value_limit(next_throttle, self.ABS_MAX_VALUE_THROTTLE)
                    # Add the cancel gravity set point with the angle compensate
                    CMDS['throttle'] += cancel_gravity_value / ((np.cos(self.imu[2][0]*np.pi*1/180)) * (np.cos(self.imu[2][1]*np.pi*1/180)))
                    value_available = True 
                    prev_altitude_sensor = altitude_corrected

            # LANDING
            if not self.TAKEOFF and self.LANDING:
                CMDS['throttle'] = cancel_gravity_value + (15/(altitude+0.5))
        
            '''Update the ToF value'''
            if control_tof_pipe_read.poll():
                if not init_altitude:
                    altitude_sensor, self.TOF_Time = control_tof_pipe_read.recv() # Flushing the old value 
                    # altitude_sensor = control_tof_pipe_read.recv() # Flushing the old value 
                else:
                    # turning the altitdue back to ground, reference as global coordinate
                    altitude_sensor, self.TOF_Time = control_tof_pipe_read.recv()
                    #DEBUG USE
                    toft=time.time()
                    # The sensor is not at the center axis of rotation
                    # The following parts are going to turn the offset bact the origin
                    # ROLL: (Measure * cos(roll_angle)) - (sensor_offset * sin(sensor_angle - roll_angle)
                    # PITCH: (Measure - offset) * cos(pitch_angle) 
                    # combine: (Measure * cos(roll) * cos(pitch)) - (offset * sin(sensor-roll) * cos(pitch))
                    # CogniFly offset -> z: -40mm, y: +38mm
                    altitude_corrected = altitude_sensor * (np.cos(self.imu[2][0]*np.pi*1/180)) * (np.cos(self.imu[2][1]*np.pi*1/180))
                    offset = self.TOFOFFSET_R * np.sin(self.TOFOFFSET_ANGLE - (self.imu[2][0]*np.pi*1/180)) * (np.cos(self.imu[2][1]*np.pi*1/180))
                    tof_filter.update([self.truncate(altitude_corrected-offset), self.truncate(((altitude_corrected-offset)-prev_altitude_sensor)/dt)])
                        
            '''Update the XY Filter'''
            # if ((not TAKEOFF) and (abs(error_altitude) < 0.2)):
            if (not self.TAKEOFF):
                # For init reading will very large, but normal case would not larger than 1s
                dt_OF = prev_time-self.OF_TIME
                dt_IMU = prev_time-self.IMU_TIME
                # KFXY.R[0,0] = (0.005+(velocity/90)) # Increase the noise for the filter when up and down 
                # KFXY.R[1,1] = (0.005+(velocity/90))
                # # For init reading will very large, but normal case would not larger than 1s
                # KFXY.F[0,2] = dt_OF if abs(dt_OF<3) else 0
                # KFXY.F[1,3] = dt_OF if abs(dt_OF<3) else 0
                # KFXY.B[2,2] = dt_IMU if abs(dt_IMU<3) else 0
                # KFXY.B[3,3] = dt_IMU if abs(dt_IMU<3) else 0
                # # Another angular speed can be optained by (atitude/dt)
                # # linear speed can be optained by angluar_speed*height
                # KFXY_u[2,0] = self.truncate(9.81*(self.imu[0][0])*np.cos(self.imu[2][1])) #imu[0][0]->ax Pitch acc #imu[2][1]->Pitch angle
                # KFXY_u[3,0] = self.truncate(9.81*(self.imu[0][1])*np.cos(self.imu[2][0])) #imu[0][1]->ay Roll acc  #imu[2][0]->Roll angle
                if control_optflow_pipe_read.poll():
                    KFXY_z[0,0], KFXY_z[1,0], of_dis, self.OF_TIME = control_optflow_pipe_read.recv() # it will block until a brand new value comes.
                    #DEBUG USE
                    oft=time.time()
                    # KFXY.update(KFXY_z*(-altitude))# To real scale # X-Y reversed
                
                # KFXY.predict(u=0)#KFXY_u) # [dx, dy, vx, vy]

                OF_DIS += of_dis*altitude
                '''X-Y control'''
                factor = 1.6 #Seem the data is scaled up 1.6 times 

                error_roll  = self.truncate((OF_DIS[1])/factor)
                error_pitch = self.truncate((OF_DIS[0])/factor)
                velocity_roll_tmp = self.truncate(KFXY_z[1,0]*(-altitude))
                velocity_pitch_tmp = self.truncate(KFXY_z[0,0]*(-altitude))
                self.a = np.exp(-abs(velocity_pitch_tmp - velocity_pitch))
                velocity_pitch += self.a * (velocity_pitch_tmp - velocity_pitch)
                self.a = np.exp(-abs(velocity_roll_tmp - velocity_roll))
                velocity_roll  += self.a * (velocity_roll_tmp - velocity_roll)
                # prev_error_roll = error_roll
                # prev_error_pitch = error_pitch
                # velocity_roll_tmp = -self.truncate((error_roll-prev_error_roll)/dt_OF)
                # velocity_pitch_tmp = -self.truncate((error_pitch-prev_error_pitch)/dt_OF)
                # velocity_roll += self.a*(velocity_roll_tmp - prev_velocity_roll)
                # velocity_pitch += self.a*(velocity_pitch_tmp - prev_velocity_pitch)
                # prev_velocity_roll = velocity_roll
                # prev_velocity_pitch = velocity_pitch
                # velocity_roll = self.truncate(KFXY.x[3,0])
                # velocity_pitch = self.truncate(KFXY.x[2,0])

                # The new cognifly is reversed the pi orientation
                next_roll = roll_pd.calc(-error_roll, velocity=velocity_roll) # Y
                next_pitch = pitch_pd.calc(-error_pitch, velocity=velocity_pitch) # X
                CMDS['roll'] = next_roll if abs(next_roll) <= self.ABS_MAX_VALUE_ROLL else (-1 if next_roll < 0 else 1)*self.ABS_MAX_VALUE_ROLL 
                CMDS['pitch'] = next_pitch if abs(next_pitch) <= self.ABS_MAX_VALUE_PITCH else (-1 if next_pitch < 0 else 1)*self.ABS_MAX_VALUE_PITCH 
                value_available = True
                
                print (">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n")     
                print ("OF Distance: ",OF_DIS)  
                print("THROTTLE :{2:.2f}    | ALT:{1:.2f}   |   ERR:{0:.2f}     |   Vec:{3:.2f}".format(error_altitude, altitude, next_throttle, velocity))
                print("dt_OF:{:.2f} |   dt_IMU:{:.2f}".format(dt_OF,dt_IMU))
                print("Angular Roll: {:.2f}     |   Pitch: {:.2f}".format(angu_roll, angu_pitch))
                print("ERROR ROLL : %2.2f  error|  %2.2f roll|  %2.2f of" %(error_roll, next_roll, 0))
                print("ERROR PITCH: %2.2f  error|  %2.2f pitch|  %2.2f of" %(error_pitch, next_pitch, 0))
                print("ROLL velocity: ", -KFXY.x[3,0], KFXY_z[1,0]*(-altitude), self.truncate((self.imu[2][0]*np.pi*1/180*altitude/dt)))
                print("PITCH velocity", -KFXY.x[2,0], KFXY_z[0,0]*(-altitude), self.truncate((self.imu[2][1]*np.pi*1/180*altitude/dt)))
                print("TIME:{0:1.2f}  |  OF:{1:.2f}   |   IMU:{2:.2f}    |   TOF:{3:.2f}".format(time.time(), 
                                                                                            (self.OF_TIME-oft), 
                                                                                            (self.IMU_TIME-imut),
                                                                                            (self.TOF_Time -toft)))
                
                self.data.append((CMDS['throttle'], CMDS['roll'], CMDS['pitch'], altitude, error_altitude, velocity, 
                                  error_roll, velocity_roll, angu_roll,
                                  error_pitch, velocity_pitch, angu_pitch))

            if self.DATA:
                np.save("control_t_auto", self.data)
                print (">>>>>>>>>>>>>>>>>>SAVED!")
                self.DATA = False
            
            '''Send out the CMDS values back to the joystick loop'''
            if value_available and (not ext_control_pipe_read.poll()):
                ext_control_pipe_write.send(CMDS)
                value_available = False
            time.sleep(self.PERIOD)
            prev_time = time.time()  