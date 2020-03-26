import os
import time
import numpy as np

from filterpy.memory import FadingMemoryFilter

from cognifly_utils import PID

NODE_STR = "[CONTROL NODE]"

PI_OVER_180 = 3.1415/180
INV_PI_OVER_180 = 1/PI_OVER_180

TIMEOUT_SENSORS = 0.005

# # Low-pass filter according to self.lowpass_cte
# dx = self.lowpass_cte * prev_dx + (1.0 - self.lowpass_cte) * dx
# prev_dx = dx
# dy = self.lowpass_cte * prev_dy + (1.0 - self.lowpass_cte) * dy
# prev_dy = dy

class Control():

    def __init__(self, control_optflow_pipe_read, 
                       control_tof_pipe_read, 
                       control_imu_pipe_read, 
                       ext_control_pipe_write, 
                       ext_control_pipe_read,
                       altitude_setpoint = 0.0,
                       pid_dict = None,
                       debug = False):

        self.control_optflow_pipe_read = control_optflow_pipe_read
        self.control_tof_pipe_read = control_tof_pipe_read
        self.control_imu_pipe_read = control_imu_pipe_read
        self.ext_control_pipe_write = ext_control_pipe_write
        self.ext_control_pipe_read = ext_control_pipe_read

        self.altitude_setpoint = altitude_setpoint 


        self.takeoff_threshold = 0.035 # in meters
        self.takeoff_gain = 1500.0

        self.x_setpoint = 0.0
        self.y_setpoint = 0.0

        self.DEBUG = debug

        #
        # Define all the parameters
        #

        # The variables below MUST be initialized with False
        self.tof_ready = False
        self.imu_ready = False
        self.optflow_ready = False


        # The value used to cancel gravity when the
        # battery is fully charged (12.60V)
        self.BASE_THROTTLE = 400 #580

        # Gravity cancelation
        # Betaflight has a vbat_pid_compensation!
        self.gravity_cancelation = self.BASE_THROTTLE


        # Basic parameters
        self.ABS_MAX_VALUE_ROLL = 10      # PID Roll limit (saturation)
        self.ABS_MAX_VALUE_PITCH = 10     # PID Pitch limit (saturation)
        self.ABS_MAX_VALUE_THROTTLE = 300 # PID Throttle limit (saturation)

        # ToF sensor offset
        self.TOFOFFSET_Z = 0.030 # [m]
        self.TOFOFFSET_X = 0.023 # [m]

        self.TOFOFFSET_R = np.sqrt(self.TOFOFFSET_Z**2 + self.TOFOFFSET_Z**2)
        self.TOFOFFSET_ANGLE = np.tan(self.TOFOFFSET_Z/self.TOFOFFSET_X)

        #
        # PID Gains
        #

        # PID Init
        # self.throttle_pid     = PID(pid_dict['kpz_p'], pid_dict['kiz_p'], pid_dict['kdz_p'], integral_threshold=1000) #throttle PID
        self.throttle_pid_vel = PID(pid_dict['kpz_v'], pid_dict['kiz_v'], pid_dict['kdz_v'], integral_threshold=100) #throttle PID
        self.throttle_pid_takeoff = PID(0.0, 1.0, 0.0, integral_threshold=300) #throttle PID for take off only
        self.roll_pid_vel     = PID(pid_dict['kpy_v'], pid_dict['kiy_v'], pid_dict['kdy_v']) #roll PID
        self.pitch_pid_vel    = PID(pid_dict['kpx_v'], pid_dict['kix_v'], pid_dict['kdx_v']) #pitch PID
        self.roll_pid_pos     = PID(pid_dict['kpy_p'], pid_dict['kiy_p'], pid_dict['kdy_p']) #roll PID
        self.pitch_pid_pos    = PID(pid_dict['kpx_p'], pid_dict['kix_p'], pid_dict['kdx_p']) #pitch PID


        self.CMDS_init = {}
        self.CMDS_init['throttle']   = 0.0 
        self.CMDS_init['roll']       = 0.0
        self.CMDS_init['pitch']      = 0.0
        self.CMDS_init['p_ctrl']     = [0.0, 0.0]
        self.CMDS_init['error_p']    = [0.0, 0.0, 0.0, 0.0] # error_x,error_y,error_z,error_h
        self.CMDS_init['error_v']    = [0.0, 0.0, 0.0, 0.0] # error_vx,error_vy,error_vz,error_vh
        self.CMDS_init['velocity']   = [0.0, 0.0, 0.0, 0.0] # vx,vy,vz,vh
        self.CMDS_init['time_stamp'] = 0.0


        self.next_throttle = 0.0
        
        self.tof_altitude = 0.0

        self.autonomous_mode = False
        self.prev_step_time = time.time() # only for the derivative

        print(NODE_STR + "Control Node Initialized!")


    def value_limit(self, output, limit):
        '''Set the value not exceed the limited value'''
        if abs(output) > limit:
            return limit*int(output/abs(output))
        else:
            return output 
    

    def truncate(self, data, dp = 2):
        ''' Truncate the value down to 2 dp as default'''
        return (int(data*(10**dp)))/(10**dp)


    def update_imu(self):
        # Update the IMU values (independently if autonomous mode is on or off)
        if self.control_imu_pipe_read.poll(2): # timeout=2 is for closing the thread at the end
                                                # ideally it should call recv directly, but it
                                                # will block the thread at the end
            self.imu, self.mean_batt_voltage, self.imu_time_stamp = self.control_imu_pipe_read.recv() 
            # imu => [[accX,accY,accZ], [gyroX,gyroY,gyroZ], [roll,pitch,yaw]]

            self.imu_ready = True
        else:
            print(NODE_STR + "Imu timeout?!?!?")

    def update_tof(self, dt=1/20, beta = 0.8):
        # Update the ToF values
        if self.control_tof_pipe_read.poll(2):
            RangingMeasurementData = self.control_tof_pipe_read.recv()

            if self.imu_ready: # the corrections depend on the imu
                altitude_sensor = RangingMeasurementData['RangeMilliMeter']/1000 #[m]
                # self.tof_time_stamp = RangingMeasurementData['time_stamp']
                self.tof_std = RangingMeasurementData['SigmaMilliMeter']
                
                if 3.0 > altitude_sensor > 0.0:  # outlier rejection...
                    #
                    # TODO: altitude compensation according to ROLL and PITCH angles.
                    #
                    # The sensor is not at the center axis of rotation (offset on pitch, but very small)
                    # ...ignore the offset to make calculations faster
                    self.tof_altitude = np.cos(self.imu[2][0]*PI_OVER_180)*np.cos(self.imu[2][1]*PI_OVER_180)*altitude_sensor

                if not self.tof_ready:
                    self.tof_ready = True

                    self.tof_vel = 0.0
                    self._filter_tof = FadingMemoryFilter([self.tof_altitude, 0.0, 0.0], dt, order=2, beta=beta)
                    self._filter_tof_v = FadingMemoryFilter([0.0, 0.0, 0.0], dt, order=2, beta=beta)

                    self.tof_time_stamp = time.time()
                    self.tof_prev_altitude = self.tof_altitude
                    self.tof_prev_time_stamp = self.tof_time_stamp
                    return

        if self.tof_ready:
            self._filter_tof.update(self.tof_altitude)
            self.tof_altitude = self._filter_tof.x[0]
            self.tof_time_stamp = time.time()
            self.tof_diff = (self.tof_altitude - self.tof_prev_altitude)
            self.tof_dt = (self.tof_time_stamp - self.tof_prev_time_stamp)
            self._filter_tof_v.update(self.tof_diff / self.tof_dt)
            self.tof_vel = self._filter_tof_v.x[0]

            self.tof_prev_altitude = self.tof_altitude
            self.tof_prev_time_stamp = self.tof_time_stamp



    def update_optflow(self, optflow_min_quality = 30, dt = 1/20, beta = 0.8, gyro_threshold = 100):
        # Update the Optical Flow values
        if self.control_optflow_pipe_read.poll(2):
            OptFlowMeasurementData = self.control_optflow_pipe_read.recv() # it will block until a brand new value comes.

            if self.tof_ready and self.imu_ready: # the corrections depend on the tof and imu
                dx = OptFlowMeasurementData['dx']
                dy = OptFlowMeasurementData['dy']
                self.optflow_quality = OptFlowMeasurementData['Quality']
                # self.optflow_time_stamp = OptFlowMeasurementData['time_stamp']

                # See equations 6.55 and 6.56 for the compensations on the optical flow:
                # Modelling and Control of the Crazyflie Quadrotor for Aggressive and
                # Autonomous Flight by Optical Flow Driven State Estimation 
                # http://lup.lub.lu.se/luur/download?func=downloadFile&recordOId=8905295&fileOId=8905299
                # and also see Crazyflie firmware optical flow: https://git.io/Je59e

                # the FC gives values in degrees/sec => so, convert to rad/s
                gyro_y = self.imu[1][0]*PI_OVER_180 # roll
                gyro_x = self.imu[1][1]*PI_OVER_180 # pitch

                altitude = self.tof_altitude


                #
                # DX and DY NEED TO BE ROTATED ACCORDING TO YAW ANGLE... 
                #
                #

                if not self.optflow_ready:
                    self.optflow_ready = True
                    # Optical Flow sensor (PMW3901) details
                    NofPixels = 30.0
                    Aperture = (4.2*PI_OVER_180) # to rad
                    self.ap_npix = Aperture/NofPixels

                    self.optflow_prev_vel_y = 0.0
                    self.optflow_prev_vel_x = 0.0
                    self.optflow_time_stamp = time.time()
                    self.optflow_prev_time_stamp = self.optflow_time_stamp
                    self._optflow_time_stamp = self.optflow_time_stamp
                    self._optflow_prev_time_stamp = self._optflow_time_stamp
                    self.optflow_dt = 0.0
                    self.optflow_vel_y = 0.0
                    self.optflow_vel_x = 0.0
                    self.optflow_x = 0.0
                    self.optflow_y = 0.0
                    self._filter_optflow_vx = FadingMemoryFilter([0.0, 0.0, 0.0], dt, order=2, beta=beta)
                    self._filter_optflow_vy = FadingMemoryFilter([0.0, 0.0, 0.0], dt, order=2, beta=beta)
                    return
                else:
                    self._optflow_time_stamp = time.time()
                    self._optflow_dt = (self._optflow_time_stamp - self._optflow_prev_time_stamp)
                    if self.optflow_quality > optflow_min_quality:
                        if abs(gyro_x) < gyro_threshold: # simple outlier rejection...
                            self.optflow_vel_x = altitude*(dx*self.ap_npix/self._optflow_dt + gyro_x)
                        if abs(gyro_y) < gyro_threshold: # simple outlier rejection...
                            self.optflow_vel_y = altitude*(dy*self.ap_npix/self._optflow_dt + gyro_y)
                    self._optflow_prev_time_stamp = self._optflow_time_stamp

        if self.optflow_ready:
            self.optflow_time_stamp = time.time()
            self.optflow_dt = (self.optflow_time_stamp - self.optflow_prev_time_stamp)
            self._filter_optflow_vx.update(self.optflow_vel_x)
            self._filter_optflow_vy.update(self.optflow_vel_y)
            self.optflow_vel_x = self._filter_optflow_vx.x[0]
            self.optflow_vel_y = self._filter_optflow_vy.x[0]
            self.optflow_x += self.optflow_dt*self.optflow_vel_x
            self.optflow_y += self.optflow_dt*self.optflow_vel_y

            self.optflow_prev_vel_x = self.optflow_vel_x
            self.optflow_prev_vel_y = self.optflow_vel_y
            self.optflow_prev_time_stamp = self.optflow_time_stamp


    def step(self, min_dt = 0.1):
        # Resets the last commands
        value_available = False

        # This order is important because it follows the dependencies.
        # There's no guarantee they will update the values, but the PID
        # controller will generate new control outputs after the first
        # time the values are available.
        self.update_imu()

        self.update_tof(beta = 0.8)
        self.update_optflow(beta = 0.8) # this filter will actuate on the ground velocity

        #
        # Here I need to process setpoints received from a pipe 
        # to allow an external process (running something like a CNN) to control the drone.
        # It will always update the last setpoints and if nothing is received the controller
        # will just keep seeking the last setpoints received or set by default.

        # Read the joystick trigger: auto mode or not
        if self.ext_control_pipe_write.poll():
            # cognifly_main will send True or False
            # When cognifly_main sends False, it means all future 
            # commands sent from the control_node will be ignored.
            self.autonomous_mode = self.ext_control_pipe_write.recv()
            print(NODE_STR + f"Received self.autonomous_mode value: {self.autonomous_mode} - [imu:{self.imu_ready}, tof:{self.tof_ready}, optflow:{self.optflow_ready}]")
            if self.autonomous_mode:
                self.CMDS = self.CMDS_init.copy()
                if self.tof_ready:
                    if (self.tof_altitude < 0.05):
                        self.throttle_pid_takeoff.integrate = True        
                        self.last_takeoff_altitude = 0.0
                else:
                    print(NODE_STR + "Tof was not available / ready!")
                    return 1 # this will force the system to shutdown

        # Everything depends on tof, so no tof no pid...
        if self.autonomous_mode and self.tof_ready:

            # Calculate next Throttle
            dt = time.time() - self.prev_step_time
            self.prev_step_time = time.time()

            error_z =  self.altitude_setpoint - self.tof_altitude

            # trying to keep the velocity below 0.5x the max velocity that gravity alone could stop
            # before reaching the error_z = 0 (that's the "/2.0")
            error_vz = np.sign(error_z)*np.sqrt(2*9.81*abs(error_z))/2.0 - self.tof_vel

            self.CMDS['error_p'][2] = error_z
            self.CMDS['error_v'][2] = error_vz
            self.CMDS['velocity'][2] = self.tof_vel

            if (self.tof_altitude > self.takeoff_threshold) and self.throttle_pid_takeoff.integrate:
                self.throttle_pid_takeoff.integrate = False
                self.gravity_cancelation = self.gravity_cancelation + self.value_limit(self.next_throttle, self.ABS_MAX_VALUE_THROTTLE)
                print(f"{NODE_STR} - TAKE OFF FINISHED >> {self.gravity_cancelation:.4f}, {self.tof_altitude:.4f}, {self.imu[0][2]:.4f}")

            if self.throttle_pid_takeoff.integrate:
                if self.tof_altitude >= self.last_takeoff_altitude:
                    # a high value of self.takeoff_gain will not be a problem IFF a controller takes over just after it
                    # otherwise it will probably generate a value that will accelerate upwards instead of only cancelling gravity...
                    self.next_throttle = self.throttle_pid_takeoff.calc(self.takeoff_gain*(self.takeoff_threshold-self.tof_altitude), dt=0.1)
                    self.last_takeoff_altitude = self.tof_altitude
            else:
                self.next_throttle = self.throttle_pid_vel.calc(error_vz, dt)
                print(f'{NODE_STR} - HOVERING >> P:{self.throttle_pid_vel.p_value:.4f}, I:{self.throttle_pid_vel.i_value:.4f}, D:{self.throttle_pid_vel.d_value:.4f}, dt:{dt:.4f}, t:{(self.prev_step_time):.4f}')
            
            print(f'{NODE_STR} - batt:{self.mean_batt_voltage:.4f}, alt:{self.tof_altitude:.4f},{self.tof_vel:.4f}, thr:{self.next_throttle:.4f}, imu:{self.imu[0][2]:.4f}')
            
            # Gravity cancelation
            # self.gravity_cancelation = self.gravity_cancelation / ((np.cos(self.imu[2][0]*INV_PI_OVER_180)) * (np.cos(self.imu[2][1]*INV_PI_OVER_180)))

            self.CMDS['throttle'] = self.value_limit(self.next_throttle, self.ABS_MAX_VALUE_THROTTLE) + self.gravity_cancelation

            # Calculate next Roll and Pitch
            if self.optflow_ready:      

                # Position control (outer loop)
                error_x =  (self.x_setpoint - self.optflow_x)
                self.CMDS['error_p'][0] = error_x
                # next_pitch = self.pitch_pid_pos.calc(error_x, dt)
                # self.CMDS['p_ctrl'][0] = self.value_limit(next_pitch, self.ABS_MAX_VALUE_PITCH)
                # self.CMDS['pitch'] = self.value_limit(next_pitch, self.ABS_MAX_VALUE_ROLL)

                error_y =  (self.y_setpoint - self.optflow_y)
                self.CMDS['error_p'][1] = error_y
                # next_roll = self.roll_pid_pos.calc(-error_y, dt)
                # self.CMDS['p_ctrl'][1] = self.value_limit(next_roll, self.ABS_MAX_VALUE_ROLL)
                # self.CMDS['roll'] = self.value_limit(next_roll, self.ABS_MAX_VALUE_ROLL)

                # # Velocity control (inner loop)
                error_vx = (0.0 - self.optflow_vel_x) #(self.CMDS['p_ctrl'][0] - self.optflow_vel_x)
                self.CMDS['error_v'][0] = error_vx
                self.CMDS['velocity'][0] = self.optflow_vel_x
                next_pitch = self.pitch_pid_vel.calc(error_vx, dt)
                self.CMDS['pitch'] = self.value_limit(next_pitch, self.ABS_MAX_VALUE_PITCH)

                print(f'{NODE_STR} - POS.X >> P:{self.pitch_pid_vel.p_value:.4f}, I:{self.pitch_pid_vel.i_value:.4f}, D:{self.pitch_pid_vel.d_value:.4f}, dt:{dt:.4f}, t:{(self.prev_step_time):.4f}')

                error_vy = (0.0 - self.optflow_vel_y) #(self.CMDS['p_ctrl'][1] - self.optflow_vel_y)
                self.CMDS['error_v'][1] = error_vy
                self.CMDS['velocity'][1] = self.optflow_vel_y
                next_roll = self.roll_pid_vel.calc(-error_vy, dt)
                self.CMDS['roll'] = self.value_limit(next_roll, self.ABS_MAX_VALUE_ROLL)

                print(f'{NODE_STR} - POS.Y >> P:{self.roll_pid_vel.p_value:.4f}, I:{self.roll_pid_vel.i_value:.4f}, D:{self.roll_pid_vel.d_value:.4f}, dt:{dt:.4f}, t:{(self.prev_step_time):.4f}')


                print(f'{NODE_STR} - x:{self.optflow_x}, y:{self.optflow_y}')

            self.CMDS['time_stamp'] = self.prev_step_time

            # TODO: Decide if it's better to reset tof_ready and optflow_ready to avoid PID calc using old values...
            value_available = True 


        # Send out the CMDS values back to the joystick loop
        if value_available:
            if not self.ext_control_pipe_read.poll(): # verify if the last value was consumed
                self.ext_control_pipe_write.send(self.CMDS)

        return 0