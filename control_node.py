import os
import time

FREQ = 20
PERIOD = 1/FREQ
ABS_MAX_VALUE_ROLL = 50
ABS_MAX_VALUE_PITCH = 50
ABS_MAX_VALUE_THROTTLE = 50


Y_GAIN = 40
X_GAIN = 40
Z_GAIN = 75

def control_process(*args):

    control_optflow_pipe_read, control_cv_pipe_read, control_tof_pipe_read, control_imu_pipe_read, ext_control_pipe_write, ext_control_pipe_read, nice_level = args

    os.nice(nice_level)

    value_available = False
    altitude = None
    postition_hold = False
    prev_altitude = 0
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
            
        # This is reading the ToF output (around 30Hz)
        if control_tof_pipe_read.poll():
            altitude = control_tof_pipe_read.recv()

            if postition_hold:
                # Remember to reset integrator here too!
                prev_altitude = altitude
                postition_hold = False
                continue

            if prev_altitude:
                error = altitude - prev_altitude
                next_throttle = -Z_GAIN*error
                CMDS['throttle'] = next_throttle if abs(next_throttle) <= ABS_MAX_VALUE_THROTTLE else (-1 if next_throttle < 0 else 1)*ABS_MAX_VALUE_THROTTLE 
                value_available = True

        if control_imu_pipe_read.poll():
            _ = control_imu_pipe_read.recv()

        # This is reading the opticalflow output (around 10Hz)
        if control_optflow_pipe_read.poll():
            x_motion, y_motion = control_optflow_pipe_read.recv()

            next_roll = -Y_GAIN*y_motion
            CMDS['roll'] = next_roll if abs(next_roll) <= ABS_MAX_VALUE_ROLL else (-1 if next_roll < 0 else 1)*ABS_MAX_VALUE_ROLL 

            next_pitch = -X_GAIN*x_motion
            CMDS['pitch'] = next_pitch if abs(next_pitch) <= ABS_MAX_VALUE_PITCH else (-1 if next_pitch < 0 else 1)*ABS_MAX_VALUE_PITCH 

            value_available = True

        # This is just to check the speed... (around 2Hz)
        if control_cv_pipe_read.poll():
            x_motion, y_motion = control_cv_pipe_read.recv()

            if y_motion:
                next_roll = -Y_GAIN*y_motion
                CMDS['roll'] = next_roll if abs(next_roll) <= ABS_MAX_VALUE_ROLL else (-1 if next_roll < 0 else 1)*ABS_MAX_VALUE_ROLL 

            if y_motion:
                next_pitch = -X_GAIN*x_motion
                CMDS['pitch'] = next_pitch if abs(next_pitch) <= ABS_MAX_VALUE_PITCH else (-1 if next_pitch < 0 else 1)*ABS_MAX_VALUE_PITCH 

            value_available = True
            print("Received from camera...")


        if value_available and (not ext_control_pipe_read.poll()):
            ext_control_pipe_write.send(CMDS)
            value_available = False

        time.sleep(PERIOD)
