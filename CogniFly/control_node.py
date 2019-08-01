import os
import time
import pid
# control_optflow_pipe_in: receive velocities from opticalflow (x,y) # in unit
# control_cv_pipe_in     : receive displacement from opencv    ([x,y], (area_different)) # in unit
# control_tof_pipe_in    : receive altitude from the ToF       (h) # in mm

CMDS = {
        'throttle': 1000,
        'roll':     1500,
        'pitch':    1500
}


FREQ = 20
PERIOD = 1/FREQ
ABS_MAX_VALUE_ROLL = 50
ABS_MAX_VALUE_PITCH = 50

altitude = None

Y_GAIN = 40
X_GAIN = 40
def control_process(control_optflow_pipe_in, control_cv_pipe_in, 
                    control_ToF_pipe_in, loop_pipe_out, nice_level):

    os.nice(nice_level)
    while True:

        # # This is just to check the speed... (around 2Hz)
        # if control_cv_pipe_in.poll():
        #     x_motion, y_motion = control_cv_pipe_in.recv()

        #     if y_motion:
        #         next_roll = -Y_GAIN*y_motion
        #         CMDS['roll'] = next_roll if abs(next_roll) <= ABS_MAX_VALUE_ROLL else (-1 if next_roll < 0 else 1)*ABS_MAX_VALUE_ROLL 

        #     if y_motion:
        #         next_pitch = -X_GAIN*x_motion
        #         CMDS['pitch'] = next_pitch if abs(next_pitch) <= ABS_MAX_VALUE_PITCH else (-1 if next_pitch < 0 else 1)*ABS_MAX_VALUE_PITCH 

        #     loop_pipe_out.send(CMDS)

        # This is reading the opticalflow output (around 10Hz)
        if control_ToF_pipe_in.poll():
            global altitude
            altitude = control_ToF_pipe_in.recv()


        if control_optflow_pipe_in.poll():
            x_motion, y_motion = control_optflow_pipe_in.recv()
            # the optical flow data multiple altitude will become ground truth value

            next_roll = -Y_GAIN*(y_motion*altitude)
            CMDS['roll'] = next_roll if abs(next_roll) <= ABS_MAX_VALUE_ROLL else (-1 if next_roll < 0 else 1)*ABS_MAX_VALUE_ROLL 

            next_pitch = -X_GAIN*(x_motion*altitude)
            CMDS['pitch'] = next_pitch if abs(next_pitch) <= ABS_MAX_VALUE_PITCH else (-1 if next_pitch < 0 else 1)*ABS_MAX_VALUE_PITCH 

            loop_pipe_out.send(CMDS)

        time.sleep(PERIOD)
