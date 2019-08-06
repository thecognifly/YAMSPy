import os
import time

CMDS = {
        'roll':     1500,
        'pitch':    1500
}


ABS_MAX_VALUE_ROLL = 50
ABS_MAX_VALUE_PITCH = 50

Y_GAIN = 100
X_GAIN = 100
def control_process(control_optflow_pipe_in, control_cv_pipe_in,loop_pipe_out, nice_level):
    os.nice(nice_level)

    while True:
        new_cmd = False
        if control_optflow_pipe_in.poll():
            new_cmd = True
            x_motion, y_motion = control_optflow_pipe_in.recv()

            next_roll = -Y_GAIN*y_motion
            CMDS['roll'] = next_roll if next_roll <=  ABS_MAX_VALUE_ROLL else +ABS_MAX_VALUE_ROLL 
            CMDS['roll'] = next_roll if next_roll >= -ABS_MAX_VALUE_ROLL else -ABS_MAX_VALUE_ROLL 

            next_pitch = -X_GAIN*x_motion
            CMDS['pitch'] = next_pitch if next_pitch <= +ABS_MAX_VALUE_PITCH else +ABS_MAX_VALUE_PITCH 
            CMDS['pitch'] = next_pitch if next_pitch >= -ABS_MAX_VALUE_PITCH else -ABS_MAX_VALUE_PITCH 

        if new_cmd:
            loop_pipe_out.send(CMDS)

        time.sleep(0.5)
