import asyncio
import time
import signal
from collections import deque
from multiprocessing import Process, Pipe
import os
import numpy as np

# pip install uvloop
import uvloop

# pip3 install git+https://github.com/gvalkov/python-evdev.git
import evdev
from evdev import InputDevice, ecodes, ff

from yamspy import MSPy


from camera import Camera

from tof_node import ToF

from control_node import control

"""
$ sudo find /. -name "evtest.py"
and run it to get a list of valid stuff available.
"""

PRINT_VALUES_FREQ = 1
JOYSTICK_FREQ = 20
MAIN_FREQ = 50
READ_VOLT_FC_FREQ = 1

DATA = []

# TIMEOUT = 1/MAIN_FREQ
# TIMEOUT_JOYSTICK = 1/JOYSTICK_FREQ # this is only useful to avoid locking at shutdown


# List of inputs that will be taken over when in autonomous mode
AUTONOMOUS_INPUT = ['roll', 'pitch','throttle'] #['roll', 'pitch', 'throttle']

# Using MSP controller it's possible to have more auxiliary inputs than this.
CMDS_init = {
        'roll':     1500,
        'pitch':    1500,
        'throttle': 1000,
        'yaw':      1500,
        'aux1':     1000, # DISARMED (1000) / ARMED (1800)
        'aux2':     1000, # ANGLE (1000) / HORIZON (1500) / FLIP (1800)
        'aux3':     1000, # FAILSAFE (1800)
        'aux4':     1000  # HEADFREE (1800)
        }

CMDS = CMDS_init.copy()

# I'm not sure if this order changes according to the configurations made to betaflight...
CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2', 'aux3', 'aux4']

try:
    gamepad = InputDevice('/dev/input/event2')
except FileNotFoundError:
    raise Exception('The PS4 Remote not connected')

shutdown = False

fc_reboot = False

board = None

mean_voltage = -1

min_voltage = -1

controller_battery = -1

frequencies_keys = ['send_cmds_to_fc', 
                    'joystick_interface', 
                    'autonomous', 
                    'read_voltage_from_fc', 
                    'print_values']

frequencies_measurement = {'joystick_interface':-1,
                           'send_cmds_to_fc':-1,
                           'autonomous':-1,
                           'read_voltage_from_fc':-1,
                           'print_values':-1}

# Setup for the vibration
rumble = ff.Rumble(strong_magnitude=0x0000, weak_magnitude=0xffff)
duration_ms = 1000

effect = ff.Effect(
    ecodes.FF_RUMBLE, -1, 0,
    ff.Trigger(0, 0),
    ff.Replay(duration_ms, 0),
    ff.EffectType(ff_rumble_effect=rumble)
)
effect_id = gamepad.upload_effect(effect)


async def joystick_interface(dev, ext_contr_pipe = None):
    """Use PS4 Bluetooth controller to control the drone

    Here it is expected that the flight controller (Betaflight) has the modes configured as:
    Aux1==1800 => ARM
    Aux1==1000 => DISARM

    Aux2==1000 => ANGLE MODE
    Aux2==1500 => HORIZON MODE
    Aux2==1800 => FLIP OVER AFTER CRASH MODE

    Aux3==1800 => FAILSAFE ON

    Aux4==1800 => HEADFREE ON

    And it's configured to use Receiver as MSP RX input 
    """
    global fc_reboot
    global DATA

    print("joystick_interface started...")

    reboot_event = [0,0]
    joystick_lost = False
    failsafe = False
    headfree = False
    autonomous = False
    prev_time = time.time()
    prev_time_ext = time.time()
    while not shutdown:
        try:
            if ext_contr_pipe: 
                # process info from external controller
                if ext_contr_pipe.poll():
                    cmds_pipe = ext_contr_pipe.recv()
                    if ('pitch' in AUTONOMOUS_INPUT):
                        # The received commands will always actuate around the center positions
                        CMDS['pitch'] = CMDS_init['pitch'] + cmds_pipe['pitch']
                    if ('roll' in AUTONOMOUS_INPUT):
                        # The received commands will always actuate around the center positions
                        CMDS['roll'] = CMDS_init['roll'] + cmds_pipe['roll']
                    if ('throttle' in AUTONOMOUS_INPUT):
                        if cmds_pipe['throttle'] >= 0.0: # it doesn't make sense negative values here
                            # The received commands will always actuate FROM the default min value
                            CMDS['throttle'] = CMDS_init['throttle'] + cmds_pipe['throttle']
            
                    frequencies_measurement['autonomous'] = time.time() - prev_time_ext
                    prev_time_ext = time.time()
            
            events = dev.read()
            # This seems to be the best option, but it raises BlockinIOError when I try to process
            # events and there's none

            # events = await asyncio.wait_for(dev.async_read(), timeout=TIMEOUT_JOYSTICK)
            # Using the wait_for gives this error asyncio.base_futures.InvalidStateError
            # and I was not able to catch it!

            # events = await dev.async_read()
            # Reading async_read directly, it hangs during shutdown if there no events and the other
            # tasks are closed.
            for event in events:
                # It uses a list because both buttons (L2 and R2) need to be pressed to reboot
                if reboot_event[0] == True and reboot_event[1] == True:
                    if not fc_reboot:
                        print('REBOOTING FC...')
                        fc_reboot = True

                if event.type == 3:
                    if event.code == 3:
                        if ('roll' not in AUTONOMOUS_INPUT) or not autonomous:
                            # LEFT / RIGHT: code 00 val from 0 (left) to 255 (right)
                            CMDS['roll'] = 1000+1000*event.value/255 # RIGHT left / right
                    elif event.code == 4:
                        if ('pitch' not in AUTONOMOUS_INPUT) or not autonomous:
                            # UP / DOWN: code 04 val from 255 (down) to 0 (up)
                            CMDS['pitch'] = 1000+1000*(255-event.value)/255 # RIGHT up / down
                    elif event.code == 1:
                        if ('throttle' not in AUTONOMOUS_INPUT) or not autonomous:
                            # UP / DOWN: code 01 val from 255 (down) to 0 (up)
                            CMDS['throttle'] = 1000+1000*(255-event.value)/255 # LEFT up / down
                    elif event.code == 0:
                        if ('yaw' not in AUTONOMOUS_INPUT) or not autonomous:
                            if not headfree:
                                # LEFT / RIGHT: code 00 val from 0 (left) to 255 (right)
                                CMDS['yaw'] = 1000+1000*event.value/255 # LEFT left / right
                    # Both, L2 and R2 need to be fully pressed to reboot
                    elif event.code == 2: # L2 - FULLY PRESSED => REBOOT
                        if event.value > 250:
                            reboot_event[0] = True
                        else:
                            reboot_event[0] = False
                    elif event.code == 5: # R2 - FULLY PRESSED => REBOOT
                        if event.value > 250:
                            reboot_event[1] = True
                        else:
                            reboot_event[1] = False

                if event.type == 1:
                    if event.value == 1:
                        if event.code == 311:
                            CMDS['aux1'] = 1000 # R1 - DISARM
                            autonomous = False
                            print('DISarming...')
                            print (">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>RUN TIME: ", (time.time() - start))
                            dev.write(ecodes.EV_FF, effect_id, 2)
                        if event.code == 310:
                            if CMDS['throttle']<=1000:
                                CMDS['aux1'] = 1800 # L1 - ARM
                                print('ARMing...')
                                start = time.time()
                                dev.write(ecodes.EV_FF, effect_id, 1)
                            else:
                                dev.write(ecodes.EV_FF, effect_id, 5)
                                print('Throttle too high!!!')
                        if event.code == 307:
                            CMDS['aux2'] = 1000 # TRIANGLE - ANGLE MODE
                            print('ANGLE MODE...')
                            dev.write(ecodes.EV_FF, effect_id, 1)
                        if event.code == 304:
                            CMDS['aux2'] = 1500 # CROSS - HORIZON MODE
                            print('HORIZON MODE...')
                            dev.write(ecodes.EV_FF, effect_id, 1)
                        if event.code == 308: # SQUARE - HEADFREE
                            # HEADFREE will turn off yaw reading from joystick
                            if not headfree:
                                # CMDS['aux4'] = 1800 # HEADFREE ON
                                print('HEADFREE ON...')
                                headfree = True
                                CMDS['yaw'] = CMDS_init['yaw']
                                dev.write(ecodes.EV_FF, effect_id, 1)
                            else:
                                # CMDS['aux4'] = 1000 # HEADFREE OFF
                                print('HEADFREE OFF...')
                                headfree = False
                                dev.write(ecodes.EV_FF, effect_id, 2)
                        if event.code == 314:
                            CMDS['aux2'] = 1800 # share button - FLIP OVER AFTER CRASH
                            print('FLIP OVER AFTER CRASH MODE...')
                        if event.code == 315:
                            if not failsafe:
                                CMDS['aux3'] = 1800 # options button - FAILSAFE
                                print('FAILSAFE ON...')
                                failsafe = True
                                dev.write(ecodes.EV_FF, effect_id, 1)
                            else:
                                CMDS['aux3'] = 1000 # options button - FAILSAFE
                                print('FAILSAFE OFF...')
                                failsafe = False
                                dev.write(ecodes.EV_FF, effect_id, 2)
                        if event.code == 305: # CIRCLE - AUTONOMOUS MODE
                            if not autonomous:
                                autonomous = True
                                if ext_contr_pipe:
                                    # Indicates to the external controller it needs to save the current
                                    # altitude
                                    ext_contr_pipe.send(True)
                                print('>>>>>>>>>>>>>AUTONOMOUS MODE...')
                                dev.write(ecodes.EV_FF, effect_id, 5) # vibrate for longer here
                            else:
                                autonomous = False
                                if ext_contr_pipe:
                                    # Indicates to the external controller it needs to reset the current
                                    # altitude
                                    ext_contr_pipe.send(False)
                                CMDS['roll'] = CMDS_init['roll']
                                CMDS['pitch'] = CMDS_init['pitch']
                                CMDS['throttle'] = CMDS_init['throttle']
                                CMDS['yaw'] = CMDS_init['yaw']
                                print('>>>>>>>>>>>>>MANUAL MODE...')
                                dev.write(ecodes.EV_FF, effect_id, 1)

            DATA.append((CMDS['throttle'], CMDS['roll'], CMDS['pitch'], time.time()))
        # except asyncio.TimeoutError:
            # continue
        except BlockingIOError:
            pass
        except OSError:
            print("Joystick lost...")
            joystick_lost = True
            break
        except:
            print("Something else broke joystick_interface...")
            joystick_lost = True
            break

        frequencies_measurement['joystick_interface'] = time.time() - prev_time
        prev_time = time.time()
        await asyncio.sleep(1/JOYSTICK_FREQ)

    print("joystick_interface closing...")
    if joystick_lost:
        fc_reboot = True
        CMDS['roll'] = CMDS_init['roll']
        CMDS['pitch'] = CMDS_init['pitch']
        CMDS['throttle'] = CMDS_init['throttle']
        CMDS['yaw'] = CMDS_init['yaw']
        dev.write(ecodes.EV_FF, effect_id, 5)


async def print_values():
    print("print_values started...")

    prev_time = time.time()
    while not shutdown:
        print("Frequencies:")
        print(["{} - {:.2f}Hz".format(keys, 1/frequencies_measurement[keys]) for keys in frequencies_keys])
        if board:
            print("Voltage / Accelerometer / Attitude:")
            values = [mean_voltage, min_voltage] + board.SENSOR_DATA['accelerometer'] + board.SENSOR_DATA['kinematics']
            print("{:.2f}V ({:.2f}V) - AccX:{:.2f}, AccY:{:.2f}, AccZ:{:.2f} - R:{:.2f}, P:{:.2f}, Y:{:.2f}".format(*values))
        print("Commands:")
        print(["{} - {:.2f}".format(cmd, CMDS[cmd]) for cmd in CMDS_ORDER])

        frequencies_measurement['print_values'] = time.time() - prev_time
        prev_time = time.time()
        await asyncio.sleep(1/PRINT_VALUES_FREQ)

    print("print_values closing...")


async def read_voltage_from_fc(dev):
    print("read_voltage_from_fc started...")

    global mean_voltage, min_voltage, controller_battery

    command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
                    'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
                    'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES', 'MSP_ANALOG']
    while not board:
        await asyncio.sleep(1/READ_VOLT_FC_FREQ)

    if board:
        for msg in command_list: 
            if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                dataHandler = board.receive_msg()
                board.process_recv_data(dataHandler)

        min_voltage = board.BATTERY_CONFIG['vbatmincellvoltage']*board.BATTERY_STATE['cellCount']
        warn_voltage = board.BATTERY_CONFIG['vbatwarningcellvoltage']*board.BATTERY_STATE['cellCount']
        max_voltage = board.BATTERY_CONFIG['vbatmaxcellvoltage']*board.BATTERY_STATE['cellCount']
        voltage = board.ANALOG['voltage']
        avg_voltage_deque = deque([voltage]*5)

    # dataReady = False
    prev_time = time.time()
    while not shutdown:
        # if not dataReady: #make sure the data received is processed only in the next loop
        #     if board.send_RAW_msg(MSPy.MSPCodes['MSP_ANALOG'], data=[]):
        #         dataHandler = board.receive_msg()
        #         dataReady = True
        # else:
        #     board.process_recv_data(dataHandler)
        #     voltage = board.ANALOG['voltage']
        #     dataReady = False

        with open("/sys/class/power_supply/sony_controller_battery_90:89:5f:3f:d7:a6/capacity", 'r') as controller:
            controller_battery = int(controller.read())

        board.fast_read_analog()
        voltage = board.ANALOG['voltage']

        avg_voltage_deque.appendleft(voltage)
        avg_voltage_deque.pop()

        mean_voltage = sum(avg_voltage_deque)/5
        if mean_voltage <= min_voltage:
            dev.write(ecodes.EV_FF, effect_id, 5)
        elif mean_voltage >= max_voltage:
            dev.write(ecodes.EV_FF, effect_id, 1)
        elif mean_voltage <= warn_voltage:
            dev.write(ecodes.EV_FF, effect_id, 1)

        frequencies_measurement['read_voltage_from_fc'] = time.time() - prev_time
        prev_time = time.time()
        await asyncio.sleep(1/READ_VOLT_FC_FREQ)

    print("read_voltage_from_fc closing...")


async def send_cmds_to_fc(pipes):
    global fc_reboot
    global shutdown
    global board
    global mean_voltage
    global controller_battery
    print("send_cmds_to_fc started...")

    pipe_write, pipe_read = pipes 

    prev_time = time.time()
    while not shutdown:
        print("Connecting to the FC...")
        with MSPy(device="/dev/ttyACM0", loglevel='WARNING', baudrate=115200) as board:
            if board == 1: # an error occurred...
                print("Connecting to the FC... FAILED!")
                await asyncio.sleep(1)
                continue
            else:
                print("Connecting to the FC... OK!")
                try:
                    while not shutdown:
                        if fc_reboot:
                            shutdown = True
                            print('REBOOTING...')
                            break
                        if controller_battery < 20:
                            print (">>>>>>>>>>>>>>>>>>>>>>>>>>>>>PS4 Controller capacity too low !")
                            # shutdown = True
                            # break

                        CMDS_RC = [CMDS[ki] for ki in CMDS_ORDER]

                        # if board.send_RAW_RC(CMDS_RC):
                        #     dataHandler = board.receive_msg()
                        #     board.process_recv_data(dataHandler)

                        board.fast_msp_rc_cmd(CMDS_RC)
                        board.fast_read_imu()
                        board.fast_read_attitude()

                        if not pipe_read.poll():
                            # accelerometer = board.SENSOR_DATA['accelerometer']
                            # gyroscope = board.SENSOR_DATA['gyroscope']
                            # attitude = board.SENSOR_DATA['kinematics']
                            pipe_write.send(((board.SENSOR_DATA['accelerometer'],
                                             board.SENSOR_DATA['gyroscope'],
                                             board.SENSOR_DATA['kinematics']),
                                             mean_voltage,
                                             time.time()))

                        frequencies_measurement['send_cmds_to_fc'] = time.time() - prev_time
                        prev_time = time.time()
                        await asyncio.sleep(1/MAIN_FREQ)
                finally:
                    shutdown = True
                    board.reboot()

    print("send_cmds_to_fc closing...")


async def ask_exit(signame, loop):
    global shutdown

    print("Got signal %s: exit" % signame)
    shutdown = True
    await asyncio.sleep(0)

def run_loop(pipes):
    joystick_pipe, imu_pipes = pipes
    # ioloop = asyncio.new_event_loop()
    ioloop = uvloop.new_event_loop() # should be faster...
    tasks = [
        joystick_interface(gamepad, joystick_pipe),
        read_voltage_from_fc(gamepad),
        print_values(),
        send_cmds_to_fc(imu_pipes)
    ]

    for sig in ['SIGHUP','SIGINT', 'SIGTERM']:
        ioloop.add_signal_handler(
            getattr(signal, sig),
            lambda s=sig: ioloop.create_task(ask_exit(s, ioloop)))

    ioloop.run_until_complete(asyncio.wait(tasks))
    ioloop.close()

if __name__ == '__main__':

    os.nice(-10) # this process has the highest priority

    print("Creating Pipes...")
    ext_control_pipe_write, ext_control_pipe_read = Pipe()
    control_optflow_pipe_write, control_optflow_pipe_read = Pipe()
    control_cv_pipe_write, control_cv_pipe_read = Pipe()
    control_tof_pipe_write, control_tof_pipe_read = Pipe()
    control_imu_pipe_write, control_imu_pipe_read = Pipe()
    print("Creating Pipes... done!")
    
    #
    # Launch all processes
    #
    # - Control_node(opticalflow) => Main_node
    # - OpticalFlow_node(camera) => Control_node

    print("Creating Processes...")
    # nice_level > -10
    # control_pipe_in: receive velocities from opticalflow
    # ext_control_pipe_read: write throttle, yaw, roll and pitch to main node.
    control = control()
    nice_level_control = 10
    control_process = Process(target=control.control_process, 
                              args=(control_optflow_pipe_read, 
                                    control_cv_pipe_read,
                                    control_tof_pipe_read,
                                    control_imu_pipe_read,
                                    ext_control_pipe_write,
                                    ext_control_pipe_read, 
                                    nice_level_control))
    
    # nice_level > nice_level_control_node
    # control_pipe_out: write velocities to control node
    # opticalflow_process = Process(target=opticalflow_node, args=(control_pipe_out, nice_level))
    camera = Camera((control_optflow_pipe_write, control_optflow_pipe_read),
                    (control_cv_pipe_write, control_cv_pipe_read),
                    frameWidth=720,
                    frameHeight=720,
                    frameRate=30,
                    DEBUG=False)
    nice_level_cam = 10 
    camera_process = Process(target=camera.run, args=(nice_level_cam,))
    
    # nice_level > nice_level_control_node
    # control_pipe_out: write altitude to control node
    ToF = ToF(control_tof_pipe_write, control_tof_pipe_read)
    nice_level_ToF = 10
    ToF_process = Process(target=ToF.run, args=(nice_level_ToF,))
    print("Creating Processes... done!")

    print("Launching Processes...")
    control_process.start()
    camera_process.start()
    ToF_process.start()
    print("Launching Processes... done!")
    
    try:
        run_loop((ext_control_pipe_read, (control_imu_pipe_write, control_imu_pipe_read)))
    
    finally:
        np.save("joy", DATA)
        print("Killing Processes...")
        control_process.terminate()
        time.sleep(3)
        camera_process.terminate()
        ToF_process.terminate()
        print("Killing Processes... done!")
