import time
import signal
from collections import deque
import numpy as np
from multiprocessing import Process, Pipe
from yamspy import MSPy
import os

DEBUG = True
board = None

PRINT_VALUES_FREQ = 5
JOYSTICK_FREQ = 20
MAIN_FREQ = 50
READ_VOLT_FC_FREQ = 1
READ_IMU_FC_FREQ = 15

wfc = []
rfc = []
rfcbat = []

CMDS_init = {
        'roll':     1500,
        'pitch':    1500,
        'throttle': 900,
        'yaw':      1500,
        'aux1':     1000, # DISARMED (1000) / ARMED (1800)
        'aux2':     1000, # ANGLE (1000) / HORIZON (1500) / FLIP (1800)
        'aux3':     1000, # FAILSAFE (1800)
        'aux4':     1000  # HEADFREE (1800)
        }

CMDS = CMDS_init.copy()

command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES', 'MSP_ANALOG']

CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2', 'aux3', 'aux4']
shutdown = False
fc_reboot = False

try:
    while not shutdown:
        with MSPy(device="/dev/ttyACM0", loglevel='WARNING', baudrate=500000) as board:
            if board == 1: # an error occurred...
                print("Not connected to the FC...")              
                continue
            else:
                try:
                    prev_time = time.time()
                    while not shutdown:
                        CMDS_RC = [CMDS[ki] for ki in CMDS_ORDER]

                        #if board.send_RAW_RC(CMDS_RC):
                        #    dataHandler = board.receive_msg()
                        #    print(dataHandler)
                        #    #board.process_recv_data(dataHandler)
                        
                        board.fast_msp_rc_cmd(CMDS_RC)
                        board.fast_read_analog()
                        board.fast_read_attitude()
                        board.fast_read_imu()
                        #board.fast_cmd_and_read(CMDS_RC)
                        accelerometer = board.SENSOR_DATA['accelerometer']
                        gyroscope = board.SENSOR_DATA['gyroscope']
                        voltage = board.ANALOG['voltage']
                        attitude = board.SENSOR_DATA['kinematics']

                        print(accelerometer)
                        print(gyroscope)
                        print(attitude)
                        print(voltage)

                        #if board.send_RAW_msg(MSPy.MSPCodes['MSP_RAW_IMU'], data=[]):
                        #    dataHandler = board.receive_msg()
                        #    board.process_recv_data(dataHandler)
                        #    accelerometer = board.SENSOR_DATA['accelerometer']
                        #    gyroscope = board.SENSOR_DATA['gyroscope']  
                        
                        #if board.send_RAW_msg(MSPy.MSPCodes['MSP_ANALOG'], data=[]):
                        #    dataHandler = board.receive_msg()
                        #    board.process_recv_data(dataHandler)
                        #    voltage = board.ANALOG['voltage']
                
                        print ("Read speed: %2.2f Hz"%(1/(time.time()-prev_time)))
                        prev_time = time.time()

                except KeyboardInterrupt:
                    shutdown = True
finally:
    print("FINISHED")
