import time
import signal
from collections import deque
import numpy as np
from multiprocessing import Process, Pipe
from betaflightmspy import MSPy
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
                    # init read imu
                    if board.send_RAW_msg(MSPy.MSPCodes['MSP_RAW_IMU'], data=[]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)
                
                    accelerometer = board.SENSOR_DATA['accelerometer']
                    gyroscope = board.SENSOR_DATA['gyroscope']
                
                    imudataReady = False
                    prev_time = time.time()

                    # init read BAT
                    for msg in command_list: 
                        if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                            dataHandler = board.receive_msg()
                            board.process_recv_data(dataHandler)
            
                    min_voltage = board.BATTERY_CONFIG['vbatmincellvoltage']*board.BATTERY_STATE['cellCount']
                    warn_voltage = board.BATTERY_CONFIG['vbatwarningcellvoltage']*board.BATTERY_STATE['cellCount']
                    max_voltage = board.BATTERY_CONFIG['vbatmaxcellvoltage']*board.BATTERY_STATE['cellCount']
                    voltage = board.ANALOG['voltage']
                    avg_voltage_deque = deque([voltage]*5)
                    batdataReady = False


                    while not shutdown:
                        # send RC
                        if fc_reboot:
                            shutdown = True
                            print('REBOOTING...')
                            break
                        
                        CMDS_RC = [CMDS[ki] for ki in CMDS_ORDER]

                        if board.send_RAW_RC(CMDS_RC):
                            dataHandler = board.receive_msg()
                            board.process_recv_data(dataHandler)

                        if DEBUG:
                            wfc.append(1/(time.time()-prev_time))
                            print ("Write to FC: %2.2f Hz"%(1/(time.time()-prev_time)))
                        prev_time = time.time()

                        # Read IMU
                        if board.send_RAW_msg(MSPy.MSPCodes['MSP_RAW_IMU'], data=[]):
                            dataHandler = board.receive_msg()
                            board.process_recv_data(dataHandler)
                            accelerometer = board.SENSOR_DATA['accelerometer']
                            gyroscope = board.SENSOR_DATA['gyroscope']  
                        
                        if DEBUG:
                            rfc.append(1/(time.time()-prev_time))
                            print ("Read IMU: %2.2f Hz"%(1/(time.time()-prev_time)))
                        else:
                            print ("IMU: ", imudataReady)
                        
                        prev_time = time.time()

                        # read bat
                        if board.send_RAW_msg(MSPy.MSPCodes['MSP_ANALOG'], data=[]):
                            dataHandler = board.receive_msg()
                            board.process_recv_data(dataHandler)
                            voltage = board.ANALOG['voltage']
                
                        avg_voltage_deque.appendleft(voltage)
                        avg_voltage_deque.pop()
                
                        mean_voltage = sum(avg_voltage_deque)/5
                        if DEBUG:
                            rfcbat.append(1/(time.time()-prev_time))
                            print ("Read BAT: %2.2f Hz"%(1/(time.time()-prev_time)))
                            # print (accelerometer, voltage)
                        else:
                            print("BAT", batdataReady)
                        prev_time = time.time()
                except KeyboardInterrupt:
                    shutdown = True
finally:
    np.save('write', wfc)
    np.save('read', rfc)
    np.save("read_bat",rfcbat)
    print ("Finish")
