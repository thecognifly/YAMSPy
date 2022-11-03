import time
import datetime
import struct
import os

from yamspy import MSPy

# $ python -m yamspy.msp_proxy --ports 54310 54320 54330 54340 54350

serial_port = '/dev/ttyACM0'
# serial_port = 54340

FC_SEND_LOOP_TIME = 1/10

with MSPy(device=serial_port, loglevel='WARNING', baudrate=115200, use_tcp=False) as board:
# with MSPy(device=serial_port, loglevel='WARNING', baudrate=115200, use_tcp=True, min_time_between_writes=1/30) as board:
    command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO',
                    'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS',
                    'MSP_STATUS_EX','MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES']
    for msg in command_list:
        if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
            dataHandler = board.receive_msg()
            board.process_recv_data(dataHandler)
    try:       
        while True:
            os.system('clear')
            now = datetime.datetime.now()
            print(now)

            # Ask MSP2_NAV_DEBUG data
            if board.send_RAW_msg(MSPy.MSPCodes['MSP2_INAV_DEBUG'], data=[]):
                print("MSP2_INAV_DEBUG data sent!")
                dataHandler = board.receive_msg()
                print("MSP2_INAV_DEBUG ACK data received!")
                board.process_recv_data(dataHandler)
                print("MSP2_INAV_DEBUG data processed!")
            else:
                print("MSP2_INAV_DEBUG not sent!")

            print(board.SENSOR_DATA['debug'])

            try:
                for i,v in enumerate(['x', 'y', 'z']):
                    print(f"{v}={board.SENSOR_DATA['debug'][i]/1000.0}")
            
                for i,v in enumerate(['vx', 'vy', 'vz']):
                    print(f"{v}={board.SENSOR_DATA['debug'][i+3]/1000.0}")
                
                print(f"yaw={board.SENSOR_DATA['debug'][6]/10}degrees")

                print(f"navEPV={board.SENSOR_DATA['debug'][7] & 0xFFFF}")
                print(f"navEPH={(board.SENSOR_DATA['debug'][7] & 0xFFFF0000)>>16}")
            except TypeError as err:
                pass


            time.sleep(FC_SEND_LOOP_TIME)

    except KeyboardInterrupt:
        print("stop")
    finally:
        pass
        #board.reboot()