
import time
import struct

from yamspy import MSPy, msp_ctrl

serial_port = 54330
FC_SEND_LOOP_TIME = 1/20


msp2_range_format = '<Bi' # https://docs.python.org/3/library/struct.html#format-characters
range_template = {
             'quality': 0,      # uint8_t - [0;255] - INAV 5.1 ignores this value!
             'distanceMm': 0,   # int32_t - Negative value for out of range
}

with MSPy(device=serial_port, loglevel='WARNING', baudrate=115200, use_tcp=True) as board:
    command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO',
                    'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS',
                    'MSP_STATUS_EX','MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES']
    for msg in command_list:
        if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
            dataHandler = board.receive_msg()
            board.process_recv_data(dataHandler)
    try:
        # https://www.mide.com/air-pressure-at-altitude-calculator
        # Pressure: 100745.83 Pa
        # Temp: 25oC
        # Altitude: 50m

        mspSensorRangefinderDataMessage = range_template.copy()
        mspSensorRangefinderDataMessage['quality'] = 200
        mspSensorRangefinderDataMessage['distanceMm'] = 50*1000
        while True:
            print(time.monotonic())
            baro_data = struct.pack(msp2_range_format, *mspSensorRangefinderDataMessage.values())

            # Ask GPS data
            if board.send_RAW_msg(MSPy.MSPCodes['MSP_ALTITUDE'], data=[]):
                print("MSP_ALTITUDE data sent!")
                dataHandler = board.receive_msg()
                print("MSP_ALTITUDE ACK data received!")
                board.process_recv_data(dataHandler)
                print("MSP_ALTITUDE data processed!")

            # Received altitude data
            print(board.SENSOR_DATA['altitude'])
            print(board.SENSOR_DATA['altitude_vel'])

            # Send GPS data
            if board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_RANGEFINDER'], data=baro_data):
                print(f"MSP2_SENSOR_RANGEFINDER data {baro_data} sent!")

            time.sleep(FC_SEND_LOOP_TIME)

    except KeyboardInterrupt:
        print("stop")
    finally:
        pass
        #board.reboot()
