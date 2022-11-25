
import time
import struct

from yamspy import MSPy, msp_ctrl

# $ python -m yamspy.msp_proxy --ports 54310 54320 54330 54340
serial_port = 54320
# serial_port = '/dev/ttyACM0'
FC_SEND_LOOP_TIME = 1/10


msp2_range_format = '<Bi' # https://docs.python.org/3/library/struct.html#format-characters
range_template = {
             'quality': 0,      # uint8_t - [0;255] - INAV 5.1 ignores this value!
             'distanceMm': 0,   # int32_t - Negative value for out of range
}

msp2_baro_format = '<BIfh' # https://docs.python.org/3/library/struct.html#format-characters
baro_template = {
             'instance': 0,      # uint8_t
             'timeMs': 0,        # uint32_t - ignored by the FC
             'pressurePa': 0.0,  # float
             'temp': 0           # int16_t - centi-degrees C
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
        mspSensorRangefinderDataMessage = range_template.copy()
        mspSensorRangefinderDataMessage['quality'] = 200
        mspSensorRangefinderDataMessage['distanceMm'] = 50

        # https://www.mide.com/air-pressure-at-altitude-calculator
        # 101325.00 @ 25oC = 0m
        # Allow the FC to calculate the zero????
        mspSensorBaroDataMessage = baro_template.copy()
        mspSensorBaroDataMessage['instance'] = 1
        mspSensorBaroDataMessage['pressurePa'] = 101208.95 # 10m above sea level
        mspSensorBaroDataMessage['temp'] = 25*100 # centi-degrees C
        
        baro_data = struct.pack(msp2_baro_format, *mspSensorBaroDataMessage.values())
        range_data = struct.pack(msp2_range_format, *mspSensorRangefinderDataMessage.values())

        # Send some messages to initialize / calibrate the barometer
        for i in range(50):
            print("Initial messages ", time.monotonic())

            # Send Baro data
            if board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_BAROMETER'], data=baro_data):
                print(f"MSP2_SENSOR_BAROMETER data {baro_data} sent!")

            # Send rangefinder data
            if board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_RANGEFINDER'], data=range_data):
                print(f"MSP2_SENSOR_RANGEFINDER data {range_data} sent!")

            time.sleep(FC_SEND_LOOP_TIME)

        while True:
            # Ask altitude data (maybe I should ask for MSP_SONAR_ALTITUDE as well)
            prev_time = time.monotonic()

            # Send Baro data
            if board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_BAROMETER'], data=baro_data):
                print(f"MSP2_SENSOR_BAROMETER data {baro_data} sent!")

            # Send rangefinder data
            if board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_RANGEFINDER'], data=range_data):
                print(f"MSP2_SENSOR_RANGEFINDER data {range_data} sent!")


            time.sleep(FC_SEND_LOOP_TIME)

    except KeyboardInterrupt:
        print("stop")
    finally:
        pass
        #board.reboot()
