
import time
import struct

from yamspy import MSPy, msp_ctrl

# $ python -m yamspy.msp_proxy --ports 54310 54320 54330 54340
# serial_port = 54340
FC_SEND_LOOP_TIME = 1/20


if __name__ == '__main__':
    from argparse import ArgumentParser

    parser = ArgumentParser(description='Command line example.')
    parser.add_argument('--serialport', action='store', default="/dev/serial0", help='serial port')
    parser.add_argument('--use-tcp', action='store_true', help='TCP')
    arguments = parser.parse_args()
    serial_port = arguments.serialport
    use_tcp = arguments.use_tcp

    msp2_baro_format = '<BIfh' # https://docs.python.org/3/library/struct.html#format-characters
    baro_template = {
                 'instance': 0,      # uint8_t
                 'timeMs': 0,        # uint32_t - ignored by the FC
                 'pressurePa': 0.0,  # float
                 'temp': 0           # int16_t - centi-degrees C
    }

    with MSPy(device=serial_port, loglevel='WARNING', baudrate=115200, use_tcp=use_tcp) as board:
        command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO',
                        'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS',
                        'MSP_STATUS_EX','MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES']
        for msg in command_list:
            if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                dataHandler = board.receive_msg()
                board.process_recv_data(dataHandler)
        try:
            # https://www.mide.com/air-pressure-at-altitude-calculator
            # 101325.00 @ 25oC = 0m
            # Allow the FC to calculate the zero????
            mspSensorBaroDataMessage = baro_template.copy()
            mspSensorBaroDataMessage['instance'] = 1
            mspSensorBaroDataMessage['pressurePa'] = 101325.00
            mspSensorBaroDataMessage['temp'] = 25*100 # centi-degrees C

            for i in range(1000):
                print("Initial messages ", time.monotonic())
                baro_data = struct.pack(msp2_baro_format, *mspSensorBaroDataMessage.values())

                # Ask altitude data
                if board.send_RAW_msg(MSPy.MSPCodes['MSP_ALTITUDE'], data=[]):
                    print("MSP_ALTITUDE data sent!")
                    dataHandler = board.receive_msg()
                    print("MSP_ALTITUDE ACK data received!")
                    board.process_recv_data(dataHandler)
                    print("MSP_ALTITUDE data processed!")

                # Received altitude data
                print(board.SENSOR_DATA['altitude'])
                print(board.SENSOR_DATA['altitude_vel'])

                # Send Baro data
                if board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_BAROMETER'], data=baro_data):
                    print(f"MSP2_SENSOR_BAROMETER data {baro_data} sent!")

                time.sleep(FC_SEND_LOOP_TIME)


            # https://www.mide.com/air-pressure-at-altitude-calculator
            # Pressure: 100745.83 Pa
            # Temp: 25oC
            # Altitude: 50m
            mspSensorBaroDataMessage = baro_template.copy()
            mspSensorBaroDataMessage['instance'] = 1
            mspSensorBaroDataMessage['pressurePa'] = 100745.83
            mspSensorBaroDataMessage['temp'] = 25*100 # centi-degrees C
            while True:
                print(time.monotonic())
                baro_data = struct.pack(msp2_baro_format, *mspSensorBaroDataMessage.values())

                # Ask altitude data
                if board.send_RAW_msg(MSPy.MSPCodes['MSP_ALTITUDE'], data=[]):
                    print("MSP_ALTITUDE data sent!")
                    dataHandler = board.receive_msg()
                    print("MSP_ALTITUDE ACK data received!")
                    board.process_recv_data(dataHandler)
                    print("MSP_ALTITUDE data processed!")

                # Received altitude data
                print(board.SENSOR_DATA['altitude'])
                print(board.SENSOR_DATA['altitude_vel'])

                # Send Baro data
                if board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_BAROMETER'], data=baro_data):
                    print(f"MSP2_SENSOR_BAROMETER data {baro_data} sent!")

                time.sleep(FC_SEND_LOOP_TIME)

        except KeyboardInterrupt:
            print("stop")
        finally:
            pass
            #board.reboot()
