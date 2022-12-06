
import time
import struct

from yamspy import MSPy, msp_ctrl

# $ python -m yamspy.msp_proxy --ports 54310 54320 54330 54340
# serial_port = 54330
FC_SEND_LOOP_TIME = 1/100


msp2_range_format = '<Bi' # https://docs.python.org/3/library/struct.html#format-characters
range_template = {
             'quality': 0,      # uint8_t - [0;255] - INAV 5.1 ignores this value!
             'distanceMm': 0,   # int32_t - Negative value for out of range
}

if __name__ == '__main__':
    from argparse import ArgumentParser

    parser = ArgumentParser(description='Command line example.')
    parser.add_argument('--serialport', action='store', default="/dev/serial0", help='serial port')
    parser.add_argument('--use-tcp', action='store_true', help='TCP')
    arguments = parser.parse_args()
    serial_port = arguments.serialport
    use_tcp = arguments.use_tcp

    with MSPy(device=serial_port, loglevel='WARNING', baudrate=115200, use_tcp=use_tcp) as board:
        try:
            mspSensorRangefinderDataMessage = range_template.copy()
            mspSensorRangefinderDataMessage['quality'] = 200
            mspSensorRangefinderDataMessage['distanceMm'] = 50*1000
            while True:
                print(time.monotonic())
                range_data = struct.pack(msp2_range_format, *mspSensorRangefinderDataMessage.values())

                # Ask altitude data (maybe I should ask for MSP_SONAR_ALTITUDE as well)
                if board.send_RAW_msg(MSPy.MSPCodes['MSP_ALTITUDE'], data=[]):
                    print("MSP_ALTITUDE data sent!")
                    dataHandler = board.receive_msg()
                    print("MSP_ALTITUDE ACK data received!")
                    board.process_recv_data(dataHandler)
                    print("MSP_ALTITUDE data processed!")

                # Received altitude data
                print(board.SENSOR_DATA['altitude'])
                print(board.SENSOR_DATA['altitude_vel'])

                # Send Range data
                if board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_RANGEFINDER'], data=range_data):
                    print(f"MSP2_SENSOR_RANGEFINDER data {range_data} sent!")

                time.sleep(FC_SEND_LOOP_TIME)

        except KeyboardInterrupt:
            print("stop")
        finally:
            pass
            #board.reboot()
