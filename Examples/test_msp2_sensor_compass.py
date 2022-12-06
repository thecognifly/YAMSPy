
import time
import struct

from yamspy import MSPy, msp_ctrl

# $ python -m yamspy.msp_proxy --ports 54310 54320 54330 54340
# serial_port = 54360
FC_SEND_LOOP_TIME = 1/10


msp2_compass_format = '<BIhhh' # https://docs.python.org/3/library/struct.html#format-characters
compass_template = {
             'instance': 0,      # uint8_t
             'timeMs': 0,        # uint32_t - ignored by the FC
             'magX': 0,          # int16_t - mGauss
             'magY': 0,          # int16_t - mGauss
             'magZ': 0           # int16_t - mGauss
}


if __name__ == '__main__':
    from argparse import ArgumentParser

    parser = ArgumentParser(description='Command line example.')
    parser.add_argument('--serialport', action='store', default="/dev/serial0", help='serial port')
    parser.add_argument('--use-tcp', action='store_true', help='TCP')
    arguments = parser.parse_args()
    serial_port = arguments.serialport
    use_tcp = arguments.use_tcp

    with MSPy(device=serial_port, loglevel='WARNING', baudrate=115200, use_tcp=use_tcp, min_time_between_writes=1/30) as board:
        try:
            mspSensorCompassDataMessage = compass_template.copy()
            mspSensorCompassDataMessage['instance'] = 1

            for i in range(50):
                print("Initial messages ", time.monotonic())
                print("magX, magY, magZ: ", mspSensorCompassDataMessage['magX'], mspSensorCompassDataMessage['magY'], mspSensorCompassDataMessage['magZ'])
                compass_data = struct.pack(msp2_compass_format, *mspSensorCompassDataMessage.values())

                # Send Compass data
                if board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_COMPASS'], data=compass_data):
                    print(f"MSP2_SENSOR_COMPASS data {compass_data} sent!")

                time.sleep(FC_SEND_LOOP_TIME)


            mspSensorCompassDataMessage['magX'] = 0 #mGauss
            mspSensorCompassDataMessage['magY'] = 450 #mGauss
            while True:
                print(time.monotonic())
                print("magX, magY, magZ: ", mspSensorCompassDataMessage['magX'], mspSensorCompassDataMessage['magY'], mspSensorCompassDataMessage['magZ'])
                compass_data = struct.pack(msp2_compass_format, *mspSensorCompassDataMessage.values())


                # Send Compass data
                if board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_COMPASS'], data=compass_data):
                    print(f"MSP2_SENSOR_COMPASS data {compass_data} sent!")

                time.sleep(FC_SEND_LOOP_TIME)

        except KeyboardInterrupt:
            print("stop")
        finally:
            pass
            #board.reboot()
