import time
import datetime
import os

from yamspy import MSPy

# $ python -m yamspy.msp_proxy --ports 54310 54320 54330 54340 54350

# serial_port = '/dev/ttyACM0'
# serial_port = 54340

FC_SEND_LOOP_TIME = 1/10

EST_GPS_XY_VALID            = (1 << 0)
EST_GPS_Z_VALID             = (1 << 1)
EST_BARO_VALID              = (1 << 2)
EST_SURFACE_VALID           = (1 << 3)
EST_FLOW_VALID              = (1 << 4)
EST_XY_VALID                = (1 << 5)
EST_Z_VALID                 = (1 << 6)

if __name__ == '__main__':
    from argparse import ArgumentParser

    parser = ArgumentParser(description='Command line example.')
    parser.add_argument('--serialport', action='store', default="/dev/serial0", help='serial port')
    parser.add_argument('--use-tcp', action='store_true', help='TCP')
    arguments = parser.parse_args()
    serial_port = arguments.serialport
    use_tcp = arguments.use_tcp

    # with MSPy(device=serial_port, loglevel='WARNING', baudrate=115200, use_tcp=False) as board:
    with MSPy(device=serial_port, loglevel='WARNING', baudrate=115200, use_tcp=use_tcp, min_time_between_writes=1/30) as board:
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

                    print(f"navEPV={board.SENSOR_DATA['debug'][7] & 0x3FF}")
                    print(f"navEPH={(board.SENSOR_DATA['debug'][7] & 0xFFC00)>>10}")
                    flags = (board.SENSOR_DATA['debug'][7] & 0xFFF00000)>>20
                    print(f"EST_GPS_XY_VALID  = {bool(flags & EST_GPS_XY_VALID)}")
                    print(f"EST_GPS_Z_VALID   = {bool(flags & EST_GPS_Z_VALID)}")
                    print(f"EST_BARO_VALID    = {bool(flags & EST_BARO_VALID)}")
                    print(f"EST_SURFACE_VALID = {bool(flags & EST_SURFACE_VALID)}")
                    print(f"EST_FLOW_VALID    = {bool(flags & EST_FLOW_VALID)}")
                    print(f"EST_XY_VALID      = {bool(flags & EST_XY_VALID)}")
                    print(f"EST_Z_VALID       = {bool(flags & EST_Z_VALID)}")
                except TypeError as err:
                    pass


                time.sleep(FC_SEND_LOOP_TIME)

        except KeyboardInterrupt:
            print("stop")
        finally:
            pass
            #board.reboot()