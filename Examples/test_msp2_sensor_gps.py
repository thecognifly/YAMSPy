from math import cos
import time
import datetime
import struct
import os

from yamspy import MSPy, msp_ctrl

# $ python -m yamspy.msp_proxy --ports 54310 54320 54330 54340
serial_port = 54330
FC_SEND_LOOP_TIME = 1/5

DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR = 1.113195
long_origin = -73.61319383049725 * 10000000
lat_origin = 45.50496682273918 * 10000000
gpsScaleLonDown = cos((abs(lat_origin) / 10000000) * 0.0174532925)

lat_displace = lambda x: x/DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR + lat_origin
long_displace = lambda y: y/(gpsScaleLonDown*DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR) + long_origin

msp2_gps_format = '<BHIBBHHHHiiiiiiHHHBBBBB' # https://docs.python.org/3/library/struct.html#format-characters
gps_template = {
             'instance': 0,                  # uint8 -  sensor instance number to support multi-sensor setups
             'gpsWeek':  0,                  # uint16 - GPS week, 0xFFFF if not available
             'msTOW': 0,                     # uint32
             'fixType': 0,                   # uint8
             'satellitesInView': 0,          # uint8
             'horizontalPosAccuracy': 0,      # uint16 - [cm]
             'verticalPosAccuracy': 0,        # uint16 - [cm]
             'horizontalVelAccuracy': 0,      # uint16 - [cm/s]
             'hdop': 0,                       # uint16
             'longitude': 0,                  # int32
             'latitude': 0,                   # int32
             'mslAltitude': 0,                # int32 - [cm]
             'nedVelNorth': 0,                # int32 - [cm/s]
             'nedVelEast': 0,                 # int32
             'nedVelDown': 0,                 # int32
             'groundCourse': 0,               # uint16 - deg * 100, 0..36000
             'trueYaw': 0,                    # uint16 - deg * 100, values of 0..36000 are valid. 65535 = no data available
             'year': 0,                       # uint16
             'month': 0,                      # uint8
             'day': 0,                        # uint8
             'hour': 0,                       # uint8
             'min': 0,                        # uint8
             'sec': 0,                        # uint8
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
            mspSensorGpsDataMessage = gps_template.copy()
            mspSensorGpsDataMessage['instance'] = 1
            mspSensorGpsDataMessage['fixType'] = 99
            mspSensorGpsDataMessage['satellitesInView'] = mspSensorGpsDataMessage['fixType']
            mspSensorGpsDataMessage['gpsWeek'] = 0xFFFF

            ############ SEND GPS DATA #############
            # gpsSol.llh.lon   = pkt->longitude;
            mspSensorGpsDataMessage['longitude'] = long_origin
            # gpsSol.llh.lat   = pkt->latitude;
            mspSensorGpsDataMessage['latitude'] = lat_origin
            # gpsSol.llh.alt   = pkt->mslAltitude;
            mspSensorGpsDataMessage['mslAltitude'] = 0 # [cm]
            # gpsSol.velNED[X] = pkt->nedVelNorth;
            mspSensorGpsDataMessage['nedVelNorth'] = 0
            # gpsSol.velNED[Y] = pkt->nedVelEast;
            mspSensorGpsDataMessage['nedVelEast'] = 0
            # gpsSol.velNED[Z] = pkt->nedVelDown;
            mspSensorGpsDataMessage['nedVelDown'] = 0
            # gpsSol.groundSpeed = calc_length_pythagorean_2D((float)pkt->nedVelNorth, (float)pkt->nedVelEast);
            # gpsSol.groundCourse = pkt->groundCourse / 10;   // in deg * 10
            mspSensorGpsDataMessage['groundCourse'] = 15000 # deg * 100, 0..36000
            mspSensorGpsDataMessage['trueYaw'] = 15000
            # gpsSol.eph = gpsConstrainEPE(pkt->horizontalPosAccuracy / 10);
            mspSensorGpsDataMessage['horizontalPosAccuracy'] = 10
            # gpsSol.epv = gpsConstrainEPE(pkt->verticalPosAccuracy / 10);
            mspSensorGpsDataMessage['verticalPosAccuracy'] = 10
            # gpsSol.hdop = gpsConstrainHDOP(pkt->hdop);
            mspSensorGpsDataMessage['hdop'] = 10

            count = 0
            while True:
                os.system('clear')
                now = datetime.datetime.now()
                print(now)
                mspSensorGpsDataMessage['year'] = now.year
                mspSensorGpsDataMessage['month'] = now.month
                mspSensorGpsDataMessage['day'] = now.day
                mspSensorGpsDataMessage['hour'] = now.hour
                mspSensorGpsDataMessage['min'] = now.minute
                mspSensorGpsDataMessage['sec'] = now.second
                gps_data = struct.pack(msp2_gps_format, *[int(i) for i in mspSensorGpsDataMessage.values()])

                # # Ask GPS data
                # if board.send_RAW_msg(MSPy.MSPCodes['MSP_RAW_GPS'], data=[]):
                #     print("MSP_RAW_GPS data sent!")
                #     dataHandler = board.receive_msg()
                #     print("MSP_RAW_GPS ACK data received!")
                #     board.process_recv_data(dataHandler)
                #     print("MSP_RAW_GPS data processed!")
                # else:
                #     print("MSP_RAW_GPS not sent!")

                # # Received GPS data
                # print(board.GPS_DATA)

                # Send GPS data
                if board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_GPS'], data=gps_data):
                    print(f"MSP2_SENSOR_GPS data {gps_data} sent!")
                else:
                    print("MSP2_SENSOR_GPS not sent!")

                MAX_COUNT = 50
                if count>MAX_COUNT:
                    mspSensorGpsDataMessage['latitude'] = lat_displace(666)
                    mspSensorGpsDataMessage['longitude'] = long_displace(999)
                    mspSensorGpsDataMessage['mslAltitude'] = 100 #cm => this is not working after isImuHeadingValid
                else:
                    print(f"Countdown: {MAX_COUNT-count}")
                count += 1
                time.sleep(FC_SEND_LOOP_TIME)

        except KeyboardInterrupt:
            print("stop")
        finally:
            pass
            #board.reboot()