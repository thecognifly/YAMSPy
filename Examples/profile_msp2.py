from math import cos

from math import cos
import time
import datetime
import struct
import os

import cProfile
import pstats
from pstats import SortKey

from yamspy import MSPy

#
# On Linux, your serial port will probably be something like
# /dev/ttyACM0 or /dev/ttyS0 or the same names with numbers different from 0
#
# On Windows, I would expect it to be 
# COM1 or COM2 or COM3...
#
# This library uses pyserial, so if you have more questions try to check its docs:
# https://pyserial.readthedocs.io/en/latest/shortintro.html
#
#
serial_port = "/dev/ttyACM0"

profile = cProfile.Profile()


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

def foo():
    with MSPy(device=serial_port, loglevel='DEBUG', baudrate=115200) as board:
        prev = time.monotonic()
        for i in range(100):
            # Read info from the FC
            # Please, pay attention to the way it works:
            # 1. Message is sent without any payload (data=[])
            if board.send_RAW_msg(MSPy.MSPCodes['MSP2_INAV_STATUS'], data=[]):
                # 2. Response msg from the flight controller is received
                dataHandler = board.receive_msg()
                # 3. The msg is parsed
                # board.process_recv_data(dataHandler)
                # 4. After the parser, the instance is populated.
                # In this example, SENSOR_DATA has its altitude value updated.

            if board.send_RAW_msg(MSPy.MSPCodes['MSP2_INAV_ANALOG'], data=[]):
                # 2. Response msg from the flight controller is received
                dataHandler = board.receive_msg()
                # 3. The msg is parsed
                # board.process_recv_data(dataHandler)
                # 4. After the parser, the instance is populated.
                # In this example, SENSOR_DATA has its altitude value updated.

            now = datetime.datetime.now()
            mspSensorGpsDataMessage['year'] = now.year
            mspSensorGpsDataMessage['month'] = now.month
            mspSensorGpsDataMessage['day'] = now.day
            mspSensorGpsDataMessage['hour'] = now.hour
            mspSensorGpsDataMessage['min'] = now.minute
            mspSensorGpsDataMessage['sec'] = now.second
            gps_data = struct.pack(msp2_gps_format, *[int(i) for i in mspSensorGpsDataMessage.values()])
            # Send GPS data
            if not board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_GPS'], data=gps_data):
                print("MSP2_SENSOR_GPS not sent!")
        print(time.monotonic()-prev)
# profile.runcall(foo)
profile.run('foo()')
ps = pstats.Stats(profile)

ps.strip_dirs().sort_stats(SortKey.CUMULATIVE).print_stats()