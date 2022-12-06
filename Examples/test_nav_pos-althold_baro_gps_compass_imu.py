
import time
import struct
import datetime
from math import cos


from yamspy import MSPy, msp_ctrl

# $ python -m yamspy.msp_proxy --ports 54310 54320 54330 54340
serial_port = 54320
# serial_port = '/dev/ttyACM0'
FC_SEND_LOOP_TIME = 1/20


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

msp2_baro_format = '<BIfh' # https://docs.python.org/3/library/struct.html#format-characters
baro_template = {
             'instance': 0,      # uint8_t
             'timeMs': 0,        # uint32_t - ignored by the FC
             'pressurePa': 0.0,  # float
             'temp': 0           # int16_t - centi-degrees C
}

msp2_compass_format = '<BIhhh' # https://docs.python.org/3/library/struct.html#format-characters
compass_template = {
             'instance': 0,      # uint8_t
             'timeMs': 0,        # uint32_t - ignored by the FC
             'magX': 0,          # int16_t - mGauss
             'magY': 0,          # int16_t - mGauss
             'magZ': 0           # int16_t - mGauss
}


msp2_imu_format = '<hhhhhhBhhh' # https://docs.python.org/3/library/struct.html#format-characters

HITL_USE_IMU = (1 << 3) # ignore roll, pitch and yaw
imu_template = {
             'accX': 0,          # int16_t - (float*1000) G's
             'accY': 0,          # int16_t - (float*1000) G's
             'accZ': 0,          # int16_t - (float*1000) G's
             'gyroX': 0,         # int16_t - (float*1000) deg/s
             'gyroY': 0,         # int16_t - (float*1000) deg/s
             'gyroZ': 0,         # int16_t - (float*1000) deg/s
             'flags': 0,         # int8_t
             'roll':  0,         # int16_t - (float*1000) deg*10
             'pitch': 0,         # int16_t - (float*1000) deg*10
             'yaw':   0,         # int16_t - (float*1000) deg*10
}


def update_gps(now):
    mspSensorGpsDataMessage['year'] = now.year
    mspSensorGpsDataMessage['month'] = now.month
    mspSensorGpsDataMessage['day'] = now.day
    mspSensorGpsDataMessage['hour'] = now.hour
    mspSensorGpsDataMessage['min'] = now.minute
    mspSensorGpsDataMessage['sec'] = now.second
    gps_data = struct.pack(msp2_gps_format, *[int(i) for i in mspSensorGpsDataMessage.values()])

    return gps_data

with MSPy(device=serial_port, loglevel='WARNING', baudrate=115200, use_tcp=True, min_time_between_writes=1/50) as board:
    # It's necessary to send some messages or the RX failsafe will be activated
    # and it will not be possible to arm.
    
    try:
        #
        # GPS
        #
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

        #
        # Barometer
        #
        # https://www.mide.com/air-pressure-at-altitude-calculator
        # 101325.00 @ 25oC = 0m
        # Allow the FC to calculate the zero????
        mspSensorBaroDataMessage = baro_template.copy()
        mspSensorBaroDataMessage['instance'] = 1
        mspSensorBaroDataMessage['pressurePa'] = 101208.95 # 10m above sea level
        mspSensorBaroDataMessage['temp'] = 25*100 # centi-degrees C
        
        baro_data = struct.pack(msp2_baro_format, *mspSensorBaroDataMessage.values())

        mspSensorCompassDataMessage = compass_template.copy()
        mspSensorCompassDataMessage['instance'] = 1
        mspSensorCompassDataMessage['magX'] = int((-107E-6)*10E6) #mGauss
        mspSensorCompassDataMessage['magY'] = int((-46E-6)*10E6) #mGauss
        mspSensorCompassDataMessage['magZ'] = int((-284E-6)*10E6) #mGauss
        compass_data = struct.pack(msp2_compass_format, *mspSensorCompassDataMessage.values())

        mspSensorImuDataMessage = imu_template.copy()
        mspSensorImuDataMessage['accX'] = int(0)
        mspSensorImuDataMessage['accY'] = int(0)
        mspSensorImuDataMessage['accZ'] = int(1000)
        mspSensorImuDataMessage['gyroX'] = int(0)
        mspSensorImuDataMessage['gyroY'] = int(0)
        mspSensorImuDataMessage['gyroZ'] = int(0)
        mspSensorImuDataMessage['flags'] = HITL_USE_IMU
        imu_data = struct.pack(msp2_imu_format, *mspSensorImuDataMessage.values())

        # Send some messages to initialize / calibrate the barometer
        for i in range(50):
            print("Initial messages ", time.monotonic())
            
            # Send GPS
            now = datetime.datetime.now()
            gps_data = update_gps(now)
            if board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_GPS'], data=gps_data):
                print(f"MSP2_SENSOR_GPS data {gps_data} sent!")

            # Send Baro data
            if board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_BAROMETER'], data=baro_data):
                print(f"MSP2_SENSOR_BAROMETER data {baro_data} sent!")

            # Send Compass data
            if board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_COMPASS'], data=compass_data):
                print(f"MSP2_SENSOR_COMPASS data {compass_data} sent!")

            # Send IMU data
            if board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_IMU'], data=imu_data):
                print(f"MSP2_SENSOR_IMU data {imu_data} sent!")

            time.sleep(FC_SEND_LOOP_TIME)

        count = 0
        while True:
            # Ask altitude data (maybe I should ask for MSP_SONAR_ALTITUDE as well)
            prev_time = time.monotonic()

            if count == 3:
                # Send GPS
                now = datetime.datetime.now()
                gps_data = update_gps(now)
                if board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_GPS'], data=gps_data):
                    print(f"MSP2_SENSOR_GPS data {gps_data} sent!")
                count = 0

            # Send Baro data
            if board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_BAROMETER'], data=baro_data):
                print(f"MSP2_SENSOR_BAROMETER data {baro_data} sent!")

            # Send Compass data
            if board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_COMPASS'], data=compass_data):
                print(f"MSP2_SENSOR_COMPASS data {compass_data} sent!")

            # Send IMU data
            if board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_IMU'], data=imu_data):
                print(f"MSP2_SENSOR_IMU data {imu_data} sent!")

            count += 1
            time.sleep(FC_SEND_LOOP_TIME)

    except KeyboardInterrupt:
        print("stop")
    finally:
        pass
        #board.reboot()
