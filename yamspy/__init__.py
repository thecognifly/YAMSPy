"""YAMSPy: Yet Another Implementation of Multiwii Serial Protocol Python Interface for Betaflight, iNAV, etc.

Copyright (C) 2019 Ricardo de Azambuja

This file is part of YAMSPy.

YAMSPy is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

YAMSPy is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with YAMSPy.  If not, see <https://www.gnu.org/licenses/>.

Deeply based on code from Betaflight and iNAV.

Acknowledgement:
This work was possible thanks to the financial support from IVADO.ca (postdoctoral scholarship 2019/2020).

Disclaimer (adapted from Wikipedia):
None of the authors, contributors, supervisors, administrators, employers, friends, family, vandals, or anyone else 
connected (or not) with this project, in any way whatsoever, can be made responsible for your use of the information (code) 
contained or linked from here.

TODO:
1) Add more support to iNAV.
2) Add the possibility to register a callback functions when a msg is transmitted.
3) Improve the arming confirmation.
4) This file is way too big... it needs to be broken down into smaller ones.
"""

__author__ = "Ricardo de Azambuja"
__copyright__ = "Copyright 2019, MISTLab.ca"
__credits__ = [""]
__license__ = "GPL"
__version__ = "0.3.3"
__maintainer__ = "Ricardo de Azambuja"
__email__ = "ricardo.azambuja@gmail.com"
__status__ = "Development"

import logging
import struct
import time
import sys
from threading import Lock

if "linux" in sys.platform:
    import ctypes
    ffs = ctypes.cdll.LoadLibrary('libc.so.6').ffs # this is only for ffs... it should be directly implemented.
else:
    def ffs(x): # modified from https://stackoverflow.com/a/36059264
        return (x&-x).bit_length()

import serial # pyserial version???

class MSPy:

    # Dictionary with all the possible codes
    # Based on betaflight-configurator and inav-configurator 
    # Latest versions can be found:
    #  https://github.com/betaflight/betaflight-configurator/blob/master/src/js/msp/MSPCodes.js
    #  https://github.com/iNavFlight/inav-configurator/blob/master/js/msp/MSPCodes.js

    MSPCodes = {
        'MSP_API_VERSION':                1,
        'MSP_FC_VARIANT':                 2,
        'MSP_FC_VERSION':                 3,
        'MSP_BOARD_INFO':                 4,
        'MSP_BUILD_INFO':                 5,

        'MSP_NAME':                       10,
        'MSP_SET_NAME':                   11,

        'MSP_BATTERY_CONFIG':             32,
        'MSP_SET_BATTERY_CONFIG':         33,
        'MSP_MODE_RANGES':                34,
        'MSP_SET_MODE_RANGE':             35,
        'MSP_FEATURE_CONFIG':             36,
        'MSP_SET_FEATURE_CONFIG':         37,
        'MSP_BOARD_ALIGNMENT_CONFIG':     38,
        'MSP_SET_BOARD_ALIGNMENT_CONFIG': 39,
        'MSP_CURRENT_METER_CONFIG':       40,
        'MSP_SET_CURRENT_METER_CONFIG':   41,
        'MSP_MIXER_CONFIG':               42,
        'MSP_SET_MIXER_CONFIG':           43,
        'MSP_RX_CONFIG':                  44,
        'MSP_SET_RX_CONFIG':              45,
        'MSP_LED_COLORS':                 46,
        'MSP_SET_LED_COLORS':             47,
        'MSP_LED_STRIP_CONFIG':           48,
        'MSP_SET_LED_STRIP_CONFIG':       49,
        'MSP_RSSI_CONFIG':                50,
        'MSP_SET_RSSI_CONFIG':            51,
        'MSP_ADJUSTMENT_RANGES':          52,
        'MSP_SET_ADJUSTMENT_RANGE':       53,
        'MSP_CF_SERIAL_CONFIG':           54,
        'MSP_SET_CF_SERIAL_CONFIG':       55,
        'MSP_VOLTAGE_METER_CONFIG':       56,
        'MSP_SET_VOLTAGE_METER_CONFIG':   57,
        'MSP_SONAR':                      58,
        'MSP_PID_CONTROLLER':             59,
        'MSP_SET_PID_CONTROLLER':         60,
        'MSP_ARMING_CONFIG':              61,
        'MSP_SET_ARMING_CONFIG':          62,
        'MSP_RX_MAP':                     64,
        'MSP_SET_RX_MAP':                 65,
        #'MSP_BF_CONFIG':                  66, # DEPRECATED
        #'MSP_SET_BF_CONFIG':              67, # DEPRECATED
        'MSP_SET_REBOOT':                 68,
        #'MSP_BF_BUILD_INFO':              69, # Not used
        'MSP_DATAFLASH_SUMMARY':          70,
        'MSP_DATAFLASH_READ':             71,
        'MSP_DATAFLASH_ERASE':            72,
        'MSP_LOOP_TIME':                  73,
        'MSP_SET_LOOP_TIME':              74,
        'MSP_FAILSAFE_CONFIG':            75,
        'MSP_SET_FAILSAFE_CONFIG':        76,
        'MSP_RXFAIL_CONFIG':              77,
        'MSP_SET_RXFAIL_CONFIG':          78,
        'MSP_SDCARD_SUMMARY':             79,
        'MSP_BLACKBOX_CONFIG':            80,
        'MSP_SET_BLACKBOX_CONFIG':        81,
        'MSP_TRANSPONDER_CONFIG':         82,
        'MSP_SET_TRANSPONDER_CONFIG':     83,
        'MSP_OSD_CONFIG':                 84,
        'MSP_SET_OSD_CONFIG':             85,
        'MSP_OSD_CHAR_READ':              86,
        'MSP_OSD_CHAR_WRITE':             87,
        'MSP_VTX_CONFIG':                 88,
        'MSP_SET_VTX_CONFIG':             89,
        'MSP_ADVANCED_CONFIG':            90,
        'MSP_SET_ADVANCED_CONFIG':        91,
        'MSP_FILTER_CONFIG':              92,
        'MSP_SET_FILTER_CONFIG':          93,
        'MSP_PID_ADVANCED':               94,
        'MSP_SET_PID_ADVANCED':           95,
        'MSP_SENSOR_CONFIG':              96,
        'MSP_SET_SENSOR_CONFIG':          97,
        #'MSP_SPECIAL_PARAMETERS':         98, // DEPRECATED
        'MSP_ARMING_DISABLE':             99,
        #'MSP_SET_SPECIAL_PARAMETERS':     99, // DEPRECATED
        #'MSP_IDENT':                      100, // DEPRECTED
        'MSP_STATUS':                     101,
        'MSP_RAW_IMU':                    102,
        'MSP_SERVO':                      103,
        'MSP_MOTOR':                      104,
        'MSP_RC':                         105,
        'MSP_RAW_GPS':                    106,
        'MSP_COMP_GPS':                   107,
        'MSP_ATTITUDE':                   108,
        'MSP_ALTITUDE':                   109,
        'MSP_ANALOG':                     110,
        'MSP_RC_TUNING':                  111,
        'MSP_PID':                        112,
        #'MSP_BOX':                        113, // DEPRECATED 
        'MSP_MISC':                       114, # DEPRECATED
        'MSP_BOXNAMES':                   116,
        'MSP_PIDNAMES':                   117,
        'MSP_WP':                         118, # Not used
        'MSP_BOXIDS':                     119,
        'MSP_SERVO_CONFIGURATIONS':       120,
        'MSP_MOTOR_3D_CONFIG':            124,
        'MSP_RC_DEADBAND':                125,
        'MSP_SENSOR_ALIGNMENT':           126,
        'MSP_LED_STRIP_MODECOLOR':        127,

        'MSP_VOLTAGE_METERS':             128,
        'MSP_CURRENT_METERS':             129,
        'MSP_BATTERY_STATE':              130,
        'MSP_MOTOR_CONFIG':               131,
        'MSP_GPS_CONFIG':                 132,
        'MSP_COMPASS_CONFIG':             133,
        'MSP_GPS_RESCUE':                 135,

        'MSP_STATUS_EX':                  150,

        'MSP_UID':                        160,
        'MSP_GPS_SV_INFO':                164,

        'MSP_GPSSTATISTICS':              166,

        'MSP_DISPLAYPORT':                182,

        'MSP_COPY_PROFILE':               183,

        'MSP_BEEPER_CONFIG':              184,
        'MSP_SET_BEEPER_CONFIG':          185,

        'MSP_SET_RAW_RC':                 200,
        'MSP_SET_RAW_GPS':                201, # Not used
        'MSP_SET_PID':                    202,
        #'MSP_SET_BOX':                    203, // DEPRECATED
        'MSP_SET_RC_TUNING':              204,
        'MSP_ACC_CALIBRATION':            205,
        'MSP_MAG_CALIBRATION':            206,
        'MSP_SET_MISC':                   207, # DEPRECATED
        'MSP_RESET_CONF':                 208,
        'MSP_SET_WP':                     209, # Not used
        'MSP_SELECT_SETTING':             210,
        'MSP_SET_HEADING':                211, # Not used
        'MSP_SET_SERVO_CONFIGURATION':    212,
        'MSP_SET_MOTOR':                  214,
        'MSP_SET_MOTOR_3D_CONFIG':        217,
        'MSP_SET_RC_DEADBAND':            218,
        'MSP_SET_RESET_CURR_PID':         219,
        'MSP_SET_SENSOR_ALIGNMENT':       220,
        'MSP_SET_LED_STRIP_MODECOLOR':    221,
        'MSP_SET_MOTOR_CONFIG':           222,
        'MSP_SET_GPS_CONFIG':             223,
        'MSP_SET_COMPASS_CONFIG':         224,
        'MSP_SET_GPS_RESCUE':             225,

        'MSP_MODE_RANGES_EXTRA':          238,
        'MSP_SET_ACC_TRIM':               239,
        'MSP_ACC_TRIM':                   240,
        'MSP_SERVO_MIX_RULES':            241,
        'MSP_SET_SERVO_MIX_RULE':         242, # Not used
        'MSP_SET_4WAY_IF':                245, # Not used
        'MSP_SET_RTC':                    246,
        'MSP_RTC':                        247, # Not used
        'MSP_SET_BOARD_INFO':             248, # Not used
        'MSP_SET_SIGNATURE':              249, # Not used

        'MSP_EEPROM_WRITE':               250,
        'MSP_DEBUGMSG':                   253, # Not used
        'MSP_DEBUG':                      254,

        # INAV specific codes
        'MSPV2_SETTING':                      0x1003,
        'MSPV2_SET_SETTING':                  0x1004,

        'MSP2_COMMON_MOTOR_MIXER':            0x1005,
        'MSP2_COMMON_SET_MOTOR_MIXER':        0x1006,

        'MSP2_COMMON_SETTING_INFO':           0x1007,
        'MSP2_COMMON_PG_LIST':                0x1008,

        'MSP2_CF_SERIAL_CONFIG':              0x1009,
        'MSP2_SET_CF_SERIAL_CONFIG':          0x100A,

        'MSPV2_INAV_STATUS':                  0x2000,
        'MSPV2_INAV_OPTICAL_FLOW':            0x2001,
        'MSPV2_INAV_ANALOG':                  0x2002,
        'MSPV2_INAV_MISC':                    0x2003,
        'MSPV2_INAV_SET_MISC':                0x2004,
        'MSPV2_INAV_BATTERY_CONFIG':          0x2005,
        'MSPV2_INAV_SET_BATTERY_CONFIG':      0x2006,
        'MSPV2_INAV_RATE_PROFILE':            0x2007,
        'MSPV2_INAV_SET_RATE_PROFILE':        0x2008,
        'MSPV2_INAV_AIR_SPEED':               0x2009,
        'MSPV2_INAV_OUTPUT_MAPPING':          0x200A,

        'MSP2_INAV_MIXER':                    0x2010,
        'MSP2_INAV_SET_MIXER':                0x2011,

        'MSP2_INAV_OSD_LAYOUTS':              0x2012,
        'MSP2_INAV_OSD_SET_LAYOUT_ITEM':      0x2013,
        'MSP2_INAV_OSD_ALARMS':               0x2014,
        'MSP2_INAV_OSD_SET_ALARMS':           0x2015,
        'MSP2_INAV_OSD_PREFERENCES':          0x2016,
        'MSP2_INAV_OSD_SET_PREFERENCES':      0x2017,

        'MSP2_INAV_MC_BRAKING':               0x200B,
        'MSP2_INAV_SET_MC_BRAKING':           0x200C,

        'MSP2_INAV_SELECT_BATTERY_PROFILE':   0x2018,

        'MSP2_INAV_DEBUG':                    0x2019,

        'MSP2_BLACKBOX_CONFIG':               0x201A,
        'MSP2_SET_BLACKBOX_CONFIG':           0x201B,

        'MSP2_INAV_TEMP_SENSOR_CONFIG':       0x201C,
        'MSP2_INAV_SET_TEMP_SENSOR_CONFIG':   0x201D,
        'MSP2_INAV_TEMPERATURES':             0x201E,

        'MSP2_INAV_SERVO_MIXER':              0x2020,
        'MSP2_INAV_SET_SERVO_MIXER':          0x2021,
        'MSP2_INAV_LOGIC_CONDITIONS':         0x2022,
        'MSP2_INAV_SET_LOGIC_CONDITIONS':     0x2023,
        'MSP2_INAV_LOGIC_CONDITIONS_STATUS':  0x2026,

        'MSP2_PID':                           0x2030,
        'MSP2_SET_PID':                       0x2031,

        'MSP2_INAV_OPFLOW_CALIBRATION':       0x2032
    }

    # The idea is to automate, so only the dictionary above needs to be updated
    MSPCodes2Str = {v: i for i,v in MSPCodes.items()}

    SIGNATURE_LENGTH = 32

    JUMBO_FRAME_SIZE_LIMIT = 255

    def __init__(self, device, baudrate=115200, trials=1, 
                 logfilename='MSPy.log', logfilemode='a', loglevel='DEBUG'):
        """
        Parameters
        ----------
        device : str
            The location of the serial device (e.g. "/dev/ttyACM0")
        baudrate : int, optional
            Serial connection speed (default is 115200)
        trials : int, optional
            Number of times it should try openning the serial port before giving up (default is 1)
        logfilename : str, optional
            Name of the file where the log is saved (default is 'MSPy.log').
            If logfilename=None, it will use stdout instead of a file.
        logfilemode : str, optional
            Use 'a' for append and 'w' for overriding (default is 'a'). 
        loglevel : str, optional
            The loglevel passed to logging (default is 'DEBUG')
        """

        self.dataHandler_init = {
            'msp_version':                1,
            'state':                      0,
            'message_direction':          -1,
            'code':                       0,
            'dataView':                   0,
            'message_length_expected':    0,
            'message_length_received':    0,
            'message_buffer':             [],
            'message_buffer_uint8_view':  [],
            'message_checksum':           0,
            'messageIsJumboFrame':        False,
            'crcError':                   False,

            'callbacks':                  [],
            'packet_error':               0,
            'unsupported':                0,

            'last_received_timestamp':   None,
            'listeners':                  []
        }

        self.CONFIG = {
            'apiVersion':                       "0.0.0",
            'flightControllerIdentifier':       '',
            'flightControllerVersion':          '',
            'version':                          0,
            'buildInfo':                        '',
            'multiType':                        0,
            'msp_version':                      0, # not specified using semantic versioning
            'capability':                       0,
            'cycleTime':                        0,
            'i2cError':                         0,
            'activeSensors':                    0,
            'mode':                             0,
            'profile':                          0,
            'uid':                              [0, 0, 0],
            'accelerometerTrims':               [0, 0],
            'name':                             '',
            'displayName':                      'JOE PILOT',
            'numProfiles':                      3,
            'rateProfile':                      0,
            'boardType':                        0,
            'armingDisableCount':               0,
            'armingDisableFlags':               0,
            'armingDisabled':                   False,
            'runawayTakeoffPreventionDisabled': False,
            'boardIdentifier':                  "",
            'boardVersion':                     0,
            'commCapabilities':                 0,
            'targetName':                       "",
            'boardName':                        "",
            'manufacturerId':                   "",
            'signature':                        [],
            'mcuTypeId':                        255,
        }

        self.SENSOR_DATA = {
            'gyroscope':                  [0, 0, 0],
            'accelerometer':              [0, 0, 0],
            'magnetometer':               [0, 0, 0],
            'altitude':                   0,
            'sonar':                      0,
            'kinematics':                 [0.0, 0.0, 0.0],
            'debug':                      [0, 0, 0, 0, 0, 0, 0, 0], # 8 values for special situations like MSP2_INAV_DEBUG
        }

        self.MOTOR_DATA = [0]*8

        # defaults
        # roll, pitch, yaw, throttle, aux 1, ... aux n
        self.RC = {
            'active_channels':            0,
            'channels':                   [0]*32,
        }

        self.GPS_DATA = {
            'fix':                        0,
            'numSat':                     0,
            'lat':                        0,
            'lon':                        0,
            'alt':                        0,
            'speed':                      0,
            'ground_course':              0,
            'distanceToHome':             0,
            'ditectionToHome':            0,
            'update':                     0,
            'chn':                        [],
            'svid':                       [],
            'quality':                    [],
            'cno':                        []
        }

        self.ANALOG = {}

        self.VOLTAGE_METERS = []

        self.CURRENT_METERS = []

        self.BATTERY_STATE = {}

        self.SENSOR_ALIGNMENT = {
            'align_gyro':                 0,
            'align_acc':                  0,
            'align_mag':                  0,
            'gyro_detection_flags':       0,
            'gyro_to_use':                0,
            'gyro_1_align':               0,
            'gyro_2_align':               0,
        }


        self.BOARD_ALIGNMENT_CONFIG = {
            'roll':                       0,
            'pitch':                      0,
            'yaw':                        0,
        }

        self.ARMING_CONFIG = {
            'auto_disarm_delay':          0,
            'disarm_kill_switch':         0,
            'small_angle':                0,
        }

        self.FEATURE_CONFIG = {
            'featuremask':                   0,

            'features':                     {
            0: { 'group': 'rxMode', 'name': 'RX_PPM', 'enabled': False},
            2: { 'group': 'other', 'name': 'INFLIGHT_ACC_CAL', 'enabled': False},
            3: { 'group': 'rxMode', 'name': 'RX_SERIAL', 'enabled': False},
            4: { 'group': 'esc', 'name': 'MOTOR_STOP', 'enabled': False},
            5: { 'group': 'other', 'name': 'SERVO_TILT', 'enabled': False},
            6: { 'group': 'other', 'name': 'SOFTSERIAL', 'enabled': False},
            7: { 'group': 'gps', 'name': 'GPS', 'enabled': False},
            9: { 'group': 'other', 'name': 'SONAR', 'enabled': False},
            10: { 'group': 'other', 'name': 'TELEMETRY', 'enabled': False},
            12: { 'group': '3D', 'name': '3D', 'enabled': False},
            13: { 'group': 'rxMode', 'name': 'RX_PARALLEL_PWM', 'enabled': False},
            14: { 'group': 'rxMode', 'name': 'RX_MSP', 'enabled': False},
            15: { 'group': 'rssi', 'name': 'RSSI_ADC', 'enabled': False},
            16: { 'group': 'other', 'name': 'LED_STRIP', 'enabled': False},
            17: { 'group': 'other', 'name': 'DISPLAY', 'enabled': False},
            19: { 'group': 'other', 'name': 'BLACKBOX', 'enabled': False},
            20: { 'group': 'other', 'name': 'CHANNEL_FORWARDING', 'enabled': False},
            21: { 'group': 'other', 'name': 'TRANSPONDER', 'enabled': False},
            22: { 'group': 'other', 'name': 'AIRMODE', 'enabled': False},
            18: { 'group': 'other', 'name': 'OSD', 'enabled': False},
            25: { 'group': 'rxMode', 'name': 'RX_SPI', 'enabled': False},
            27: { 'group': 'other', 'name': 'ESC_SENSOR', 'enabled': False},
            28: { 'group': 'other', 'name': 'ANTI_GRAVITY', 'enabled': False},
            29: { 'group': 'other', 'name': 'DYNAMIC_FILTER', 'enabled': False}
            }
        }

        self.RX_CONFIG = {
            'serialrx_provider':            0,
            'stick_max':                    0,
            'stick_center':                 0,
            'stick_min':                    0,
            'spektrum_sat_bind':            0,
            'rx_min_usec':                  0,
            'rx_max_usec':                  0,
            'rcInterpolation':              0,
            'rcInterpolationInterval':      0,
            'rcInterpolationChannels':      0,
            'airModeActivateThreshold':     0,
            'rxSpiProtocol':                0,
            'rxSpiId':                      0,
            'rxSpiRfChannelCount':          0,
            'fpvCamAngleDegrees':           0,
            'rcSmoothingType':              0,
            'rcSmoothingInputCutoff':       0,
            'rcSmoothingDerivativeCutoff':  0, 
            'rcSmoothingInputType':         0,
            'rcSmoothingDerivativeType':    0,
        }

        self.RC_MAP = []

        self.AUX_CONFIG_IDS = []

        self.MODE_RANGES = []

        self.MODE_RANGES_EXTRA = []

        self.ADJUSTMENT_RANGES = []

        self.RXFAIL_CONFIG = []

        self.FAILSAFE_CONFIG = {
            'failsafe_delay':                 0,
            'failsafe_off_delay':             0,
            'failsafe_throttle':              0,
            'failsafe_switch_mode':           0,
            'failsafe_throttle_low_delay':    0,
            'failsafe_procedure':             0,
        }

        self.SERVO_DATA = []*8

        self.VOLTAGE_METER_CONFIGS = []

        self.CURRENT_METER_CONFIGS = []

        self.BATTERY_CONFIG = {
            'vbatmincellvoltage':         0,
            'vbatmaxcellvoltage':         0,
            'vbatwarningcellvoltage':     0,
            'capacity':                   0,
            'voltageMeterSource':         0,
            'currentMeterSource':         0,
        }

        self.RC_TUNING = {
            'RC_RATE':                    0,
            'RC_EXPO':                    0,
            'roll_pitch_rate':            0, # pre 1.7 api only
            'roll_rate':                  0,
            'pitch_rate':                 0,
            'yaw_rate':                   0,
            'dynamic_THR_PID':            0,
            'throttle_MID':               0,
            'throttle_EXPO':              0,
            'dynamic_THR_breakpoint':     0,
            'RC_YAW_EXPO':                0,
            'rcYawRate':                  0,
            'rcPitchRate':                0,
            'RC_PITCH_EXPO':              0,
            'roll_rate_limit':            1998,
            'pitch_rate_limit':           1998,
            'yaw_rate_limit':             1998,
        }

        self.PIDs = []

        self.PID = {
            'controller':                 0
        }

        self.FC_CONFIG = {
            'loopTime':                   0
        }

        self.MOTOR_CONFIG = {
            'minthrottle':                0,
            'maxthrottle':                0,
            'mincommand':                 0,
        }

        self.MISC = {
            # DEPRECATED = only used to store values that are written back to the fc as-is, do NOT use for any other purpose
            'failsafe_throttle':          0,
            'gps_baudrate':               0,
            'multiwiicurrentoutput':      0,
            'placeholder2':               0,
            'vbatscale':                  0,
            'vbatmincellvoltage':         0,
            'vbatmaxcellvoltage':         0,
            'vbatwarningcellvoltage':     0,
            'batterymetertype':           1, # 1=ADC, 2=ESC
        }

        self.GPS_CONFIG = {
            'provider':                   0,
            'ublox_sbas':                 0,
            'auto_config':                0,
            'auto_baud':                  0,
        }

        self.RSSI_CONFIG = {
            'channel':                    0,
        }

        self.COMPASS_CONFIG = {
            'mag_declination':            0,
        }

        self.GPS_RESCUE = {
            'angle':                          0,
            'initialAltitudeM':               0,
            'descentDistanceM':               0,
            'rescueGroundspeed':              0,
            'throttleMin':                    0,
            'throttleMax':                    0,
            'throttleHover':                  0,
            'sanityChecks':                   0,
            'minSats':                        0,
        }

        self.MOTOR_3D_CONFIG = {
            'deadband3d_low':             0,
            'deadband3d_high':            0,
            'neutral':                    0,
        }


        self.AUX_CONFIG = []

        self.PIDNAMES = []

        self.SERVO_CONFIG = []

        self.RC_DEADBAND_CONFIG = {
            'deadband':                   0,
            'yaw_deadband':               0,
            'alt_hold_deadband':          0,
            'deadband3d_throttle':        0,
        }

        self.BEEPER_CONFIG = {
            'beepers':                    0,
            'dshotBeaconTone':            0,
            'dshotBeaconConditions':      0,
        }

        self.MIXER_CONFIG = {
            'mixer':                      0,
            'reverseMotorDir':            0,
        }

        self.REBOOT_TYPES = {
            'FIRMWARE': 0,
            'BOOTLOADER': 1,
            'MSC': 2,
            'MSC_UTC': 3
        }

        # 0 based index, must be identical to 'baudRates' in 'src/main/io/serial.c' in betaflight
        self.BAUD_RATES = ['AUTO', '9600', '19200', '38400', '57600', '115200',
                           '230400', '250000', '400000', '460800', '500000', '921600', '1000000',
                           '1500000', '2000000', '2470000']

        # needs to be identical to 'serialPortFunction_e' in 'src/main/io/serial.h' in betaflight
        self.SERIAL_PORT_FUNCTIONS = {
            'MSP': 0,
            'GPS': 1,
            'TELEMETRY_FRSKY': 2,
            'TELEMETRY_HOTT': 3,
            'TELEMETRY_MSP': 4,
            'TELEMETRY_LTM': 4, # LTM replaced MSP
            'TELEMETRY_SMARTPORT': 5,
            'RX_SERIAL': 6,
            'BLACKBOX': 7,
            'TELEMETRY_MAVLINK': 9,
            'ESC_SENSOR': 10,
            'TBS_SMARTAUDIO': 11,
            'TELEMETRY_IBUS': 12,
            'IRC_TRAMP': 13,
            'RUNCAM_DEVICE_CONTROL': 14, # support communicate with RunCam Device
            'LIDAR_TF': 15
        }

        self.SERIAL_CONFIG = {
            'ports':                      [],

            # pre 1.6 settings
            'mspBaudRate':                0,
            'gpsBaudRate':                0,
            'gpsPassthroughBaudRate':     0,
            'cliBaudRate':                0,
        }

        self.PID_ADVANCED_CONFIG = {
            'gyro_sync_denom':            0,
            'pid_process_denom':          0,
            'use_unsyncedPwm':            0,
            'fast_pwm_protocol':          0,
            'motor_pwm_rate':             0,
            'digitalIdlePercent':         0,
            'gyroUse32kHz':               0,
        }

        self.FILTER_CONFIG = {
            'gyro_hardware_lpf':          0,
            'gyro_32khz_hardware_lpf':    0,
            'gyro_lowpass_hz':            0,
            'gyro_lowpass_dyn_min_hz':    0,
            'gyro_lowpass_dyn_max_hz':    0,
            'gyro_lowpass_type':          0,
            'gyro_lowpass2_hz':           0,
            'gyro_lowpass2_type':         0,
            'gyro_notch_hz':              0,
            'gyro_notch_cutoff':          0,
            'gyro_notch2_hz':             0,
            'gyro_notch2_cutoff':         0,
            'dterm_lowpass_hz':           0,
            'dterm_lowpass_dyn_min_hz':   0,
            'dterm_lowpass_dyn_max_hz':   0,
            'dterm_lowpass_type':         0,
            'dterm_lowpass2_hz':          0,
            'dterm_lowpass2_type':        0,
            'dterm_notch_hz':             0,
            'dterm_notch_cutoff':         0,
            'yaw_lowpass_hz':             0,
        }

        self.ADVANCED_TUNING = {
            'rollPitchItermIgnoreRate':   0,
            'yawItermIgnoreRate':         0,
            'yaw_p_limit':                0,
            'deltaMethod':                0,
            'vbatPidCompensation':        0,
            'dtermSetpointTransition':    0,
            'dtermSetpointWeight':        0,
            'toleranceBand':              0,
            'toleranceBandReduction':     0,
            'itermThrottleGain':          0,
            'pidMaxVelocity':             0,
            'pidMaxVelocityYaw':          0,
            'levelAngleLimit':            0,
            'levelSensitivity':           0,
            'itermThrottleThreshold':     0,
            'itermAcceleratorGain':       0,
            'itermRotation':              0,
            'smartFeedforward':           0,
            'itermRelax':                 0,
            'itermRelaxType':             0,
            'absoluteControlGain':        0,
            'throttleBoost':              0,
            'acroTrainerAngleLimit':      0,
            'feedforwardRoll':            0,
            'feedforwardPitch':           0,
            'feedforwardYaw':             0,
            'feedforwardTransition':      0,
            'antiGravityMode':            0,
            'dMinRoll':                   0,
            'dMinPitch':                  0,
            'dMinYaw':                    0,
            'dMinGain':                   0,
            'dMinAdvance':                0,
            'useIntegratedYaw':           0,
            'integratedYawRelax':         0,
        }

        self.SENSOR_CONFIG = {
            'acc_hardware':               0,
            'baro_hardware':              0,
            'mag_hardware':               0,
        }

        self.DATAFLASH = {
            'ready':                      False,
            'supported':                  False,
            'sectors':                    0,
            'totalSize':                  0,
            'usedSize':                   0
        }

        self.SDCARD = {
            'supported':                  False,
            'state':                      0,
            'filesystemLastError':        0,
            'freeSizeKB':                 0,
            'totalSizeKB':                0,
        }

        self.BLACKBOX = {
            'supported':                  False,
            'blackboxDevice':             0,
            'blackboxRateNum':            1,
            'blackboxRateDenom':          1,
            'blackboxPDenom':             0,
        }

        self.TRANSPONDER = {
            'supported':                  False,
            'data':                       [],
            'provider':                   0,
            'providers':                  [],
        }

        self.armingDisableFlagNames_BF = {
            0: "NOGYRO",
            1: "FAILSAFE",
            2: "RXLOSS",
            3: "BADRX",
            4: "BOXFAILSAFE",
            5: "RUNAWAY",
            6: "CRASH",
            7: "THROTTLE",
            8: "ANGLE",
            9: "BOOTGRACE",
            10: "NOPREARM",
            11: "LOAD",
            12: "CALIB",
            13: "CLI",
            14: "CMS",
            15: "BST",
            16: "MSP",
            17: "PARALYZE",
            18: "GPS",
            19: "RESCUE SW",
            20: "RPMFILTER",
            21: "ARMSWITCH"
            }

        self.armingDisableFlagNames_INAV = {
            0: "OK_TO_ARM",
            1: "PREVENT_ARMING",
            2: "ARMED",
            3: "WAS_EVER_ARMED",
            8: "BLOCKED_UAV_NOT_LEVEL",
            9: "BLOCKED_SENSORS_CALIBRATING",
            10: "BLOCKED_SYSTEM_OVERLOADED",
            11: "BLOCKED_NAVIGATION_SAFETY",
            12: "BLOCKED_COMPASS_NOT_CALIBRATED",
            13: "BLOCKED_ACCELEROMETER_NOT_CALIBRATED",
            15: "BLOCKED_HARDWARE_FAILURE",
            26: "BLOCKED_INVALID_SETTING"
            }

        if logfilename:
            logging.basicConfig(format="[%(levelname)s] [%(asctime)s]: %(message)s",
                                filename=logfilename, 
                                filemode=logfilemode,
                                level=getattr(logging, loglevel.upper()))
        else:
            logging.basicConfig(format="[%(levelname)s] [%(asctime)s]: %(message)s",
                                level=getattr(logging, loglevel.upper()),
                                stream=sys.stdout)

        # For some reason, passing the arguments doesn't work and
        # the serial fails...
        # self.conn = serial.Serial(port=device,
        #                           baudrate=baudrate,
        #                           bytesize=serial.EIGHTBITS,
        #                           parity=serial.PARITY_NONE,
        #                           stopbits=serial.STOPBITS_ONE,
        #                           timeout=0, # 0 for non-blocking mode (read)
        #                           xonxoff=False,
        #                           rtscts=False,
        #                           write_timeout=1, # it will raise serial.SerialTimeoutException
        #                           dsrdtr=False,
        #                           inter_byte_timeout=None,
        #                           exclusive=None)

        self.conn = serial.Serial()
        self.conn.port = device
        self.conn.baudrate = baudrate
        self.conn.bytesize = serial.EIGHTBITS
        self.conn.parity = serial.PARITY_NONE
        self.conn.stopbits = serial.STOPBITS_ONE
        self.conn.timeout = 1
        self.conn.xonxoff = False
        self.conn.rtscts = False
        self.conn.dsrdtr = False
        self.conn.writeTimeout = 1

        self.ser_trials = trials

        self.serial_port_write_lock = Lock()
        self.serial_port_read_lock = Lock()


    def __enter__(self):
        self.is_serial_open = not self.connect(trials=self.ser_trials)

        if self.is_serial_open:
            return self
        else:
            logging.warning("Serial port ({}) not ready/available".format(self.conn.port))
            return 1


    def __exit__(self, exc_type, exc_value, traceback):
        if not self.conn.closed:
            self.conn.close()


    def connect(self, trials=100, delay=0.5):
        """Opens the serial connection with the board

        Parameters
        ----------
        trials : int
            Number of times it should try openning the serial port before giving up (default is 1)

        delay : float
            Time between trials
            
        Returns
        -------
        int
            0 if successful or 1 if not
        """

        for _ in range(trials):
            try:
                self.conn.open()
                self.basic_info()
                return 0
                
            except serial.SerialException as err:
                logging.warning("Error opening the serial port ({0}): {1}".format(self.conn.port, err))
            
            except FileNotFoundError as err:
                logging.warning("Port ({0}) not found: {1}".format(self.conn.port, err))

            time.sleep(delay)
        
        return 1


    def basic_info(self):
        """Basic info about the flight controller to distinguish between the many flavours.
        """
        for msg in ['MSP_API_VERSION', 'MSP_FC_VARIANT']:
            if self.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                dataHandler = self.receive_msg()
                self.process_recv_data(dataHandler)

        if 'INAV' in self.CONFIG['flightControllerIdentifier']:
            self.INAV = True
        else:
            self.INAV = False

        basic_info_cmd_list = ['MSP_FC_VERSION', 'MSP_BUILD_INFO', 'MSP_BOARD_INFO', 'MSP_UID', 
                               'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX']
        if self.INAV:
            basic_info_cmd_list.append('MSPV2_INAV_ANALOG')
            basic_info_cmd_list.append('MSP_VOLTAGE_METER_CONFIG')

        for msg in basic_info_cmd_list:
            if self.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                dataHandler = self.receive_msg()
                self.process_recv_data(dataHandler)
    
        print(self.CONFIG)

    def fast_read_altitude(self):
        # Request altitude
        if self.send_RAW_msg(MSPy.MSPCodes['MSP_ALTITUDE']):
            # $ + M + < + data_length + msg_code + data + msg_crc
            # 6 bytes + data_length
            data_length = 4
            msg = self.receive_raw_msg(size = (6+data_length))[5:]
            converted_msg = struct.unpack('<i', msg[:-1])[0]
            self.SENSOR_DATA['altitude'] = round((converted_msg / 100.0), 2) # correct scale factor

    def fast_read_imu(self):
        """Request, read and process RAW IMU
        """

        # Request IMU values
        if self.send_RAW_msg(MSPy.MSPCodes['MSP_RAW_IMU']):
            # $ + M + < + data_length + msg_code + data + msg_crc
            # 6 bytes + data_length
            # data_length: 9 x 2 = 18 bytes
            data_length = 18
            msg = self.receive_raw_msg(size = (6+data_length))[5:]
            converted_msg = struct.unpack('<%dh' % (data_length/2) , msg[:-1])

            # /512 for mpu6050, /256 for mma
            # currently we are unable to differentiate between the sensor types, so we are going with 512
            # And what about SENSOR_CONFIG???
            self.SENSOR_DATA['accelerometer'][0] = converted_msg[0]
            self.SENSOR_DATA['accelerometer'][1] = converted_msg[1]
            self.SENSOR_DATA['accelerometer'][2] = converted_msg[2]

            # properly scaled (INAV and BF use the same * (4 / 16.4))
            # but this is supposed to be RAW, so raw it is!
            self.SENSOR_DATA['gyroscope'][0] = converted_msg[3]
            self.SENSOR_DATA['gyroscope'][1] = converted_msg[4]
            self.SENSOR_DATA['gyroscope'][2] = converted_msg[5]

            # no clue about scaling factor (/1090), so raw
            self.SENSOR_DATA['magnetometer'][0] = converted_msg[6]
            self.SENSOR_DATA['magnetometer'][1] = converted_msg[7]
            self.SENSOR_DATA['magnetometer'][2] = converted_msg[8]


    def fast_read_attitude(self):
        """Request, read and process the ATTITUDE (Roll, Pitch and Yaw in degrees)
        """

        # Request ATTITUDE values
        if self.send_RAW_msg(MSPy.MSPCodes['MSP_ATTITUDE']):
            # $ + M + < + data_length + msg_code + data + msg_crc
            # 6 bytes + data_length
            # data_length: 3 x 2 = 6 bytes
            data_length = 6
            msg = self.receive_raw_msg(size = (6+data_length))[5:]
            converted_msg = struct.unpack('<%dh' % (data_length/2) , msg[:-1])

            self.SENSOR_DATA['kinematics'][0] = converted_msg[0] / 10.0 # x
            self.SENSOR_DATA['kinematics'][1] = converted_msg[1] / 10.0 # y
            self.SENSOR_DATA['kinematics'][2] = converted_msg[2] # z
    
    
    def fast_read_analog(self):
        """Request, read and process the ANALOG message
        """

        # Request ANALOG values
        if self.send_RAW_msg(MSPy.MSPCodes['MSP_ANALOG']):
            # $ + M + < + data_length + msg_code + data + msg_crc
            # 6 bytes + data_length
            if not self.INAV:
                # data_length: 1 + 2 + 2 + 2 + 2 = 9 bytes
                data_length = 9
                msg = self.receive_raw_msg(size = (6+data_length))[5:]
                converted_msg = struct.unpack('<B2HhH', msg[:-1])

            else:
                # data_length: 1 + 2 + 2 + 2 = 7 bytes
                data_length = 7
                msg = self.receive_raw_msg(size = (6+data_length))[5:]
                converted_msg = struct.unpack('<B2Hh', msg[:-1])

            self.ANALOG['voltage'] = converted_msg[0] / 10 # iNAV uses a MSPV2 message to get a precise value.
            self.ANALOG['mAhdrawn'] = converted_msg[1]
            self.ANALOG['rssi'] = converted_msg[2] # 0-1023
            self.ANALOG['amperage'] = converted_msg[3] / 100 # A
            self.ANALOG['last_received_timestamp'] = int(time.time()) # why not monotonic? where is time synchronized?
            if not self.INAV:
                self.ANALOG['voltage'] = converted_msg[4] / 100 # BF has this 2 bytes value here


    def fast_msp_rc_cmd(self, cmds):
        """Send, read and process the RAW RC considering the MSP_RX

        Parameters
        ----------
        cmds : list
            List with RC values to be sent
            * The number of values is 4 + number of AUX channels enabled (max 14) 
        """
        cmds = [int(cmd) for cmd in cmds]
        data = struct.pack('<%dH' % len(cmds), *cmds)
        if self.send_RAW_msg(MSPy.MSPCodes['MSP_SET_RAW_RC'], data):
            # $ + M + < + data_length + msg_code + data + msg_crc
            # 6 bytes + data_length

            # The FC will send a code 0 message until it received enough RC msgs, then it
            # will return a code 200. However, the message is always empty (data_length = 0).
            _ = self.receive_raw_msg(size = 6)

    def receive_raw_msg(self, size, timeout = 10):
        """Receive multiple bytes at once when it's not a jumbo frame.

        Returns
        -------
        bytes
            data received
        """
        with self.serial_port_read_lock: # It's necessary to lock everything because order is important
            local_read = self.conn.read
            timeout = time.time() + timeout
            while True:
                if time.time() >= timeout:
                    logging.warning("Timeout occured when receiving a message")
                    break
                msg_header = local_read()
                if msg_header:
                    if ord(msg_header) == 36: # $
                        break

            msg = local_read(size - 1) # -1 to compensate for the $
            return msg_header + msg

    def receive_msg(self):
        """Receive an MSP message from the serial port
        Based on betaflight-configurator (https://git.io/fjRAz)

        Returns
        -------
        dict
            dataHandler with the received data pre-parsed
        """

        dataHandler = self.dataHandler_init.copy()
        received_bytes = self.receive_raw_msg(3)
        dataHandler['last_received_timestamp'] = time.time()

        local_read = self.conn.read
        with self.serial_port_read_lock: # It's necessary to lock everything because order is important
            di = 0
            while True:
                try:
                    data = received_bytes[di]
                    di += 1
                    logging.debug("State: {1} - byte received (at {0}): {2}".format(dataHandler['last_received_timestamp'], 
                                                                            dataHandler['state'], 
                                                                            data))
                except IndexError:
                    # Instead of crashing everything, let's just ignore this msg...
                    # ... and hope for the best :)
                    logging.debug('IndexError detected on state: {}'.format(dataHandler['state']))
                    dataHandler['state'] = -1
                    break # Sends it to the error state

                # it will always fall in the first state by default
                if dataHandler['state'] == 0: # sync char 1
                    if (data == 36): # $ - a new MSP message begins with $
                        dataHandler['state'] = 1

                elif dataHandler['state'] == 1: # sync char 2
                    if (data == 77): # M - followed by an M => MSP V1
                        dataHandler['msp_version'] = 1
                        dataHandler['state'] = 2
                    elif (data == 88): # X => MSP V2
                        dataHandler['msp_version'] = 2
                        dataHandler['state'] = 2
                    else: # something went wrong, no M received...
                        logging.debug('Something went wrong, no M received.')
                        break # sends it to the error state

                elif dataHandler['state'] == 2: # direction (should be >)
                    dataHandler['unsupported'] = 0
                    if (data == 33): # !
                        # FC reports unsupported message error
                        logging.debug('FC reports unsupported message error.')
                        dataHandler['unsupported'] = 1
                        break # sends it to the error state
                    else:
                        if (data == 62): # > FC to PC
                            dataHandler['message_direction'] = 1
                        elif (data == 60): # < PC to FC
                            dataHandler['message_direction'] = 0
                            
                        if dataHandler['msp_version'] == 1:
                            dataHandler['state'] = 3
                            received_bytes += local_read(2)
                        elif dataHandler['msp_version'] == 2:
                            dataHandler['state'] = 2.1
                            received_bytes += local_read(5)

                elif dataHandler['state'] == 2.1: # MSP V2: flag (ignored)
                    dataHandler['flags'] = data # 4th byte 
                    dataHandler['state'] = 2.2

                elif dataHandler['state'] == 2.2: # MSP V2: code LOW
                    dataHandler['code'] = data
                    dataHandler['state'] = 2.3

                elif dataHandler['state'] == 2.3: # MSP V2: code HIGH
                    dataHandler['code'] |= data << 8
                    dataHandler['state'] = 3.1

                elif dataHandler['state'] == 3:
                    dataHandler['message_length_expected'] = data # 4th byte
                    if dataHandler['message_length_expected'] == MSPy.JUMBO_FRAME_SIZE_LIMIT:
                        logging.debug("JumboFrame received.")
                        dataHandler['messageIsJumboFrame'] = True

                    # start the checksum procedure
                    dataHandler['message_checksum'] = data
                    dataHandler['state'] = 4

                elif dataHandler['state'] == 3.1: # MSP V2: msg length LOW
                    dataHandler['message_length_expected'] = data
                    dataHandler['state'] = 3.2

                elif dataHandler['state'] == 3.2: # MSP V2: msg length HIGH
                    dataHandler['message_length_expected'] |= data << 8
                    # setup buffer according to the message_length_expected
                    dataHandler['message_buffer_uint8_view'] = dataHandler['message_buffer'] # keep same names from betaflight-configurator code
                    if dataHandler['message_length_expected'] > 0:
                        dataHandler['state'] = 7
                        received_bytes += local_read(dataHandler['message_length_expected']+2) # +2 for CRC
                    else:
                        dataHandler['state'] = 9
                        received_bytes += local_read(2) # 2 for CRC

                elif dataHandler['state'] == 4:
                    dataHandler['code'] = data
                    dataHandler['message_checksum'] ^= data

                    if dataHandler['message_length_expected'] > 0:
                        # process payload
                        if dataHandler['messageIsJumboFrame']:
                            dataHandler['state'] = 5
                            received_bytes += local_read()
                        else:
                            received_bytes += local_read(dataHandler['message_length_expected']+1) # +1 for CRC
                            dataHandler['state'] = 7
                    else:
                        # no payload
                        dataHandler['state'] = 9
                        received_bytes += local_read()

                elif dataHandler['state'] == 5:
                    # this is a JumboFrame
                    dataHandler['message_length_expected'] = data

                    dataHandler['message_checksum'] ^= data

                    dataHandler['state'] = 6
                    received_bytes += local_read()

                elif dataHandler['state'] == 6:
                    # calculates the JumboFrame size
                    dataHandler['message_length_expected'] +=  256 * data
                    logging.debug("JumboFrame message_length_expected: {}".format(dataHandler['message_length_expected']))
                    # There's no way to check for transmission errors here...
                    # In the worst scenario, it will try to read 255 + 256*255 = 65535 bytes

                    dataHandler['message_checksum'] ^= data

                    dataHandler['state'] = 7
                    received_bytes += local_read(dataHandler['message_length_expected']+1) # +1 for CRC

                elif dataHandler['state'] == 7:
                    # setup buffer according to the message_length_expected
                    dataHandler['message_buffer'] = bytearray(dataHandler['message_length_expected'])
                    dataHandler['message_buffer_uint8_view'] = dataHandler['message_buffer'] # keep same names from betaflight-configurator code

                    # payload
                    dataHandler['message_buffer_uint8_view'][dataHandler['message_length_received']] = data
                    dataHandler['message_checksum'] ^= data
                    dataHandler['message_length_received'] += 1

                    if dataHandler['message_length_received'] == dataHandler['message_length_expected']:
                        dataHandler['state'] = 9
                    else:
                        dataHandler['state'] = 8

                elif dataHandler['state'] == 8:
                    # payload
                    dataHandler['message_buffer_uint8_view'][dataHandler['message_length_received']] = data
                    dataHandler['message_checksum'] ^= data
                    dataHandler['message_length_received'] += 1

                    if dataHandler['message_length_received'] == dataHandler['message_length_expected']:
                        dataHandler['state'] = 9

                elif dataHandler['state'] == 9:
                        if dataHandler['msp_version'] == 1:
                            if dataHandler['message_checksum'] == data:
                                # checksum is correct, message received, store dataview
                                logging.debug("Message received (length {1}) - Code {0}".format(dataHandler['code'], dataHandler['message_length_received']))
                                dataHandler['dataView'] = dataHandler['message_buffer'] # keep same names from betaflight-configurator code
                                return dataHandler
                            else:
                                # wrong checksum
                                logging.debug('Code: {0} - crc failed (received {1}, calculated {2})'.format(dataHandler['code'], 
                                                                                                            data,
                                                                                                            dataHandler['message_checksum']))
                                dataHandler['crcError'] = True
                                break # sends it to the error state
                        elif dataHandler['msp_version'] == 2:
                            dataHandler['message_checksum'] = 0
                            dataHandler['message_checksum'] = self._crc8_dvb_s2(dataHandler['message_checksum'], 0) # flag
                            dataHandler['message_checksum'] = self._crc8_dvb_s2(dataHandler['message_checksum'], dataHandler['code'] & 0xFF) # code LOW
                            dataHandler['message_checksum'] = self._crc8_dvb_s2(dataHandler['message_checksum'], (dataHandler['code'] & 0xFF00) >> 8) # code HIGH
                            dataHandler['message_checksum'] = self._crc8_dvb_s2(dataHandler['message_checksum'], dataHandler['message_length_expected'] & 0xFF) #  HIGH
                            dataHandler['message_checksum'] = self._crc8_dvb_s2(dataHandler['message_checksum'], (dataHandler['message_length_expected'] & 0xFF00) >> 8) #  HIGH
                            for si in range(dataHandler['message_length_received']):
                                dataHandler['message_checksum'] = self._crc8_dvb_s2(dataHandler['message_checksum'], dataHandler['message_buffer'][si])
                            if dataHandler['message_checksum'] == data:
                                # checksum is correct, message received, store dataview
                                logging.debug("Message received (length {1}) - Code {0}".format(dataHandler['code'], dataHandler['message_length_received']))
                                dataHandler['dataView'] = dataHandler['message_buffer'] # keep same names from betaflight-configurator code
                                return dataHandler
                            else:
                                # wrong checksum
                                logging.debug('Code: {0} - crc failed (received {1}, calculated {2})'.format(dataHandler['code'], 
                                                                                                            data,
                                                                                                            dataHandler['message_checksum']))
                                dataHandler['crcError'] = True
                                break # sends it to the error state

            # it means an error occurred
            logging.debug('Error detected on state: {}'.format(dataHandler['state']))
            dataHandler['packet_error'] = 1

            return dataHandler


    @staticmethod
    def readbytes(data, size=8, unsigned=False, read_as_float=False):
        """Unpack bytes according to size / type

        Parameters
        ----------
        data : bytearray
            Data to be unpacked
        size : int, optional
            Number of bits (8, 16 or 32) (default is 8)
        unsigned : bool, optional
            Indicates if data is unsigned or not (default is False)
        read_as_float: bool, optional
            Indicates if data is read as float or not (default is False)
            
        Returns
        -------
        int
            unpacked bytes according to input options
        """
        buffer = bytearray()

        for _ in range(int(size/8)):
            buffer.append(data.pop(0))
        
        if size==8:
            unpack_format = 'b'
        elif size==16:
            if read_as_float: # for special situations like MSP2_INAV_DEBUG
                unpack_format = 'e'
            else:   
                unpack_format = 'h'
        elif size==32:
            if read_as_float: # for special situations like MSP2_INAV_DEBUG
                unpack_format = 'f'
            else:
                unpack_format = 'i'
        
        if unsigned:
            unpack_format = unpack_format.upper()

        return struct.unpack('<' + unpack_format, buffer)[0]


    def process_armingDisableFlags(self, flags):
        result = []
        while (flags):
            bitpos = ffs(flags) - 1
            flags &= ~(1 << bitpos)
            if self.INAV:
                result.append(self.armingDisableFlagNames_INAV.get(bitpos, ""))
            else:
                result.append(self.armingDisableFlagNames_BF.get(bitpos, ""))
        return result


    def process_mode(self, flag):
        """Translate the value from CONFIG['mode']
        """
        result = []
        for i in range(len(self.AUX_CONFIG)):
            if (self.bit_check(flag, i)):
                result.append(self.AUX_CONFIG[i])

        return result


    @staticmethod
    def bit_check(mask, bit):
        return ((mask>>bit)%2) != 0


    def serialPortFunctionMaskToFunctions(self, functionMask):
        functions = []

        keys = self.SERIAL_PORT_FUNCTIONS.keys()
        for key in keys:
            bit = self.SERIAL_PORT_FUNCTIONS[key]
            if (self.bit_check(functionMask, bit)):
                functions.append(key)
        return functions


    @staticmethod
    def convert(val_list, n=16): 
        """Convert to n*bits (8 multiple) list

        Parameters
        ----------
        val_list : list
            List with values to be converted
        
        n: int, optional
            Number of bits (multiple of 8) (default is 16)
            
        Returns
        -------
        list
            List where each item is the equivalent byte value
        """ 
        buffer = []
        for val in val_list:
            for i in range(int(n/8)): 
                buffer.append((int(val)>>i*8) & 255) 
        return buffer 

    def save2eprom(self):
        logging.info("Save to EPROM requested") # some configs also need reboot to be applied (not online).
        return self.send_RAW_msg(MSPy.MSPCodes['MSP_EEPROM_WRITE'], data=[])


    def reboot(self):
        logging.info("Reboot requested")
        return self.send_RAW_msg(MSPy.MSPCodes['MSP_SET_REBOOT'], data=[])


    def set_ARMING_DISABLE(self, armingDisabled=0, runawayTakeoffPreventionDisabled=0):
        """Disable Arming or runaway takeoff prevention
        
        runawayTakeoffPreventionDisabled will be ignored if armingDisabled is true
        https://github.com/betaflight/betaflight/wiki/Runaway-Takeoff-Prevention
        """
        data = bytearray([armingDisabled, runawayTakeoffPreventionDisabled])
        return self.send_RAW_msg(MSPy.MSPCodes['MSP_ARMING_DISABLE'], data)


    def set_RX_MAP(self, new_rc_map):
        assert(type(new_rc_map)==list)
        assert(len(new_rc_map)==8)

        return self.send_RAW_msg(MSPy.MSPCodes['MSP_SET_RX_MAP'], new_rc_map)


    def set_FEATURE_CONFIG(self, mask):
        assert(type(mask)==int)

        data = self.convert([mask], 32)
        return self.send_RAW_msg(MSPy.MSPCodes['MSP_SET_FEATURE_CONFIG'], data)


    def send_RAW_MOTORS(self, data=[]):
        assert(type(data)==list)
        assert(len(data)==8)

        data = self.convert(data, 16) # any values bigger than 255 need to be converted.
                                      # RC and Motor commands go from 0 to 2000.

        return self.send_RAW_msg(MSPy.MSPCodes['MSP_SET_MOTOR'], data)


    def send_RAW_RC(self, data=[]):
        """
        When using RX_SERIAL:
        [roll, pitch, yaw, throttle, aux1, aux2,...,aux10]

        When using RX_MSP:
        [roll, pitch, yaw, throttle, aux1, aux2,...,aux14]

        Considering RC_MAP==[0, 1, 3, 2, 4, 5, 6, 7]
        """
        data = self.convert(data, 16) # any values bigger than 255 need to be converted.
                                      # RC and Motor commands go from 0 to 2000.

        return self.send_RAW_msg(MSPy.MSPCodes['MSP_SET_RAW_RC'], data)


    def send_RAW_msg(self, code, data=[], blocking=True, timeout=-1):
        """Send a RAW MSP message through the serial port
        Based on betaflight-configurator (https://git.io/fjRxz)

        Parameters
        ----------
        code : int
            MSP Code
        
        data: list or bytearray, optional
            Data to be sent (default is [])
            
        Returns
        -------
        int
            number of bytes of data actually written (including 6 bytes header)
        """

        res = -1

        # Always reserve 6 bytes for protocol overhead
        # $ + M + < + data_length + msg_code + data + msg_crc
        len_data = len(data)
        if code < 255: # MSP V1
            size = len_data + 6
            checksum = 0

            bufView = bytearray([0]*size)

            bufView[0] = 36 #$
            bufView[1] = 77 #M
            bufView[2] = 60 #<
            bufView[3] = len_data
            bufView[4] = code

            checksum = bufView[3] ^ bufView[4]

            for i in range(len_data):
                bufView[i + 5] = data[i]
                checksum ^= bufView[i + 5]

            bufView[-1] = checksum

        elif code > 255: # MSP V2
            size = len_data + 9
            checksum = 0
            bufView = bytearray([0]*size)
            bufView[0] = 36 #$ 
            bufView[1] = 88 #X
            bufView[2] = 60 #<
            bufView[3] = 0 #flag: reserved, set to 0
            bufView[4] = code & 0xFF #code lower byte
            bufView[5] = (code & 0xFF00) >> 8 #code upper byte
            bufView[6] = len_data & 0xFF #len_data lower byte
            bufView[7] = (len_data & 0xFF00) >> 8 #len_data upper byte
            for di in range(len_data):
                bufView[8+di] = data[di]
            for si in range(3, size-1):
                checksum = self._crc8_dvb_s2(checksum, bufView[si])
            bufView[-1] = checksum

        if self.serial_port_write_lock.acquire(blocking, timeout):
            try:
                res = self.conn.write(bufView)
            finally:
                self.serial_port_write_lock.release()
                if res>0:
                    logging.debug("RAW message sent: {}".format(bufView))

                return res
    
    @staticmethod
    def _crc8_dvb_s2(crc, ch):
        """CRC for MSPV2
        *copied from inav-configurator
        """
        crc ^= ch
        for _ in range(8):
            if (crc & 0x80):
                crc = ((crc << 1) & 0xFF) ^ 0xD5
            else:
                crc = (crc << 1) & 0xFF
        return crc

    def process_recv_data(self, dataHandler):
        """Process the dataHandler from receive_msg consuming (pop!) dataHandler['dataView'] as it goes.
        Based on betaflight-configurator (https://git.io/fjRAV)

        Parameters
        ----------
        dataHandler : dict
            Dictionary generated by receive_msg
            
        Returns
        -------
        int
            len(data) when successful or -(error type) if not
        """

        data = dataHandler['dataView'] # DataView (allowing us to view arrayBuffer as struct/union)
        code = dataHandler['code']
        if code == 0: # code==0 means nothing was received...
            logging.debug("Nothing was received - Code 0")
            return -1
        elif dataHandler['crcError']:
            logging.debug("dataHandler has a crcError.")
            return -2
        elif dataHandler['packet_error']:
            logging.debug("dataHandler has a packet_error.")
            return -3
        else:
            if (not dataHandler['unsupported']):
                processor = MSPy.__dict__.get("process_" + MSPy.MSPCodes2Str[code])
                if processor: # if nothing is found, should be None
                    try:
                        if data:
                            processor(self,data) # use it..
                            return len(data)
                        else:
                            return 0 # because a valid message may contain no data...
                    except IndexError as err:
                        logging.debug('Received data processing error: {}'.format(err))
                        return -4
                else:
                    logging.debug('Unknown code received: {}'.format(code))
                    return -5
            else:
                logging.debug('FC reports unsupported message error - Code {}'.format(code))
                return -6
        

    def process_MSP_STATUS(self, data):
        self.CONFIG['cycleTime'] = self.readbytes(data, size=16, unsigned=True)
        self.CONFIG['i2cError'] = self.readbytes(data, size=16, unsigned=True)
        self.CONFIG['activeSensors'] = self.readbytes(data, size=16, unsigned=True)
        self.CONFIG['mode'] = self.readbytes(data, size=32, unsigned=True)
        self.CONFIG['profile'] = self.readbytes(data, size=8, unsigned=True)
        
    def process_MSP_STATUS_EX(self, data):
        self.CONFIG['cycleTime'] = self.readbytes(data, size=16, unsigned=True)
        self.CONFIG['i2cError'] = self.readbytes(data, size=16, unsigned=True)
        self.CONFIG['activeSensors'] = self.readbytes(data, size=16, unsigned=True)
        self.CONFIG['mode'] = self.readbytes(data, size=32, unsigned=True)

        self.CONFIG['profile'] = self.readbytes(data, size=8, unsigned=True)
        self.CONFIG['cpuload'] = self.readbytes(data, size=16, unsigned=True)
        
        if not self.INAV:
            self.CONFIG['numProfiles'] = self.readbytes(data, size=8, unsigned=True)
            self.CONFIG['rateProfile'] = self.readbytes(data, size=8, unsigned=True)

            # Read flight mode flags
            byteCount = self.readbytes(data, size=8, unsigned=True)
            self.CONFIG['flightModeFlags'] = [] # this was not implemented on betaflight-configurator
            for _ in range(byteCount):
                # betaflight-configurator would just discard these bytes
                self.CONFIG['flightModeFlags'].append(self.readbytes(data, size=8, unsigned=True))

            # Read arming disable flags
            self.CONFIG['armingDisableCount'] = self.readbytes(data, size=8, unsigned=True) # Flag count
            self.CONFIG['armingDisableFlags'] = self.readbytes(data, size=32, unsigned=True)
        else:
            self.CONFIG['armingDisableFlags'] = self.readbytes(data, size=16, unsigned=True)

    def process_MSP_RAW_IMU(self, data):
        # /512 for mpu6050, /256 for mma
        # currently we are unable to differentiate between the sensor types, so we are going with 512
        # And what about SENSOR_CONFIG???
        self.SENSOR_DATA['accelerometer'][0] = self.readbytes(data, size=16, unsigned=False)
        self.SENSOR_DATA['accelerometer'][1] = self.readbytes(data, size=16, unsigned=False)
        self.SENSOR_DATA['accelerometer'][2] = self.readbytes(data, size=16, unsigned=False)

        # properly scaled (INAV and BF use the same * (4 / 16.4))
        # but this is supposed to be RAW, so raw it is!
        self.SENSOR_DATA['gyroscope'][0] = self.readbytes(data, size=16, unsigned=False)
        self.SENSOR_DATA['gyroscope'][1] = self.readbytes(data, size=16, unsigned=False)
        self.SENSOR_DATA['gyroscope'][2] = self.readbytes(data, size=16, unsigned=False)

        # no clue about scaling factor (/1090), so raw
        self.SENSOR_DATA['magnetometer'][0] = self.readbytes(data, size=16, unsigned=False)
        self.SENSOR_DATA['magnetometer'][1] = self.readbytes(data, size=16, unsigned=False)
        self.SENSOR_DATA['magnetometer'][2] = self.readbytes(data, size=16, unsigned=False)

    def process_MSP_SERVO(self, data):
        servoCount = int(len(data) / 2)
        self.SERVO_DATA = [self.readbytes(data, size=16, unsigned=True) for _ in range(servoCount)]

    def process_MSP_MOTOR(self, data):
        motorCount = int(len(data) / 2)
        self.MOTOR_DATA = [self.readbytes(data, size=16, unsigned=True) for i in range(motorCount)]

    def process_MSP_RC(self, data):
        n_channels = int(len(data) / 2)
        self.RC['active_channels'] = n_channels
        self.RC['channels'] = [self.readbytes(data, size=16, unsigned=True) for i in range(n_channels)]

    def process_MSP_RAW_GPS(self, data):
        self.GPS_DATA['fix'] = self.readbytes(data, size=8, unsigned=True)
        self.GPS_DATA['numSat'] = self.readbytes(data, size=8, unsigned=True)
        self.GPS_DATA['lat'] = self.readbytes(data, size=32, unsigned=False)
        self.GPS_DATA['lon'] = self.readbytes(data, size=32, unsigned=False)
        self.GPS_DATA['alt'] = self.readbytes(data, size=16, unsigned=True)
        self.GPS_DATA['speed'] = self.readbytes(data, size=16, unsigned=True)
        self.GPS_DATA['ground_course'] = self.readbytes(data, size=16, unsigned=True)

        if self.INAV:
            self.GPS_DATA['hdop'] = self.readbytes(data, size=16, unsigned=True)

    def process_MSP_COMP_GPS(self, data):
        self.GPS_DATA['distanceToHome'] = self.readbytes(data, size=16, unsigned=True)
        self.GPS_DATA['directionToHome'] = self.readbytes(data, size=16, unsigned=True)
        self.GPS_DATA['update'] = self.readbytes(data, size=8, unsigned=True)

    def process_MSP_GPSSTATISTICS(self, data):
        self.GPS_DATA['messageDt'] = self.readbytes(data, size=16, unsigned=True)
        self.GPS_DATA['errors'] = self.readbytes(data, size=32, unsigned=True)
        self.GPS_DATA['timeouts'] = self.readbytes(data, size=32, unsigned=True)
        self.GPS_DATA['packetCount'] = self.readbytes(data, size=32, unsigned=True)
        self.GPS_DATA['hdop'] = self.readbytes(data, size=16, unsigned=True)
        self.GPS_DATA['eph'] = self.readbytes(data, size=16, unsigned=True)
        self.GPS_DATA['epv'] = self.readbytes(data, size=16, unsigned=True)

    def process_MSP_ATTITUDE(self, data):
        self.SENSOR_DATA['kinematics'][0] = self.readbytes(data, size=16, unsigned=False) / 10.0 # x
        self.SENSOR_DATA['kinematics'][1] = self.readbytes(data, size=16, unsigned=False) / 10.0 # y
        self.SENSOR_DATA['kinematics'][2] = self.readbytes(data, size=16, unsigned=False) # z

    def process_MSP_ALTITUDE(self, data):
        self.SENSOR_DATA['altitude'] = round((self.readbytes(data, size=32, unsigned=False) / 100.0), 2) # correct scale factor

    def process_MSP_SONAR(self, data):
        self.SENSOR_DATA['sonar'] = self.readbytes(data, size=32, unsigned=False)

    def process_MSP_ANALOG(self, data):
        self.ANALOG['voltage'] = self.readbytes(data, size=8, unsigned=True) / 10.0
        self.ANALOG['mAhdrawn'] = self.readbytes(data, size=16, unsigned=True)
        self.ANALOG['rssi'] = self.readbytes(data, size=16, unsigned=True) # 0-1023
        self.ANALOG['amperage'] = self.readbytes(data, size=16, unsigned=False) / 100 # A
        self.ANALOG['last_received_timestamp'] = int(time.time()) # why not monotonic? where is time synchronized?
        if not self.INAV:
            self.ANALOG['voltage'] = self.readbytes(data, size=16, unsigned=True) / 100
    
    def process_MSPV2_INAV_ANALOG(self, data):
        if self.INAV:
            tmp = self.readbytes(data, size=8, unsigned=True)
            self.ANALOG['battery_full_when_plugged_in'] = True if (tmp & 1) else False
            self.ANALOG['use_capacity_thresholds'] = True if ((tmp & 2) >> 1) else False
            self.ANALOG['battery_state'] = (tmp & 12) >> 2
            self.ANALOG['cell_count'] = (tmp & 0xF0) >> 4

            self.ANALOG['voltage'] = self.readbytes(data, size=16, unsigned=True) / 100
            self.ANALOG['amperage'] = self.readbytes(data, size=16, unsigned=True) / 100 # A
            self.ANALOG['power'] = self.readbytes(data, size=32, unsigned=True) / 100
            self.ANALOG['mAhdrawn'] = self.readbytes(data, size=32, unsigned=True)
            self.ANALOG['mWhdrawn'] = self.readbytes(data, size=32, unsigned=True)
            self.ANALOG['battery_remaining_capacity'] = self.readbytes(data, size=32, unsigned=True)
            self.ANALOG['battery_percentage'] = self.readbytes(data, size=8, unsigned=True)
            self.ANALOG['rssi'] = self.readbytes(data, size=16, unsigned=True) # 0-1023

            # TODO: update both BF and INAV variables
            self.BATTERY_STATE['cellCount'] = self.ANALOG['cell_count']

    def process_MSP_VOLTAGE_METERS(self, data):
        total_bytes_per_meter = (8+8)/8 # just to make it clear where it comes from...
        self.VOLTAGE_METERS = [{'id':self.readbytes(data, size=8, unsigned=True),
                                'voltage':self.readbytes(data, size=8, unsigned=True) / 10.0
                                } for _ in range(int(len(data) / total_bytes_per_meter))]

    def process_MSP_CURRENT_METERS(self, data):
        total_bytes_per_meter = (8+16+16)/8 # just to make it clear where it comes from...
        self.CURRENT_METERS = [{'id':self.readbytes(data, size=8, unsigned=True),
                                'mAhDrawn':self.readbytes(data, size=16, unsigned=True), # mAh
                                'amperage':self.readbytes(data, size=16, unsigned=True) / 1000 # A
                                } for _ in range(int(len(data) / total_bytes_per_meter))]

    def process_MSP_BATTERY_STATE(self, data):
        self.BATTERY_STATE['cellCount'] = self.readbytes(data, size=8, unsigned=True)
        self.BATTERY_STATE['capacity'] = self.readbytes(data, size=16, unsigned=True) # mAh
        # BATTERY_STATE.voltage = data.readU8() / 10.0; // V
        self.BATTERY_STATE['mAhDrawn'] = self.readbytes(data, size=16, unsigned=True) # mAh
        self.BATTERY_STATE['amperage'] = self.readbytes(data, size=16, unsigned=True) / 100 # A
        self.BATTERY_STATE['batteryState'] = self.readbytes(data, size=8, unsigned=True)
        self.BATTERY_STATE['voltage'] = self.readbytes(data, size=16, unsigned=True) / 100 # V

    def process_MSP_VOLTAGE_METER_CONFIG(self, data):
        self.VOLTAGE_METER_CONFIGS = []
        if self.INAV:
            voltageMeterConfig = {}
            voltageMeterConfig['vbatscale'] = self.readbytes(data, size=8, unsigned=True)/10
            self.VOLTAGE_METER_CONFIGS.append(voltageMeterConfig)
            self.BATTERY_CONFIG['vbatmincellvoltage'] = self.readbytes(data, size=8, unsigned=True)/10
            self.BATTERY_CONFIG['vbatmaxcellvoltage'] = self.readbytes(data, size=8, unsigned=True)/10
            self.BATTERY_CONFIG['vbatwarningcellvoltage'] = self.readbytes(data, size=8, unsigned=True)/10
        else:
            voltage_meter_count = self.readbytes(data, size=8, unsigned=True)

            for i in range(voltage_meter_count):
                subframe_length = self.readbytes(data, size=8, unsigned=True)
                if (subframe_length != 5):
                    for j in range(subframe_length):
                        self.readbytes(data, size=8, unsigned=True)
                else:
                    voltageMeterConfig = {}
                    voltageMeterConfig['id'] = self.readbytes(data, size=8, unsigned=True)
                    voltageMeterConfig['sensorType'] = self.readbytes(data, size=8, unsigned=True)
                    voltageMeterConfig['vbatscale'] = self.readbytes(data, size=8, unsigned=True)
                    voltageMeterConfig['vbatresdivval'] = self.readbytes(data, size=8, unsigned=True)
                    voltageMeterConfig['vbatresdivmultiplier'] = self.readbytes(data, size=8, unsigned=True)

                    self.VOLTAGE_METER_CONFIGS.append(voltageMeterConfig)

    def process_MSP_CURRENT_METER_CONFIG(self, data):
        self.CURRENT_METER_CONFIGS = []
        if self.INAV:
            currentMeterConfig = {}
            currentMeterConfig['scale'] = self.readbytes(data, size=16, unsigned=True)
            currentMeterConfig['offset'] = self.readbytes(data, size=16, unsigned=True)
            currentMeterConfig['sensorType'] = self.readbytes(data, size=8, unsigned=True)
            self.CURRENT_METER_CONFIGS.append(currentMeterConfig)
            self.BATTERY_CONFIG['capacity'] = self.readbytes(data, size=16, unsigned=True)
        else:
            current_meter_count = self.readbytes(data, size=8, unsigned=True)
            for i in range(current_meter_count):
                currentMeterConfig = {}
                subframe_length = self.readbytes(data, size=8, unsigned=True)

                if (subframe_length != 6):
                    for j in range(subframe_length):
                        self.readbytes(data, size=8, unsigned=True)
                else:
                    currentMeterConfig['id'] = self.readbytes(data, size=8, unsigned=True)
                    currentMeterConfig['sensorType'] = self.readbytes(data, size=8, unsigned=True)
                    currentMeterConfig['scale'] = self.readbytes(data, size=16, unsigned=False)
                    currentMeterConfig['offset'] = self.readbytes(data, size=16, unsigned=False)

                    self.CURRENT_METER_CONFIGS.append(currentMeterConfig)

    def process_MSP_BATTERY_CONFIG(self, data):
        self.BATTERY_CONFIG['vbatmincellvoltage'] = self.readbytes(data, size=8, unsigned=True) / 10 # 10-50
        self.BATTERY_CONFIG['vbatmaxcellvoltage'] = self.readbytes(data, size=8, unsigned=True) / 10 # 10-50
        self.BATTERY_CONFIG['vbatwarningcellvoltage'] = self.readbytes(data, size=8, unsigned=True) / 10 # 10-50
        self.BATTERY_CONFIG['capacity'] = self.readbytes(data, size=16, unsigned=True)
        self.BATTERY_CONFIG['voltageMeterSource'] = self.readbytes(data, size=8, unsigned=True)
        self.BATTERY_CONFIG['currentMeterSource'] = self.readbytes(data, size=8, unsigned=True)

        self.BATTERY_CONFIG['vbatmincellvoltage'] = self.readbytes(data, size=16, unsigned=True) / 100
        self.BATTERY_CONFIG['vbatmaxcellvoltage'] = self.readbytes(data, size=16, unsigned=True) / 100
        self.BATTERY_CONFIG['vbatwarningcellvoltage'] = self.readbytes(data, size=16, unsigned=True) / 100

    def process_MSP_RC_TUNING(self, data):
        self.RC_TUNING['RC_RATE'] = round((self.readbytes(data, size=8, unsigned=True) / 100.0), 2)
        self.RC_TUNING['RC_EXPO'] = round((self.readbytes(data, size=8, unsigned=True) / 100.0), 2)

        self.RC_TUNING['roll_pitch_rate'] = 0
        self.RC_TUNING['roll_rate'] = round((self.readbytes(data, size=8, unsigned=True) / 100.0), 2)
        self.RC_TUNING['pitch_rate'] = round((self.readbytes(data, size=8, unsigned=True) / 100.0), 2)

        self.RC_TUNING['yaw_rate'] = round((self.readbytes(data, size=8, unsigned=True) / 100.0), 2)
        self.RC_TUNING['dynamic_THR_PID'] = round((self.readbytes(data, size=8, unsigned=True) / 100.0), 2)
        self.RC_TUNING['throttle_MID'] = round((self.readbytes(data, size=8, unsigned=True) / 100.0), 2)
        self.RC_TUNING['throttle_EXPO'] = round((self.readbytes(data, size=8, unsigned=True) / 100.0), 2)

        self.RC_TUNING['dynamic_THR_breakpoint'] = self.readbytes(data, size=16, unsigned=True)

        self.RC_TUNING['RC_YAW_EXPO'] = round((self.readbytes(data, size=8, unsigned=True) / 100.0), 2)

        if not self.INAV:
            self.RC_TUNING['rcYawRate'] = round((self.readbytes(data, size=8, unsigned=True) / 100.0), 2)

            self.RC_TUNING['rcPitchRate'] = round((self.readbytes(data, size=8, unsigned=True) / 100.0), 2)
            self.RC_TUNING['RC_PITCH_EXPO'] = round((self.readbytes(data, size=8, unsigned=True) / 100.0), 2)

            self.RC_TUNING['throttleLimitType'] = self.readbytes(data, size=8, unsigned=True)
            self.RC_TUNING['throttleLimitPercent'] = self.readbytes(data, size=8, unsigned=True)

            if int("".join((self.CONFIG['apiVersion'].rsplit('.')))) >= 1420:
                self.RC_TUNING['roll_rate_limit'] = self.readbytes(data, size=16, unsigned=True)
                self.RC_TUNING['pitch_rate_limit'] = self.readbytes(data, size=16, unsigned=True)
                self.RC_TUNING['yaw_rate_limit'] = self.readbytes(data, size=16, unsigned=True)

    def process_MSP_PID(self, data):
        self.PIDs = [
            [
                self.readbytes(data, size=8, unsigned=True) for _ in range(3)
            ] 
            for _ in range(int(len(data)/3))
        ]

    def process_MSP2_PID(self, data):
        self.PIDs = [
            [
                self.readbytes(data, size=8, unsigned=True) for _ in range(4)
            ] 
            for _ in range(int(len(data)/4))
        ]

    def process_MSP_ARMING_CONFIG(self, data):
        self.ARMING_CONFIG['auto_disarm_delay'] = self.readbytes(data, size=8, unsigned=True)
        self.ARMING_CONFIG['disarm_kill_switch'] = self.readbytes(data, size=8, unsigned=True)
        if not self.INAV:
            self.ARMING_CONFIG['small_angle'] = self.readbytes(data, size=8, unsigned=True)

    def process_MSP_LOOP_TIME(self, data):
        if self.INAV:
            self.FC_CONFIG['loopTime'] = self.readbytes(data, size=16, unsigned=True)

    def process_MSP_MISC(self, data): # 22 bytes
        if self.INAV:
            self.MISC['midrc'] = self.RX_CONFIG['midrc'] = self.readbytes(data, size=16, unsigned=True)
            self.MISC['minthrottle'] = self.MOTOR_CONFIG['minthrottle'] = self.readbytes(data, size=16, unsigned=True) # 0-2000
            self.MISC['maxthrottle'] = self.MOTOR_CONFIG['maxthrottle'] = self.readbytes(data, size=16, unsigned=True) # 0-2000
            self.MISC['mincommand'] = self.MOTOR_CONFIG['mincommand'] = self.readbytes(data, size=16, unsigned=True) # 0-2000
            self.MISC['failsafe_throttle'] = self.readbytes(data, size=16, unsigned=True) # 1000-2000
            self.MISC['gps_type'] = self.GPS_CONFIG['provider'] = self.readbytes(data, size=8, unsigned=True)
            self.MISC['sensors_baudrate'] = self.MISC['gps_baudrate'] = self.readbytes(data, size=8, unsigned=True)
            self.MISC['gps_ubx_sbas'] = self.GPS_CONFIG['ublox_sbas'] = self.readbytes(data, size=8, unsigned=True)
            self.MISC['multiwiicurrentoutput'] = self.readbytes(data, size=8, unsigned=True)
            self.MISC['rssi_channel'] = self.RSSI_CONFIG['channel'] = self.readbytes(data, size=8, unsigned=True)
            self.MISC['placeholder2'] = self.readbytes(data, size=8, unsigned=True)

            self.COMPASS_CONFIG['mag_declination'] = self.readbytes(data, size=16, unsigned=False) / 100 # -18000-18000
            
            self.MISC['mag_declination'] = self.COMPASS_CONFIG['mag_declination']*10

            self.MISC['vbatscale'] = self.readbytes(data, size=8, unsigned=True) # 10-200
            self.MISC['vbatmincellvoltage'] = self.readbytes(data, size=8, unsigned=True) / 10 # 10-50
            self.MISC['vbatmaxcellvoltage'] = self.readbytes(data, size=8, unsigned=True) / 10 # 10-50
            self.MISC['vbatwarningcellvoltage'] = self.readbytes(data, size=8, unsigned=True) / 10 # 10-50

    def process_MSPV2_INAV_MISC(self, data):
        if self.INAV:
            self.MISC['midrc'] = self.RX_CONFIG['midrc'] = self.readbytes(data, size=16, unsigned=True)
            self.MISC['minthrottle'] = self.MOTOR_CONFIG['minthrottle'] = self.readbytes(data, size=16, unsigned=True) # 0-2000
            self.MISC['maxthrottle'] = self.MOTOR_CONFIG['maxthrottle'] = self.readbytes(data, size=16, unsigned=True) # 0-2000
            self.MISC['mincommand'] = self.MOTOR_CONFIG['mincommand'] = self.readbytes(data, size=16, unsigned=True) # 0-2000
            self.MISC['failsafe_throttle'] = self.readbytes(data, size=16, unsigned=True) # 1000-2000
            self.MISC['gps_type'] = self.GPS_CONFIG['provider'] = self.readbytes(data, size=8, unsigned=True)
            self.MISC['sensors_baudrate'] = self.MISC['gps_baudrate'] = self.readbytes(data, size=8, unsigned=True)
            self.MISC['gps_ubx_sbas'] = self.GPS_CONFIG['ublox_sbas'] = self.readbytes(data, size=8, unsigned=True)
            self.MISC['rssi_channel'] = self.RSSI_CONFIG['channel'] = self.readbytes(data, size=8, unsigned=True)

            self.MISC['mag_declination'] = self.readbytes(data, size=16, unsigned=False) / 10 # -18000-18000
            self.MISC['vbatscale'] = self.readbytes(data, size=16, unsigned=True)
            self.MISC['voltage_source'] = self.readbytes(data, size=8, unsigned=True)
            self.MISC['battery_cells'] = self.readbytes(data, size=8, unsigned=True)
            self.MISC['vbatdetectcellvoltage'] = self.readbytes(data, size=16, unsigned=True) / 100
            self.MISC['vbatmincellvoltage'] = self.readbytes(data, size=16, unsigned=True) / 100
            self.MISC['vbatmaxcellvoltage'] = self.readbytes(data, size=16, unsigned=True) / 100
            self.MISC['vbatwarningcellvoltage'] = self.readbytes(data, size=16, unsigned=True) / 100
            self.MISC['battery_capacity'] = self.readbytes(data, size=32, unsigned=True)
            self.MISC['battery_capacity_warning'] = self.readbytes(data, size=32, unsigned=True)
            self.MISC['battery_capacity_critical'] = self.readbytes(data, size=32, unsigned=True)
            self.MISC['battery_capacity_unit'] = 'mWh' if self.readbytes(data, size=8, unsigned=True) else 'mAh'

    def process_MSP_MOTOR_CONFIG(self, data):
        self.MOTOR_CONFIG['minthrottle'] = self.readbytes(data, size=16, unsigned=True) # 0-2000
        self.MOTOR_CONFIG['maxthrottle'] = self.readbytes(data, size=16, unsigned=True) # 0-2000
        self.MOTOR_CONFIG['mincommand'] = self.readbytes(data, size=16, unsigned=True) # 0-2000

        self.MOTOR_CONFIG['motor_count'] = self.readbytes(data, size=8, unsigned=True)
        self.MOTOR_CONFIG['motor_poles'] = self.readbytes(data, size=8, unsigned=True)
        self.MOTOR_CONFIG['use_dshot_telemetry'] = (self.readbytes(data, size=8, unsigned=True) != 0)
        self.MOTOR_CONFIG['use_esc_sensor'] = (self.readbytes(data, size=8, unsigned=True) != 0)

    def process_MSP_COMPASS_CONFIG(self, data):
        self.COMPASS_CONFIG['mag_declination'] = self.readbytes(data, size=16, unsigned=False) / 100 # -18000-18000

    def process_MSP_GPS_CONFIG(self, data):
        self.GPS_CONFIG['provider'] = self.readbytes(data, size=8, unsigned=True)
        self.GPS_CONFIG['ublox_sbas'] = self.readbytes(data, size=8, unsigned=True)
        
        self.GPS_CONFIG['auto_config'] = self.readbytes(data, size=8, unsigned=True)
        self.GPS_CONFIG['auto_baud'] = self.readbytes(data, size=8, unsigned=True)

    def process_MSP_GPS_RESCUE(self, data):
        self.GPS_RESCUE['angle']             = self.readbytes(data, size=16, unsigned=True)
        self.GPS_RESCUE['initialAltitudeM']  = self.readbytes(data, size=16, unsigned=True)
        self.GPS_RESCUE['descentDistanceM']  = self.readbytes(data, size=16, unsigned=True)
        self.GPS_RESCUE['rescueGroundspeed'] = self.readbytes(data, size=16, unsigned=True)
        self.GPS_RESCUE['throttleMin']       = self.readbytes(data, size=16, unsigned=True)
        self.GPS_RESCUE['throttleMax']       = self.readbytes(data, size=16, unsigned=True)
        self.GPS_RESCUE['throttleHover']     = self.readbytes(data, size=16, unsigned=True)
        self.GPS_RESCUE['sanityChecks']      = self.readbytes(data, size=8, unsigned=True)
        self.GPS_RESCUE['minSats']           = self.readbytes(data, size=8, unsigned=True)

    def process_MSP_RSSI_CONFIG(self, data):
        self.RSSI_CONFIG['channel'] = self.readbytes(data, size=8, unsigned=True)

    def process_MSP_MOTOR_3D_CONFIG(self, data):
        self.MOTOR_3D_CONFIG['deadband3d_low'] = self.readbytes(data, size=16, unsigned=True)
        self.MOTOR_3D_CONFIG['deadband3d_high'] = self.readbytes(data, size=16, unsigned=True)
        self.MOTOR_3D_CONFIG['neutral'] = self.readbytes(data, size=16, unsigned=True)

    def process_MSP_BOXNAMES(self, data):
        self.AUX_CONFIG = [] # empty the array as new data is coming in

        buff = ""
        for i in range(len(data)):
            char = self.readbytes(data, size=8, unsigned=True)
            if (char == 0x3B): # ; (delimeter char)
                self.AUX_CONFIG.append(buff) # convert bytes into ASCII and save as strings

                # empty buffer
                buff = ""
            else:
                buff += chr(char)

    def process_MSP_PIDNAMES(self, data):
        self.PIDNAMES = [] # empty the array as new data is coming in

        buff = ""
        for i in range(len(data)):
            char = self.readbytes(data, size=8, unsigned=True)
            if (char == 0x3B):  # ; (delimeter char)
                self.PIDNAMES.append(buff) # convert bytes into ASCII and save as strings

                # empty buffer
                buff = ""
            else:
                buff += chr(char)

    def process_MSP_BOXIDS(self, data):
        self.AUX_CONFIG_IDS = [] # empty the array as new data is coming in

        for i in range(len(data)):
            self.AUX_CONFIG_IDS.append(self.readbytes(data, size=8, unsigned=True))

    def process_MSP_SERVO_CONFIGURATIONS(self, data):
        self.SERVO_CONFIG = [] # empty the array as new data is coming in
        if (len(data) % 12 == 0):
            for i in range(0, len(data), 12):
                arr = {
                    'min':                      self.readbytes(data, size=16, unsigned=True),
                    'max':                      self.readbytes(data, size=16, unsigned=True),
                    'middle':                   self.readbytes(data, size=16, unsigned=True),
                    'rate':                     self.readbytes(data, size=8, unsigned=False),
                    'indexOfChannelToForward':  self.readbytes(data, size=8, unsigned=True),
                    'reversedInputSources':     self.readbytes(data, size=32, unsigned=True)
                }

                self.SERVO_CONFIG.append(arr)

    def process_MSP_RC_DEADBAND(self, data):
        self.RC_DEADBAND_CONFIG['deadband'] = self.readbytes(data, size=8, unsigned=True)
        self.RC_DEADBAND_CONFIG['yaw_deadband'] = self.readbytes(data, size=8, unsigned=True)
        self.RC_DEADBAND_CONFIG['alt_hold_deadband'] = self.readbytes(data, size=8, unsigned=True)

        self.RC_DEADBAND_CONFIG['deadband3d_throttle'] = self.readbytes(data, size=16, unsigned=True)

    def process_MSP_SENSOR_ALIGNMENT(self, data):
        self.SENSOR_ALIGNMENT['align_gyro'] = self.readbytes(data, size=8, unsigned=True)
        self.SENSOR_ALIGNMENT['align_acc'] = self.readbytes(data, size=8, unsigned=True)
        self.SENSOR_ALIGNMENT['align_mag'] = self.readbytes(data, size=8, unsigned=True)

        if self.INAV:
            self.SENSOR_ALIGNMENT['align_opflow'] = self.readbytes(data, size=8, unsigned=True)
        else:
            self.SENSOR_ALIGNMENT['gyro_detection_flags'] = self.readbytes(data, size=8, unsigned=True)
            self.SENSOR_ALIGNMENT['gyro_to_use'] = self.readbytes(data, size=8, unsigned=True)
            self.SENSOR_ALIGNMENT['gyro_1_align'] = self.readbytes(data, size=8, unsigned=True)
            self.SENSOR_ALIGNMENT['gyro_2_align'] = self.readbytes(data, size=8, unsigned=True)

    # def process_MSP_DISPLAYPORT(self, data):

    def process_MSP_SET_RAW_RC(self, data):
        logging.debug('RAW RC values updated')

    def process_MSP_SET_PID(self, data):
        logging.info('PID settings saved')

    def process_MSP_SET_RC_TUNING(self, data):
        logging.info('RC Tuning saved')

    def process_MSP_ACC_CALIBRATION(self, data):
        logging.info('Accel calibration executed')

    def process_MSP_MAG_CALIBRATION(self, data):
        logging.info('Mag calibration executed')

    def process_MSP_SET_MOTOR_CONFIG(self, data):
        logging.info('Motor Configuration saved')

    def process_MSP_SET_GPS_CONFIG(self, data):
        logging.info('GPS Configuration saved')

    def process_MSP_SET_RSSI_CONFIG(self, data):
        logging.info('RSSI Configuration saved')

    def process_MSP_SET_FEATURE_CONFIG(self, data):
        logging.info('Features saved')

    def process_MSP_SET_BEEPER_CONFIG(self, data):
        logging.info('Beeper Configuration saved')

    def process_MSP_RESET_CONF(self, data):
        logging.info('Settings Reset')

    def process_MSP_SELECT_SETTING(self, data):
        logging.info('Profile selected')

    def process_MSP_SET_SERVO_CONFIGURATION(self, data):
        logging.info('Servo Configuration saved')

    def process_MSP_EEPROM_WRITE(self, data):
        logging.info('Settings Saved in EEPROM')

    def process_MSP_SET_CURRENT_METER_CONFIG(self, data):
        logging.info('Amperage Settings saved')

    def process_MSP_SET_VOLTAGE_METER_CONFIG(self, data):
        logging.info('Voltage config saved')
        
    def process_MSP_DEBUG(self, data):
        for i in range(4):
            self.SENSOR_DATA['debug'][i] = self.readbytes(data, size=16, unsigned=False)

    def process_MSP2_INAV_DEBUG(self, data):
        for i in range(8):
            self.SENSOR_DATA['debug'][i] = self.readbytes(data, size=32, unsigned=False)

    def process_MSP_SET_MOTOR(self, data):
        logging.info('Motor Speeds Updated')

    def process_MSP_UID(self, data):
        for i in range(3):
            self.CONFIG['uid'][i] = self.readbytes(data, size=32, unsigned=True)
    
    def process_MSP_ACC_TRIM(self, data):
        self.CONFIG['accelerometerTrims'][0] = self.readbytes(data, size=16, unsigned=False) # pitch
        self.CONFIG['accelerometerTrims'][1] = self.readbytes(data, size=16, unsigned=False) # roll

    def process_MSP_SET_ACC_TRIM(self, data):
        logging.info('Accelerometer trimms saved.')

    def process_MSP_GPS_SV_INFO(self, data):
        if (len(data) > 0):
            numCh = self.readbytes(data, size=8, unsigned=True)

            for i in range(numCh):
                self.GPS_DATA['chn'].append(self.readbytes(data, size=8, unsigned=True))
                self.GPS_DATA['svid'].append(self.readbytes(data, size=8, unsigned=True))
                self.GPS_DATA['quality'].append(self.readbytes(data, size=8, unsigned=True))
                self.GPS_DATA['cno'].append(self.readbytes(data, size=8, unsigned=True))

    def process_MSP_RX_MAP(self, data):
        self.RC_MAP = [] # empty the array as new data is coming in

        for i in range(len(data)):
            self.RC_MAP.append(self.readbytes(data, size=8, unsigned=True))

    def process_MSP_SET_RX_MAP(self, data):
        logging.debug('RCMAP saved')
        
    def process_MSP_MIXER_CONFIG(self, data):
        self.MIXER_CONFIG['mixer'] = self.readbytes(data, size=8, unsigned=True)
        if not self.INAV:                    
            self.MIXER_CONFIG['reverseMotorDir'] = self.readbytes(data, size=8, unsigned=True)

    def process_MSP_FEATURE_CONFIG(self, data):
        self.FEATURE_CONFIG['featuremask']  = self.readbytes(data, size=32, unsigned=True)
        for idx in range(32):
            enabled = self.bit_check(self.FEATURE_CONFIG['featuremask'], idx)
            if idx in self.FEATURE_CONFIG['features'].keys():
                self.FEATURE_CONFIG['features'][idx]['enabled'] = enabled
            else:
                self.FEATURE_CONFIG['features'][idx] = {'enabled': enabled}

    def process_MSP_BEEPER_CONFIG(self, data):
        self.BEEPER_CONFIG['beepers'] = self.readbytes(data, size=32, unsigned=True)
            
        self.BEEPER_CONFIG['dshotBeaconTone'] = self.readbytes(data, size=8, unsigned=True)

        self.BEEPER_CONFIG['dshotBeaconConditions'] = self.readbytes(data, size=32, unsigned=True)

    def process_MSP_BOARD_ALIGNMENT_CONFIG(self, data):
        self.BOARD_ALIGNMENT_CONFIG['roll'] = self.readbytes(data, size=16, unsigned=False) # -180 - 360
        self.BOARD_ALIGNMENT_CONFIG['pitch'] = self.readbytes(data, size=16, unsigned=False) # -180 - 360
        self.BOARD_ALIGNMENT_CONFIG['yaw'] = self.readbytes(data, size=16, unsigned=False) # -180 - 360

    def process_MSP_SET_REBOOT(self, data):
        rebootType = self.readbytes(data, size=8, unsigned=True)

        if ((rebootType == self.REBOOT_TYPES['MSC']) or (rebootType == self.REBOOT_TYPES['MSC_UTC'])):
            if (self.readbytes(data, size=8, unsigned=True) == 0):
                logging.warning('Storage device not ready for reboot.')

        logging.info('Reboot request accepted')

    def process_MSP_API_VERSION(self, data):
        self.CONFIG['mspProtocolVersion'] = self.readbytes(data, size=8, unsigned=True)
        self.CONFIG['apiVersion'] = str(self.readbytes(data, size=8, unsigned=True)) + '.' + str(self.readbytes(data, size=8, unsigned=True)) + '.0'

    def process_MSP_FC_VARIANT(self, data):
        identifier = ''
        for i in range(4):
            identifier += chr(self.readbytes(data, size=8, unsigned=True))
        self.CONFIG['flightControllerIdentifier'] = identifier

    def process_MSP_FC_VERSION(self, data):
        self.CONFIG['flightControllerVersion'] =  str(self.readbytes(data, size=8, unsigned=True)) + '.'
        self.CONFIG['flightControllerVersion'] += str(self.readbytes(data, size=8, unsigned=True)) + '.'
        self.CONFIG['flightControllerVersion'] += str(self.readbytes(data, size=8, unsigned=True))

    def process_MSP_BUILD_INFO(self, data):
        dateLength = 11
        buff = []
        for i in range(dateLength):
            buff.append(self.readbytes(data, size=8, unsigned=True))
        
        buff.append(32) # ascii space

        timeLength = 8
        for i in range(timeLength):
            buff.append(self.readbytes(data, size=8, unsigned=True))

        self.CONFIG['buildInfo'] = bytearray(buff).decode("utf-8")

    def process_MSP_BOARD_INFO(self, data):
        identifier = ''
        for i in range(4):
            identifier += chr(self.readbytes(data, size=8, unsigned=True))

        self.CONFIG['boardIdentifier'] = identifier
        self.CONFIG['boardVersion'] = self.readbytes(data, size=16, unsigned=True)

        self.CONFIG['boardType'] = self.readbytes(data, size=8, unsigned=True)

        self.CONFIG['targetName'] = ""

        self.CONFIG['commCapabilities'] = self.readbytes(data, size=8, unsigned=True)

        length = self.readbytes(data, size=8, unsigned=True)
        
        for i in range(length):
            self.CONFIG['targetName'] += chr(self.readbytes(data, size=8, unsigned=True))

        self.CONFIG['boardName'] = ""
        self.CONFIG['manufacturerId'] = ""
        self.CONFIG['signature'] = []
        self.CONFIG['boardName'] = ""
        self.CONFIG['mcuTypeId'] = ""

        if data:
            length = self.readbytes(data, size=8, unsigned=True)
            for i in range(length):
                self.CONFIG['boardName'] += chr(self.readbytes(data, size=8, unsigned=True))

            length = self.readbytes(data, size=8, unsigned=True)
            for i in range(length):
                self.CONFIG['manufacturerId'] += chr(self.readbytes(data, size=8, unsigned=True))

            for i in range(MSPy.SIGNATURE_LENGTH):
                self.CONFIG['signature'].append(self.readbytes(data, size=8, unsigned=True))

            self.CONFIG['mcuTypeId'] = self.readbytes(data, size=8, unsigned=True)

    def process_MSP_NAME(self, data):
        self.CONFIG['name'] = ''
    
        while len(data)>0:
            char = self.readbytes(data, size=8, unsigned=True)
            self.CONFIG['name'] += chr(char)

    # def process_MSP_SET_CHANNEL_FORWARDING(self, data):
    #     logging.info('Channel forwarding saved')

    def process_MSP_CF_SERIAL_CONFIG(self, data):
        self.SERIAL_CONFIG['ports'] = []
        bytesPerPort = 1 + 2 + (1 * 4)
        serialPortCount = int(len(data) / bytesPerPort)

        for i in range(serialPortCount):
            serialPort = {
                'identifier': self.readbytes(data, size=8, unsigned=True),
                'functions': self.serialPortFunctionMaskToFunctions(self.readbytes(data, size=16, unsigned=True)),
                'msp_baudrate': self.BAUD_RATES[self.readbytes(data, size=8, unsigned=True)],
                'gps_baudrate': self.BAUD_RATES[self.readbytes(data, size=8, unsigned=True)],
                'telemetry_baudrate': self.BAUD_RATES[self.readbytes(data, size=8, unsigned=True)],
                'blackbox_baudrate': self.BAUD_RATES[self.readbytes(data, size=8, unsigned=True)]
            }

            self.SERIAL_CONFIG['ports'].append(serialPort)

    def process_MSP_SET_CF_SERIAL_CONFIG(self, data):
        logging.info('Serial config saved')

    def process_MSP_MODE_RANGES(self, data):
        self.MODE_RANGES = [] # empty the array as new data is coming in

        modeRangeCount = int(len(data) / 4) # 4 bytes per item.

        for i in range(modeRangeCount):
            modeRange = {
                'id': self.readbytes(data, size=8, unsigned=True),
                'auxChannelIndex': self.readbytes(data, size=8, unsigned=True),
                'range': {
                    'start': 900 + (self.readbytes(data, size=8, unsigned=True) * 25),
                    'end': 900 + (self.readbytes(data, size=8, unsigned=True) * 25)
                            }
                }
            self.MODE_RANGES.append(modeRange)

    def process_MSP_MODE_RANGES_EXTRA(self, data):
        self.MODE_RANGES_EXTRA = [] # empty the array as new data is coming in

        modeRangeExtraCount = self.readbytes(data, size=8, unsigned=True)

        for i in range(modeRangeExtraCount):
            modeRangeExtra = {
                'id': self.readbytes(data, size=8, unsigned=True),
                'modeLogic': self.readbytes(data, size=8, unsigned=True),
                'linkedTo': self.readbytes(data, size=8, unsigned=True)
            }
            self.MODE_RANGES_EXTRA.append(modeRangeExtra)

    def process_MSP_ADJUSTMENT_RANGES(self, data):
        self.ADJUSTMENT_RANGES = [] # empty the array as new data is coming in

        adjustmentRangeCount = int(len(data) / 6) # 6 bytes per item.

        for i in range(adjustmentRangeCount):
            adjustmentRange = {
                'slotIndex': self.readbytes(data, size=8, unsigned=True),
                'auxChannelIndex': self.readbytes(data, size=8, unsigned=True),
                'range': {
                    'start': 900 + (self.readbytes(data, size=8, unsigned=True) * 25),
                    'end': 900 + (self.readbytes(data, size=8, unsigned=True) * 25)
                },
                'adjustmentFunction': self.readbytes(data, size=8, unsigned=True),
                'auxSwitchChannelIndex': self.readbytes(data, size=8, unsigned=True)
            }
            self.ADJUSTMENT_RANGES.append(adjustmentRange)

    def process_MSP_RX_CONFIG(self, data):
        self.RX_CONFIG['serialrx_provider'] = self.readbytes(data, size=8, unsigned=True)
        # maxcheck for INAV
        self.RX_CONFIG['stick_max'] = self.readbytes(data, size=16, unsigned=True)
        # midrc for INAV
        self.RX_CONFIG['stick_center'] = self.readbytes(data, size=16, unsigned=True)
        # mincheck for INAV
        self.RX_CONFIG['stick_min'] = self.readbytes(data, size=16, unsigned=True)
        self.RX_CONFIG['spektrum_sat_bind'] = self.readbytes(data, size=8, unsigned=True)
        self.RX_CONFIG['rx_min_usec'] = self.readbytes(data, size=16, unsigned=True)
        self.RX_CONFIG['rx_max_usec'] = self.readbytes(data, size=16, unsigned=True)
        self.RX_CONFIG['rcInterpolation'] = self.readbytes(data, size=8, unsigned=True)
        self.RX_CONFIG['rcInterpolationInterval'] = self.readbytes(data, size=8, unsigned=True)
        self.RX_CONFIG['airModeActivateThreshold'] = self.readbytes(data, size=16, unsigned=True)  
        # spirx_protocol for INAV
        self.RX_CONFIG['rxSpiProtocol'] = self.readbytes(data, size=8, unsigned=True)
        # spirx_id for INAV
        self.RX_CONFIG['rxSpiId'] = self.readbytes(data, size=32, unsigned=True)
        # spirx_channel_count for INAV
        self.RX_CONFIG['rxSpiRfChannelCount'] = self.readbytes(data, size=8, unsigned=True)
        self.RX_CONFIG['fpvCamAngleDegrees'] = self.readbytes(data, size=8, unsigned=True)
        if self.INAV:
            self.RX_CONFIG['receiver_type'] = self.readbytes(data, size=8, unsigned=True)
        else:
            self.RX_CONFIG['rcInterpolationChannels'] = self.readbytes(data, size=8, unsigned=True)
            self.RX_CONFIG['rcSmoothingType'] = self.readbytes(data, size=8, unsigned=True)
            self.RX_CONFIG['rcSmoothingInputCutoff'] = self.readbytes(data, size=8, unsigned=True)
            self.RX_CONFIG['rcSmoothingDerivativeCutoff'] = self.readbytes(data, size=8, unsigned=True)
            self.RX_CONFIG['rcSmoothingInputType'] = self.readbytes(data, size=8, unsigned=True)
            self.RX_CONFIG['rcSmoothingDerivativeType'] = self.readbytes(data, size=8, unsigned=True)
    def process_MSP_FAILSAFE_CONFIG(self, data):
        self.FAILSAFE_CONFIG['failsafe_delay'] = self.readbytes(data, size=8, unsigned=True)
        self.FAILSAFE_CONFIG['failsafe_off_delay'] = self.readbytes(data, size=8, unsigned=True)
        self.FAILSAFE_CONFIG['failsafe_throttle'] = self.readbytes(data, size=16, unsigned=True)
        self.FAILSAFE_CONFIG['failsafe_switch_mode'] = self.readbytes(data, size=8, unsigned=True)
        self.FAILSAFE_CONFIG['failsafe_throttle_low_delay'] = self.readbytes(data, size=16, unsigned=True)
        self.FAILSAFE_CONFIG['failsafe_procedure'] = self.readbytes(data, size=8, unsigned=True)

    def process_MSP_RXFAIL_CONFIG(self, data):
        self.RXFAIL_CONFIG = [] # empty the array as new data is coming in

        channelCount = int(len(data) / 3)
        for i in range(channelCount):
            rxfailChannel = {
                'mode':  self.readbytes(data, size=8, unsigned=True),
                'value': self.readbytes(data, size=16, unsigned=True)
            }
            self.RXFAIL_CONFIG.append(rxfailChannel)

    def process_MSP_ADVANCED_CONFIG(self, data):
        self.PID_ADVANCED_CONFIG['gyro_sync_denom'] = self.readbytes(data, size=8, unsigned=True)
        self.PID_ADVANCED_CONFIG['pid_process_denom'] = self.readbytes(data, size=8, unsigned=True)
        self.PID_ADVANCED_CONFIG['use_unsyncedPwm'] = self.readbytes(data, size=8, unsigned=True)
        self.PID_ADVANCED_CONFIG['fast_pwm_protocol'] = self.readbytes(data, size=8, unsigned=True)
        self.PID_ADVANCED_CONFIG['motor_pwm_rate'] = self.readbytes(data, size=16, unsigned=True)

        self.PID_ADVANCED_CONFIG['digitalIdlePercent'] = self.readbytes(data, size=16, unsigned=True) / 100

    def process_MSP_FILTER_CONFIG(self, data):
        self.FILTER_CONFIG['gyro_lowpass_hz'] = self.readbytes(data, size=8, unsigned=True)
        self.FILTER_CONFIG['dterm_lowpass_hz'] = self.readbytes(data, size=16, unsigned=True)
        self.FILTER_CONFIG['yaw_lowpass_hz'] = self.readbytes(data, size=16, unsigned=True)
        
        self.FILTER_CONFIG['gyro_notch_hz'] = self.readbytes(data, size=16, unsigned=True)
        self.FILTER_CONFIG['gyro_notch_cutoff'] = self.readbytes(data, size=16, unsigned=True)
        self.FILTER_CONFIG['dterm_notch_hz'] = self.readbytes(data, size=16, unsigned=True)
        self.FILTER_CONFIG['dterm_notch_cutoff'] = self.readbytes(data, size=16, unsigned=True)

        self.FILTER_CONFIG['gyro_notch2_hz'] = self.readbytes(data, size=16, unsigned=True)
        self.FILTER_CONFIG['gyro_notch2_cutoff'] = self.readbytes(data, size=16, unsigned=True)

        if not self.INAV:
            self.FILTER_CONFIG['dterm_lowpass_type'] = self.readbytes(data, size=8, unsigned=True)

            self.FILTER_CONFIG['gyro_hardware_lpf'] = self.readbytes(data, size=8, unsigned=True)
            
            self.readbytes(data, size=8, unsigned=True) # must consume this byte

            self.FILTER_CONFIG['gyro_lowpass_hz'] = self.readbytes(data, size=16, unsigned=True)
            self.FILTER_CONFIG['gyro_lowpass2_hz'] = self.readbytes(data, size=16, unsigned=True)
            self.FILTER_CONFIG['gyro_lowpass_type'] = self.readbytes(data, size=8, unsigned=True)
            self.FILTER_CONFIG['gyro_lowpass2_type'] = self.readbytes(data, size=8, unsigned=True)
            self.FILTER_CONFIG['dterm_lowpass2_hz'] = self.readbytes(data, size=16, unsigned=True)

            self.FILTER_CONFIG['gyro_32khz_hardware_lpf'] = 0

            self.FILTER_CONFIG['dterm_lowpass2_type'] = self.readbytes(data, size=8, unsigned=True)
            self.FILTER_CONFIG['gyro_lowpass_dyn_min_hz'] = self.readbytes(data, size=16, unsigned=True)
            self.FILTER_CONFIG['gyro_lowpass_dyn_max_hz'] = self.readbytes(data, size=16, unsigned=True)
            self.FILTER_CONFIG['dterm_lowpass_dyn_min_hz'] = self.readbytes(data, size=16, unsigned=True)
            self.FILTER_CONFIG['dterm_lowpass_dyn_max_hz'] = self.readbytes(data, size=16, unsigned=True)
        else:
            self.FILTER_CONFIG['accNotchHz'] = self.readbytes(data, size=16, unsigned=True)
            self.FILTER_CONFIG['accNotchCutoff'] = self.readbytes(data, size=16, unsigned=True)
            self.FILTER_CONFIG['gyroStage2LowpassHz'] = self.readbytes(data, size=16, unsigned=True)

    def process_MSP_SET_PID_ADVANCED(self, data):
        logging.info("Advanced PID settings saved")

    def process_MSP_PID_ADVANCED(self, data):
        self.ADVANCED_TUNING['rollPitchItermIgnoreRate'] = self.readbytes(data, size=16, unsigned=True)
        self.ADVANCED_TUNING['yawItermIgnoreRate'] = self.readbytes(data, size=16, unsigned=True)
        self.ADVANCED_TUNING['yaw_p_limit'] = self.readbytes(data, size=16, unsigned=True)
        self.ADVANCED_TUNING['deltaMethod'] = self.readbytes(data, size=8, unsigned=True)
        self.ADVANCED_TUNING['vbatPidCompensation'] = self.readbytes(data, size=8, unsigned=True)
        if not self.INAV:
            self.ADVANCED_TUNING['feedforwardTransition'] = self.readbytes(data, size=8, unsigned=True)

            self.ADVANCED_TUNING['dtermSetpointWeight'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['toleranceBand'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['toleranceBandReduction'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['itermThrottleGain'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['pidMaxVelocity'] = self.readbytes(data, size=16, unsigned=True)
            self.ADVANCED_TUNING['pidMaxVelocityYaw'] = self.readbytes(data, size=16, unsigned=True)

            self.ADVANCED_TUNING['levelAngleLimit'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['levelSensitivity'] = self.readbytes(data, size=8, unsigned=True)

            self.ADVANCED_TUNING['itermThrottleThreshold'] = self.readbytes(data, size=16, unsigned=True)
            self.ADVANCED_TUNING['itermAcceleratorGain'] = self.readbytes(data, size=16, unsigned=True)

            self.ADVANCED_TUNING['dtermSetpointWeight'] = self.readbytes(data, size=16, unsigned=True)

            self.ADVANCED_TUNING['itermRotation'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['smartFeedforward'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['itermRelax'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['itermRelaxType'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['absoluteControlGain'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['throttleBoost'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['acroTrainerAngleLimit'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['feedforwardRoll']  = self.readbytes(data, size=16, unsigned=True)
            self.ADVANCED_TUNING['feedforwardPitch'] = self.readbytes(data, size=16, unsigned=True)
            self.ADVANCED_TUNING['feedforwardYaw']   = self.readbytes(data, size=16, unsigned=True)
            self.ADVANCED_TUNING['antiGravityMode']  = self.readbytes(data, size=8, unsigned=True)

            self.ADVANCED_TUNING['dMinRoll'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['dMinPitch'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['dMinYaw'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['dMinGain'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['dMinAdvance'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['useIntegratedYaw'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['integratedYawRelax'] = self.readbytes(data, size=8, unsigned=True)
        else:
            self.ADVANCED_TUNING['setpointRelaxRatio'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['dtermSetpointWeight'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['pidSumLimit'] = self.readbytes(data, size=16, unsigned=True)
            self.ADVANCED_TUNING['itermThrottleGain'] = self.readbytes(data, size=8, unsigned=True)
            self.ADVANCED_TUNING['axisAccelerationLimitRollPitch'] = self.readbytes(data, size=16, unsigned=True)
            self.ADVANCED_TUNING['axisAccelerationLimitYaw'] = self.readbytes(data, size=16, unsigned=True)

    def process_MSP_SENSOR_CONFIG(self, data):
        self.SENSOR_CONFIG['acc_hardware'] = self.readbytes(data, size=8, unsigned=True)
        self.SENSOR_CONFIG['baro_hardware'] = self.readbytes(data, size=8, unsigned=True)
        self.SENSOR_CONFIG['mag_hardware'] = self.readbytes(data, size=8, unsigned=True)
        if self.INAV:
            self.SENSOR_CONFIG['pitot'] = self.readbytes(data, size=8, unsigned=True)
            self.SENSOR_CONFIG['rangefinder'] = self.readbytes(data, size=8, unsigned=True)
            self.SENSOR_CONFIG['opflow'] = self.readbytes(data, size=8, unsigned=True)

    def process_MSP_DATAFLASH_SUMMARY(self, data):
        flags = self.readbytes(data, size=8, unsigned=True)
        self.DATAFLASH['ready'] = ((flags & 1) != 0)
        self.DATAFLASH['supported'] = ((flags & 2) != 0)
        self.DATAFLASH['sectors'] = self.readbytes(data, size=32, unsigned=True)
        self.DATAFLASH['totalSize'] = self.readbytes(data, size=32, unsigned=True)
        self.DATAFLASH['usedSize'] = self.readbytes(data, size=32, unsigned=True)
        # update_dataflash_global();

    def process_MSP_DATAFLASH_ERASE(self, data):
        logging.info("Data flash erase begun...")

    def process_MSP_SDCARD_SUMMARY(self, data):
        flags = self.readbytes(data, size=8, unsigned=True)

        self.SDCARD['supported'] = ((flags & 0x01) != 0)
        self.SDCARD['state'] = self.readbytes(data, size=8, unsigned=True)
        self.SDCARD['filesystemLastError'] = self.readbytes(data, size=8, unsigned=True)
        self.SDCARD['freeSizeKB'] = self.readbytes(data, size=32, unsigned=True)
        self.SDCARD['totalSizeKB'] = self.readbytes(data, size=32, unsigned=True)

    def process_MSP_BLACKBOX_CONFIG(self, data):
        if not self.INAV:
            self.BLACKBOX['supported'] = (self.readbytes(data, size=8, unsigned=True) & 1) != 0
            self.BLACKBOX['blackboxDevice'] = self.readbytes(data, size=8, unsigned=True)
            self.BLACKBOX['blackboxRateNum'] = self.readbytes(data, size=8, unsigned=True)
            self.BLACKBOX['blackboxRateDenom'] = self.readbytes(data, size=8, unsigned=True)

            self.BLACKBOX['blackboxPDenom'] = self.readbytes(data, size=16, unsigned=True)
        else:
            pass # API no longer supported (INAV 2.3.0)
    def process_MSP_SET_BLACKBOX_CONFIG(self, data):
        logging.info("Blackbox config saved")

    # TODO: This changed and it will need to check the BF version to decode things correctly
    # def process_MSP_TRANSPONDER_CONFIG(self, data):
    #     bytesRemaining = len(data)

    #     providerCount = self.readbytes(data, size=8, unsigned=True)
    #     bytesRemaining-=1

    #     self.TRANSPONDER['supported'] = providerCount > 0
    #     self.TRANSPONDER['providers'] = []

    #     for i in range(providerCount):
    #         provider = {
    #             'id': self.readbytes(data, size=8, unsigned=True),
    #             'dataLength': self.readbytes(data, size=8, unsigned=True)
    #         }
    #         bytesRemaining -= 2

    #         self.TRANSPONDER['providers'].append(provider)

    #     self.TRANSPONDER['provider'] = self.readbytes(data, size=8, unsigned=True)
    #     bytesRemaining-=1

    #     self.TRANSPONDER['data'] = []
    #     for i in range(bytesRemaining):
    #         self.TRANSPONDER['data'].append(self.readbytes(data, size=8, unsigned=True))

    def process_MSP_SET_TRANSPONDER_CONFIG(self, data):
        logging.info("Transponder config saved")

    def process_MSP_SET_MODE_RANGE(self, data):
        logging.info('Mode range saved')

    def process_MSP_SET_ADJUSTMENT_RANGE(self, data):
        logging.info('Adjustment range saved')
        
    def process_MSP_SET_BOARD_ALIGNMENT_CONFIG(self, data):
        logging.info('Board alignment saved')
        
    def process_MSP_PID_CONTROLLER(self, data):
        self.PID['controller'] = self.readbytes(data, size=8, unsigned=True)
        
    def process_MSP_SET_PID_CONTROLLER(self, data):
        logging.info('PID controller changed')
        
    def process_MSP_SET_LOOP_TIME(self, data):
        logging.info('Looptime saved')
        
    def process_MSP_SET_ARMING_CONFIG(self, data):
        logging.info('Arming config saved')
        
    def process_MSP_SET_RESET_CURR_PID(self, data):
        logging.info('Current PID profile reset')
        
    def process_MSP_SET_MOTOR_3D_CONFIG(self, data):
        logging.info('3D settings saved')
        
    def process_MSP_SET_MIXER_CONFIG(self, data):
        logging.info('Mixer config saved')
        
    def process_MSP_SET_RC_DEADBAND(self, data):
        logging.info('Rc controls settings saved')
        
    def process_MSP_SET_SENSOR_ALIGNMENT(self, data):
        logging.info('Sensor alignment saved')
        
    def process_MSP_SET_RX_CONFIG(self, data):
        logging.info('Rx config saved')
        
    def process_MSP_SET_RXFAIL_CONFIG(self, data):
        logging.info('Rxfail config saved')
        
    def process_MSP_SET_FAILSAFE_CONFIG(self, data):
        logging.info('Failsafe config saved')
        
    def process_MSP_OSD_CONFIG(self, data):
        logging.info('OSD_CONFIG received')
        
    def process_MSP_SET_OSD_CONFIG(self, data):
        logging.info('OSD config set')
        
    def process_MSP_OSD_CHAR_READ(self, data):
        logging.info('OSD char received')
        
    def process_MSP_OSD_CHAR_WRITE(self, data):
        logging.info('OSD char uploaded')
        
    def process_MSP_VTX_CONFIG(self, data):
        logging.info('VTX_CONFIG received')
        
    def process_MSP_SET_VTX_CONFIG(self, data):
        logging.info('VTX_CONFIG set')
        
    def process_MSP_SET_NAME(self, data):
        logging.info('Name set')
        
    def process_MSP_SET_FILTER_CONFIG(self, data):
        logging.info('Filter config set')
        
    def process_MSP_SET_ADVANCED_CONFIG(self, data):
        logging.info('Advanced config parameters set')
        
    def process_MSP_SET_SENSOR_CONFIG(self, data):
        logging.info('Sensor config parameters set')
        
    def process_MSP_COPY_PROFILE(self, data):
        logging.info('Copy profile')
        
    def process_MSP_ARMING_DISABLE(self, data):
        logging.info('Arming disable')
        
    def process_MSP_SET_RTC(self, data):
        logging.info('Real time clock set')