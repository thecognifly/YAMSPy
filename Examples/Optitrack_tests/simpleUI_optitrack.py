"""simpleUI.py: Simple UI (toy one really) to test YAMSPy using a SSH connection.

Copyright (C) 2020 Ricardo de Azambuja

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


WARNING:
This UI is not fit for flying your drone, it is only useful for testing stuff in a safe place, ok?

Acknowledgement:
This work was possible thanks to the financial support from IVADO.ca (postdoctoral scholarship 2019/2020).

Disclaimer (adapted from Wikipedia):
None of the authors, contributors, supervisors, administrators, employers, friends, family, vandals, or anyone else 
connected (or not) with this project, in any way whatsoever, can be made responsible for your use of the information (code) 
contained or linked from here.


TODO:
1) The integrator makes it too hard to control things (it winds up). Probably a non-linear thing would be helpful here...
2) Everything is far from optimal so it could be improved... but it's a simpleUI example after all ;)
"""

__author__ = "Ricardo de Azambuja"
__copyright__ = "Copyright 2020, MISTLab.ca"
__credits__ = [""]
__license__ = "GPL"
__version__ = "0.0.1"
__maintainer__ = "Ricardo de Azambuja"
__email__ = "ricardo.azambuja@gmail.com"
__status__ = "Development"

import traceback
import time
import curses
from collections import deque
from itertools import cycle

from yamspy import MSPy

from OptiTrackPython_CogniFly import CogniFlyNatNetClient


# Max periods for:
CTRL_LOOP_TIME = 1/100
SLOW_MSGS_LOOP_TIME = 1/5 # these messages take a lot of time slowing down the loop...

NO_OF_CYCLES_AVERAGE_GUI_TIME = 10


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


#
# Stuff to be changed...
#

SERIAL_PORT = "/dev/ttyS0"
#SERIAL_PORT = "/dev/ttyACM0"
CLIENT_IP = "192.168.0.101"
FAKE_MOCAP = 0#[0,10,20,30,40] # if different from none, it will override the optitrack values
SIM_FLOW_TOF = True # when true, it will send (start) fake optical flow (always zero) and tof (always 2.5cm) MSP2 messages
# 'w' and 'e' will turn on/off on the fly



TAKEOFF_THROTTLE = 1300
LAND_THROTTLE = 900
TAKEOFF_LOOP_TIME = 1/10
TAKEOFF_TOTAL_TIME = 5
TAKEOFF_STEP = (TAKEOFF_THROTTLE-LAND_THROTTLE)*(1/TAKEOFF_LOOP_TIME)/TAKEOFF_TOTAL_TIME
DATA = [0,0,0,0,0]
OPTITRACK_LOOP_TIME = 1/5
OPTITRACK_DEBUG_LOOP_TIME = 1/2

def run_curses(external_function):
    result=1

    try:
        # get the curses screen window
        screen = curses.initscr()

        # turn off input echoing
        curses.noecho()

        # respond to keys immediately (don't wait for enter)
        curses.cbreak()

        # non-blocking
        screen.timeout(0)

        # map arrow keys to special values
        screen.keypad(True)

        screen.addstr(1, 0, "Press 'q' to quit, 'r' to reboot, 'm' to change mode, 'a' to arm, 'd' to disarm and arrow keys to control", curses.A_BOLD)
        
        result = external_function(screen)

    finally:
        # shut down cleanly
        curses.nocbreak(); screen.keypad(0); curses.echo()
        curses.endwin()
        if result==1:
            print("An error occurred... probably the serial port is not available ;)")

def keyboard_controller(screen):
    debug_optitrack = [0,0,0,0,0,0,0,0]
    mocap_data = [0,0,0,0,0]
    sim_flow_tof = SIM_FLOW_TOF

    CMDS = {
            'roll':     1500,
            'pitch':    1500,
            'throttle': 900,
            'yaw':      1500,
            'aux1':     1000,
            'aux2':     1000
            }

    # This order is the important bit: it will depend on how your flight controller is configured.
    # Below it is considering the flight controller is set to use AETR.
    # The names here don't really matter, they just need to match what is used for the CMDS dictionary.
    # In the documentation, iNAV uses CH5, CH6, etc while Betaflight goes AUX1, AUX2...
    CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']

    # "print" doesn't work with curses, use addstr instead
    result = 1
    try:
        screen.addstr(15, 0, "Starting OptiTrack...")
        # This will start a thread, but the RPI0 has only one core. Will this thread be too much for it???
        # Check how much the main loop slows down compared to the original simpleUI...
        screen.addstr(15, 0, "Starting OptiTrack... started!")
        screen.clrtoeol()
        screen.move(1,0)


        screen.addstr(15, 0, "Connecting to the FC...")

        with MSPy(device=SERIAL_PORT, loglevel='WARNING', baudrate=115200) as board, \
             CogniFlyNatNetClient(client_ip=CLIENT_IP) as CNNC:
            if board == 1: # an error occurred...
                exit

            screen.addstr(15, 0, "Connecting to the FC... connected!")
            screen.clrtoeol()
            screen.move(1,0)

            average_cycle = deque([0]*NO_OF_CYCLES_AVERAGE_GUI_TIME)

            # It's necessary to send some messages or the RX failsafe will be activated
            # and it will not be possible to arm.
            command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
                            'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
                            'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES']

            if board.INAV:
                command_list.append('MSPV2_INAV_ANALOG')
                command_list.append('MSP_VOLTAGE_METER_CONFIG')

            for msg in command_list: 
                if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                    dataHandler = board.receive_msg()
                    board.process_recv_data(dataHandler)
            if board.INAV:
                cellCount = board.BATTERY_STATE['cellCount']
            else:
                cellCount = 0 # MSPV2_INAV_ANALOG is necessary
            min_voltage = board.BATTERY_CONFIG['vbatmincellvoltage']*cellCount
            warn_voltage = board.BATTERY_CONFIG['vbatwarningcellvoltage']*cellCount
            max_voltage = board.BATTERY_CONFIG['vbatmaxcellvoltage']*cellCount

            screen.addstr(15, 0, "apiVersion: {}".format(board.CONFIG['apiVersion']))
            screen.clrtoeol()
            screen.addstr(15, 50, "flightControllerIdentifier: {}".format(board.CONFIG['flightControllerIdentifier']))
            screen.addstr(16, 0, "flightControllerVersion: {}".format(board.CONFIG['flightControllerVersion']))
            screen.addstr(16, 50, "boardIdentifier: {}".format(board.CONFIG['boardIdentifier']))
            screen.addstr(17, 0, "boardName: {}".format(board.CONFIG['boardName']))
            screen.addstr(17, 50, "name: {}".format(board.CONFIG['name']))


            slow_msgs = cycle(['MSP_ANALOG', 'MSP_STATUS_EX', 'MSP_MOTOR', 'MSP_RC'])

            takeoff = False
            optitrack = False
            cursor_msg = ""
            last_loop_time = last_slow_msg_time = last_cycleTime = last_takeoff_msg_time = last_optitrack_msg_time = last_optitrack_debug_msg_time  = prev_mocap_timestamp = time.time()
            while True:
                start_time = time.time()

                char = screen.getch() # get keypress
                curses.flushinp() # flushes buffer
                

                #
                # Key input processing
                #

                #
                # KEYS (NO DELAYS)
                #
                if char == ord('q') or char == ord('Q'):
                    print('done!')
                    time.sleep(0.5)
                    result=0
                    break

                elif char == ord('d') or char == ord('D'):
                    cursor_msg = 'Sending Disarm command...'
                    CMDS['aux1'] = 1000

                elif char == ord('r') or char == ord('R'):
                    screen.addstr(3, 0, 'Sending Reboot command...')
                    screen.clrtoeol()
                    board.reboot()
                    time.sleep(1)
                    result=0
                    break

                elif char == ord('o') or char == ord('o'):
                    cursor_msg = 'Enable OptiTrack System...'
                    optitrack = True

                elif char == ord('p') or char == ord('P'):
                    cursor_msg = 'DISable OptiTrack System...'
                    optitrack = False

                elif char == ord('a') or char == ord('A'):
                    cursor_msg = 'Sending Arm command...'
                    CMDS['aux1'] = 1800

                elif char == ord('t') or char == ord('T'):
                    CMDS['throttle'] = 1400
                    cursor_msg = 'takeoff throttle:{}'.format(CMDS['throttle'])
                    # takeoff = True
                    
                elif char == ord('l') or char == ord('L'):
                    CMDS['throttle'] = 900
                    cursor_msg = 'landing throttle:{}'.format(CMDS['throttle'])

                #
                # The code below is expecting the drone to have the
                # modes set accordingly since everything is hardcoded.
                #
                elif char == ord('m') or char == ord('M'):
                    if CMDS['aux2'] <= 1300:
                        cursor_msg = 'SURFACE + NAV ALTHOLD Mode...'
                        CMDS['aux2'] = 1500
                    elif 1700 > CMDS['aux2'] > 1300:
                        cursor_msg = 'SURFACE + NAS POSHOLD Mode...'
                        CMDS['aux2'] = 2000
                    elif CMDS['aux2'] >= 1700:
                        cursor_msg = 'ANGLE Mode...'
                        CMDS['aux2'] = 1000

                elif char == ord('w') or char == ord('W'):
                    CMDS['throttle'] = CMDS['throttle'] + 10 if CMDS['throttle'] + 10 <= 2000 else CMDS['throttle']
                    cursor_msg = 'W Key - throttle(+):{}'.format(CMDS['throttle'])
                    sim_flow_tof = False # turn OFF the flow+tof simulator

                elif char == ord('e') or char == ord('E'):
                    CMDS['throttle'] = CMDS['throttle'] - 10 if CMDS['throttle'] - 10 >= 1000 else CMDS['throttle']
                    cursor_msg = 'E Key - throttle(-):{}'.format(CMDS['throttle'])
                    sim_flow_tof = True # turn ON the flow+tof simulator

                elif char == curses.KEY_RIGHT:
                    CMDS['roll'] = CMDS['roll'] + 10 if CMDS['roll'] + 10 <= 2000 else CMDS['roll']
                    cursor_msg = 'Right Key - roll(-):{}'.format(CMDS['roll'])

                elif char == curses.KEY_LEFT:
                    CMDS['roll'] = CMDS['roll'] - 10 if CMDS['roll'] - 10 >= 1000 else CMDS['roll']
                    cursor_msg = 'Left Key - roll(+):{}'.format(CMDS['roll'])

                elif char == curses.KEY_UP:
                    CMDS['pitch'] = CMDS['pitch'] + 10 if CMDS['pitch'] + 10 <= 2000 else CMDS['pitch']
                    cursor_msg = 'Up Key - pitch(+):{}'.format(CMDS['pitch'])

                elif char == curses.KEY_DOWN:
                    CMDS['pitch'] = CMDS['pitch'] - 10 if CMDS['pitch'] - 10 >= 1000 else CMDS['pitch']
                    cursor_msg = 'Down Key - pitch(-):{}'.format(CMDS['pitch'])

                #
                # IMPORTANT MESSAGES (CTRL_LOOP_TIME based)
                #
                if (time.time()-last_loop_time) >= CTRL_LOOP_TIME:
                    last_loop_time = time.time()
                    # Send the RC channel values to the FC
                    if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)

                    # 
                    # Simulates the optical flow and tof sensors
                    #
                    if sim_flow_tof:
                        # Fake rangefinder [uint8 quality, int32 alt_mm]
                        if board.send_RAW_msg(0x1F01, data=[255]+board.convert([300],n=32)):
                            pass
                        else:
                            exit

                        # Fake optical flow [uint8 quality, int32 motionX, int32 motionY]
                        if board.send_RAW_msg(0x1F02, data=[255]+board.convert([0,0],n=32)):
                            pass
                        else:
                            exit


                #
                # TAKEOFF - to avoid a steep step
                #
                # if takeoff:
                #     if (time.time()-last_takeoff_msg_time) >= TAKEOFF_LOOP_TIME:
                #         last_takeoff_msg_time = time.time()
                #         if CMDS['throttle'] < TAKEOFF_THROTTLE:
                #             CMDS['throttle'] += TAKEOFF_STEP
                #             cursor_msg = 'Takeoff throttle:{}'.format(CMDS['throttle'])
                #         else:
                #             cursor_msg = 'Takeoff ended:{}'.format(CMDS['throttle'])
                #             takeoff = False

                #
                # OptiTrack Updates
                #
                if (time.time()-last_optitrack_msg_time) >= OPTITRACK_LOOP_TIME:
                    last_optitrack_msg_time = time.time()
                    if optitrack: # "o" enables and "p" disables
                        mocap_data, mocap_timestamp = CNNC.read_new_data("Carbon")
                        if FAKE_MOCAP:
                            DATA[0] = DATA[0] + 1
                            DATA[1] = FAKE_MOCAP[1]
                            DATA[2] = FAKE_MOCAP[2]
                            DATA[3] = FAKE_MOCAP[3]
                            DATA[4] = FAKE_MOCAP[4]
                        elif mocap_timestamp > prev_mocap_timestamp: # avoid sending the same value twice...
                            DATA[0] = DATA[0] + 1
                            DATA[1] = int(mocap_data[0]*100)
                            DATA[2] = int(mocap_data[1]*100)
                            DATA[3] = int(mocap_data[2]*100)
                            DATA[4] = int(mocap_data[3]*100)
                            prev_mocap_timestamp = mocap_timestamp

                        screen.addstr(12, 0, f"Optitrack values (DATA): {DATA}, mocap_timestamp:{mocap_timestamp}")
                        screen.clrtoeol()

                        Data = board.convert(DATA, 16)
                        if board.send_RAW_msg(30, Data):
                            dataHandler = board.receive_msg()
                            codestr = MSPy.MSPCodes2Str.get(dataHandler['code'])
                            if codestr:
                                screen.addstr(13, 0, f"Received >>> {codestr}")
                                screen.clrtoeol()
                                board.process_recv_data(dataHandler)
                        else:
                            screen.addstr(13, 0, "Received >>> NOTHING!")
                            screen.clrtoeol()

                if (time.time()-last_optitrack_debug_msg_time) >= OPTITRACK_DEBUG_LOOP_TIME:
                    last_optitrack_debug_msg_time = time.time()
                    if board.send_RAW_msg(MSPy.MSPCodes['MSP_DEBUG'], data=[]):
                        dataHandler = board.receive_msg()
                        codestr = MSPy.MSPCodes2Str.get(dataHandler['code'])
                        if codestr == 'MSP_DEBUG':
                            board.process_recv_data(dataHandler)
                            debug_optitrack = board.SENSOR_DATA['debug']
                            

                #
                # SLOW MSG processing (user GUI)
                #
                if (time.time()-last_slow_msg_time) >= SLOW_MSGS_LOOP_TIME:
                    last_slow_msg_time = time.time()

                    next_msg = next(slow_msgs) # circular list

                    # Read info from the FC
                    if board.send_RAW_msg(MSPy.MSPCodes[next_msg], data=[]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)
                        
                    if next_msg == 'MSP_ANALOG':
                        voltage = board.ANALOG['voltage']
                        voltage_msg = ""
                        if min_voltage < voltage <= warn_voltage:
                            voltage_msg = "LOW BATT WARNING"
                        elif voltage <= min_voltage:
                            voltage_msg = "ULTRA LOW BATT!!!"
                        elif voltage >= max_voltage:
                            voltage_msg = "VOLTAGE TOO HIGH"

                        screen.addstr(8, 0, "Battery Voltage: {:2.2f}V".format(board.ANALOG['voltage']))
                        screen.clrtoeol()
                        screen.addstr(8, 24, voltage_msg, curses.A_BOLD + curses.A_BLINK)
                        screen.clrtoeol()

                    elif next_msg == 'MSP_STATUS_EX':
                        ARMED = board.bit_check(board.CONFIG['mode'],0)
                        screen.addstr(5, 0, "ARMED: {}".format(ARMED), curses.A_BOLD)
                        screen.clrtoeol()

                        screen.addstr(5, 50, "armingDisableFlags: {}".format(board.process_armingDisableFlags(board.CONFIG['armingDisableFlags'])))
                        screen.clrtoeol()

                        screen.addstr(6, 0, "cpuload: {}".format(board.CONFIG['cpuload']))
                        screen.clrtoeol()
                        screen.addstr(6, 50, "cycleTime: {}".format(board.CONFIG['cycleTime']))
                        screen.clrtoeol()

                        screen.addstr(7, 0, "mode: {}".format(board.CONFIG['mode']))
                        screen.clrtoeol()

                        screen.addstr(7, 50, "Flight Mode: {}".format(board.process_mode(board.CONFIG['mode'])))
                        screen.clrtoeol()


                    elif next_msg == 'MSP_MOTOR':
                        screen.addstr(9, 0, "Motor Values: {}".format(board.MOTOR_DATA))
                        screen.clrtoeol()

                
                    elif next_msg == 'MSP_RC':
                        screen.addstr(10, 0, "RC Channels Values: {}".format(board.RC['channels']))
                        screen.clrtoeol()

                    screen.addstr(11, 0, "GUI cycleTime: {0:2.2f}ms (average {1:2.2f}Hz)".format((last_cycleTime)*1000,
                                                                                                1/(sum(average_cycle)/len(average_cycle))))
                    screen.clrtoeol()

                    screen.addstr(19, 0, "mocap fc: {}".format(debug_optitrack))
                    screen.clrtoeol()
                    screen.addstr(20, 0, "mocap op: {}".format(mocap_data))
                    screen.clrtoeol()

                    screen.addstr(3, 0, cursor_msg)
                    screen.clrtoeol()
                    

                end_time = time.time()
                last_cycleTime = end_time-start_time
                if (end_time-start_time)<CTRL_LOOP_TIME:
                    time.sleep(CTRL_LOOP_TIME-(end_time-start_time))
                    
                average_cycle.append(end_time-start_time)
                average_cycle.popleft()

    finally:
        screen.addstr(5, 0, "Disconneced from the FC!")
        screen.clrtoeol()
        time.sleep(1)
        print('done!')
        return result

if __name__ == "__main__":
    try:
        run_curses(keyboard_controller)
    except Exception as err:
        print(err)
        traceback.print_exc()
        