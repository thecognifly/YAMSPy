"""example.py: Just an example of how to use YAMSPy.

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

Acknowledgement:
This work was possible thanks to the financial support from IVADO.ca (postdoctoral scholarship 2019/2020).

Disclaimer (adapted from Wikipedia):
None of the authors, contributors, supervisors, administrators, employers, friends, family, vandals, or anyone else 
connected (or not) with this project, in any way whatsoever, can be made responsible for your use of the information (code) 
contained or linked from here.
"""

import sys
import time
import serial

from yamspy import MSPy

"""Usage example... and testing ;)
This simple example will send all possible MSP commands, but the ones from avoid_list.
"""

# This list has commands that may cause havok... 
# I may have missed something, so you were warned ;)
avoid_list = ['MSP_DATAFLASH_ERASE','MSP_DATAFLASH_READ','MSP_EEPROM_WRITE']

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
serial_port = "/dev/serial0"

# As you run this script, it will save a file MSPy.log with a very detailed info about all the 
# things sent and received through the serial port because of the argument loglevel='DEBUG'. 
with MSPy(device=serial_port, logfilename='MSPy.log', logfilemode='a', loglevel='DEBUG') as board:
    if board==1:
        print("An error ocurred... probably the serial port is not available ;)")
        sys.exit(1)

    #
    # The commands bellow will list / test all the possible messages implemented here
    # skipping the ones that SET or try to crazy stuff... 
    for msg in MSPy.MSPCodes.keys():
        if 'SET' not in msg:
            if msg not in avoid_list:
                if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                    print('{} >>> requested!'.format(msg))
                    dataHandler = board.receive_msg()
                    codestr = MSPy.MSPCodes2Str.get(dataHandler['code'])
                    if codestr:
                        print('Received >>> {}'.format(codestr))
                        board.process_recv_data(dataHandler)
                    elif dataHandler['packet_error'] == 1:
                        print('Received >>> ERROR!')

    print("armingDisableFlags: {}".format(board.process_armingDisableFlags(board.CONFIG['armingDisableFlags'])))

    if board.reboot():
        try:
            dataHandler = board.receive_msg()
            board.process_recv_data(dataHandler)
        except serial.SerialException:
            print("Board is rebooting")