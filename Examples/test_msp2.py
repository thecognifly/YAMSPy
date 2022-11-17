import time

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
# serial_port = "/dev/ttyACM0"

if __name__ == '__main__':
    from argparse import ArgumentParser
    parser = ArgumentParser(description='Command line example.')
    parser.add_argument('--serialport', action='store', default="/dev/serial0", help='serial port')
    arguments = parser.parse_args()
    serial_port = arguments.serialport

    msg_list = ['MSP2_INAV_STATUS', 'MSP2_INAV_ANALOG']
    with MSPy(device=serial_port, loglevel='DEBUG', baudrate=115200) as board:
        while True:
            for msg in msg_list:
                prev = time.monotonic()
                # Read info from the FC
                # Please, pay attention to the way it works:
                # 1. Message is sent without any payload (data=[])
                if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                    # 2. Response msg from the flight controller is received
                    dataHandler = board.receive_msg()
                    # 3. The msg is parsed
                    # board.process_recv_data(dataHandler)
                    # 4. After the parser, the instance is populated.
                    print(dataHandler)
                print(f"{msg}: f={1/(time.monotonic()-prev):0.3}Hz")