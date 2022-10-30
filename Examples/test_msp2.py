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
serial_port = "/dev/ttyACM0"

with MSPy(device=serial_port, loglevel='DEBUG', baudrate=115200) as board:
    while True:
        prev = time.monotonic()
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
            print(dataHandler)
        print(f"MSP2_INAV_STATUS: f={1/(time.monotonic()-prev):0.3}Hz")

        prev = time.monotonic()
        if board.send_RAW_msg(MSPy.MSPCodes['MSP2_INAV_ANALOG'], data=[]):
            # 2. Response msg from the flight controller is received
            dataHandler = board.receive_msg()
            # 3. The msg is parsed
            # board.process_recv_data(dataHandler)
            # 4. After the parser, the instance is populated.
            # In this example, SENSOR_DATA has its altitude value updated.
            print(dataHandler)
        print(f"MSP2_INAV_ANALOG: f={1/(time.monotonic()-prev):0.3}Hz")