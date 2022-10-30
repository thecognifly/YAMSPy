import time
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

def foo():
    with MSPy(device=serial_port, loglevel='DEBUG', baudrate=1152000) as board:
        for i in range(100):
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

            prev = time.monotonic()
            if board.send_RAW_msg(MSPy.MSPCodes['MSP2_INAV_ANALOG'], data=[]):
                # 2. Response msg from the flight controller is received
                dataHandler = board.receive_msg()
                # 3. The msg is parsed
                # board.process_recv_data(dataHandler)
                # 4. After the parser, the instance is populated.
                # In this example, SENSOR_DATA has its altitude value updated.

# profile.runcall(foo)
profile.run('foo()')
ps = pstats.Stats(profile)

ps.strip_dirs().sort_stats(SortKey.CUMULATIVE).print_stats()