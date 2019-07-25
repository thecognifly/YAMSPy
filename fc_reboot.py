import time
from pybetaflight import pyBetaflight

if __name__ == '__main__':
    with MSPy(device="/dev/ttyACM0", loglevel='WARNING') as board:
        board.reboot()
        time.sleep(2)
