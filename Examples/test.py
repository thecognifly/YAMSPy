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
serial_port = "/dev/ttyS0"

with MSPy(device=serial_port, loglevel='DEBUG', baudrate=115200) as board:
    # Read info from the FC
    # Please, pay attention to the way it works:
    # 1. Message is sent: MSP_ALTITUDE without any payload (data=[])
    if board.send_RAW_msg(MSPy.MSPCodes['MSP_ALTITUDE'], data=[]):
        # 2. Response msg from the flight controller is received
        dataHandler = board.receive_msg()
        # 3. The msg is parsed
        board.process_recv_data(dataHandler)
        # 4. After the parser, the instance is populated.
        # In this example, SENSOR_DATA has its altitude value updated.
        print(board.SENSOR_DATA['altitude'])

# For some msgs there are available specialized methods to read them faster:
# fast_read_altitude
# fast_read_imu
# fast_read_attitude
# fast_read_analog
# fast_msp_rc_cmd
#
# Notice they all start with "fast_" ;)