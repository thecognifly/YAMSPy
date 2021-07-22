import time
from yamspy import MSPy

serial_port = "/dev/ttyACM0"

with MSPy(device=serial_port, logfilename='MSPy.log', logfilemode='a', loglevel='DEBUG') as board:
    test_mocap = [1,2,3,4,5]
    data = board.convert(test_mocap, 16)

    try:
        while True:
            if board.send_RAW_msg(30, data):
                dataHandler = board.receive_msg()
                codestr = MSPy.MSPCodes2Str.get(dataHandler['code'])
                if codestr:
                    print('Received >>> {}'.format(codestr))
                    board.process_recv_data(dataHandler)
                elif dataHandler['packet_error'] == 1:
                    print('Received >>> ERROR!')
            time.sleep(.1) # 10Hz
    except KeyboardInterrupt:
        if board.reboot():
            try:
                dataHandler = board.receive_msg()
                board.process_recv_data(dataHandler)
            except serial.SerialException:
                print("Board is rebooting")   
