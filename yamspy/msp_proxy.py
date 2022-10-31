"""MSP Proxy

This script should allow multiple apps to talk MSP to the same flight controller (FC).
To connect to the FC it needs to use a serial port.
All the other apps will connect through tcp using the address 127.0.0.1 (localhost) and
one port.
It would be useful if this same script could have the option to log all the communication
for debugging showing which app sent/received.

$ python -m yamspy.msp_proxy --ports 54310 54320
"""

from curses import raw
import logging
import argparse
import socket
import sys
from time import sleep, monotonic
from threading import Lock
from multiprocessing import Process, Pipe
from select import select

import serial

from . import msp_ctrl
from . import msp_codes


def TCPServer(pipe, HOST, PORT, timeout=1/10000, time2sleep=0):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # to avoid "Address already in use" when the port is actually free
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        buffersize = s.getsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF)
        s.settimeout(False)
        s.setblocking(True)
        s.bind((HOST, PORT))
        s.listen()
        while True:
            conn, addr = s.accept()
            conn.settimeout(timeout)
            with conn:
                print(f"Connected by {(HOST, PORT)}")
                def receive():
                    logging.debug(f"[{PORT}] waiting for data to be received from PC...")
                    _,_,_ = select([conn],[],[])  # wait for data
                    try:
                        recvbuffer = conn.recv(buffersize) # read all buffer
                        if not recvbuffer:
                            conn.close() # no data for SOCK_STREAM = dead
                    except ConnectionResetError:
                        conn.close() # no data for SOCK_STREAM = dead
                        recvbuffer = b''
                    logging.debug(f"[{PORT}] Socket ({addr}) returned recvbuffer {recvbuffer}")
                    return recvbuffer

                def send(msg):
                    try:
                        conn.sendall(msg)
                    except BrokenPipeError:
                        logging.warning(f"Socket connection broken while sending {addr}")
                        return False
                    return True

                while conn.fileno()>0:
                    sleep(time2sleep)
                    tic = monotonic()
                    # This is a slow operation... but it's needed to know
                    # where a message starts / ends, and it's useful for debugging
                    pc2fc, raw_bytes = msp_ctrl.receive_msg(receive, logging, output_raw_bytes=True) # from PC
                    logging.debug(f"[{PORT}] msp_ctrl.receive_msg time: {1000*(monotonic()-tic)}ms")
                    logging.debug(f"[{PORT}] {msp_codes.MSPCodes2Str[pc2fc['code']]} message_direction={'FC2PC' if pc2fc['message_direction'] else 'PC2FC'}, payload={len(pc2fc['dataView'])}, packet_error={pc2fc['packet_error']}")
                    if pc2fc['packet_error'] != 0:
                        logging.error(f"[{PORT}] packet_error!!!!")
                        raw_bytes = b''
                    if raw_bytes:
                        pipe.send([PORT,raw_bytes]) # to FC (proxy)
                        raw_bytes = pipe.recv() # from FC (proxy)
                        if raw_bytes:
                            res = False
                            try:
                                res = send(raw_bytes) # to PC
                            finally:
                                if res:
                                    logging.debug(f"[{PORT}] RAW message sent to PC: {raw_bytes}")
                                else:
                                    break

                logging.warning(f"[{PORT}] Connection closed!")

                


def main(ports, device, baudrate, timeout=1/1000):
    time2sleep = len(ports)*timeout
    try:
        sconn = serial.Serial(port = device, baudrate = baudrate,
                            bytesize = serial.EIGHTBITS, parity = serial.PARITY_NONE,
                            stopbits = serial.STOPBITS_ONE, timeout = timeout,
                            xonxoff = False, rtscts = False, dsrdtr = False, writeTimeout = timeout
                            )

        logging.info("Serial port open!")
    except serial.SerialException as err:
        logging.error(f"Error opening the serial port {device}.\n{err}")
        exit(1)

    def ser_read():
        _,_,_ = select([sconn],[],[])  # wait for data
        data = b''
        data = sconn.read(sconn.inWaiting()) # blocking
        return data

    servers = {}
    for p in ports:
        pipe_local, pipe_thread = Pipe()
        HOST = '127.0.0.1'
        PORT = p
        server_thread = Process(target=TCPServer, args=(pipe_thread, HOST, PORT, timeout, time2sleep))
        # Exit the server thread when the main thread terminates
        server_thread.daemon = True
        server_thread.start()
        logging.warning(f"Listening on port {PORT} in thread {server_thread.name}")
        servers[PORT] = [server_thread, pipe_local, pipe_thread, HOST]

    local_pipes = [v[1]for v in servers.values()]
    while True:
        pipes,_,_ = select(local_pipes,[],[])
        pipe_local = pipes[0]
        PORT, raw_bytes = pipe_local.recv() # from PC
        server_thread, _, pipe_thread, HOST = servers[PORT]
        if server_thread.is_alive():
            res = 0
            try:
                res = sconn.write(raw_bytes) # to FC (serial port)
                if res>0:
                    logging.debug(f"[MAIN-{PORT}] RAW message sent to FC: {raw_bytes}")
                else:
                    logging.error(f"[MAIN-{PORT}] RAW message {raw_bytes} was not sent")
                    pipe_local.send(b'') # to PC (TCP)
                    continue
            except serial.SerialTimeoutException:
                logging.error(f"[MAIN-{PORT}] RAW message {raw_bytes} was not sent")
                pipe_local.send(b'') # to PC (TCP)
                continue

            # Check for a response from the FC
            tic = monotonic()
            fc2pc, raw_bytes = msp_ctrl.receive_msg(ser_read, logging, output_raw_bytes=True) # from FC (serial port)
            logging.debug(f"[MAIN-{PORT}] msp_ctrl.receive_msg time: {1000*(monotonic()-tic)}ms")
            logging.debug(f"[MAIN-{PORT}] {msp_codes.MSPCodes2Str[fc2pc['code']]} message_direction={'FC2PC' if fc2pc['message_direction'] else 'PC2FC'}, payload={len(fc2pc['dataView'])}, packet_error={fc2pc['packet_error']}")
            if fc2pc['packet_error'] != 0:
                logging.error(f"[MAIN-{PORT}] packet_error!!!!")
                raw_bytes = b''
            pipe_local.send(raw_bytes) # to PC (TCP)
        else:
            raise RuntimeError(f"Server for {HOST, PORT} died!")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='TCP server that acts as an MSP proxy.')

    parser.add_argument('--serial', type=str, nargs='*', default='/dev/ttyACM0',
                                    help='Serial port to connect to the FC.')

    parser.add_argument('--ports', type=int, nargs='+', default=[54310],
                                   help='TCP ports to use for each client (do not share!)')

    parser.add_argument('--baudrate', type=int,  default=115200,
                        help='Baudrate used by the serial connection...')

    # parser.add_argument('--nice', type=int,
    #                     default=0,
    #                     help='Nice level (from -20 to 19, but negative numbers need sudo)')

    # parser.add_argument('--string_choices', type=str, nargs='*', 
    #                     default=[], 
    #                     choices=['opt1', 'opt2', 'opt3'], 
    #                     help='help...')

    parser.add_argument("--debug", 
                        action='store_true', 
                        help="Debug mode.")


    args = parser.parse_args()

    if args.debug:
        logmode = 'DEBUG'
    else:
        logmode = 'INFO'

    logging.basicConfig(format="[%(levelname)s] [%(asctime)s]: %(message)s",
                        level=getattr(logging, logmode),
                        stream=sys.stdout)
    
    try:
        main(args.ports, args.serial, args.baudrate, timeout=1/1000)
    except KeyboardInterrupt:
        print("\nBye!")
