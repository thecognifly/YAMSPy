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
from os import path, kill, getppid
from signal import SIGINT

import serial

from . import msp_ctrl
from . import msp_codes


def TCPServer(pipe, HOST, PORT, timeout=1/10000, time2sleep=0):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # to avoid "Address already in use" when the port is actually free
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # https://access.redhat.com/documentation/en-us/red_hat_enterprise_linux_for_real_time/7/html/tuning_guide/tcp_nodelay_and_small_buffer_writes
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
                    _,_,_ = select([conn],[],[conn],1)  # wait for data, timeout is for closed connections
                    try:
                        recvbuffer = conn.recv(buffersize) # read all buffer
                        if not recvbuffer:
                            logging.warning(f"[{PORT}] Empty recvbuffer!")
                            conn.close() # no data for SOCK_STREAM = dead
                    except ConnectionResetError:
                        logging.warning(f"[{PORT}] ConnectionResetError!")
                        conn.close() # no data for SOCK_STREAM = dead
                        recvbuffer = b''
                    except socket.timeout:
                        logging.warning(f"[{PORT}] socket.timeout!")
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

                message_number = 0
                while conn.fileno()>0:
                    sleep(time2sleep)
                    tic = monotonic()
                    # This is a slow operation... but it's needed to know
                    # where a message starts / ends, and it's useful for debugging
                    message_number += 1
                    pc2fc, raw_bytes = msp_ctrl.receive_msg(receive, logging, output_raw_bytes=True) # from PC
                    if pc2fc['pending'] == 1:
                        pc2fc, _raw_bytes = msp_ctrl.receive_msg(receive, logging, pc2fc, output_raw_bytes=True) # from PC
                        raw_bytes += _raw_bytes
                    if not raw_bytes:
                        break
                    logging.debug(f"[{PORT}] msp_ctrl.receive_msg time: {1000*(monotonic()-tic)}ms")
                    logging.debug(f"[{PORT}] {msp_codes.MSPCodes2Str[pc2fc['code']]} message_direction={'FC2PC' if pc2fc['message_direction'] else 'PC2FC'}, payload={len(pc2fc['dataView'])}, packet_error={pc2fc['packet_error']}")
                    if pc2fc['packet_error'] != 0:
                        logging.error(f"[{PORT}] packet_error receiving from TCP ({message_number})!!!!")
                    if 'MSP2_SENSOR' in msp_codes.MSPCodes2Str[pc2fc['code']]:
                        pipe.send([PORT,raw_bytes,False]) # to FC (proxy)
                    else:
                        pipe.send([PORT,raw_bytes,True]) # to FC (proxy)
                        if pipe.poll(timeout=1): # this timeout should only occur when connection is closed
                            raw_bytes = pipe.recv() # from FC (proxy)
                            res = 0
                            try:
                                res = send(raw_bytes) # to PC
                            finally:
                                if res:
                                    logging.debug(f"[{PORT}] RAW message sent to PC: {raw_bytes}")
                                else:
                                    break

                logging.warning(f"[{PORT}] Connection closed!")

                


def main(ports, device, baudrate, timeout=1/1000, min_time_between_writes=1/100):
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

    # to avoid getting stuck with select
    def check_serial_port():
        while path.exists(device):
            sleep(1)
        kill(getppid(), SIGINT) # will raise the KeyboardInterrupt
    serial_test_thread = Process(target=check_serial_port)
    serial_test_thread.daemon = True
    serial_test_thread.start()
    
    message_number = 0
    last_write = monotonic()
    while path.exists(device):
        pipes,_,_ = select(local_pipes,[],[sconn])
        pipe_local = pipes[0]
        current_write = monotonic()
        if (current_write-last_write) < min_time_between_writes:
            sleep(max(min_time_between_writes-(current_write-last_write),0))
            current_write = monotonic()
        last_write = current_write
        PORT, raw_bytes, get_reply = pipe_local.recv() # from PC
        server_thread, _, pipe_thread, HOST = servers[PORT]
        if server_thread.is_alive():
            res = 0
            while res == 0 and len(raw_bytes): # raw_bytes will never be 0 length
                try:
                    res = sconn.write(raw_bytes) # to FC (serial port)
                    # sconn.flush()
                except serial.SerialTimeoutException:
                    logging.error(f"[MAIN-{PORT}] RAW message {raw_bytes} was not sent to FC (SerialTimeoutException)!")
            message_number += 1
            logging.debug(f"[MAIN-{PORT}] RAW message ({message_number}) sent to FC: {raw_bytes}")
            
            if get_reply:
                # Check for a response from the FC
                tic = monotonic()
                fc2pc, raw_bytes = msp_ctrl.receive_msg(ser_read, logging, output_raw_bytes=True) # from FC (serial port)
                if fc2pc['pending'] == 1:
                    fc2pc, _raw_bytes = msp_ctrl.receive_msg(ser_read, logging, fc2pc, output_raw_bytes=True) # from FC (rest of the pending message)
                    raw_bytes += _raw_bytes
                logging.debug(f"[MAIN-{PORT}] msp_ctrl.receive_msg time: {1000*(monotonic()-tic)}ms")
                logging.debug(f"[MAIN-{PORT}] {msp_codes.MSPCodes2Str[fc2pc['code']]} message_direction={'FC2PC' if fc2pc['message_direction'] else 'PC2FC'}, payload={len(fc2pc['dataView'])}, packet_error={fc2pc['packet_error']}")
                if fc2pc['packet_error'] != 0:
                    logging.error(f"[MAIN-{PORT}] packet_error receiving from serial port ({msp_codes.MSPCodes2Str[fc2pc['code']]} - {message_number})!!!!")
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

    parser.add_argument('--min_time_between_writes', type=float,  default=1/100,
                        help='Minimum time between serial writes...')


    args = parser.parse_args()

    if args.debug:
        logmode = 'DEBUG'
    else:
        logmode = 'INFO'

    logging.basicConfig(format="[%(levelname)s] [%(asctime)s]: %(message)s",
                        level=getattr(logging, logmode),
                        stream=sys.stdout)
    
    try:
        main(args.ports, args.serial, args.baudrate, timeout=1/1000, min_time_between_writes=args.min_time_between_writes)
    except KeyboardInterrupt:
        print("\nBye!")
