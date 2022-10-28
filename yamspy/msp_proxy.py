"""MSP Proxy

This script should allow multiple apps to talk MSP to the same flight controller (FC).
To connect to the FC it needs to use a serial port.
All the other apps will connect through tcp using the address 127.0.0.1 (localhost) and
one port.
It would be useful if this same script could have the option to log all the communication
for debugging showing which app sent/received.

PROBLEM:
A naive implementation could simply broadcast the messages received from the FC to all apps,
and just forward any received information from the apps to the FC.
However, there are some caveats:
1. MSP messages can't be mixed or the state machine will fail.
2. The apps were written expecting they were the only thing connected to that port
therefore they may await for an acknowledgement from the FC.

SOLUTION:
1. Speak MSP, or at least understand when a message starts / ends
2. Always wait for a reply from the FC
3. Buffer other messages

Assumptions:
1. I am considering the scenario where the FC never sends a message without being asked.
2. The TCP server will use one server (port) per client

Listen to clients using one thread per client.
Process and buffer messages received from client.
First message to be processed until the end is sent to the FC. It will wait until the FC answers 
or timeout before it can send another message.



"""

import logging
import argparse
import socket
import sys
from time import sleep, monotonic
from threading import Lock
from multiprocessing import Process, Pipe

import serial

import msp_ctrl
import msp_codes


logging.basicConfig(format="[%(levelname)s] [%(asctime)s]: %(message)s",
                    level=getattr(logging, 'INFO'),
                    stream=sys.stdout)


def TCPServer(pipe, HOST, PORT, timeout=1/10000):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # to avoid "Address already in use" when the port is actually free
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.settimeout(timeout)
        #s.setblocking(False)
        s.bind((HOST, PORT))
        s.listen()
        while True:
            try:
                conn, addr = s.accept()
                conn.settimeout(timeout)
            except socket.timeout:
                sleep(timeout)
                continue
            
            with conn:
                print(f"Connected by {(HOST, PORT)}")
                def receive():
                    recvbuffer = b''
                    di = 0
                    while True:
                        try:
                            data = conn.recv(1)
                            logging.info(f"Socket ({addr}) received data {data}")
                            if data:
                                recvbuffer += data
                                di += 1
                            elif di==0:
                                conn.close() # no data for SOCK_STREAM = dead
                                break
                        except socket.timeout:
                            logging.info(f"Socket ({addr}) socket.timeout")
                            break
                    logging.debug(f"Socket ({addr}) returned recvbuffer {recvbuffer}")
                    return recvbuffer

                def send(msg):
                    sent = 0
                    try:
                        sent = conn.send(msg)
                    except BrokenPipeError:
                        logging.warning(f"Socket connection broken while sending {addr}")
                    return sent

                while conn.fileno()>0:
                    # this is a slow operation... but it's needed to know
                    # where a message starts / ends
                    tic = monotonic()
                    pc2fc = msp_ctrl.receive_msg(receive, logging)
                    logging.warning(f"[{PORT}] {msp_codes.MSPCodes2Str[pc2fc['code']]} message_direction={'FC2PC' if pc2fc['message_direction'] else 'PC2FC'}, payload={len(pc2fc['dataView'])}, packet_error={pc2fc['packet_error']}")
                    logging.warning(f"[{PORT}] msp_ctrl.receive_msg time: {1000*(monotonic()-tic)}ms")
                    if pc2fc['packet_error'] == 0:
                        pipe.send(pc2fc)
                        fc2pc = pipe.recv() # blocking
                        # process
                        bufView = msp_ctrl.prepare_RAW_msg(fc2pc['msp_version'], fc2pc['code'], fc2pc['dataView'])
                        res = 0
                        try:
                            res = send(bufView)
                        finally:
                            if res>0:
                                logging.debug(f"RAW message sent: {bufView}")
                            else:
                                break
                    else:
                        logging.warning(f"[{PORT}] packet_error!!!!")

                    sleep(1/1000)

                logging.warning(f"[{PORT}] Connection closed!")

                


def main(ports, device, baudrate, timeout=1/1000):
    try:
        sconn = serial.Serial(port = device, baudrate = baudrate,
                            bytesize = serial.EIGHTBITS, parity = serial.PARITY_NONE,
                            stopbits = serial.STOPBITS_ONE, timeout = timeout,
                            xonxoff = False, rtscts = False, dsrdtr = False, writeTimeout = timeout
                            )

        logging.info("Serial port open!")
    except serial.SerialException as err:
        logging.warning(f"Error opening the serial port ({device}): {err}")

    def ser_read():
        try:
            data = sconn.read(sconn.inWaiting())
            return data
        except serial.SerialTimeoutException as err:
            logging.warning(f"Error reading from the serial port ({device}): {err}")
            return []

    servers = []
    for p in ports:
        pipe_local, pipe_thread = Pipe()
        HOST = '127.0.0.1'
        PORT = p
        server_thread = Process(target=TCPServer, args=(pipe_thread, HOST, PORT, timeout))
        # Exit the server thread when the main thread terminates
        server_thread.daemon = True
        server_thread.start()
        logging.warning(f"Listening on port {PORT} in thread {server_thread.name}")
        servers.append([server_thread, pipe_local, pipe_thread, HOST, PORT])

    while True:
        for server_thread, pipe_local, pipe_thread, HOST, PORT in servers:
            if server_thread.is_alive():
                if pipe_local.poll():
                    pc2fc = pipe_local.recv()
                    # send this message to the FC
                    bufView = msp_ctrl.prepare_RAW_msg(pc2fc['msp_version'], pc2fc['code'], pc2fc['dataView'])
                    res = 0
                    try:
                        res = sconn.write(bufView)
                    finally:
                        if res>0:
                            logging.info(f"RAW message sent: {bufView}")
                        else:
                            raise RuntimeError("RAW message not sent")
                    # wait for a response from the FC
                    tic = monotonic()
                    fc2pc = msp_ctrl.receive_msg(ser_read, logging)
                    logging.warning(f"[MAIN] msp_ctrl.receive_msg time: {1000*(monotonic()-tic)}ms")
                    logging.warning(f"[MAIN] {msp_codes.MSPCodes2Str[fc2pc['code']]} message_direction={'FC2PC' if fc2pc['message_direction'] else 'PC2FC'}, payload={len(fc2pc['dataView'])}, packet_error={fc2pc['packet_error']}")
                    pipe_local.send(fc2pc)
            else:
                raise RuntimeError(f"Server for {HOST, PORT} died!")
        sleep(1/500)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='TCP server that acts as an MSP proxy.')

    parser.add_argument('--serial', type=str, nargs='*', default='/dev/ttyACM0',
                                    help='Serial port to connect to the FC.')

    parser.add_argument('--ports', type=int, nargs='+', required=True,
                                   help='TCP ports to use for each client (do not share!)')

    parser.add_argument('--baudrate', type=int,  default=115200,
                        help='Baudrate used by the serial connection...')

    parser.add_argument('--nice', type=int,
                        default=0,
                        help='Nice level (from -20 to 19, but negative numbers need sudo)')

    parser.add_argument('--string_choices', type=str, nargs='*', 
                        default=[], 
                        choices=['opt1', 'opt2', 'opt3'], 
                        help='help...')

    parser.add_argument("--debug", 
                        action='store_true', 
                        help="Debug mode.")


    args = parser.parse_args()


    main(args.ports, args.serial, args.baudrate, timeout=1/100)
