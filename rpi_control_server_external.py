import time
import asyncio
from concurrent.futures import TimeoutError
import signal
from itertools import cycle
import binascii

import sys

import numpy as np

from pybetaflight import pyBetaflight


BASE_VOLTAGE = 8.4 # used to compensate throttle

MSG_SIZE = 40

start = time.time()
"""
Message:
Client => Server
[timestamp, 'roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2'] => 28bytes using float32
"""

class UI_server():

    def __init__(self, board, freq = 50,
                       failsafe = 0.1, 
                       ip = '127.0.0.1', port = 8989):

        self.failsafe = failsafe
        self.ip = ip
        self.port = port

        self.min_period = 1/freq

        self.main_message_in = []

        self.slow_message_in = []
        self.slow_message_out = []

        self.board = board

        self.shutdown = False

        self.connected = 0 # it needs to be initialized here, or main_loop will crash

        self.handshake_message = np.zeros(int(MSG_SIZE/4), dtype=np.int8)
        self.handshake_message[0] = -1
        self.handshake_message[-1] = -1

        self.voltage = 0

        # It's necessary to send some messages or the RX failsafe will be active
        # and it will not be possible to arm.
        command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
                        'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
                        'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES', 'MSP_ANALOG']
        if self.board:
            for msg in command_list: 
                if self.board.send_RAW_msg(pyBetaflight.MSPCodes[msg], data=[]):
                    dataHandler = self.board.receive_msg()
                    self.board.process_recv_data(dataHandler)

            self.min_voltage = self.board.BATTERY_CONFIG['vbatmincellvoltage']*self.board.BATTERY_STATE['cellCount']
            self.warn_voltage = self.board.BATTERY_CONFIG['vbatwarningcellvoltage']*self.board.BATTERY_STATE['cellCount']
            self.max_voltage = self.board.BATTERY_CONFIG['vbatmaxcellvoltage']*self.board.BATTERY_STATE['cellCount']
            self.voltage = self.board.ANALOG['voltage']


    def tic(self):
        return 'at %1.6f seconds' % (time.time() - start)
        

    async def main_msg_server(self, reader, writer):
        """It will simply pass along the received messages from the client and ignore them if
        the timestamp is newer than the previous ones. It will also send to the client the messages
        received from the slow/main loops.
        
        The first value of every message is the timestamp. 
        """
        print("main_msg_server starting...")

        prev_time = time.time()
        prev_timestamp = 0
        self.connected = 0
        first_message = False
        while True and not self.shutdown:
            try:
                data = await reader.read(MSG_SIZE)
            except ConnectionResetError:
                break

            curr_time = time.time()
            if len(data):
                main_message_in = np.frombuffer(data,dtype=np.float32)
                if np.array_equal(self.handshake_message,main_message_in): # handshake
                    print("Handshake received...")
                    self.connected = curr_time
                    # possible future clock synchronization, but because it's asynchronous... no guarantees.
                    writer.write(np.asanyarray([curr_time], dtype=np.float32).tobytes())
                    await writer.drain()
                    first_message = True
                    continue
                elif self.connected:
                    if first_message:
                        prev_timestamp = curr_time
                        first_message = False
                        print("First message...", prev_timestamp)

                    if (curr_time - prev_timestamp) < self.failsafe:
                        # print("{} - Fast... {}".format(curr_time, (curr_time - prev_timestamp)))
                        self.connected = curr_time
                        prev_timestamp = curr_time #main_message_in[0]
                        self.main_message_in = main_message_in[:-1]
                        # Send back voltage level
                        writer.write(np.asanyarray([self.voltage], dtype=np.float32).tobytes())
                        # await writer.drain() # we don't care about this... so the comment
                    else:
                        print("Too slow...", (curr_time - prev_timestamp))
                        prev_timestamp = curr_time # resets...

            elif data == b'': # when the client closes
                break
                
            print('{} ({:2.2f}us - {:2.2f}Hz) - main_msg_server'.format(self.tic(), 1e6*(time.time()-curr_time), 1/(curr_time-prev_time)))
            prev_time = curr_time
        
        self.connected = 0 # indicates the connection was closed
        print("main_msg_server closing...")


    async def main_loop(self):
        """The main_loop is in charge of keeping things safe and keep the flight controller happy by
        sending commands mimicking a remote controller receiver (minimum frequency or the FC will failsafe).
    
        0) Keeps the Flight Controller alive, but make sure it's disarmed.
        """

        print("main_loop starting...")

        # Using MSP controller it's possible to have more auxiliary inputs than this.
        CMDS_init = {
                'roll':     1500,
                'pitch':    1500,
                'throttle': 1000,
                'yaw':      1500,
                'aux1':     1000, # DISARMED (1000) / ARMED (1800)
                'aux2':     1000, # ANGLE (1000) / HORIZON (1500) / FLIP (1800)
                'aux3':     1000, # FAILSAFE (1800)
                'aux4':     1000  # HEADFREE (1800)
                }

        CMDS = CMDS_init.copy()

        CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2', 'aux3', 'aux4']

        prev_time = time.time() 

        while True and not self.shutdown:
            curr_time = time.time()
            CMDS['roll'] = CMDS_init['roll']
            CMDS['pitch'] = CMDS_init['pitch']
            CMDS['yaw'] = CMDS_init['yaw']

            if len(self.main_message_in):
                CMDS['roll'] = self.main_message_in[1]
                CMDS['pitch'] = self.main_message_in[2]

                # # Voltage compensation
                # if self.voltage and self.main_message_in[3]>1000:
                #     # if the voltage is kept higher than 1000, it will not arm
                #     CMDS['throttle'] = (BASE_VOLTAGE/self.voltage)*self.main_message_in[3]
                #     # Here I could use an average value of the voltage to avoid crazy throttle variations
                #     # as the battery voltage swings...
                # else:
                #     CMDS['throttle'] = self.main_message_in[3]

                CMDS['throttle'] = self.main_message_in[3]
                

                CMDS['yaw'] = self.main_message_in[4]
                CMDS['aux1'] = self.main_message_in[5]
                CMDS['aux2'] = self.main_message_in[6]
                CMDS['aux3'] = self.main_message_in[7]
                CMDS['aux4'] = self.main_message_in[8]

                # erases the previous message
                self.main_message_in = []

            if not self.connected:
                CMDS['throttle'] = CMDS_init['throttle']
                CMDS['aux1'] = CMDS_init['aux1'] # ARM / DISARM
                CMDS['aux2'] = CMDS_init['aux2']
                CMDS['aux3'] = CMDS_init['aux3']
                CMDS['aux4'] = CMDS_init['aux4']

            if self.board:
                # The flight controller needs to receive a certain number of 
                # messages before one can arm it. Therefore, when there's no connection,
                # it will send the safe values
                if not self.connected:
                    print("Keep connection to flight controller alive...")
                    # Send the RC channel values to the FC without checking anything
                    self.board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER])
                    # Waits for the feedback from the FC to avoid overloading it
                    _ = self.board.receive_msg()
                else:
                    # When the client is connected, we want to make sure we only
                    # pass along messages if they respect the failsafe value leaving
                    # to the flight controller the task of actually activating its failsafe.
                    # self.connected has the time the previous message was received
                    if (curr_time - self.connected) < self.failsafe:
                        # Send the RC channel values to the FC without checking anything
                        self.board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER])
                        # Waits for the feedback from the FC only to avoid overloading it
                        self.board.receive_msg()

            print('{} ({:2.2f}us - {:2.2f}Hz) - main_loop'.format(self.tic(), 1e6*(time.time()-curr_time), 1/(curr_time-prev_time)))
            prev_time = curr_time

            await asyncio.sleep(self.min_period) # using sleep(self.min_period), 
                                                 # to avoid overloading the flight controller

        print("main_loop closing...")


    async def read_battery(self):
        dataReady = False
        while not self.shutdown:
            if self.board:
                if not dataReady:
                    if self.board.send_RAW_msg(pyBetaflight.MSPCodes['MSP_ANALOG'], data=[]):
                        dataHandler = self.board.receive_msg()
                        dataReady = True
                else:
                    self.board.process_recv_data(dataHandler)
                    self.voltage = self.board.ANALOG['voltage']
                    dataReady = False

                if self.voltage <= self.min_voltage:
                    print("LOW VOLTAGE")
                    self.shutdown = True

            await asyncio.sleep(1)


    async def ask_exit(self, signame, loop):
        print("Got signal %s: exit" % signame)
        self.shutdown = True
        await asyncio.sleep(0)


    async def main(self):
        ioloop = asyncio.get_event_loop()

        for sig in ['SIGHUP','SIGINT', 'SIGTERM']:
            ioloop.add_signal_handler(
                getattr(signal, sig),
                lambda s=sig: ioloop.create_task(self.ask_exit(s, ioloop)))


        # Use asyncio.gather to run two coroutines concurrently (if possible):
        await asyncio.gather(
            self.read_battery(),
            self.main_loop(),
            asyncio.start_server(self.main_msg_server, self.ip, self.port, loop=ioloop)
        )


    def run(self):
        ioloop = asyncio.get_event_loop()
        server1 = asyncio.start_server(self.main_msg_server, self.ip, self.port, loop=ioloop)
        tasks = [
            server1,
            ioloop.create_task(self.main_loop()),
            ioloop.create_task(self.read_battery())
        ]

        for sig in ['SIGHUP','SIGINT', 'SIGTERM']:
            ioloop.add_signal_handler(
                getattr(signal, sig),
                lambda s=sig: ioloop.create_task(self.ask_exit(s, ioloop)))

        ioloop.run_until_complete(asyncio.wait(tasks))
        ioloop.close()

if __name__ == '__main__':
    # with pyBetaflight(device="/dev/serial/by-id/usb-Betaflight_OmnibusF4_0x8000000-if00", loglevel='WARNING', baudrate=1000000) as board:
        board = None
        server = UI_server(board, ip="192.168.2.124")
        try:
            asyncio.run(server.main())
        except AttributeError:
            server.run()
        finally:
            if board:
                board.reboot()
            time.sleep(2)
