import time
import asyncio
from concurrent.futures import TimeoutError
import signal
# from itertools import cycle
# import binascii

# import sys
import os

from threading import Lock

import numpy as np

import uvloop

from pybetaflight import pyBetaflight

import picamera
from picamera.array import PiMotionAnalysis, PiRGBArray

from Optical_Flow_Filter import Filter


BASE_VOLTAGE = 8.4 # used to compensate throttle

MSG_SIZE = 36

start = time.time()


DEBUG = False


class UI_server(PiMotionAnalysis):
    def __init__(self, camera, 
                       board = None, freq = 30,
                       failsafe = 0.1, 
                       ip = '127.0.0.1', port = 8989):
        
        super(UI_server, self).__init__(camera)

        self.filter = Filter(vtl_threshold = 0.6, 
                             diff_threshold = 0.3,
                             data_threshold = 5, 
                             altitude = 100)

        self.dz, self.dx, self.dy, self.gnd_x, self.gnd_y = 0, 0, 0, 0, 0

        self.lock_opticalflow = Lock()

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

        self.CMDS_roll = self.CMDS_pitch = 0.0

        self.prev_time = 0
        self.OPTFLOW_FLAG = False
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
        while not self.shutdown:
            try:
                data = await reader.read(1000*MSG_SIZE)
            except ConnectionResetError:
                break

            curr_time = time.time()
            if len(data):
                data = data[-MSG_SIZE:]
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
                        self.main_message_in = main_message_in[:]
                        # Send back voltage level
                        writer.write(np.asanyarray([self.voltage], dtype=np.float32).tobytes())
                        # await writer.drain() # we don't care about this... so the comment
                    else:
                        print("Too slow...", (curr_time - prev_timestamp))
                        prev_timestamp = curr_time # resets...

            elif data == b'': # when the client closes
                break
            if DEBUG:
                print('{} ({:2.2f}us - {:2.2f}Hz) - main_msg_server'.format(self.tic(), 1e6*(time.time()-curr_time), 1/(curr_time-prev_time)))
            prev_time = curr_time
        
        self.connected = 0 # indicates the connection was closed
        print("main_msg_server closing...")


    def analyze(self, a):
        if not self.shutdown:
            if self.lock_opticalflow.acquire(False):
                try:
                    self.dz, self.dx, self.dy, self.gnd_x, self.gnd_y = self.filter.run(a)
                    
                    # dy => -pitch
                    # dx => +roll

                    # if (time.time()-self.prev_time)>= self.min_period:
                    #     self.main_loop(optical_flow_corrections = (self.dz, self.dx, self.dy))
                    #     self.prev_time = time.time()

                finally:
                    self.lock_opticalflow.release()
                    self.OPTFLOW_FLAG = True


    def main_loop(self, optical_flow_corrections = (0,0,0)):
        """The main_loop is in charge of keeping things safe and keep the flight controller happy by
        sending commands mimicking a remote controller receiver (minimum frequency or the FC will failsafe).
    
        0) Keeps the Flight Controller alive, but make sure it's disarmed.
        """
        if not self.connected:
            # Using MSP controller it's possible to have more auxiliary inputs than this.
            self.CMDS_init = {
                    'roll':     1500,
                    'pitch':    1500,
                    'throttle': 1000,
                    'yaw':      1500,
                    'aux1':     1000, # DISARMED (1000) / ARMED (1800)
                    'aux2':     1000, # ANGLE (1000) / HORIZON (1500) / FLIP (1800)
                    'aux3':     1000, # FAILSAFE (1800)
                    'aux4':     1000  # HEADFREE (1800)
                    }

            self.CMDS = self.CMDS_init.copy()

        CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2', 'aux3', 'aux4']

        curr_time = time.time()
        self.CMDS['roll'] = self.CMDS_init['roll']
        self.CMDS['pitch'] = self.CMDS_init['pitch']
        self.CMDS['yaw'] = self.CMDS_init['yaw']

        if len(self.main_message_in):
            self.CMDS['roll'] = self.main_message_in[1]
            self.CMDS['pitch'] = self.main_message_in[2]

            # # Voltage compensation
            # if self.voltage and self.main_message_in[3]>1000:
            #     # if the voltage is kept higher than 1000, it will not arm
            #     CMDS['throttle'] = (BASE_VOLTAGE/self.voltage)*self.main_message_in[3]
            #     # Here I could use an average value of the voltage to avoid crazy throttle variations
            #     # as the battery voltage swings...
            # else:
            #     CMDS['throttle'] = self.main_message_in[3]

            self.CMDS['throttle'] = self.main_message_in[3]
            

            self.CMDS['yaw'] = self.main_message_in[4]
            self.CMDS['aux1'] = self.main_message_in[5]
            self.CMDS['aux2'] = self.main_message_in[6]
            self.CMDS['aux3'] = self.main_message_in[7]
            self.CMDS['aux4'] = self.main_message_in[8]

            # erases the previous message
            self.main_message_in = []

        if not self.connected:
            self.CMDS['throttle'] = self.CMDS_init['throttle']
            self.CMDS['aux1'] = self.CMDS_init['aux1'] # ARM / DISARM
            self.CMDS['aux2'] = self.CMDS_init['aux2']
            self.CMDS['aux3'] = self.CMDS_init['aux3']
            self.CMDS['aux4'] = self.CMDS_init['aux4']

        # Apply optical flow corrections:
        # dy => -pitch
        # dx => +roll
        OPTFLOW_THRS_Y = 10
        OPTFLOW_THRS_X = 10

        OPTFLOW_CORR_X = 30
        OPTFLOW_CORR_Y = 30

        OPTFLOW_ABSMAX_Y = 100
        OPTFLOW_ABSMAX_X = 100
        if self.OPTFLOW_FLAG:
            optical_flow_corrections = (self.dz, self.dx, self.dy)
            self.OPTFLOW_FLAG = False
        self.CMDS_pitch += +OPTFLOW_CORR_Y if optical_flow_corrections[2]>+OPTFLOW_THRS_Y else 0
        self.CMDS_pitch += -OPTFLOW_CORR_Y if optical_flow_corrections[2]<-OPTFLOW_THRS_Y else 0 
        self.CMDS_roll  += -OPTFLOW_CORR_X if optical_flow_corrections[1]>+OPTFLOW_THRS_X else 0
        self.CMDS_roll  += +OPTFLOW_CORR_X if optical_flow_corrections[1]<-OPTFLOW_THRS_X else 0

        if self.CMDS['aux1'] == self.CMDS_init['aux1'] or self.CMDS['throttle'] < 1550: # disarmed
            self.CMDS_pitch = self.CMDS_roll = 0

        self.CMDS_roll = self.CMDS_roll if abs(self.CMDS_roll) < OPTFLOW_ABSMAX_X else np.sign(self.CMDS_roll)*OPTFLOW_ABSMAX_X
        self.CMDS_pitch = self.CMDS_pitch if abs(self.CMDS_pitch) < OPTFLOW_ABSMAX_Y else np.sign(self.CMDS_pitch)*OPTFLOW_ABSMAX_Y

        self.CMDS['roll'] += self.CMDS_roll
        self.CMDS['pitch'] += self.CMDS_pitch

        
        if self.board:
            # The flight controller needs to receive a certain number of 
            # messages before one can arm it. Therefore, when there's no connection,
            # it will send the safe values
            if not self.connected:
                # print("Keep connection to flight controller alive...")
                # Send the RC channel values to the FC without checking anything
                self.board.send_RAW_RC([self.CMDS[ki] for ki in CMDS_ORDER])
                # Waits for the feedback from the FC to avoid overloading it
                self.board.receive_msg()
            else:
                # When the client is connected, we want to make sure we only
                # pass along messages if they respect the failsafe value leaving
                # to the flight controller the task of actually activating its failsafe.
                # self.connected has the time the previous message was received
                if (curr_time - self.connected) < self.failsafe:
                    # Send the RC channel values to the FC without checking anything
                    self.board.send_RAW_RC([self.CMDS[ki] for ki in CMDS_ORDER])
                    # Waits for the feedback from the FC only to avoid overloading it
                    self.board.receive_msg()

        # print(self.CMDS)
        if DEBUG:
            print('{} ({:2.2f}us - {:2.2f}Hz) - main_loop'.format(self.tic(), 1e6*(time.time()-curr_time), 1/(curr_time-self.prev_time)))

        # await asyncio.sleep(self.min_period) # using sleep(self.min_period), 
        #                                      # to avoid overloading the flight controller


    async def read_battery(self):
        print("read_battery starting...")

        dataReady = False
        while not self.shutdown:
            if self.board:
                if self.lock_opticalflow.acquire(False):
                    try:
                        if not dataReady:
                            if self.board.send_RAW_msg(pyBetaflight.MSPCodes['MSP_ANALOG'], data=[]):
                                dataHandler = self.board.receive_msg()
                                dataReady = True
                        else:
                            self.board.process_recv_data(dataHandler)
                            self.voltage = self.board.ANALOG['voltage']
                            dataReady = False
                    finally:
                        self.lock_opticalflow.release()

                if self.voltage <= self.min_voltage:
                    print("LOW VOLTAGE")
                    self.shutdown = True

            await asyncio.sleep(1)

        print("read_battery closing...")

    async def use_opticalflow(self):
        print("use_opticalflow starting...")
        while not self.shutdown:
            if self.lock_opticalflow.acquire(False):
                try:
                    dz_lc, dx_lc, dy_lc, gnd_x_lc, gnd_y_lc = self.dz, self.dx, self.dy, self.gnd_x, self.gnd_y
                finally:
                    self.lock_opticalflow.release()

            # print ("Up(1)/Donw(-1): %3f  |  dx = %3f  |  dy = %3f" % (dz, dx, dy))
            print (gnd_x_lc, gnd_y_lc)
            time.sleep(0.02)
            await asyncio.sleep(self.min_period)

        print("use_opticalflow closing...")

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

        asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())


        # Use asyncio.gather to run two coroutines concurrently (if possible):
        await asyncio.gather(
            # self.use_opticalflow(),
            self.read_battery(),
            asyncio.start_server(self.main_msg_server, self.ip, self.port, loop=ioloop)
        )


    def run(self):
        # ioloop = asyncio.get_event_loop()
        ioloop = uvloop.new_event_loop()
        asyncio.set_event_loop(ioloop)

        server1 = asyncio.start_server(self.main_msg_server, self.ip, self.port, loop=ioloop)
        tasks = [
            server1,
            # ioloop.create_task(self.use_opticalflow()),
            ioloop.create_task(self.read_battery())
        ]

        for sig in ['SIGHUP','SIGINT', 'SIGTERM']:
            ioloop.add_signal_handler(
                getattr(signal, sig),
                lambda s=sig: ioloop.create_task(self.ask_exit(s, ioloop)))

        ioloop.run_until_complete(asyncio.wait(tasks))
        ioloop.close()

if __name__ == '__main__':
    with pyBetaflight(device="/dev/serial/by-id/usb-Betaflight_OmnibusF4_0x8000000-if00", loglevel='WARNING', baudrate=1000000) as board:
        # board = None
        print("read_cam starting...")
        with picamera.PiCamera(resolution='VGA', framerate=60) as camera:
            camera.resolution = (640,480)
            with UI_server(camera, board, ip="192.168.2.124") as server:
                camera.start_recording(
                    os.devnull, format='h264', motion_output=server)

                while True:
                    camera.wait_recording(0)
                    try:
                        asyncio.run(server.main())
                        server.main_loop()
                    except AttributeError:
                        server.run()
                    finally:
                        camera.stop_recording()
                        print("read_cam closing...")
                        if board:
                            board.reboot()
                            time.sleep(1)

                    # np.save('experiment', detector.data)
                    # np.save('experiment_raw', detector.raw_data)
                    # time.sleep(0.1)
