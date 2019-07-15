import asyncio
import numpy as np
import time
import signal
from threading import Lock
from collections import deque

from OptiTrackPython import NatNetClient, from_quaternion2rpy

TIMEOUT = 1/20
OPERATING_PERIOD = 1/20


MSG_SIZE = 36

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

# I'm not sure if this order changes according to the configurations made to betaflight...
CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2', 'aux3', 'aux4']

shutdown = False

voltage = 1

async def main_msg_server(loop):
    global voltage

    while not shutdown:
        print("main_msg_server starting...")
        try:
            reader, writer = await asyncio.wait_for(asyncio.open_connection('192.168.2.124', 8989, loop=loop), timeout=TIMEOUT)
        except (asyncio.TimeoutError, ConnectionResetError, ConnectionRefusedError):
            continue

        try:
            # Kind of handshake to synchronize time with server
            handshake_message = np.zeros(int(MSG_SIZE/4), dtype=np.int8)
            handshake_message[0] = -1
            handshake_message[-1] = -1
            writer.write(np.asanyarray(handshake_message, dtype=np.float32).tobytes())
            print("Handshake sent ({})...".format(len(np.asanyarray(handshake_message, dtype=np.float32).tobytes())))

            data = await asyncio.wait_for(reader.read(MSG_SIZE), timeout=TIMEOUT)
            print("Handshake received...")
            message = np.frombuffer(data,dtype=np.float32)
            # server_time = message[0]
            timestart = time.time()

            message = np.zeros(1+len(CMDS_ORDER), dtype=np.float32)
            while not shutdown:
                timestamp = time.time() - timestart

                message[:] = [timestamp] + [CMDS[ki] for ki in CMDS_ORDER]
                # message[-1] = binascii.crc_hqx(message[:-1],0)
                writer.write(message.tobytes())
                # print("Sent ({}): {}".format(len(message.tobytes()), message.astype(int)))

                try:
                    data = await asyncio.wait_for(reader.read(MSG_SIZE), timeout=TIMEOUT) # read the voltage level
                    voltage = np.frombuffer(data,dtype=np.float32)[0]
                    # print("Received: {}".format(msg))
                except asyncio.TimeoutError:
                    continue
                

                await asyncio.sleep(OPERATING_PERIOD)

        except asyncio.TimeoutError:
            print("main_msg_server TimeoutError...")

        except ConnectionResetError:
            continue


        finally:
            if not writer.is_closing():
                writer.close()
                await writer.wait_closed()

    print("main_msg_server closing...")


async def read_optitrack(lock_opti, optitrack_reading,
                         client_ip='192.168.2.123',
                         server_ip="192.168.2.100",
                         multicast_address="239.255.42.99",
                         rigidbody_name2track="CogniFly"):
    print("read_optitrack started...")
    streamingClient = NatNetClient(client_ip,
                                   server_ip,
                                   multicast_address)

    rbname = rigidbody_name2track
    def receiveRigidBodyFrame(timestamp, id, position, rotation, rigidBodyDescriptor):
        if rigidBodyDescriptor:
            if rbname in rigidBodyDescriptor:
                if id == rigidBodyDescriptor[rbname][0]:
                    # skips this message if still locked
                    if lock_opti.acquire(False):
                        try:
                            # rotation is a quaternion!
                            optitrack_reading[0] = [timestamp,
                                                    position,
                                                    rotation]
                        finally:
                            lock_opti.release()

    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streamingClient.rigidBodyListener = receiveRigidBodyFrame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    streamingClient.run()

    while not shutdown:
        # the method run() will start the threads, so this task just need to stay alive...
        # without eating up CPU cycles ;)
        await asyncio.sleep(1)

    streamingClient.close()
    time.sleep(1)
    print("read_optitrack closing...")

async def pid(lock_opti, optitrack_reading):
    print("pid started...")

    AVG_CYCLES_ERRORS = 1
    AVG_CYCLES_SETPNT = 1000

    setpoint_list = [
                      [0,0,0,0],
                      [0,0,0.30,0]
                    ]

    read_values = []
    smooth_errors = [deque([0]*AVG_CYCLES_ERRORS) for i in range(4)]
    smooth_setpoints = [deque([0]*AVG_CYCLES_SETPNT) for i in range(4)]
    setpoint_list.reverse() # because I'm using pop later...


    # Generates a curve for an open loop take-off
    THROTTLE_HOVER = 1680
    TC = 0.3
    TOTAL_TIME = 3
    NSTEPS = TOTAL_TIME/OPERATING_PERIOD
    takeoff_sequence = (THROTTLE_HOVER*(1-np.exp(-np.arange(NSTEPS)*OPERATING_PERIOD/TC))).tolist()
    takeoff_sequence.reverse()
    
    await asyncio.sleep(2)    

    while not shutdown:
        # this lock is necessary because the optitrack code is using threads.
        if lock_opti.acquire(False):
            try:
                # rotation is a quaternion!
                # optitrack_reading[rbname] = [timestamp,
                #                                 position,
                #                                 rotation]
                if optitrack_reading:
                    init_pos = optitrack_reading[0][1]
                    init_yaw = from_quaternion2rpy(optitrack_reading[0][2])[2]
                    timestamp = optitrack_reading[0][0]
                    break

            finally:
                lock_opti.release()

        await asyncio.sleep(1)

    while not shutdown:
        # This lock is necessary because the optitrack code (internally) is using threads instead
        # of using asyncio.
        if lock_opti.acquire(False): # non-blocking
            try:
                # Data received from the OptiTrack:
                # optitrack_reading = [timestamp, position, rotation]
                # rotation is a quaternion!

                # if the values are arriving unordered, just skip
                if optitrack_reading[0][0]>timestamp:
                    read_values = optitrack_reading[0].copy()
                    read_yaw = from_quaternion2rpy(read_values[2])[2]
                    timestamp = read_values[0]
            finally:
                lock_opti.release()

        if len(read_values):
            if len(setpoint_list):
                x_read,y_read,z_read,yaw_read = setpoint_list.pop() # it will stop on the last value

            for smooth_setpnt, setpnt_value in zip(smooth_setpoints, [x_read, y_read, z_read, yaw_read]):
                smooth_setpnt.appendleft(setpnt_value)
                smooth_setpnt.pop()

            x,y,z,yaw = [sum(v)/len(v) for v in smooth_setpoints]

            # correct discontinuity problem with yaw (-180 to 180) error calculation
            error_yaw = -(yaw - read_yaw + init_yaw)
            if error_yaw > np.pi:
                error_yaw = error_yaw - 2*np.pi
            elif error_yaw < -np.pi:
                error_yaw = error_yaw + 2*np.pi
            else:
                error_yaw = error_yaw

            orig_err_x = (read_values[1][0] - init_pos[0]) - x
            orig_err_y = (read_values[1][1] - init_pos[1]) - y # the y is inverted on the optitrack
                                                                # because it simply inverts z (wrong righthand rule).
            err_z = -(read_values[1][2] - init_pos[2] - z)


            # Keeps the drone properly aligned to the global coordinates
            # by rotating its coordinate system according to the yaw value
            yaw_corr = 0 # correction... it depends on how the rigidbody was created inside optitrack
            err_x = orig_err_x*np.sin(-read_yaw+yaw_corr) + orig_err_y*np.cos(-read_yaw+yaw_corr)
            err_y = -orig_err_y*np.sin(-read_yaw+yaw_corr) + orig_err_x*np.cos(-read_yaw+yaw_corr)
    

            for smooth_error, error_value in zip(smooth_errors, [err_x, err_y, err_z, error_yaw]):
                smooth_error.appendleft(error_value)
                smooth_error.pop()

            avg_smooth_errors = [sum(v)/len(v) for v in smooth_errors]

            # COMMANDS
            # the PID controller will only use the average errors.
            yaw_cmd = 1500 + YAW_GAIN*avg_smooth_errors[3]/(2*np.pi)
            yaw_cmd = yaw_cmd if yaw_cmd >= 1000 else 1000
            yaw_cmd = yaw_cmd if yaw_cmd <= 2000 else 2000

            if len(takeoff_sequence):
                BIAS = takeoff_sequence.pop()
                throttle_cmd = BIAS
                print("Take-off sequence")
            else:
                throttle_cmd = BIAS + Z_GAIN*avg_smooth_errors[2]
            throttle_cmd = throttle_cmd if throttle_cmd >= 1000 else 1000
            throttle_cmd = throttle_cmd if throttle_cmd <= 2000 else 2000

            roll_cmd = 1500 + Y_GAIN*avg_smooth_errors[1]
            roll_cmd = roll_cmd if roll_cmd >= 1400 else 1400
            roll_cmd = roll_cmd if roll_cmd <= 1800 else 1800

            pitch_cmd = 1500 + X_GAIN*avg_smooth_errors[0]
            pitch_cmd = pitch_cmd if pitch_cmd >= 1400 else 1400
            pitch_cmd = pitch_cmd if pitch_cmd <= 1800 else 1800


            # CMDS['yaw'] = yaw_cmd
            CMDS['throttle'] = throttle_cmd
            CMDS['pitch'] = pitch_cmd
            CMDS['roll'] = roll_cmd
            CMDS['aux1'] = 1800
            print("CONTRL (COMP):", int((8.4/voltage)*roll_cmd), int((8.4/voltage)*pitch_cmd), int((8.4/voltage)*throttle_cmd), int((8.4/voltage)*yaw_cmd))
            print("CONTRL:", int(roll_cmd), int(pitch_cmd), int(throttle_cmd), int(yaw_cmd))
            # print("AVG ERRORS:",avg_smooth_errors)
            # print("CMDS:",[CMDS[ki] for ki in CMDS_ORDER])
        
        read_values = []
        await asyncio.sleep(OPERATING_PERIOD)
    
    print("pid closing...")


async def ask_exit(signame, loop):
    global shutdown

    print("Got signal %s: exit" % signame)
    shutdown = True
    await asyncio.sleep(0)

YAW_GAIN = 0
Z_GAIN = 0
Y_GAIN = 30
X_GAIN = 30

lock_opti = Lock()

optitrack_reading = {}

ioloop = asyncio.get_event_loop()
tasks = [
    main_msg_server(ioloop),
    pid(lock_opti, optitrack_reading),
    read_optitrack(lock_opti, optitrack_reading)
]

for sig in ['SIGHUP','SIGINT', 'SIGTERM']:
    ioloop.add_signal_handler(
        getattr(signal, sig),
        lambda s=sig: ioloop.create_task(ask_exit(s, ioloop)))

ioloop.run_until_complete(asyncio.wait(tasks))
ioloop.close()
