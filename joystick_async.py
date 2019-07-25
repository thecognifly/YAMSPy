"""joystick_async.py: Use a PS4 joystick to control the drone using Bluetooth and a PiZero W.

Copyright (C) 2019 Ricardo de Azambuja

This file is part of BetaflightMSPy.

BetaflightMSPy is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

BetaflightMSPy is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with BetaflightMSPy.  If not, see <https://www.gnu.org/licenses/>.

Acknowledgement:
This work was possible thanks to the financial support from IVADO.ca (postdoctoral scholarship 2019/2020).

Disclaimer (adapted from Wikipedia):
None of the authors, contributors, supervisors, administrators, employers, friends, family, vandals, or anyone else 
connected (or not) with this project, in any way whatsoever, can be made responsible for your use of the information (code) 
contained or linked from here.
"""

import asyncio
import time
import signal

# pip install uvloop
import uvloop

# pip3 install git+https://github.com/gvalkov/python-evdev.git
import evdev
from evdev import InputDevice, ecodes, ff

# pip3 install git+https://github.com/ricardodeazambuja/BetaflightMSPy.git
from betaflightmspy import MSPy

"""
$ sudo find /. -name "evtest.py"
and run it to get a list of valid stuff available.
"""

PRINT_FREQ = 5
JOYSTICK_FREQ = 50
MAIN_FREQ = 50

TIMEOUT = 1/MAIN_FREQ
TIMEOUT_JOYSTICK = 1/JOYSTICK_FREQ # this is only useful to avoid locking at shutdown

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

gamepad = InputDevice('/dev/input/event2')

shutdown = False

fc_reboot = False

async def joystick_client(dev):
    """Use PS4 Bluetooth controller to control the drone

    Here it is expected that the flight controller (Betaflight) has the modes configured as:
    Aux1==1800 => ARM
    Aux1==1000 => DISARM

    Aux2==1000 => ANGLE MODE
    Aux2==1500 => HORIZON MODE
    Aux2==1800 => FLIP OVER AFTER CRASH MODE

    Aux3==1800 => FAILSAFE ON

    Aux4==1800 => HEADFREE ON 
    """
    global fc_reboot

    print("joystick_client started...")

    # Setup for the vibration
    rumble = ff.Rumble(strong_magnitude=0x0000, weak_magnitude=0xffff)
    duration_ms = 1000

    effect = ff.Effect(
        ecodes.FF_RUMBLE, -1, 0,
        ff.Trigger(0, 0),
        ff.Replay(duration_ms, 0),
        ff.EffectType(ff_rumble_effect=rumble)
    )
    effect_id = gamepad.upload_effect(effect)

    reboot_event = [0,0]
    joystick_lost = False
    failsafe = False
    headfree = False
    autonomous = False
    while not shutdown:
        try:
            events = dev.read()
            # This seems to be the best option, but it raises BlockinIOError when I try to process
            # events and there's none

            # events = await asyncio.wait_for(dev.async_read(), timeout=TIMEOUT_JOYSTICK)
            # Using the wait_for gives this error asyncio.base_futures.InvalidStateError
            # and I was not able to catch it!

            # events = await dev.async_read()
            # Reading async_read directly, it hangs during shutdown if there no events and the other
            # tasks are closed.

            for event in events:
                if reboot_event[0] == True and reboot_event[1] == True:
                    if not fc_reboot:
                        print('REBOOTING FC...')
                        fc_reboot = True

                if event.type == 3:
                    if (not autonomous) and (CMDS['aux1'] == 1800): # it will not read values if:
                                                                    # - it's DISarmed
                                                                    # - it's AUTONOMOUS MODE
                        if event.code == 1:
                            # UP / DOWN: code 01 val from 255 (down) to 0 (up)
                            CMDS['throttle'] = 1000+1000*(255-event.value)/255 # LEFT up / down
                        if event.code == 0:
                            # LEFT / RIGHT: code 00 val from 0 (left) to 255 (right)
                            CMDS['yaw'] = 1000+1000*event.value/255 # LEFT left / right
                            pass
                        if event.code == 4:
                            # UP / DOWN: code 04 val from 255 (down) to 0 (up)
                            CMDS['pitch'] = 1000+1000*(255-event.value)/255 # RIGHT up / down
                        if event.code == 3:
                            # LEFT / RIGHT: code 00 val from 0 (left) to 255 (right)
                            CMDS['roll'] = 1000+1000*event.value/255 # RIGHT left / right

                    # Both, L2 and R2 need to be fully pressed to reboot
                    if event.code == 2: # L2 - FULLY PRESSED => REBOOT
                        if event.value > 250:
                            reboot_event[0] = True
                        else:
                            reboot_event[0] = False
                    if event.code == 5: # R2 - FULLY PRESSED => REBOOT
                        if event.value > 250:
                            reboot_event[1] = True
                        else:
                            reboot_event[1] = False

                if event.type == 1:
                    if event.value == 1:
                        if event.code == 311:
                            CMDS['aux1'] = 1000 # R1 - DISARM
                            print('DISarming...')
                        if event.code == 310:
                            CMDS['aux1'] = 1800 # L1 - ARM
                            print('ARMing...')
                        if event.code == 307:
                            CMDS['aux2'] = 1000 # TRIANGLE - ANGLE MODE
                            print('ANGLE MODE...')
                        if event.code == 304:
                            CMDS['aux2'] = 1500 # CROSS - HORIZON MODE
                            print('HORIZON MODE...')
                        if event.code == 308: # SQUARE - HEADFREE
                            if not headfree:
                                CMDS['aux4'] = 1800 # HEADFREE ON
                                print('HEADFREE ON...')
                                headfree = True
                            else:
                                CMDS['aux4'] = 1000 # HEADFREE OFF
                                print('HEADFREE OFF...')
                                headfree = False
                        if event.code == 314:
                            CMDS['aux2'] = 1800 # share button - FLIP OVER AFTER CRASH
                            print('FLIP OVER AFTER CRASH MODE...')
                        if event.code == 315:
                            if not failsafe:
                                CMDS['aux3'] = 1800 # options button - FAILSAFE
                                print('FAILSAFE ON...')
                                failsafe = True
                            else:
                                CMDS['aux3'] = 1000 # options button - FAILSAFE
                                print('FAILSAFE OFF...')
                                failsafe = False
                        if event.code == 305: # CIRCLE - AUTONOMOUS MODE
                            if not autonomous:
                                autonomous = True
                                print('AUTONOMOUS MODE...')
                                dev.write(ecodes.EV_FF, effect_id, 5)
                            else:
                                autonomous = False
                                print('MANUAL MODE...')
                                dev.write(ecodes.EV_FF, effect_id, 1)



        # except asyncio.TimeoutError:
            # continue
        except BlockingIOError:
            pass
        except OSError:
            print("Joystick lost...")
            joystick_lost = True
            break

        await asyncio.sleep(1/JOYSTICK_FREQ)

    print("joystick_client closing...")
    if not joystick_lost:
        dev.write(ecodes.EV_FF, effect_id, 5)


async def print_values():
    print("print_values started...")

    prev_time = time.time()
    while not shutdown:
        print("{0}Hz : {1}".format(1/(time.time()-prev_time), [CMDS[cmd] for cmd in CMDS_ORDER]))
        prev_time = time.time()
        await asyncio.sleep(1/PRINT_FREQ)

    print("print_values closing...")


async def send_cmds2fc():
    global fc_reboot
    global shutdown
    print("send_cmds2fc started...")

    board = 1
    while board:
        with MSPy(device="/dev/ttyACM0", loglevel='WARNING', baudrate=115200) as board:
            if board == 1: # an error occurred...
                print("Not connected to the FC...")
                await asyncio.sleep(1)
                continue
            else:
                while not shutdown:
                    if fc_reboot:
                        shutdown = True
                        print('REBOOTING...')
                        break
                    if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)
                    await asyncio.sleep(1/MAIN_FREQ)
                board.reboot()


        board = False

    print("send_cmds2fc closing...")


async def ask_exit(signame, loop):
    global shutdown

    print("Got signal %s: exit" % signame)
    shutdown = True
    await asyncio.sleep(0)


ioloop = uvloop.new_event_loop() # should be faster...
tasks = [
    joystick_client(gamepad),
    print_values(),
    send_cmds2fc()
]

for sig in ['SIGHUP','SIGINT', 'SIGTERM']:
    ioloop.add_signal_handler(
        getattr(signal, sig),
        lambda s=sig: ioloop.create_task(ask_exit(s, ioloop)))

ioloop.run_until_complete(asyncio.wait(tasks))
ioloop.close()