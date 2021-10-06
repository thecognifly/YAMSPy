"""ps4_joystick_async.py: Use a PS4 joystick to control the drone using Bluetooth and a RPI Zero W.

Copyright (C) 2020 Ricardo de Azambuja

This file is part of YAMSPy.

YAMSPy is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

YAMSPy is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with YAMSPy.  If not, see <https://www.gnu.org/licenses/>.

Acknowledgement:
This work was possible thanks to the financial support from IVADO.ca (postdoctoral scholarship 2019/2020).

Disclaimer (adapted from Wikipedia):
None of the authors, contributors, supervisors, administrators, employers, friends, family, vandals, or anyone else 
connected (or not) with this project, in any way whatsoever, can be made responsible for your use of the information (code) 
contained or linked from here.

How to pair the PS4 controller:
0. On the PS4 controller, press and hold "share" and 
then press and hold the "PS button" to enter pairing mode
(the joystick will start blinking as a airplane preparing to land...)
(if you take too long it will stop blinking, so no more pairing)
1. Launch bluetoothctl
$ sudo bluetoothctl
2. Start scanning for devices
# scan on
It will print the unique address (id), something like this:
??:??:??:??:??:?? Wireless Controller
3. Now you can turn off the annoying scanning messages
# scan off
4. Pair to the RPI Zero W
# pair ??:??:??:??:??:??
5. Connect to the RPI Zero W
# connect ??:??:??:??:??:??
6. To allow the RPI to automatically connect next time,
it is necessary to trust the device
# trust ??:??:??:??:??:??
7. Finally quit and you are done!
# quit

To remove a device, instead of "pair" use "remove".



TODO:
- Improve documentation.
- Use fstrings instead of .format (the current RPI image is using Python 3.7 after all...)
- Make it easier to set/customize the channels and values in the joystick node.
- Change the tof, optflow and control methods to make them generic and use metaclasses to enforce 
some basic stuff so the user will need to extend the class and implement those methods. The tof should
become a generic method that reads an altitude and optflow a method that spits vx and vy. Currently, some
calculations related to the sensors are done inside the control method. And I think those three methods should
use contextmanager to clean after themselves at the end automatically.
"""

__author__ = "Ricardo de Azambuja"
__copyright__ = "Copyright 2020, MISTLab.ca"
__credits__ = [""]
__license__ = "GPL"
__version__ = "0.0.1"
__maintainer__ = "Ricardo de Azambuja"
__email__ = "ricardo.azambuja@gmail.com"
__status__ = "Development"


import argparse
import time
from collections import deque
from threading import Thread, BrokenBarrierError
from multiprocessing import Pipe, Lock, Barrier
import os
import numpy as np

# pip3 install git+https://github.com/gvalkov/python-evdev.git
import evdev
from evdev import InputDevice, ecodes, ff
# Easy way to test the joystick:
# $ sudo find /. -name "evtest.py"
# and run it to get a list of valid stuff available.


# pip3 install git+https://github.com/ricardodeazambuja/YAMSPy.git
from yamspy import MSPy

from tof_node import ToF
from optflow_node import OptFlow
from control_node import Control


class CogniFly():
    def __init__(self, options_dict = None, 
                       autonomous = [],
                       print_freq = 1.0,
                       joystick_id = '??:??:??:??:??:??',
                       debug = False, save_data = False):
        """
        joystick_id: see instruction on how to pair the joystick to the RPI Zero W
        """

        # List of inputs that will be taken over when in autonomous mode
        # For testing purposes, it is possible to pass even an empty list
        # or a combination of these possible values: ['roll', 'pitch', 'throttle']
        self.autonomous_inputs_enabled = autonomous

        self.joystick_id = joystick_id
        self.print_freq = print_freq
        self.debug = debug
        self.save_data = save_data
        if self.save_data:
            self.DATA = deque([]) # list where the CMDS data will be temporarily stored

        # Using MSP controller it's possible to have more auxiliary inputs than this.
        # In fact, I think it's possible to have up to AUX14 using RX_MSP.
        # Below are the default values for all the cmds:
        self.CMDS_init = {
                'roll':     1500,
                'pitch':    1500,
                'throttle': 1000, # throttle bellow a certain value disarms the FC
                'yaw':      1500,
                'CH5':     1000, # DISARMED (1000) / ARMED (1800)
                'CH6':     1500, # ANGLE (1000) / NAV_POSHOLD (1500)
                'CH7':     1000, # FAILSAFE (1800)
                'CH8':     1000  # HEADING HOLD (1800)
                }
        # This order changes according to the configurations on betaflight:
        # Receiver=>Channel Map=>AETR1234
        # A=>roll, E=>pitch, T=>throttle, R=>yaw and 1234...
        self.CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'CH5', 'CH6', 'CH7', 'CH8']

        self.controller_init = {
                'kpx_v': 0.0, 'kix_v': 0.0, 'kdx_v': 0.0, # PID values velocity control: x
                'kpx_p': 0.0, 'kix_p': 0.0, 'kdx_p': 0.0, # PID values position control: x

                'kpy_v': 0.0, 'kiy_v': 0.0, 'kdy_v': 0.0, # PID values velocity control: y
                'kpy_p': 0.0, 'kiy_p': 0.0, 'kdy_p': 0.0, # PID values position control: y

                'kpz_v': 0.0, 'kiz_v': 0.0, 'kdz_v': 0.0, # PID values velocity control: z
                'kpz_p': 0.0, 'kiz_p': 0.0, 'kdz_p': 0.0, # PID values position control: z

                'kph_v': 0.0, 'kih_v': 0.0, 'kdh_v': 0.0, # PID values velocity control: heading
                'kph_p': 0.0, 'kih_p': 0.0, 'kdh_p': 0.0, # PID values position control: heading

                'beta_tof': 0.0,
                'beta_optflow': 0.0
                }

        self.sensors_init = {
            'beta_tof': 0.0,
            'beta_optflow': 0.0
            }

        if options_dict != None:
            if 'Input_Config' in options_dict:
                self.CMDS_init = options_dict['Input_Config']
            if 'Input_Order' in options_dict:
                self.CMDS_ORDER = options_dict['Input_Order']
            if 'Controller_Config' in options_dict:
                self.controller_init = options_dict['Controller_Config']
            if 'Sensors_Config' in options_dict:
                self.sensors_init = options_dict['Sensors_Config']

        self.CMDS_init['error_p']    = [0.0, 0.0, 0.0, 0.0] # error_x,error_y,error_z,error_h
        self.CMDS_init['error_v']    = [0.0, 0.0, 0.0, 0.0] # error_vx,error_vy,error_vz,error_vh
        self.CMDS_init['velocity']   = [0.0, 0.0, 0.0, 0.0] # vx,vy,vz,vh
        self.CMDS_init['time_stamp'] = 0.0

        self.CMDS = self.CMDS_init.copy()

        try:
            self.gamepad = InputDevice('/dev/input/event2')
        except FileNotFoundError:
            raise Exception('I would bet the PS4 joystick is not connected... Did you turn it on?')

        # Initialization of some global constants...
        self.shutdown = False
        self.board = None
        self.mean_batt_voltage = -1
        self.min_batt_voltage = -1


        # Controls the order how things are printed
        # from the dict frequencies_measurement
        self.frequencies_keys = ['send_cmds_to_fc', 
                                 'joystick_interface', 
                                 'autonomous', 
                                 'control',
                                 'read_voltage_from_fc', 
                                 'print_values']

        # It starts with 100000 just because 0 would crash and -1 looks weird...
        # TODO: I think it will not crash with zero anymore because there's a barrier
        self.frequencies_measurement = {'joystick_interface':100000,
                                        'send_cmds_to_fc':100000,
                                        'autonomous':100000,
                                        'control':100000,
                                        'read_voltage_from_fc':100000,
                                        'print_values':100000}

        # Setup for the joystick vibration used to alert the user (warnings)
        rumble = ff.Rumble(strong_magnitude=0x0000, weak_magnitude=0xffff)
        duration_ms = 1000

        effect = ff.Effect(
            ecodes.FF_RUMBLE, -1, 0,
            ff.Trigger(0, 0),
            ff.Replay(duration_ms, 0),
            ff.EffectType(ff_rumble_effect=rumble)
        )
        self.effect_id = self.gamepad.upload_effect(effect)

        self.NofCHEV = 50 # just used when printing info on the screen


        # Limits the max frequency these nodes can run
        self.joystick_min_period = 1/50.0
        self.fc_min_period = 1/100.0 # too many cmds may overload the FC



    def joystick_interface(self, dev, ext_contr_pipe, CMDS_lock):
        """Use PS4 Bluetooth controller to control the drone
        """

        local_CMDS = self.CMDS_init.copy() # "local_" is for trying to optimize python for speed
                                           # by using local versions avoiding dictionary search...
        local_ecodes_EV_FF = ecodes.EV_FF

        print("joystick_interface started...")

        EVENTS_COUNTDOWN_INIT = 100 # if the loop goes more than this amount of
                                    # times without receiving any event from the joystick
                                    # it will shutdown

        no_events_countdown = EVENTS_COUNTDOWN_INIT
        joystick_reboot_event_trigger = [0,0] # both buttons (L2 and R2) need to be pressed to reboot
        joystick_lost = False
        failsafe = False
        headfree = False
        autonomous = False # TODO: this information could be useful for other methods too...
        cmds_pipe = None
        local_perf_counter = time.perf_counter
        local_sleep = time.sleep
        tend = local_perf_counter()
        prev_time = prev_time_ext = arm_time = time.time()
        while not self.shutdown:
            tbegin = local_perf_counter()
            tdiff = (tbegin-tend)
            if tdiff < self.joystick_min_period:
                local_sleep(self.joystick_min_period-tdiff) # make sure other threads have time to do
                                                            # their own stuff - RPI Zero's CPU has only one core.

            received_cmds = [0,0,0,0] # roll, pitch, throttle, yaw where 1 indicates a cmd was received
            try:
                # Process info from external controller according to which inputs are enabled
                # This must happen before dev.read() because the situation where no events
                # were received from the joystick would also block the external controller.
                if ext_contr_pipe.poll() and autonomous:
                    cmds_pipe = ext_contr_pipe.recv()
                    if ('roll' in self.autonomous_inputs_enabled):
                        # The received commands will always actuate around the center positions
                        local_CMDS['roll'] = self.CMDS_init['roll'] + cmds_pipe['roll']
                        received_cmds[0] = 1
                    if ('pitch' in self.autonomous_inputs_enabled):
                        # The received commands will always actuate around the center positions
                        local_CMDS['pitch'] = self.CMDS_init['pitch'] + cmds_pipe['pitch']
                        received_cmds[1] = 1
                    if ('throttle' in self.autonomous_inputs_enabled):
                        # The received commands will always actuate FROM the default min value
                        local_CMDS['throttle'] = self.CMDS_init['throttle'] + cmds_pipe['throttle']
                        received_cmds[2] = 1
                    if ('yaw' in self.autonomous_inputs_enabled and not headfree):
                        # The received commands will always actuate around the center positions
                        local_CMDS['yaw'] = self.CMDS_init['yaw'] + cmds_pipe['yaw']
                        received_cmds[3] = 1
                    # TODO: check if these attributions slow down too much...
                    local_CMDS['error_p'] = cmds_pipe['error_p']
                    local_CMDS['error_v'] = cmds_pipe['error_v']
                    local_CMDS['velocity'] = cmds_pipe['velocity']
                    local_CMDS['time_stamp'] = cmds_pipe['time_stamp']

                    # here we measure the frequency of the commands received from the autonomous controller.
                    self.frequencies_measurement['autonomous'] = time.time() - prev_time_ext
                    prev_time_ext = time.time()
                
                events = dev.read()
                # This seems to be the best option, but it raises BlockinIOError when I try to process
                # events and there's none
                # TODO: would it be faster if I create a local variable before the while loop like dev = dev??

                for event in events:
                    no_events_countdown = EVENTS_COUNTDOWN_INIT # resets to indicate the joystick is alive...
                    # ATTENTION:
                    # The way the values are adjusted were only tested with the PS4 joystick!

                    # It uses a list because both buttons (L2 and R2) need to be pressed to reboot
                    if joystick_reboot_event_trigger[0] and joystick_reboot_event_trigger[1]:
                        print(">"*self.NofCHEV + "REBOOTING FC...")
                        self.shutdown = True

                    if event.type == 3:
                        if event.code == 3:
                            # the test below allows us to keep controlling stuff using joystick
                            # even when the autonomous mode is enabled if the input was not selected
                            if ('roll' not in self.autonomous_inputs_enabled) or not autonomous:
                                # LEFT / RIGHT: code 00 val from 0 (left) to 255 (right)
                                if ((255/2+30) < event.value) or (event.value < (255/2-30)): 
                                    local_CMDS['roll'] = 1000+1000*event.value/255 # RIGHT left / right
                                else:
                                    local_CMDS['roll'] = 1500
                        elif event.code == 4:
                            if ('pitch' not in self.autonomous_inputs_enabled) or not autonomous:
                                # UP / DOWN: code 04 val from 255 (down) to 0 (up)
                                if ((255/2+30) < event.value) or (event.value < (255/2-30)):
                                    local_CMDS['pitch'] = 1000+1000*(255-event.value)/255 # RIGHT up / down
                                else:
                                    local_CMDS['pitch'] = 1500
                        elif event.code == 1:
                            if ('throttle' not in self.autonomous_inputs_enabled) or not autonomous:
                                # UP / DOWN: code 01 val from 255 (down) to 0 (up)
                                if ((255/2+10) < event.value) or (event.value < (255/2-10)):
                                    local_CMDS['throttle'] = 1000+1000*(255-event.value)/255
                                else:
                                    local_CMDS['throttle'] = 1500
                        elif event.code == 0:
                            if ('yaw' not in self.autonomous_inputs_enabled) or not autonomous:
                                if not headfree:
                                    if ((255/2+30) < event.value) or (event.value < (255/2-30)):
                                    # LEFT / RIGHT: code 00 val from 0 (left) to 255 (right)
                                        local_CMDS['yaw'] = 1000+1000*event.value/255 # LEFT left / right
                                    else:
                                        local_CMDS['yaw'] = 1500
                        # Both, L2 and R2 need to be fully pressed to reboot
                        elif event.code == 2: # L2 - FULLY PRESSED => REBOOT
                            if event.value > 250:
                                joystick_reboot_event_trigger[0] = True
                            else:
                                joystick_reboot_event_trigger[0] = False
                        elif event.code == 5: # R2 - FULLY PRESSED => REBOOT
                            if event.value > 250:
                                joystick_reboot_event_trigger[1] = True
                            else:
                                joystick_reboot_event_trigger[1] = False

                        # event.code == 17 is the digital up / down
                        # when it's pressed down: event.value == 1
                        #                   up  : event.value == -1
                        #           not pressed : event.value == 0
                        # It could be used to indicate take off / landing

                    if event.type == 1:
                        if event.value == 1:
                            if event.code == 311:
                                local_CMDS['CH5'] = 1000 # R1 - DISARM
                                autonomous = False
                                print(">"*self.NofCHEV + "MANUAL MODE...")
                                print(">"*self.NofCHEV + "DISarming... {:0.2f}V".format(self.mean_batt_voltage))
                                print(">"*self.NofCHEV + "RUN TIME: ", (time.time() - arm_time))
                                dev.write(local_ecodes_EV_FF, self.effect_id, 2) # vibration
                                # TODO: ideally MSP_STATUS_EX should be received to confirm
                                # the drone actually DISarmed...
                            elif event.code == 310:
                                # if local_CMDS['throttle']<=1000:
                                local_CMDS['CH5'] = 1900 # L1 - ARM
                                print(">"*self.NofCHEV + "ARMing... ")
                                arm_time = time.time()
                                dev.write(local_ecodes_EV_FF, self.effect_id, 1) # vibration
                                # TODO: ideally MSP_STATUS_EX should be received to confirm
                                # the drone actually armed...
                                # else:
                                #     dev.write(local_ecodes_EV_FF, self.effect_id, 5) # vibration
                                #     print(">"*self.NofCHEV + "Throttle too high ({}) for arming!!!".format(local_CMDS['throttle']))
                            elif event.code == 307:
                                local_CMDS['CH6'] = 1000 # TRIANGLE - ANGLE MODE
                                print(">"*self.NofCHEV + "ANGLE MODE... ")
                                dev.write(local_ecodes_EV_FF, self.effect_id, 1) # vibration
                            elif event.code == 304:
                                local_CMDS['CH6'] = 1500 # CROSS - NAV_ALTHOLD MODE
                                print(">"*self.NofCHEV + "NAV_ALTHOLD MODE... ")
                                dev.write(local_ecodes_EV_FF, self.effect_id, 1) # vibration
                            elif event.code == 308: # SQUARE - HEADING HOLD
                                # HEADING HOLD will turn off yaw reading from joystick
                                # and turn on FC's headfree mode. 
                                if not headfree:
                                    local_CMDS['CH8'] = 1900 # HEADING HOLD ON
                                    print(">"*self.NofCHEV + "HEADING HOLD ON... ")
                                    headfree = True
                                    local_CMDS['yaw'] = self.CMDS_init['yaw']
                                    dev.write(local_ecodes_EV_FF, self.effect_id, 1) # vibration
                                else:
                                    local_CMDS['CH8'] = 1000 # HEADING HOLD OFF
                                    print(">"*self.NofCHEV + "HEADING HOLD OFF... ")
                                    headfree = False
                                    dev.write(local_ecodes_EV_FF, self.effect_id, 2) # vibration
                            elif event.code == 315:
                                local_CMDS['CH6'] = 1900 # options button - NAV_POSHOLD MODE
                                print(">"*self.NofCHEV + "NAV_POSHOLD MODE... ")
                                dev.write(local_ecodes_EV_FF, self.effect_id, 1) # vibration
                            elif event.code == 314:
                                if not failsafe:
                                    local_CMDS['CH7'] = 1900 # share button - FAILSAFE
                                    print(">"*self.NofCHEV + "FAILSAFE ON... ")
                                    failsafe = True
                                    dev.write(local_ecodes_EV_FF, self.effect_id, 1) # vibration
                                else:
                                    local_CMDS['CH7'] = 1000 # share button - FAILSAFE
                                    print(">"*self.NofCHEV + "FAILSAFE OFF... ")
                                    failsafe = False
                                    dev.write(local_ecodes_EV_FF, self.effect_id, 2) # vibration
                            elif event.code == 305: # CIRCLE - AUTONOMOUS MODE
                                if not autonomous:
                                    autonomous = True
                                    # Indicates to the external controller
                                    # autonomous mode is ON by sending True
                                    ext_contr_pipe.send(True)
                                    print(">"*self.NofCHEV + "AUTONOMOUS MODE...")
                                    dev.write(local_ecodes_EV_FF, self.effect_id, 5) # vibrate for 5s here
                                else:
                                    autonomous = False
                                    # Indicates to the external controller
                                    # autonomous mode is OFF by sending False
                                    ext_contr_pipe.send(False)
                                    local_CMDS['roll'] = self.CMDS_init['roll']
                                    local_CMDS['pitch'] = self.CMDS_init['pitch']
                                    local_CMDS['throttle'] = self.CMDS_init['throttle']
                                    local_CMDS['yaw'] = self.CMDS_init['yaw']
                                    print(">"*self.NofCHEV + "MANUAL MODE...")
                                    dev.write(local_ecodes_EV_FF, self.effect_id, 1) # vibrate only for 1s here
                if self.save_data:
                    # self.mean_batt_voltage is written by another thread, so unless it's atomic...
                    # however, the risk of a glitch in the voltage value is better than slowing down by waiting for a lock, etc.
                    self.DATA.append((time.time(), self.mean_batt_voltage, autonomous, received_cmds, 
                                      local_CMDS['roll'], local_CMDS['pitch'], local_CMDS['throttle'], local_CMDS['yaw'], 
                                      local_CMDS['error_p'], local_CMDS['error_v'], local_CMDS['velocity'], local_CMDS['time_stamp']))

            # except BlockingIOError: # if no events were generated by the joystick
            # except OSError: # if you turn off the joystick or go too far away
            except Exception as e:
                no_events_countdown -= 1
                if no_events_countdown < 0 or e.errno == 19: # error 19 is when the joystick is off
                    print(">"*self.NofCHEV + "Joystick lost - Last Exception:{}".format(e))
                    joystick_lost = True
                    break
                else:
                    print(">"*self.NofCHEV + "Joystick Exception:{} ({} events left)".format(e, no_events_countdown))

            
            if (autonomous and not any(received_cmds)) or no_events_countdown != EVENTS_COUNTDOWN_INIT:
                with CMDS_lock:
                    self.CMDS = {}  # tells send_cmds_to_fc to skip
                                    # if autonomous mode failed to send commands
                                    # or if the joystick didn't receive any events either
                                    # and it will force the FC to failsafe after a while.
            else:
                with CMDS_lock:
                    for k,v in local_CMDS.items():
                        self.CMDS[k] = v

            self.frequencies_measurement['joystick_interface'] = time.time() - prev_time
            prev_time = time.time()
            tend = local_perf_counter()

        print("joystick_interface closing...")
        if joystick_lost:
            self.shutdown = True
            dev.write(local_ecodes_EV_FF, self.effect_id, 5) # vibration
            print(">"*self.NofCHEV + "JOYSTICK LOST?!?")



    def print_values(self, barrier_init_values, CMDS_lock, freq = 1):
        print("print_values started...")

        time2sleep = 1/freq

        prev_time = time.time()
        try:
            barrier_init_values.wait(timeout=5) # indicates the initial values are available
        except BrokenBarrierError:
            print("print_values barrier timeout...")
            self.shutdown = True
            return
        
        local_CMDS = []
        while not self.shutdown:
            print("Frequencies:")
            print(["{} - {:.2f}Hz".format(keys, 1/self.frequencies_measurement[keys]) for keys in self.frequencies_keys])
            print("Voltage / Accelerometer / Attitude:")
            values = [self.mean_batt_voltage, self.min_batt_voltage] + self.board.SENSOR_DATA['accelerometer'] + self.board.SENSOR_DATA['kinematics']
            print("{:.2f}V ({:.2f}V) - AccX:{:.2f}, AccY:{:.2f}, AccZ:{:.2f} - R:{:.2f}, P:{:.2f}, Y:{:.2f}".format(*values))
            with CMDS_lock:
                local_CMDS = self.CMDS.copy()
            if len(local_CMDS):
                print("Commands:")
                print(["{} - {:.2f}".format(cmd, local_CMDS[cmd]) for cmd in self.CMDS_ORDER])

            self.frequencies_measurement['print_values'] = time.time() - prev_time
            prev_time = time.time()
            time.sleep(time2sleep)

        print("print_values closing...")


    def read_voltage_from_fc(self, dev, barrier_init_values, freq = 1/3):
        print("read_voltage_from_fc started...")
        
        JOYSTICK_MIN_BATT_LEVEL = 30 # goes up to 100
        MOV_AVG_VOLTAGE_LENGTH = 5
        time2sleep = 1/freq

        f_controller = open("/sys/class/power_supply/sony_controller_battery_"+self.joystick_id+"/capacity", 'r') 

        try:
            barrier_init_values.wait(timeout=5) # indicates the initial values are available
        except BrokenBarrierError:
            print("read_voltage_from_fc barrier timeout...")
            self.shutdown = True
            return

        self.min_batt_voltage = self.board.BATTERY_CONFIG['vbatmincellvoltage']*self.board.BATTERY_STATE['cellCount']
        warn_batt_voltage = self.board.BATTERY_CONFIG['vbatwarningcellvoltage']*self.board.BATTERY_STATE['cellCount']
        max_batt_voltage = self.board.BATTERY_CONFIG['vbatmaxcellvoltage']*self.board.BATTERY_STATE['cellCount']
        voltage = self.board.ANALOG['voltage']
        avg_voltage_deque = deque([voltage]*MOV_AVG_VOLTAGE_LENGTH)

        prev_time = time.time()
        local_fast_read_analog = self.board.fast_read_analog
        local_ANALOG = self.board.ANALOG
        while not self.shutdown:
            local_fast_read_analog()
            voltage = local_ANALOG['voltage']

            avg_voltage_deque.appendleft(voltage)
            avg_voltage_deque.pop()

            self.mean_batt_voltage = sum(avg_voltage_deque)/MOV_AVG_VOLTAGE_LENGTH
            if self.mean_batt_voltage <= self.min_batt_voltage:
                print (">"*self.NofCHEV + "Drone battery too low ({})!!!".format(self.mean_batt_voltage))
                dev.write(ecodes.EV_FF, self.effect_id, 5) # vibration
            elif self.mean_batt_voltage >= max_batt_voltage:
                dev.write(ecodes.EV_FF, self.effect_id, 1) # vibration
                print (">"*self.NofCHEV + "Drone battery too high ({})!!!".format(self.mean_batt_voltage))
            elif self.mean_batt_voltage <= warn_batt_voltage:
                dev.write(ecodes.EV_FF, self.effect_id, 1) # vibration
                print (">"*self.NofCHEV + "Drone battery almost too low ({})...".format(self.mean_batt_voltage))

            f_controller.seek(0)
            controller_batt_level = int(f_controller.read())
            if controller_batt_level < JOYSTICK_MIN_BATT_LEVEL:
                print (">"*self.NofCHEV + "Joystick battery too low ({})!!!".format(controller_batt_level))
                dev.write(ecodes.EV_FF, self.effect_id, 5) # vibration


            self.frequencies_measurement['read_voltage_from_fc'] = time.time() - prev_time
            prev_time = time.time()
            time.sleep(time2sleep)

        print("read_voltage_from_fc closing...")


    def send_cmds_to_fc(self, pipe_write, pipe_read, CMDS_lock, barrier_init_values, barrier_sensor_values):
        print("send_cmds_to_fc started...")

        # (control_imu_pipe_write, control_imu_pipe_read)

        prev_time = time.time()
        while not self.shutdown:
            print("Connecting to the FC...")
            with MSPy(device='/dev/serial0', loglevel='WARNING', baudrate=115200) as self.board:
                if self.board == 1: # an error occurred...
                    print(">"*self.NofCHEV + "Connecting to the FC... FAILED!")
                    print(">"*self.NofCHEV + "Trying again...")
                    time.sleep(1)
                    continue
                else:
                    print(">"*self.NofCHEV + "Connecting to the FC... OK!")

                    command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
                                    'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
                                    'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES', 'MSP_ANALOG']

                    # Requests the full command_list because we are just starting and most probably not flying / armed.
                    for msg in command_list: 
                        if self.board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                            dataHandler = self.board.receive_msg()
                            self.board.process_recv_data(dataHandler)
                        else:
                            pass # do something in case it fails...
                    try:
                        barrier_init_values.wait(timeout=5) # indicates the initial values are available
                                                            # in a normal situation this should be the last
                                                            # member to reach the barrier...
                    except BrokenBarrierError:
                        print("send_cmds_to_fc barrier failed...")
                        self.shutdown = True

                    local_fast_msp_rc_cmd = self.board.fast_msp_rc_cmd
                    local_fast_read_imu = self.board.fast_read_imu
                    local_fast_read_attitude = self.board.fast_read_attitude
                    local_SENSOR_DATA = self.board.SENSOR_DATA
                    local_perf_counter = time.perf_counter
                    local_sleep = time.sleep
                    tend = local_perf_counter()
                    try:
                        while not self.shutdown:
                            CMDS_RC = []
                            imu_ready = False
                            
                            with CMDS_lock:
                                if self.CMDS:
                                    CMDS_RC = [self.CMDS[ki] for ki in self.CMDS_ORDER] # the order must match Betaflight configuration!

                            if not self.shutdown: # if shutdown is True something crazy may be awaiting in CMDS_RC...
                                tbegin = local_perf_counter()
                                tdiff = (tbegin-tend)
                                if tdiff < self.fc_min_period:
                                    local_sleep(self.fc_min_period-tdiff)
                                    # it may overload the FC if it sends too many commands

                                if CMDS_RC:
                                    local_fast_msp_rc_cmd(CMDS_RC)
                                if barrier_sensor_values.n_waiting == 2: # 2=>tof and optflow
                                    local_fast_read_imu()  # SENSOR_DATA['accelerometer'] and SENSOR_DATA['gyroscope']
                                    barrier_sensor_values.reset() # the best way I found... hackish though
                                    imu_ready = True
                                local_fast_read_attitude()  # SENSOR_DATA['kinematics'] => roll, pitch, yaw 

                                tend = local_perf_counter()

                            if not pipe_read.poll():
                                if imu_ready:
                                    pipe_write.send(((local_SENSOR_DATA['accelerometer'],
                                                      local_SENSOR_DATA['gyroscope'],
                                                      local_SENSOR_DATA['kinematics']),
                                                      self.mean_batt_voltage,
                                                      time.time()))

                            self.frequencies_measurement['send_cmds_to_fc'] = time.time() - prev_time
                            prev_time = time.time()
                    finally:
                        print(">"*self.NofCHEV + "SHUTTING DOWN...")
                        self.shutdown = True
                        print(">"*self.NofCHEV + "REBOOTING FC...")
                        self.board.reboot()

        print("send_cmds_to_fc closing...")

    def tof(self, control_tof_pipe_write, control_tof_pipe_read, 
                barrier_sensor_values, beta = 0.0,
                time_budget = 40, time_period = 50):

        ToF_obj = ToF(control_tof_pipe_write, 
                      control_tof_pipe_read,
                      dt = time_period/1000, 
                      beta = beta,
                      time_budget = time_budget, 
                      time_period = time_period)
        
        try:
            while not self.shutdown:
                try:
                    barrier_sensor_values.wait(timeout=2) # timeout is only to close the thread
                except BrokenBarrierError: # this will be activated by the reset method
                    ToF_obj.read()
        finally:
            ToF_obj.close()

    def optflow(self, control_optflow_pipe_write, control_optflow_pipe_read, 
                    barrier_sensor_values,
                    beta = 0.0, time_period = 50):

        OptFlow_obj = OptFlow(control_optflow_pipe_write, 
                              control_optflow_pipe_read,
                              dt = time_period/1000,
                              beta = beta)
        
        try:
            while not self.shutdown:
                try:
                    barrier_sensor_values.wait(timeout=2) # timeout is only to close the thread
                except BrokenBarrierError: # this will be activated by the reset method
                    OptFlow_obj.read()
        finally:
            OptFlow_obj.close()

    def control(self, control_optflow_pipe_read, 
                    control_tof_pipe_read, 
                    control_imu_pipe_read, 
                    ext_control_pipe_write, 
                    ext_control_pipe_read,
                    altitude,
                    pid_dict,
                    debug):
        
        Control_obj = Control(control_optflow_pipe_read, 
                              control_tof_pipe_read, 
                              control_imu_pipe_read, 
                              ext_control_pipe_write, 
                              ext_control_pipe_read,
                              altitude, # altitude setpoint
                              pid_dict,
                              debug)
        
        prev_time = time.time()
        try:
            while not self.shutdown:
                if not Control_obj.step():
                    self.frequencies_measurement['control'] = time.time() - prev_time
                    prev_time = time.time()
                    # time.sleep(self.control_timer.time2sleep())
                else:
                    break 
        finally:
            self.shutdown = True


    def run(self, nice_level = -10):

        print("Setting nice level...", end='')
        try:
            os.nice(nice_level) # this process (main_node) has the highest priority
        except PermissionError:
            raise Exception("You must be root to use nice smaller than 0")
        print("done!")

        print("Creating Pipes, Locks, Barriers...", end='')
        ext_control_pipe_write, ext_control_pipe_read = Pipe()
        control_optflow_pipe_write, control_optflow_pipe_read = Pipe()
        control_tof_pipe_write, control_tof_pipe_read = Pipe()
        control_imu_pipe_write, control_imu_pipe_read = Pipe()
        CMDS_lock = Lock()
        barrier_init_values = Barrier(3) # print_values, read_voltage_from_fc and send_cmds_to_fc
        barrier_sensor_values = Barrier(3) # tof, optflow and send_cmds_to_fc
        print("done!")

        threads = [
            Thread(target=self.joystick_interface, 
                   args=(self.gamepad, 
                         ext_control_pipe_read, 
                         CMDS_lock)),
            Thread(target=self.read_voltage_from_fc, 
                   args=(self.gamepad, 
                         barrier_init_values)),
            Thread(target=self.print_values, 
                   args=(barrier_init_values, 
                         CMDS_lock, self.print_freq)),
            Thread(target=self.send_cmds_to_fc, 
                   args=(control_imu_pipe_write, control_imu_pipe_read, 
                         CMDS_lock, 
                         barrier_init_values, barrier_sensor_values))#,
            # Thread(target=self.tof, 
            #        args=(control_tof_pipe_write, control_tof_pipe_read, 
            #              barrier_sensor_values, 
            #              self.sensors_init['beta_tof'], 40, 50)),
            # Thread(target=self.optflow, 
            #        args=(control_optflow_pipe_write, control_optflow_pipe_read, 
            #              barrier_sensor_values, 
            #              self.sensors_init['beta_optflow'], 50)),
            # Thread(target=self.control, 
            #        args=(control_optflow_pipe_read, control_tof_pipe_read, 
            #              control_imu_pipe_read, 
            #              ext_control_pipe_write, ext_control_pipe_read,
            #              0.50, self.controller_init, self.debug))
                         ]

        thread_names = ['joystick_interface', 'read_voltage_from_fc', 
                        'print_values', 'send_cmds_to_fc']
                        #, 'tof', 'optflow',
                        #'control']

        try:
            print("Launching threads...")
            for ti in threads:
                ti.start()
            threads[0].join() # the idea is to kill all threads if this one (joystick_interface) stops
        finally:
            self.shutdown = True
            while any([ti.is_alive() for ti in threads]):
                print("Waiting for threads to gracefully die...", [thread_names[i] for i,ti in enumerate(threads) if ti.is_alive()])
                time.sleep(0.1) # just to avoid letting the loop going wild...

            # Saving to the sdcard is slow, so it must be the last thing to avoid
            # locking something important.
            if self.save_data:
                saved_cmds_filename = "CMDS-"+time.ctime().replace(" ", "_").replace(":",".")+".npy"
                self.DATA.appendleft(self.controller_init)
                self.DATA.appendleft(self.sensors_init)
                self.DATA.appendleft({'nice_level':nice_level})
                print("Saving CMDS to file: ", saved_cmds_filename, end='')
                try:
                    np.save(saved_cmds_filename, self.DATA) # TODO: maybe I should change to pickle...
                    print(' ...done!')
                except Exception as e:
                    print(" ...failed to save data!")
                    print(e)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Launches CogniFly control system (needs sudo if nice < 0).')

    parser.add_argument('--nice', type=int,
                        default=0,
                        help='Nice level (from -20 to 19, but negative numbers need sudo)')

    parser.add_argument('--screen-frequency', type=float,
                        default=1,
                        help='Frequency info is printed on the screen.')

    parser.add_argument('--autonomous-mode', type=str, nargs='*', 
                        default=[], 
                        choices=['roll', 'pitch', 'throttle'], 
                        help='CMDS taken over by autonomous mode (e.g. --autonomous-mode throttle roll).')

    parser.add_argument("--debug", 
                        action='store_true', 
                        help="Debug mode.")

    parser.add_argument("--save", 
                        action='store_true', 
                        help="Save CMDS sent to the fligh controller to 'CMDS-<truncated monotonic time>.npy'.")

    args = parser.parse_args()

    controller_init = {
            'kpx_v': 0.0, 'kix_v': 20.0, 'kdx_v': 0.0, # PID values velocity control: x
            'kpx_p': 0.0, 'kix_p': 0.0, 'kdx_p': 0.0, # PID values position control: x

            'kpy_v': 0.0, 'kiy_v': 20.0, 'kdy_v': 0.0, # PID values velocity control: y
            'kpy_p': 0.0, 'kiy_p': 0.0, 'kdy_p': 0.0, # PID values position control: y

            'kpz_v': 35.0, 'kiz_v': 2.5, 'kdz_v': 4.0, # PID values velocity control: z
            'kpz_p': 0.0, 'kiz_p': 0.0, 'kdz_p': 0.0, # PID values position control: z

            'kph_v': 0.0, 'kih_v': 0.0, 'kdh_v': 0.0, # PID values velocity control: heading
            'kph_p': 0.0, 'kih_p': 0.0, 'kdh_p': 0.0, # PID values position control: heading

            'beta_tof': 0.0,
            'beta_optflow': 0.8
            }

    sensors_init = {
        'beta_tof': 0.0,
        'beta_optflow': 0.0
        }

    options_dict = {}
    options_dict['Controller_Config'] = controller_init
    options_dict['Sensors_Config'] = sensors_init

    drone = CogniFly(options_dict, print_freq = args.screen_frequency, 
                                   autonomous = args.autonomous_mode, 
                                   debug = args.debug, 
                                   save_data = args.save)
    
    drone.run(nice_level = args.nice)

