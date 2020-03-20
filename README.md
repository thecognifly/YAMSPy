# YAMSPy
Yet Another Implementation of [Multiwii](https://github.com/multiwii) Serial Protocol Python Interface for [Betaflight](https://github.com/betaflight/betaflight), [iNAV](https://github.com/iNavFlight/inav), etc.

## <span style="color:orange;">SAFETY FIRST!</span>
This work is ***EXPERIMENTAL*** and it has been made publicly available expecting to be used only by responsible legally capable adults, etc, etc, etc. Since it's experimental, crazy things may happen mid-flight or even after the drone has landed or, even worse, while the drone is sitting on the floor! Drones, by default, can be **VERY** dangerous if not handled properly. Here are some links that may help you understand what I mean:
- https://www.tc.gc.ca/en/services/aviation/drone-safety.html
- https://store.dji.com/guides/10-drone-safety-tips-safe-flight/
- http://knowbeforeyoufly.org/
- [LiPo batteries themselves need TLC to be safe](https://www.robotshop.com/media/files/pdf/hyperion-g5-50c-3s-1100mah-lipo-battery-User-Guide.pdf)

### <span style="color:orange;">**Disclaimer (adapted from Wikipedia):** None of the authors, contributors, supervisors administrators, employers, friends, family, vandals, or anyone else connected (or not) with this project, in any way whatsoever, can be made responsible for the use of the information (code) contained or linked from here.</span>

## Installation (Python3 because [you should not use Python2 anymore](https://www.python.org/doc/sunset-python-2/)):
```
$ git clone https://github.com/ricardodeazambuja/YAMSPy.git
$ cd YAMSPy
$ sudo pip3 install .
```

On Linux you may need to add your user to the dialout group:  
```
$ sudo usermod -a -G dialout $USER
```

## Examples:
The directory [Examples](https://github.com/ricardodeazambuja/YAMSPy/tree/master/Examples) (duh!) has some commented examples showing how to use the library.

## Setting up your flight controller (FC):
Until this point you should have installed YAMSPy, but it will not magically work without some extra steps. In order to make it work, you need to set up your FC correctly.

If you are just starting with iNAV or Betaflight you will to install one of the configurators: [inav-configurator](https://github.com/iNavFlight/inav-configurator/releases) or [betaflight-configurator](https://github.com/betaflight/betaflight-configurator/releases). 

### How to check which firmware is installed in your flight controller
In general, the most popular flight controllers (so far I have tested [Heli-nation Talon F7 Fusion](https://www.heli-nation.com/talon-f7-fusion-flight-controller) and [Kakute F7 mini](http://www.holybro.com/product/kakute-f7-mini/) with success) come with some version of Betaflight and if you try to use inav-configurator it will only show the CLI (Command Line Interface) tab. The other way around, iNAV installed and betaflight-configurator, it will still allow you to use the CLI. The good thing is that using the CLI you can enter [DFU (Device Firmware Update) mode](https://www.youtube.com/watch?v=XKoZ_qrOtXg) to reflash the firmware by simply typing ```dfu``` if iNAV firmware or ```bl``` or ```dfu``` if Betaflight. I noticed that my flight controlllers would only enter DFU mode if they were powered exclusivelly by the USB. To flash (install) a new firmware you can follow the instructions for [Betaflight](https://github.com/betaflight/betaflight/wiki/Installing-Betaflight) or [iNAV](https://github.com/iNavFlight/inav/blob/master/docs/Installation.md). BTW, once you are using the CLI mode you can just type ```status``` to receive a lot of useful info like firmware version.

### More details about how YAMSPy works
The YAMSPy works by communicating with the flight controller using MSP ([MultiWii Serial Protocol](https://github.com/multiwii)). Therefore, you need to use an UART (Universal asynchronous receiver-transmitter) that has MSP enabled. By default (AFAIK), the USB connector will be seen as an UART and it comes set as MSP, so it would allow you to use YAMSPy. You can easily enable MSP on other UART using the configurator software.

Ok, the paragraph above covers the basics, but to be able to use YAMSPy to control your drone it is necessary to configure the flight controller to use a receiver that talks MSP. This is easily done using one of the configurators. Connect to your flight controller using the configurator (inav or betaflight) and enter the "Configuration" tab. Inside this tab there's a field somewhere called "Receiver"  or "Receiver Mode". Click on the pull-down list and select "MSP RX (control via MSP port)". After any changes you almost always need to click on the "Save and Reboot". When the FC reboots it will be expecting to receive commands from one of the UARTs with MSP enabled, but that is NOT all. You still need to make sure your receiver is configured to use the correct channel map, AETR (Aileron, Elevator, Throttle and Rudder) is the default for YAMSPy and all RC commands will be expected to be Roll, Pitch, Throttle and Yaw. Additionally, the auxiliary channels will come just after Yaw following their own numbering (Aux1, Aux2... for Betaflight or CH5, CH6... for iNAV).  

## Advanced stuff
There are two possible ways to change the settings: using the configurator app (inav-configurator / betaflight-configurator) or using the CLI (Command Line Interface). The configurators are very handy, but the CLI allows you to automate stuff and save the settings in an easy to read text file. 

Flight controllers using Betaflight or iNAV will allow you to connect to its CLI using the default MSP enabled port (normally the one with a micro USB connector) and a simple terminal emulator like [PuTTy](https://www.putty.org/) (on Linux you have many options, but [PuTTy is super easy to install on Ubuntu](https://numato.com/blog/how-to-install-putty-on-linux/)) and setting the baud-rate to 115200 (the default value).  After connecting it is necessary to type ```#``` to start the CLI mode.


## TODO:
Currently the main library is made of a HUGE single file. That is just terrible, but I didn't have time to split it and test. So if you want to help, please, be my guest. Talking about testing, I haven't implemented any automated test besides the example files, again, be my guest ;)