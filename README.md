[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.4306818.svg)](https://doi.org/10.5281/zenodo.4306818)

# YAMSPy
Yet Another Implementation of [Multiwii](https://github.com/multiwii) Serial Protocol Python Interface for [Betaflight](https://github.com/betaflight/betaflight), [iNAV](https://github.com/iNavFlight/inav), etc.

## <span style="color:orange;">SAFETY FIRST!</span>
This work is ***EXPERIMENTAL*** and it has been made publicly available expecting to be used only by responsible legally capable adults, etc, etc, etc. Since it's experimental, crazy things may happen mid-flight or even after the drone has landed or, even worse, while the drone is sitting on the floor! Drones, by default, can be **VERY** dangerous if not handled properly. Here are some links that may help you understand what I mean:
- https://www.tc.gc.ca/en/services/aviation/drone-safety.html
- https://store.dji.com/guides/10-drone-safety-tips-safe-flight/
- http://knowbeforeyoufly.org/
- [LiPo batteries themselves need TLC to be safe](https://www.robotshop.com/media/files/pdf/hyperion-g5-50c-3s-1100mah-lipo-battery-User-Guide.pdf)
- [Videos where they test drone impacts against... pork](https://www.youtube.com/channel/UCu_iJC8iWK9VYzrXeAvzi1g/videos)
- [Finally, your classic Mythbusters video](https://www.youtube.com/watch?v=xvyMmKKSGFg)

### <span style="color:orange;">**Disclaimer (adapted from Wikipedia):** None of the authors, contributors, supervisors administrators, employers, friends, family, vandals, or anyone else connected (or not) with this project, in any way whatsoever, can be made responsible for the use of the information (code) contained or linked from here.</span>

## Installation (Python3 because [you should not use Python2 anymore](https://www.python.org/doc/sunset-python-2/)):
Option #1: Clone the repo so you will have the examples
```
$ git clone https://github.com/ricardodeazambuja/YAMSPy.git
$ cd YAMSPy
$ sudo pip3 install .
```

Option #2: Install directly from git (the `--upgrade` is to make sure it will install the last commit, even if the version number didn't increase)
```
$ sudo pip3 install git+git://github.com/ricardodeazambuja/yamspy --upgrade
```

On Linux you may need to add your user to the dialout group:  
```
$ sudo usermod -a -G dialout $USER
```

## Examples:
The directory [Examples](https://github.com/ricardodeazambuja/YAMSPy/tree/master/Examples) (duh!) has some commented examples showing how to use the library.

## Setting up your flight controller (FC):
Until this point you should have installed YAMSPy, but it will not magically work without some extra steps. In order to make it work, you need to set up your FC correctly.

If you are just starting with iNAV or Betaflight you will need to install one of the configurators: [inav-configurator](https://github.com/iNavFlight/inav-configurator/releases) or [betaflight-configurator](https://github.com/betaflight/betaflight-configurator/releases). 

### How to check which firmware is installed in your flight controller
In general, the most popular flight controllers (so far I have tested [Heli-nation Talon F7 Fusion](https://www.heli-nation.com/talon-f7-fusion-flight-controller) and [Kakute F7 mini](http://www.holybro.com/product/kakute-f7-mini/) with success) come with some version of Betaflight and if you try to use inav-configurator it will only show the CLI (Command Line Interface) tab. The other way around, iNAV installed and betaflight-configurator, it will still allow you to use the CLI. The good thing is that using the CLI you can enter [DFU (Device Firmware Update) mode](https://www.youtube.com/watch?v=XKoZ_qrOtXg) to reflash the firmware by simply typing ```dfu``` if iNAV firmware or ```bl``` or ```dfu``` if Betaflight. I noticed that my flight controlllers would only enter DFU mode if they were powered exclusivelly by the USB. To flash (install) a new firmware you can follow the instructions for [Betaflight](https://github.com/betaflight/betaflight/wiki/Installing-Betaflight) or [iNAV](https://github.com/iNavFlight/inav/blob/master/docs/Installation.md). BTW, once you are using the CLI mode you can just type ```status``` to receive a lot of useful info like firmware version.

### More details about how YAMSPy works
YAMSPy was designed to communicate with or control a flight controller from a Single Board Computer (SBC) like a Raspberry Pi as well as a normal PC. It works by communicating with the flight controller through a serial connection using MSP ([MultiWii Serial Protocol](https://github.com/multiwii)). Therefore, you need to have a free UART (Universal asynchronous receiver-transmitter, commonly just called "serial") that has MSP enabled on it. By default (AFAIK), the micro USB connector located on the flight controller will be seen as an UART and it comes configured as MSP, so it should allow you to use YAMSPy out of the box. You can easily enable MSP on other UART using the configurator software (see "Ports" tab).

Ok, the paragraph above covers the basics, but to be able to use YAMSPy to control your drone it is necessary to configure the flight controller to use a receiver that talks MSP. This is easily done using one of the configurators mentioned above. Connect to your flight controller using the configurator (inav or betaflight) and enter the "Configuration" tab. Inside this tab there's a field somewhere called "Receiver" or "Receiver Mode". Click on the pull-down list and select "MSP RX (control via MSP port)". After any changes you always need to click on the "Save and Reboot". When the FC reboots it will be expecting to receive commands from one of the UARTs with MSP enabled, but that is NOT all. You still need to make sure your receiver is configured to use the correct channel map. AETR (Aileron, Elevator, Throttle and Rudder) is the default for YAMSPy and all RC commands will be expected to start with Roll, Pitch, Throttle and Yaw, exactly in this order. Additionally, the auxiliary channels will come just after Yaw following their own numbering (Aux1, Aux2... for Betaflight or CH5, CH6... for iNAV).  

Finally, I would strongly suggest you to set, at least, a channel exclusivelly for arming / disarming the drone instead of using [stick commands](https://github.com/martinbudden/betaflight/blob/master/docs/Controls.md). This can be done using the configurator and the "Modes" tab.  

## Advanced stuff
### CLI
There are two possible ways to change the settings: using the configurator app (inav-configurator / betaflight-configurator) or using the CLI (Command Line Interface). The configurators are very handy, but the CLI allows you to automate stuff and save the settings in an easy to read text file. 

Flight controllers using Betaflight or iNAV will allow you to connect to its CLI using the default MSP enabled port (normally the one with a micro USB connector) and a simple terminal emulator like [PuTTy](https://www.putty.org/) (on Linux you have many options, but [PuTTy is super easy to install on Ubuntu](https://numato.com/blog/how-to-install-putty-on-linux/)) and setting the baud-rate to 115200 (the default value).  After connecting it is necessary to type ```#``` to start the CLI mode. The FC will NOT exit CLI mode if you disconnect your terminal session, what can be useful sometimes.  

If you feel adventurous, you can try to connect using [linux screen](https://linuxize.com/post/how-to-use-linux-screen/).
```
$ screen /dev/ttyACM0 115200
```
Screen can be as annoying as [VIM](https://stackoverflow.blog/2017/05/23/stack-overflow-helping-one-million-developers-exit-vim/) sometimes, so my commands:
- ```ctrl+a and then \``` to exit.
- ```ctrl+a and the ESC key``` to move around using the arrow keys (to stop this behaviour just press the ESC key again.
- ```ctrl+a, then : and finally H``` to save everything printed on the screen after that point.


## Troubleshooting
If you can't connect (talk) to the FC:
1. Check if you enabled MSP in the correct UART using inav-configurator (or betaflight-configurator)
2. Make sure you connected the cables correctly: TX => RX and RX => TX
3. Verify the devices available using ```ls -lh /dev/serial*``` and change it in the Python script if needed.

## Stargazers over time
[![Stargazers over time](https://starchart.cc/thecognifly/YAMSPy.svg)](https://starchart.cc/thecognifly/YAMSPy)

## TODO:
Currently the main library is made of a HUGE single file. That is just terrible, but I didn't have time to split it and test. So if you want to help, please, be my guest. Talking about testing, I haven't implemented any automated test besides the example files, again, be my guest ;)


## Acknowledgments:
Many people from [MISTLab](http://mistlab.ca/) helped during the development of this library. Special thanks goes to [Tom](https://github.com/cmftom) and [Yann](https://github.com/yannbouteiller).


This work was possible thanks to the financial support from [IVADO.ca (postdoctoral scholarship 2019/2020)](https://ivado.ca/en/ivado-scholarships/postdoctoral-scholarships/).
