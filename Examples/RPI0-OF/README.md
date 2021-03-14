This directory has some example code on autonomous flight using a RPI Zero W, a Raspicam and a ToF sensor for position control developed during [Tom's](https://github.com/cmftom) internship at [MISTLab.ca](https://github.com/MISTLab). It was added here because we consider it an interesting stepping stone for someone planning to only use the RPI Zero W + Raspicam without the need for the Matek Optical Flow and Lidar sensor.

### TODO
- Test if it still works with the current version of YAMSPy.
- Improve documentation.
- Clean the code.
- Change the low level control from the RPI to the flight controller: send optical flow and altitude values (rangefinder) to be read directly by the flight controller as the [Matek Optical Flow and Lidar sensor](http://www.mateksys.com/?portfolio=3901-l0x) does (MSP).



This code was strongly inspired by many online sources (e.g. [PiDrone](https://github.com/h2r/pidrone_pkg) and [piDrone](https://github.com/PiStuffing/Quadcopter)).
