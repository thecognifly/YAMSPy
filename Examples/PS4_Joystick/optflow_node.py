import os
import time

from filterpy.memory import FadingMemoryFilter
# https://filterpy.readthedocs.io/en/latest/memory/FadingMemoryFilter.html

#pip3 install git+https://github.com/ricardodeazambuja/vl53l1x-python
from pmw3901 import PMW3901


NODE_STR = "[OPTFLOW NODE]"

class OptFlow():
    def __init__(self, pipe_write, pipe_read, dt = None, beta = 0.0):
        self.pipe_read = pipe_read # it's useful for checking if the last value was consumed
        self.pipe_write = pipe_write

        # spi_port=1, spi_cs=2, spi_cs_gpio=16 (pin 36) => compatible with the hat
        # Using /boot/config.txt and dtoverlay=spi1-3cs
        keep_trying = True
        while keep_trying:
            try:
                self.flo = PMW3901(spi_port=1, spi_cs=2, spi_cs_gpio=16, secret_sauce=3)
                print(NODE_STR + "Sensor initialized!")
                keep_trying = False
            except RuntimeError:
                print(NODE_STR + "Sensor not initialized...")        
                time.sleep(0.5)
        self.flo.set_rotation(180) # X points to the same direction as the drone (Z up)

        self.OptFlowMeasurementData = {'dx':0, 'dy':0, 'Quality':0, 'time_stamp':0, 'flow_cte': 30.0/(4.2*3.14/180.0)}

        freq = 1/self.flo.SAMPLE_INTERVAL

        self.prev_dx = 0
        self.prev_dy = 0

        dt = dt if dt else freq
        self.filter_x = FadingMemoryFilter([self.prev_dx, 0.], dt, order=1, beta=beta)
        self.filter_y = FadingMemoryFilter([self.prev_dy, 0.], dt, order=1, beta=beta)

        print(NODE_STR + "OptFlow Node Initialized @ {}Hz!".format(freq))

    def close(self):
        self.flo.reset()
        print(NODE_STR + "Shutting down OptFlow Node...")

    def read(self):
        dx, dy, q = self.flo.get_motion_with_quality()
        self.OptFlowMeasurementData['time_stamp'] = time.time()        

        if not self.pipe_read.poll(): #timeout=None => blocking
            self.filter_x.update(dx)
            self.filter_y.update(dy)
            dx = self.filter_x.x[0]
            self.prev_dx = dx
            dy = self.filter_y.x[0]
            self.prev_dy = dy

            # print(NODE_STR + "New reading!")
            self.OptFlowMeasurementData['dx'] = dx
            self.OptFlowMeasurementData['dy'] = dy
            self.OptFlowMeasurementData['Quality'] = q
            self.pipe_write.send(self.OptFlowMeasurementData)
        # else:
        #     print(NODE_STR + "No readings...")

        return
