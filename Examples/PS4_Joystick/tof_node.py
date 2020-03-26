import os
import time

from filterpy.memory import FadingMemoryFilter
# https://filterpy.readthedocs.io/en/latest/memory/FadingMemoryFilter.html

#pip3 install git+https://github.com/ricardodeazambuja/vl53l1x-python
import VL53L1X


NODE_STR = "[TOF NODE]"

class ToF():
    def __init__(self, pipe_write, pipe_read, time_budget = 40, time_period = 50, dt = None, beta = 0.0):
        self.pipe_read = pipe_read # it's useful for checking if the last value was consumed
        self.pipe_write = pipe_write 
        
        self.tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
        self.tof.open(reset=True)


        self.time_budget = time_budget
        self.time_period = time_period # TIME_PERIOD MUST BE > THAN TIME_BUDGET        

        freq = 1/(self.time_period/1000)

        self.tof.set_timing(self.time_budget*1000, self.time_period)
        self.tof.start_ranging(0)

        self.prev_d = 0

        dt = dt if dt else freq
        self.filter_d = FadingMemoryFilter([self.prev_d, 0.], dt, order=1, beta=beta)

        print(NODE_STR + "ToF Node Initialized @ {}Hz!".format(freq))


    def close(self):
        self.tof.stop_ranging()
        print(NODE_STR + "Shutting down ToF...")

    def read(self):
        RangingMeasurementData = self.tof.get_RangingMeasurementData()
        RangingMeasurementData['time_stamp'] = time.time()
        
        if not self.pipe_read.poll(): #timeout=None => blocking
            # print(NODE_STR + "New reading!")
            if not RangingMeasurementData['RangeStatus']:
                d = RangingMeasurementData['RangeMilliMeter']
                self.filter_d.update(d)
                d = self.filter_d.x[0]
                self.prev_d = d
                RangingMeasurementData['RangeMilliMeter'] = d

                self.pipe_write.send(RangingMeasurementData) 
                # Important Keys:
                # - RangeMilliMeter
                # - SigmaMilliMeter
                # - RangeStatus
            else:
                print(NODE_STR + "RangeStatus problem")
