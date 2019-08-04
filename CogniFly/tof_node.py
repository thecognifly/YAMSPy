import os
import time
import VL53L0X
import kf_tof

class ToF():
    '''
    This class is going to feedback the altitude captured by ToF sensor
    '''
    def __init__(self, pipe_read, pipe_write):
        # Create Pipe
        self.pipe_read = pipe_read
        self.pipe_write = pipe_write 
        # Create a VL53L0X object
        self.tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)
        # I2C Address can change before tof.open()
        # tof.change_address(0x32)
        self.tof.open()
        # Start ranging
        # 'BEST', 'BETTER', 'GOOD', 'HIGH_SPEED', 'LONG_RANGE'
        self.tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)

        self.timing = self.tof.get_timing()
        if self.timing < 20000:
            self.timing = 20000
        # print("Timing %d ms" % (self.timing/1000))

    def run(self, nice_level):
        os.nice(nice_level)
        try:
            while True:
                st = time.time()
                data = (self.tof.get_distance()/10)
                t = (st-time.time())
                kf_tof.update(data, t)
                if not self.pipe_read.poll():
                    self.pipe_write.send((kf_tof.tof()), t) # in cm, cm/s
                time.sleep(self.timing/1000000.00)

        finally:
            self.tof.stop_ranging()
            self.tof.close()
