import os
import time
import VL53L0X

class ToF():
    '''
    This class is going to feedback the altitude captured by ToF sensor
    '''
    def __init__(self, pipe_write, pipe_read):
        # Create Pipe
        self.pipe_read = pipe_read
        self.pipe_write = pipe_write 
        # Create a VL53L0X object
        self.tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)

        self.tof.open()
        
        # Start ranging
        # 'BEST', 'BETTER', 'GOOD', 'HIGH_SPEED', 'LONG_RANGE'
        self.tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.LONG_RANGE)

        self.timing = self.tof.get_timing()
        if self.timing < 20000:
            self.timing = 20000
        # print("Timing %d ms" % (self.timing/1000))

    def run(self, nice_level):
        os.nice(nice_level)
        try:
            while True:
                if not self.pipe_read.poll(): 
                    distance = self.tof.get_distance()
                    if distance > 0:
                        # print("%d mm, %d cm" % (distance, (distance/10)))
                        distance = int(distance/10) # Truncate 2 d.p.
                        self.pipe_write.send((distance/100, # in meters
                                                time.time())) 
                self.pipe_write.recv()
                # time.sleep(self.timing/1000000.00)

        finally:
            self.tof.stop_ranging()
            self.tof.close()
