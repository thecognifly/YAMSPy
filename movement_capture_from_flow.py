import os
from time import sleep
import numpy as np
import picamera
from picamera.array import PiMotionAnalysis

class MovementDetector(PiMotionAnalysis):
    QUEUE_SIZE = 10 # the number of consecutive frames to analyze
    THRESHOLD = 4.0 # the minimum average motion required in either axis
    SAD_THRESHOLD = 50
    data = []
    raw_data = []

    def __init__(self, camera):
        super(MovementDetector, self).__init__(camera)
        self.x_queue = np.zeros(self.QUEUE_SIZE, dtype=np.float)
        self.y_queue = np.zeros(self.QUEUE_SIZE, dtype=np.float)
        self.last_x = None

    def analyze(self, a):
        # Roll the queues and overwrite the first element with a new
        # mean (equivalent to pop and append, but faster)
        self.x_queue[1:] = self.x_queue[:-1]
        self.y_queue[1:] = self.y_queue[:-1]
        sad_mean = a['sad'].mean()
        if sad_mean > self.SAD_THRESHOLD:
            self.x_queue[0] = a['x'].mean()
            self.y_queue[0] = a['y'].mean()
        else:
            self.x_queue[0] = 0
            self.y_queue[0] = 0

        # Calculate the mean of both queues
        x_mean = self.x_queue.mean()
        y_mean = self.y_queue.mean()
        # Convert left/up to -1, right/down to 1, and movement below
        # the threshold to 0
        x_move = (
            '' if abs(x_mean) < self.THRESHOLD else
            'left' if x_mean < 0.0 else
            'right')
        y_move = (
            '' if abs(y_mean) < self.THRESHOLD else
            'down' if y_mean < 0.0 else
            'up')
        # Update the display
        movement = ('%s %s' % (x_move, y_move)).strip()
        print(movement)
        print(x_mean,y_mean, sad_mean)
        self.data.append((x_mean,y_mean,sad_mean))
        self.raw_data.append(a.copy())

with picamera.PiCamera(resolution='VGA', framerate=24) as camera:
    with MovementDetector(camera) as detector:
        camera.start_recording(
            os.devnull, format='h264', motion_output=detector)
        try:
            while True:
                camera.wait_recording(1)
        except KeyboardInterrupt:
            pass
        finally:
            camera.stop_recording()
            np.save('experiment', detector.data)
            np.save('experiment_raw', detector.raw_data)
            sleep(0.1)
