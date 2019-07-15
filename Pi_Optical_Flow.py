import os
import numpy as np
import time
import cv2
import picamera
from picamera.array import PiMotionAnalysis, PiRGBArray
from Optical_Flow_Filter import Filter


class GestureDetector(PiMotionAnalysis):
    QUEUE_SIZE = 10 # the number of consecutive frames to analyze
    THRESHOLD = 4.0 # the minimum average motion required in either axis

    def __init__(self, camera):
        super(GestureDetector, self).__init__(camera)
        self.x_queue = np.zeros(self.QUEUE_SIZE, dtype=np.float)
        self.y_queue = np.zeros(self.QUEUE_SIZE, dtype=np.float)
        self.last_move = ''

    def analyze(self, a):
        Filter().run(a)


def snap_shot(cam):
    camera = cam
    rawCapture = PiRGBArray(camera)
    camera.capture(rawCapture, format="bgr")
    img = rawCapture.array
    return img

with picamera.PiCamera(resolution='VGA', framerate=60) as camera:
    camera.resolution(640,480)
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
