import os
import time
from multiprocessing import Pool, Process, Pipe

import picamera

from analyze_camera import Flow, Poss


class Camera():
    '''
    This class is going to lauch the picamera
    '''
    def __init__(self,  pipe_of,
                        pipe_cv,
                        frameWidth=240, 
                        frameHeight=240,
                        frameRate=20,

                        DEBUG = False):
        '''
        Request frameWidth, frameHeight, frameRate
        '''
        # @ Set the video parameter
        self.width = frameWidth
        self.height = frameHeight
        self.framerate = frameRate
        self.contrast = 42
        # @ The Pipe for feeding the motion data and opencv data
        self.pipe_write_of, self.pipe_read_of = pipe_of
        self.pipe_write_cv, self.pipe_read_cv = pipe_cv

        # @ For Debug use
        self.DEBUG = DEBUG
       
    
    def run(self, nice_level):
        '''
        Launch the camera start streaming
        Having Pipe for write and read
        '''
        
        os.nice(nice_level)

        # @ Set the picamera parametertaob
        camera = picamera.PiCamera()
        camera.resolution = (self.width, self.height)
        camera.framerate = self.framerate
        camera.contrast = self.contrast
        # Start the video process
        # with Poss(self.pipe_read_cv, self.pipe_write_cv,frameWidth=self.width,frameHeight=self.height, DEBUG = self.DEBUG) as poss:
        with Flow(self.pipe_read_of, self.pipe_write_of, frameWidth=self.width,frameHeight=self.height, DEBUG = self.DEBUG) as flow:
                # camera.start_recording(poss, format='bgr', splitter_port = 2)
            camera.start_recording("/dev/null", format='h264', splitter_port=1, motion_output=flow)
            try:
                while True:
                    camera.wait_recording()
                    time.sleep(1)
            except KeyboardInterrupt:
                pass
            finally:
                camera.stop_recording(splitter_port=1)
                # camera.stop_recording(splitter_port=2)

# The value return in pipe read end
# from camera import Camera
# from multiprocessing import Pool, Process, Pipe
# pipe_read_of, pipe_write_of = Pipe()
# pipe_read_cv, pipe_write_cv = Pipe()
# camera = Camera((pipe_read_of, pipe_write_of),
#                 (pipe_read_cv, pipe_write_cv),
#                 frameWidth=240,
#                 frameHeight=240,
#                 frameRate = 30,
#                 DEBUG=False)
# nice_level = 5
# camera_start = Process(target=camera.run,
#                             args=(nice_level,))
# if pipe_read.poll():
#     return (pipe_read_of.recv())
# camera_start.terminate()
# camera_start.join()
