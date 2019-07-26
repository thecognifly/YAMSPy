import picamera
import os
from motion import Motion
from multiprocessing import Pool, Process, Pipe

class Camera():
    '''
    This class is going to lauch the picamera
    '''
    def __init__(self, frameWidth=240, 
                        frameHeight=240,
                        frameRate=20):
        '''
        Request frameWidth, frameHeight, frameRate
        '''
        # @ Set the video parameter
        self.width = frameWidth
        self.height = frameHeight
        self.framerate = frameRate
        self.contrast = 42
        # @ The motion data
        self.pipe_read, self.pipe_write = Pipe()
        # @ The Pipe for feeding the motion data
    
    def run(self):
        '''
        Launch the camera start streaming
        Having Pipe for write and read
        '''
        # @ Set the picamera parameter
        camera = picamera.PiCamera()
        camera.resolution = (self.width, self.height)
        camera.framerate = self.framerate
        camera.contrast = self.contrast
        # Start the video process
        with Motion(self.width, self.height) as motion:
            camera.start_recording(os.devnull, format='h264', motion_output=motion)
            try:
                while True:
                    camera.wait_recording(.01)
                    if motion.data is not None:
                        if not self.pipe_read.poll(): #If the pipe is clean, write the data in
                            self.pipe_write.send(motion.data)
            except KeyboardInterrupt:
                pass
            finally:
                camera.stop_recording()

    def read_motion(self):
        if self.pipe_read.poll():
            return (self.pipe_read.recv())
        else:
            return None
