import picamera
from Analyze import Flow, Poss
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
        # @ The Pipe for feeding the motion data
        self.pipe_read, self.pipe_write = Pipe()
       
    
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
        with Poss(frameWidth=self.width,frameHeight=self.height) as poss:
            with Flow(self.pipe_read, self.pipe_write, frameWidth=self.width,frameHeight=self.height) as flow:
                camera.start_recording(poss, format='bgr', splitter_port = 1)
                camera.start_recording("/dev/null", format='h264', splitter_port=2, motion_output=flow)
                try:
                    while True:
                        camera.wait_recording(.01)
                except KeyboardInterrupt:
                    pass
                finally:
                    camera.stop_recording(splitter_port=1)
                    camera.stop_recording(splitter_port=2)

    def read_motion(self):
        # return (self.pipe_read.recv())
        if self.pipe_read.poll():
            return (self.pipe_read.recv())
        else:
            return None
