import picamera
from Analyze import Flow, Poss
from multiprocessing import Pool, Process, Pipe

class Camera():
    '''
    This class is going to lauch the picamera
    '''
    def __init__(self, frameWidth=240, 
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
        self.pipe_read_of, self.pipe_write_of = Pipe()
        self.pipe_read_cv, self.pipe_write_cv = Pipe()

        # @ For Debug use
        self.DEBUG = DEBUG
       
    
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
        with Poss(self.pipe_read_cv, self.pipe_write_cv,frameWidth=self.width,frameHeight=self.height, DEBUG = self.DEBUG) as poss:
            with Flow(self.pipe_read_of, self.pipe_write_of, frameWidth=self.width,frameHeight=self.height, DEBUG = self.DEBUG) as flow:
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
        if self.pipe_read_of.poll():
            return (self.pipe_read_of.recv())
        else:
            return None

    def read_cv(self):
        if self.pipe_read_cv.poll():
            return (self.pipe_read_cv.recv())
        else:
            return None
