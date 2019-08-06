import numpy as np
import io

class Motion(io.IOBase):
    """
    Reference from PiMotionAnalyse
    """

    def __init__(self, frameWidth=240, 
                    frameHeight=240, 
                    size=None):
        # @ Init the io.IOBase
        super(Motion, self).__init__()
        # @ Set the video frame parameter
        self.frameWidth = frameWidth
        self.frameHeight = frameHeight
        self.cols = self.rows = None
        # @ Set the motion array parameter
        self.data = None
        self.motion_dtype = np.dtype([
            (str('x'),   np.int8),
            (str('y'),   np.int8),
            (str('sad'), np.uint16),
            ])

    def writable(self):
        '''
        Act like a file-like class
        '''
        return True

    def write(self, b):
        '''
        Act like a file-like class
        self.data is the motion we want!
        '''
        if self.cols is None:
            self.cols = ((self.frameWidth + 15) // 16) + 1
            self.rows = (self.frameHeight + 15) // 16
        self.data = (np.frombuffer(b, dtype=self.motion_dtype).reshape((self.rows, self.cols)))
        return  len(b)      

 