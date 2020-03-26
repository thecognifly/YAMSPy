import io
import time
import numpy as np

import picamera

class FlowCap(io.IOBase):
    """
    Capturing Motion (Optical Flow'ish) from Raspiberry Pi h.264 hardware encoder
    in a "real-time" way.

    Heavily inspired by PiMotionAnalysis from picamera.array
    Some examples: https://picamera.readthedocs.io/en/release-1.13/recipes2.html

    Extra info:
    https://dspace.cvut.cz/bitstream/handle/10467/67320/F3-DP-2017-Heinrich-Adam-DP_Adam_Heinrich_2017_01.pdf
    https://github.com/h2r/pidrone_project_sensors/blob/master/student_analyze_flow.py
    https://www.edmundoptics.com/knowledge-center/application-notes/imaging/
    """

    def __init__(self, frameWidth=240, frameHeight=240,DEBUG = False):
        # Init the stuff we are inheriting from
        super(__class__, self).__init__()

        self.DEBUG = DEBUG

        # Used to filter noisy readings
        self.min_vel_x = 0.01
        self.min_vel_y = 0.01

        # Motion var
        self.x_motion = 0
        self.y_motion = 0

        # Set video frame parameters
        self.frameWidth = frameWidth
        self.frameHeight = frameHeight
        self.cols = self.rows = None

        # Set calc parameters
        self.max_flow = (((self.frameWidth) / 16) * ((self.frameHeight) / 16) * (64+64)) # 128 is the maximum value of the flow (+- 64)        
        self.flow_scale = .165
        self.flow_coeff  = 100 * self.flow_scale / self.max_flow # to get in meters

        # Set motion array parameters
        self.displacement = np.array([0., 0.])
        self.motion_dtype = np.dtype([
            (str('x'),   np.int8),
            (str('y'),   np.int8),
            (str('sad'), np.uint16),
            ])
            
        self.prev_time = time.time()

    def writable(self):
        '''
        To be a nice file, you must have this method
        '''
        return True

    def write(self, b):
        '''
        Here is where the motion / optical flow magic happens
        '''
        try:
            # Config the size of the numpy array according to frame size
            if self.cols is None:
                self.cols = ((self.frameWidth + 15) // 16) + 1
                self.rows = (self.frameHeight + 15) // 16

            data = (np.frombuffer(b, dtype=self.motion_dtype).reshape((self.rows, self.cols)))

            self.x_motion = np.sum(data['x'])*self.flow_coeff # this should be multiplied by the distance to the surface [m]
            self.y_motion = np.sum(data['y'])*self.flow_coeff # this should be multiplied by the distance to the surface [m]

            # Ignore any motion smaller than self.min_vel_x
            self.x_motion = 0 if abs(self.x_motion) < self.min_vel_x else self.x_motion

            # Ignore any motion smaller than self.min_vel_y
            self.y_motion = 0 if abs(self.y_motion) < self.min_vel_y else self.y_motion

            # Velocity to displacement
            self.displacement[0] += (self.x_motion*(time.time()-self.prev_time))
            self.displacement[1] += (self.y_motion*(time.time()-self.prev_time))                
             
            if self.DEBUG:
                print("FlowCap - x:{}, y:{}, d:{}, t:{}".format(self.x_motion, self.y_motion, self.displacement, time.time()))
                print("FlowCap - Running at {:2.2f} Hz".format(1/(time.time()-self.prev_time)))

            self.displacement.fill(0) # Reset the displacement
            self.prev_time = time.time()
        
        except Exception as e:
            print("FlowCap error: {}".format(e))
        finally:
            return  len(b) # to be a well behaved "file", you must return length


class ImgCap(io.IOBase):
    '''
    Capturing Image from a Raspicam (V2.1)
    '''
    def __init__(self, frameWidth=240, frameHeight=240, DEBUG = False):
        # Init the stuff we are inheriting from
        super(__class__, self).__init__()

        self.DEBUG = DEBUG

        # Set video frame parameters
        self.frameWidth = frameWidth
        self.frameHeight = frameHeight

        self.prev_time = time.time()

        self.output = None

    def writable(self):
        '''
        To be a nice file, you must have this method
        '''
        return True

    def write(self, b):
        '''
        Here is where the image data is received and made available at self.output
        '''

        try:
            # b is the numpy array of the image, 3 bytes of color depth
            self.output = np.reshape(np.frombuffer(b, dtype=np.uint8), (self.frameHeight, self.frameWidth, 3))

            if self.DEBUG:
                print("ImgCap - Image.shape {}".format(self.output.shape))
                print("ImgCap - Running at {:2.2f} Hz".format(1/(time.time()-self.prev_time)))

            self.prev_time = time.time()

        except Exception as e:
            print("ImgCap error: {}".format(e))
        finally:
            return len(b)


if __name__ == "__main__":

    import argparse


    DEBUG = True

    # See https://picamera.readthedocs.io/en/release-1.13/api_camera.html
    # for details about the parameters:
    frameWidth = 240 
    frameHeight = 240
    frameRate = 20
    contrast = 40
    rotation = 180
        
    # Set the picamera parametertaob
    # https://picamera.readthedocs.io/en/release-1.13/fov.html#
    camera = picamera.PiCamera()
    camera.resolution = (frameWidth, frameHeight)
    camera.framerate = frameRate
    camera.contrast = contrast
    camera.rotation = rotation

    # Start the video process
    with FlowCap(frameWidth, frameHeight, DEBUG) as flow,  ImgCap(frameWidth, frameHeight, DEBUG) as img:
        camera.start_recording(img, format='rgb', splitter_port = 2)
        camera.start_recording("/dev/null", format='h264', splitter_port = 1, motion_output=flow)
        try:
            while True:
                camera.wait_recording(timeout=0) # using timeout=0, default, it'll return immediately  
                # if img.output is not None:
                #     print(img.output[0,0,0])

        except KeyboardInterrupt:
            pass
        finally:
            camera.stop_recording(splitter_port = 1)
            camera.stop_recording(splitter_port = 2)
