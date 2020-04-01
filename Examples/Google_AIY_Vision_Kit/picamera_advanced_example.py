import io
import time
import sys
import numpy as np

import cv2

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
        super().__init__()

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
            # https://stackoverflow.com/a/41642105
            print(f'FlowCap: {type(e).__name__} {e} on line {sys.exc_info()[-1].tb_lineno}')
        finally:
            return  len(b) # to be a well behaved "file", you must return length


class ImgCap(io.IOBase):
    '''
    Capturing Image from a Raspicam (V2.1)
    '''
    def __init__(self, frameWidth=240, frameHeight=240, DEBUG = False):
        # Init the stuff we are inheriting from
        super().__init__()

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
            print(f'ImgCap: {type(e).__name__} {e} on line {sys.exc_info()[-1].tb_lineno}')
        finally:
            return len(b)


class AbsPosition(io.IOBase):
    '''
    Using Opencv estimateRigidTransform to calc the position between two frames.

    algorithm2use can be 'ORB', 'BRIEF', 'FAST', 'ORB', 'SURF' or 'SIFT'
    '''
    def __init__(self, frameWidth=240, frameHeight=240, 
                       number_of_features = 50, algorithm2use = 'ORB', 
                       match_dist = 38, DEBUG = False):
        # Init the stuff we are inheriting from
        super().__init__()

        self.DEBUG = DEBUG

        self.algorithm2use = algorithm2use
        self.match_dist = match_dist

        # Set video frame parameters
        self.frameWidth = frameWidth
        self.frameHeight = frameHeight

        # Pixel size of Raspicam (V2.1)
        self.PIXEL_SIZE_M = (16*1.4) * 10e-6 # 1.4um 4x4 binning

        # Focal Length of Raspicam (V2.1)
        self.FOCAL_LENGTH_M = 3.6 * 10e-3 # 3.6mm

        # Unit to ground meter 
        self.scale = self.PIXEL_SIZE_M/self.FOCAL_LENGTH_M

        self.img_prev = None

        # Set the opencv parameter
        algorithms = {'ORB':  cv2.ORB_create,
                      'SIFT': cv2.xfeatures2d.SIFT_create,
                      'FAST': cv2.FastFeatureDetector_create,
                      'SURF': cv2.xfeatures2d.SURF_create,
                      'BRIEF':cv2.xfeatures2d.BriefDescriptorExtractor_create}

        self.fea_det = algorithms[algorithm2use](number_of_features) 

        if algorithm2use in ['ORB', 'BRISK', 'BRIEF']:
            self.bf = cv2.BFMatcher(cv2.NORM_HAMMING2, crossCheck=False)
        else:
            self.bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)

    def img2kpt(self, img):
        '''
        Find all the keypoint inside the image
        '''
        kpts, des = self.fea_det.detectAndCompute(img, None)
        return (kpts, des)       

    def BF_filter(self, kp_curr, des_curr, kp_prev, des_prev): # kt, dt, kq, dq):
        '''
        This function is using Brute-Force to calc the displacement difference.
        Calc triangle different between two frames to know the altitude change
        and the position drift.
        return (drift_pos, area_diff)
        '''

        if self.algorithm2use in ['ORB', 'BRISK', 'BRIEF']:
            matches = self.bf.match(np.asarray(des_prev, np.uint8), np.asarray(des_curr, np.uint8))
        else:
            matches = self.bf.match(np.asarray(des_prev, np.float32), np.asarray(des_curr, np.float32))

        print(f'Number of matches: {len(matches)}')
        if (len(matches)) > 3 :
            # Sort them in the order of their distance.
            matches = sorted(matches, key = lambda x:x.distance)
            ctn = 0
            for m in matches:
                if (m.distance <= self.match_dist) and ctn < 3:
                    ctn+=1
                else:
                    break
            
            if ctn == 3: # At least 3 points for calc triangle 
                # numpy array [x, y], [x, y], [x, y]
                pt_pre = np.array([[kp_prev[matches[0].queryIdx].pt[0], kp_prev[matches[0].queryIdx].pt[1]],
                                   [kp_prev[matches[1].queryIdx].pt[0], kp_prev[matches[1].queryIdx].pt[1]],
                                   [kp_prev[matches[2].queryIdx].pt[0], kp_prev[matches[2].queryIdx].pt[1]]])

                pt_now = np.array([[kp_curr[matches[0].trainIdx].pt[0], kp_curr[matches[0].trainIdx].pt[1]],
                                   [kp_curr[matches[1].trainIdx].pt[0], kp_curr[matches[1].trainIdx].pt[1]],
                                   [kp_curr[matches[2].trainIdx].pt[0], kp_curr[matches[2].trainIdx].pt[1]]])
                # drift_pos = [drift_x, drift_y]
                drift_pos = ((np.sum((pt_pre-pt_now), axis=0))/3)
                # length = [a, b, c]
                length_pre = np.array([ (np.sqrt(np.sum((pt_pre[0]-pt_pre[1])**2))),
                                        (np.sqrt(np.sum((pt_pre[1]-pt_pre[2])**2))),
                                        (np.sqrt(np.sum((pt_pre[2]-pt_pre[0])**2)))])

                length_now = np.array([ (np.sqrt(np.sum((pt_now[0]-pt_now[1])**2))),
                                        (np.sqrt(np.sum((pt_now[1]-pt_now[2])**2))),
                                        (np.sqrt(np.sum((pt_now[2]-pt_now[0])**2)))])
                # Heron's Formula (https://en.wikipedia.org/wiki/Heron%27s_formula)
                # Area = sqrt (s*(s-a)*(s-b)*(s-c))
                s_pre = (np.sum(length_pre))/2
                s_now = (np.sum(length_now))/2
                # area_diff = area_now - area_pre
                area_diff = np.sqrt(s_now*np.prod(s_now-length_now)) - np.sqrt(s_pre*np.prod(s_pre-length_pre))
                return (drift_pos*self.scale, area_diff)
        
        return ((None, None), None)

    def writable(self):
        '''
        Act like a file-like class
        '''
        return True

    def write(self, b):
        '''
        Act like a file-like class
        '''
        try:
            self.prev_time = time.time()
            # b is the numpy array of the image, 3 bytes of color depth
            img_curr = cv2.cvtColor(np.reshape(np.frombuffer(b, dtype=np.uint8), (self.frameHeight, self.frameWidth, 3)), cv2.COLOR_BGR2GRAY)
            if not isinstance(self.img_prev, np.ndarray):
                self.img_prev = img_curr
            else:
                # https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_matcher/py_matcher.html
                kp_prev, des_prev = self.fea_det.detectAndCompute(self.img_prev, None) 
                kp_curr, des_curr = self.fea_det.detectAndCompute(img_curr, None)
                if len(kp_prev) and len(kp_curr):
                    print("AbsPosition - ", self.BF_filter(kp_curr, des_curr, kp_prev, des_prev))
                self.img_prev = img_curr
                
            if self.DEBUG:
                print("AbsPosition - Running at {:2.2f} Hz".format(1/(time.time()-self.prev_time)))
        
        except Exception as e:
            print(f'AbsPosition: {type(e).__name__} {e} on line {sys.exc_info()[-1].tb_lineno}')
        
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
    with FlowCap(frameWidth=frameWidth, frameHeight=frameHeight, DEBUG=DEBUG) as flow, \
         AbsPosition(frameWidth=frameWidth, frameHeight=frameHeight, DEBUG=DEBUG) as abspos:
    # with FlowCap(frameWidth=frameWidth, frameHeight=frameHeight, DEBUG=DEBUG) as flow, \
    #      ImgCap(frameWidth=frameWidth, frameHeight=frameHeight, DEBUG=DEBUG) as img:
        camera.start_recording(abspos, format='bgr', splitter_port = 2)
        # camera.start_recording(img, format='rgb', splitter_port = 2)
        camera.start_recording("/dev/null", format='h264', splitter_port = 1, motion_output=flow)
        try:
            while True:
                camera.wait_recording(timeout=0) # using timeout=0, default, it'll return immediately  
                # if img.output is not None:
                #     print(img.output[0,0,0])

        except KeyboardInterrupt:
            pass
        finally:
            print("Stopping picamera...")
            camera.stop_recording(splitter_port = 1)
            camera.stop_recording(splitter_port = 2)
