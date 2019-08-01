import numpy as np
import cv2
import time
import io

class Flow(io.IOBase):
    """
    Reference from PiMotionAnalyse
    Calc the velocity
    """

    def __init__(self, pipe_read, pipe_write,
                    frameWidth=240, frameHeight=240,
                    altitude = 100, #cm
                    DEBUG = False
                    ):
        # @ Init the io.IOBase
        super(__class__, self).__init__()

        # @ For Debug use
        self.DEBUG = DEBUG

        # @ Set the video frame parameter
        self.frameWidth = frameWidth
        self.frameHeight = frameHeight
        self.cols = self.rows = None

        # @ Set the calc parameter
        self.max_flow = (((self.frameWidth) / 16) * ((self.frameHeight) / 16) * 2**7)
        self.flow  = .165 / self.max_flow
        self.ALTITUDE = altitude

        # @ Set the motion array parameter
        self.data = None
        self.motion_dtype = np.dtype([
            (str('x'),   np.int8),
            (str('y'),   np.int8),
            (str('sad'), np.uint16),
            ])
            
        # @ The Pipe for feeding the motion data
        self.pipe_read = pipe_read
        self.pipe_write = pipe_write

    def writable(self):
        '''
        Act like a file-like class
        '''
        return True

    def write(self, b):
        '''
        Act like a file-like class
        '''
        # Config the size of the numpy array
        if self.DEBUG:
            start = time.time()
        if self.cols is None:
            self.cols = ((self.frameWidth + 15) // 16) + 1
            self.rows = (self.frameHeight + 15) // 16
        # If the pipe is clean, write the data in
        # We do all the calc when pipe is usable, help to save processing power
        if not self.pipe_read.poll(): 
            data = (np.frombuffer(b, dtype=self.motion_dtype).reshape((self.rows, self.cols)))
            x_motion = np.sum(data['x'])*self.flow*self.ALTITUDE
            y_motion = np.sum(data['y'])*self.flow*self.ALTITUDE
            x_motion = 0 if abs(x_motion) < 0.1 else x_motion
            y_motion = 0 if abs(y_motion) < 0.1 else y_motion
            self.pipe_write.send((x_motion, y_motion))
        if self.DEBUG:
            print("FLOW - Running at %2.2f Hz"%(1/(time.time()-start)))
        return  len(b)      

class Poss(io.IOBase):
    '''
    Using Opencv estimateRigidTransform to calc the position.
    '''
    def __init__(self, pipe_read, pipe_write,
                    frameWidth=240, frameHeight=240,
                    DEBUG = False):
        # @ Init the io.IOBase
        super(__class__, self).__init__()

        # @ For Debug use
        self.DEBUG = DEBUG

        # @ Set the video frame parameter
        self.frameWidth = frameWidth
        self.frameHeight = frameHeight

        # @ The Pipe for feeding the motion data
        self.pipe_read = pipe_read
        self.pipe_write = pipe_write

        # @ Set the opencv parameter
        self.flag_fi = True # Capture the first  frame
        self.imgPost = self.imgNow = None
        algorithms = {'ORB':  cv2.ORB_create,
                      'SIFT': cv2.xfeatures2d.SIFT_create,
                      'FAST': cv2.FastFeatureDetector_create,
                      'SURF': cv2.xfeatures2d.SURF_create,
                      'BRIEF':cv2.xfeatures2d.BriefDescriptorExtractor_create}
        algorithm2use = 'ORB' # 'BRIEF', 'FAST', 'ORB', 'SURF', 'SIFT'
        number_of_features = 50 # Limited in 50
        self.fea_det = algorithms[algorithm2use](number_of_features)       

    def img2kpt(self, img):
        '''
        Find all the keypoint inside the image
        '''
        kpts, des = self.fea_det.detectAndCompute(img, None)
        return (kpts, des)       

    def BF_filter(self, kt, dt, kq, dq):
        '''
        This function is using Brute-Force to calc the displacement different 
        calc triangle different between two frame can know the altitude change
        and calc the position drift
        return (drift_pos, area_diff)
        '''
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(dq,dt)
        if (len(matches)) > 3 :
            matches = sorted(matches, key = lambda x:x.distance)
            ctn = 0
            for m in matches:
                if m.distance <= 38 : # if the match distance larger than 38
                    ctn+=1            # Normally is not match correctly
                else:
                    break # stop the loop for faster loop
            
            if ctn > 3: # At least 3 points for calc triangle 
                # >> having a numpy array for faster calc
                # numpy array [x, y], [x, y], [x, y]
                pt_pre = np.array([[kq[matches[0].queryIdx].pt[0], kq[matches[0].queryIdx].pt[1]],
                                [kq[matches[1].queryIdx].pt[0], kq[matches[1].queryIdx].pt[1]],
                                [kq[matches[2].queryIdx].pt[0], kq[matches[2].queryIdx].pt[1]]])

                pt_now = np.array([[kt[matches[0].trainIdx].pt[0], kt[matches[0].trainIdx].pt[1]],
                                [kt[matches[1].trainIdx].pt[0], kt[matches[1].trainIdx].pt[1]],
                                [kt[matches[2].trainIdx].pt[0], kt[matches[2].trainIdx].pt[1]]])
                # >>> drift_pos = [drift_x, drift_y]
                drift_pos = ((np.sum((pt_pre-pt_now), axis=0))/3)
                # >>> length = [a, b, c]
                length_pre = np.array([ (np.sqrt(np.sum((pt_pre[0]-pt_pre[1])**2))),
                                        (np.sqrt(np.sum((pt_pre[1]-pt_pre[2])**2))),
                                        (np.sqrt(np.sum((pt_pre[2]-pt_pre[0])**2)))])

                length_now = np.array([ (np.sqrt(np.sum((pt_now[0]-pt_now[1])**2))),
                                        (np.sqrt(np.sum((pt_now[1]-pt_now[2])**2))),
                                        (np.sqrt(np.sum((pt_now[2]-pt_now[0])**2)))])
                # Heron's Flormula
                # Area = sqrt (s*(s-a)*(s-b)*(s-c))
                s_pre = (np.sum(length_pre))/2
                s_now = (np.sum(length_now))/2
                # area_diff = area_now - area_pre
                area_diff= np.sqrt(s_now*np.prod(s_now-length_now)) - np.sqrt(s_pre*np.prod(s_pre-length_pre))
                return (drift_pos, area_diff)
        else:
            return (None, None)

    def writable(self):
        '''
        Act like a file-like class
        '''
        return True

    def write(self, b):
            '''
            Act like a file-like class
            '''
            if self.DEBUG:
                start = time.time()
            # b is the numpy array of the image, 3 bytes of color depth
            if not self.pipe_read.poll(): 
                img = np.reshape(np.fromstring(b, dtype=np.uint8), (self.frameHeight, self.frameWidth, 3))
                if self.flag_fi:
                    # Capture the first image
                    self.flag_fi = False
                    self.imgNow = self.imgPost = img
                else:
                    # Finding the feature inside the frame
                    # First image is Query Image
                    # Second image is Train Image
                    try:
                        # Capture feature
                        kq, dq = self.fea_det.detectAndCompute(self.imgNow, None) 
                        kt, dt = self.fea_det.detectAndCompute(img, None)
                        self.pipe_write.send(self.BF_filter(kt, dt, kq, dq))
                        self.imgNow = img
                    except:
                        pass
            if self.DEBUG:
                print("POSS - Running at %2.2f Hz"%(1/(time.time()-start)))
            return len(b)
