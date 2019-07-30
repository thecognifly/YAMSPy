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
                    altitude = 100 #cm
                    ):
        # @ Init the io.IOBase
        super(__class__, self).__init__()

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
        print("FLOW - Running at %2.2f Hz"%(1/(time.time()-start)))
        return  len(b)      

class Poss(io.IOBase):
    '''
    Using Opencv estimateRigidTransform to calc the position.
    '''
    def __init__(self, frameWidth=240, frameHeight=240
                    ):
        # @ Init the io.IOBase
        super(__class__, self).__init__()

        # @ Set the video frame parameter
        self.frameWidth = frameWidth
        self.frameHeight = frameHeight

        # @ Set the opencv parameter
        self.flag_1 = True
        self.imgPost = self.imgNow = None
        algorithms = {'ORB':  cv2.ORB_create,
                            'SIFT': cv2.xfeatures2d.SIFT_create,
                            'FAST': cv2.FastFeatureDetector_create,
                            'SURF': cv2.xfeatures2d.SURF_create,
                            'BRIEF':cv2.xfeatures2d.BriefDescriptorExtractor_create}
        algorithm2use = 'ORB' # 'BRIEF', 'FAST', 'ORB', 'SURF', 'SIFT'
        number_of_features = 50 # Limited in 50
        self.fea_det = algorithms[algorithm2use](number_of_features)
        # Set FLANN parameters 
        FLANN_INDEX_LSH = 6
        index_params= dict(algorithm = FLANN_INDEX_LSH,
                        table_number = 6, # 12
                        key_size = 12,     # 20
                        multi_probe_level = 1) #2
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params,search_params)
        

    def img2kpt(self, img):
        '''
        Find all the keypoint inside the image
        '''
        kpts, des = self.fea_det.detectAndCompute(img, None)
        return (kpts, des)

    def flann_filter(self, kt, dt, kq, dq):
        '''
        Using flann_filter
        '''
        matches = self.flann.knnMatch(dq, dt, k=2)
        matchesMask = [[0,0] for i in range(len(matches)) if len(matches[i])>1]
        filtered_matches = []
        # ratio test as per Lowe's paper
        for i,mn in enumerate(matches):
            if len(mn)>1:
                m = mn[0]
                n = mn[1]
                if m.distance < 0.7*n.distance:
                    matchesMask[i]=[1,0]
                    filtered_matches.append(i)
       

    def BF_filter(self, kt, dt, kq, dq):
        '''
        This function is using Brute-Force to calc the displacement different 
        return x_diff, y_diff
        '''
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(dq,dt)
        if (len(matches)) > 5 :
            matches = sorted(matches, key = lambda x:x.distance)
            img1_pos = (kq[matches[0].queryIdx].pt)
            img2_pos = (kt[matches[0].trainIdx].pt)
            return ((img2_pos[0]-img1_pos[0]), (img2_pos[1]-img1_pos[1]))
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
        start = time.time()
        # b is the numpy array of the image, 3 bytes of color depth
        img = np.reshape(np.fromstring(b, dtype=np.uint8), (self.frameHeight, self.frameWidth, 3))
        M = np.float32([[1,0,100],[0,1,100]])
        img = cv2.warpAffine(img, M, (img.shape[0],img.shape[1]))
        if self.flag_1:
            # Capture the first image
            self.flag_1 = False
            self.imgNow = self.imgPost = img
        else:
            # Finding the feature inside the frame
            # First image is Query Image
            # Second image is Train Image
            try:
                # Caputre feature
                kq, dq = self.fea_det.detectAndCompute(self.imgNow, None) 
                kt, dt = self.fea_det.detectAndCompute(img, None)
                x_pos, y_pos = self.BF_filter(kt, dt, kq, dq)
                # print(x_pos, y_pos)
                self.imgNow = img
            except:
                pass
        print("POSS - Running at %2.2f Hz"%(1/(time.time()-start)))

