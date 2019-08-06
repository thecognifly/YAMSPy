'''
This library is built base on cython format
Raspberry Pi Zero Optical Flow Fitler
Work with camera.py and motion.py
This program used SAD filter for filter the not usable data_
The vertical movement is depending on the amount of x-axis vector
Fitler send back
self.dx | self.dy -> Mode_Median Filtering of the x-y direction movement
self.dz -> UP (1) Down(-1) Hove(-1)
self.gnd_x, self.gnd_y is ground truth in cm
>> 28 July 2019
>> Chan Man Fung Tom
>> @MistLab
'''
import numpy as np

cdef class Filter:
    '''
    Raspberry Pi Zero Optical Flow Fitler
    '''
    # Define the main config varible
    cdef float VTL_THRESHOLD, DIFF_THRESHOLD
    cdef int DATA_THRESHOLD, ALTITUDE, QUEUE_SIZE
    # Define the program varible
    cdef float PIXEL_SIZE_CM, FOCAL_LENGTH_CM
    # Define output varible
    cdef float dx, dy, dz, gnd_x, gnd_y

    def __init__(self, float vtl_threshold = 0.3, float diff_threshold = 0.3,
                        int data_threshold = 150, int altitude = 100, int  queue_size = 10):
        # @ the minimum precentage of the up and down vector different
        self.VTL_THRESHOLD = vtl_threshold

        # @ the minimum precentage of the pos and neg vector different
        self.DIFF_THRESHOLD = diff_threshold

        # @ the minimum number of the usable data
        self.DATA_THRESHOLD = data_threshold

        # @ The altitude of the frame
        self.ALTITUDE = altitude # cm

        # @ Queue Size
        self.QUEUE_SIZE = queue_size

        # @ PIXEL SIZE of Pi camera
        self.PIXEL_SIZE_CM = (16*1.4) / 10000 # 1.4um 4x4 binning

        # @ Focal Length of Pi camera
        self.FOCAL_LENGTH_CM = 0.36 # 3.6mm

        # @ The movement of the frame
        self.dx = self.dy = self.dz = 0

        # @ The ground truth
        self.gnd_x = self.gnd_y = 0

    def import_data(self, data):
        '''
        # >>> Import Data
        # >>> data.[a:b] a = row, b = columns
        '''
        self.x = np.copy(data['x']) 
        self.y = np.copy(data['y']) 
        self.sad = np.copy(data['sad']) # Sum of Absolute Difference >> hows the data reliable

    def update(self, j):
        '''
        # >>> Refresh the data, only act when using data base
        '''
        self.x = self.x[j]
        self.y = self.y[j]
        self.sad = self.sad[j]

    def ten_cut_off(self, data):
        '''
        # >>> remove 20% of the data from the begining and the end for remove outlier
        '''
        return data[ int((len(data))*0.1):int((len(data))*(0.9))]
    
    def sort(self, data): # Rearrange of order of the sample
        '''
        # >>> Sort the data set
        '''
        return np.sort(data)

    def mode(self, data):
        '''
        # >>> Find the most frequenly data
        '''
        # return stats.mode(data)[0][0]
        unique, counts = np.unique(data, return_counts=True)
        return unique[np.argmax(counts)]

    def twoD2oneD(self, data): # This function only work on each frame. In real time, delect all the frame argument
        '''
        # >>> Creating 1D array for x and y instead of 2D
        '''
        return data.ravel()

    def sad_filter (self, k = 1.5): # This function only work on each frame. In real time, delect all the frame argument
        '''
        # >>> Setting SAD Filter # Using the average sad value of each set
        '''
        sad_threshold = np.mean(self.sad) * k # smaller the sad value, the better of data. k is use to reduce the limit
        sad_filter = np.where(self.sad>sad_threshold) # sad_filter is mean the data not reliable
        self.x[sad_filter] = 0                        # therefore, set those data to zero.
        self.y[sad_filter] = 0
    
    cdef vtl_filter(self):
        '''
        # >>> Finding the different of abs x and abs y
        '''
        cdef int x_1, x1, y_1, y1, px, py

        x_1 = len(self.x[np.where(self.x < 0)[0]])
        x1 = len(self.x[np.where(self.x > 0)[0]])
        y_1 = len(self.y[np.where(self.y < 0)[0]])
        y1 = len(self.y[np.where(self.y > 0)[0]])
        px = py = 0
        if x_1 > 0 or x1 > 0 : px = (1-(abs(x_1 - x1)/(x_1+x1)))
        if y_1 > 0 or y1 > 0 : py = (1-(abs(y_1 - y1)/(y_1+y1)))
        if ((px > self.VTL_THRESHOLD) and ((x_1+x1) > self.DATA_THRESHOLD)) or ((py > self.VTL_THRESHOLD) and ((y_1+y1) > self.DATA_THRESHOLD)):
            return False
        else:
            return True

    def zero_filter(self, data):
        '''
        # >>> Filter out too zero vector
        '''
        return (data[(np.where(data!= 0)[0])]) # return nonzero data

    cdef px2gnd(self):
        '''
        # >>> px displacement to ground displacement
        # >>> GND_DIS = PIXEL_SIZE * PX_DIS * altitude / FOCAL_LENGTH
        '''
        if self.dx == 0 :
            self.gnd_x = 0
        else:
            self.gnd_x = ((self.PIXEL_SIZE_CM * self.dx * self.altitude)/self.FOCAL_LENGTH_CM)*1.25 / 10
        if self.dy == 0 :
            self.gnd_y = 0
        else:
            self.gnd_y = ((self.PIXEL_SIZE_CM * self.dy * self.altitude)/self.FOCAL_LENGTH_CM)*1.25 / 10
    
    cdef vtl_dir(self):
        '''
        # >>> Dectect the left side of the frame
        # >>> Return -1 = Down , Return 1 = Up
        '''
        cdef int x_1, x1, x_1_, x1_, temp1, temp2
        self.dx = self.dy = 0
        data = ((self.x[:,:int(((len(self.x[0,:]))/2))]))
        x_1 = len(data[np.where(data < 0)[0]])
        x1 = len(data[np.where(data > 0)[0]])
        data_ = ((self.x[:,int(((len(self.x[0,:]))/2)):]))
        x_1_ = len(data_[np.where(data_ < 0)[0]])
        x1_ = len(data_[np.where(data_ > 0)[0]])
        if (x_1_ + x1_) > 0 and (x_1 + x1) > 0:
            temp1 = abs(x_1 - x1)/(x_1 + x1)
            temp2 = abs(x_1_ - x1_)/(x_1_ + x1_)
            if  temp1 > self.DIFF_THRESHOLD and  temp2 > self.DIFF_THRESHOLD:
                if temp1 > temp2:
                    if (x_1 - x1) < 0 :
                        self.dz = -1
                    else:
                        self.dz = 1
                else:
                    if (x_1_ - x1_) < 0 :
                        self.dz = 1
                    else:
                        self.dz = -1
        else:
            self.dz = 0

    def mode_median(self, data, q1, mid, q3):
        '''
        # Checking the median isn't the most populated value with q1 q3
        # If not, take mean at with q1 and q3
        '''
        if (mid != q1 != q3):
            a = np.where(data == q1)[0][0]  # First arrays of q1 in list
            b = np.where(data == q3)[0][-1] # Last array of q3 in list
            Data = np.mean(data[a:b])
        else:
            Data = mid
        return Data

    cdef hrz_dir(self):
        '''
        # >>> Calc the detla x and y
        '''
        cdef float x_q1, x_q3, y_q1, y_q3
        # Covert and sort the data to 1D, remove the outlier and take mean
        x = (self.ten_cut_off((self.zero_filter(self.sort(self.twoD2oneD(self.x))))))
        y = (self.ten_cut_off((self.zero_filter(self.sort(self.twoD2oneD(self.y))))))
        self.dz = 0
        if len(x)>self.DATA_THRESHOLD :
            x_q1 = x[int(len(x)/4)] # 25% of the data
            medianx =  x[int(len(x)/2)] # Median of data
            x_q3 = x[int(3*(len(x))/4)] # 75% of the data
            self.dx = self.mode_median(x, x_q1, medianx, x_q3)
        else:
            self.dx = 0
        if len(y)>self.DATA_THRESHOLD :
            y_q1 = y[int(len(y)/4)] # 25% of the data
            mediany = y[int(len(y)/2)] # Median of data
            y_q3 = y[int(3*(len(y))/4)] # 75% of the data
            self.dy = self.mode_median(y, y_q1, mediany, y_q3)
        else:
            self.dy = 0

    def run(self, data, DoubleMean = False):
        '''
        # >>> Main Flow for filtering
        # >>> Using Double Mean just set it True
        '''
        self.import_data(data)
        if DoubleMean:
            self.x_queue[1:] = self.x_queue[:-1]
            self.y_queue[1:] = self.y_queue[:-1]
            self.x_queue[0] = self.x.mean()
            self.y_queue[0] = self.y.mean()
            # Calculate the mean of both queues
            self.dx = self.x_queue.mean()
            self.dy = self.y_queue.mean()
        else:
            self.sad_filter() # using k = 1.8 gain to lower the SAD limit. Default is 1.5
            if (self.vtl_filter()): # Return True if not vertical movement
                self.hrz_dir()
            else:
                self.vtl_dir()
        self.px2gnd()        
        return (self.dz, self.dx, self.dy, self.gnd_x, self.gnd_y)
