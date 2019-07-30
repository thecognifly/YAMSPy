from camera import Camera
from multiprocessing import Pool, Process, Pipe
import time

camera = Camera(frameWidth=240,
                    frameHeight=240,
                    frameRate = 30,
                    DEBUG=True)
camera_start = Process(target=camera.run,
                            args=())

def motion_data():
    ctn = 0
    data_ = camera.read_motion()
    data = camera.read_motion()
    if data_ is not None:
        while data is None and ctn < 5:
            data = camera.read_motion()
            ctn += 1
        if data is None:
            data = data_
    return data

def cv_data():
    ctn = 0
    data_ = camera.read_cv()
    data = camera.read_cv()
    if data_ is not None:
        while data is None and ctn < 5:
            data = camera.read_cv()
            ctn += 1
        if data is None:
            data = data_
    return data

try:
    camera_start.start()
    while True:
        start = time.time()
        time.sleep(.1)
        motion_data()
        cv_data()
        # print("MAIN - Running at {}Hz".format(1/(time.time()-start)))
        

except KeyboardInterrupt:
    camera_start.terminate()
    camera_start.join()
