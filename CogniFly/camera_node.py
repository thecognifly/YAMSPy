import os
import time
from multiprocessing import Pool, Process, Pipe
from of.camera import Camera

os.nice(-10)
DEBUG = True
pipe_read_of, pipe_write_of = Pipe()
pipe_read_cv, pipe_write_cv = Pipe()
camera = Camera((pipe_read_of, pipe_write_of),
                (pipe_read_cv, pipe_write_cv),
                frameWidth=240,
                frameHeight=240,
                frameRate = 30,
                DEBUG=DEBUG)
camera_start = Process(target=camera.run,
                            args=(10, ))

def motion_data():
    ctn = 0
    data_ = pipe_read_of.recv()
    data = pipe_read_of.recv()
    if data_ is not None:
        while data is None and ctn < 5:
            data = pipe_read_of.recv()
            ctn += 1
        if data is None:
            data = data_
    return data

def cv_data():
    ctn = 0
    data_ = pipe_read_cv.recv()
    data = pipe_read_cv.recv()
    if data_ is not None:
        while data is None and ctn < 5:
            data = pipe_read_cv.recv()
            ctn += 1
        if data is None:
            data = data_
    return data

try:
    camera_start.start()
    while True:
        start = time.time()
        motion_data()
        cv_data()
        if DEBUG:
            print("MAIN - Running at {}Hz".format(1/(time.time()-start)))

except KeyboardInterrupt:
    camera_start.terminate()
    camera_start.join()