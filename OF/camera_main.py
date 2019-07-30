from camera import Camera

from multiprocessing import Pool, Process, Pipe
import time

camera = Camera(frameWidth=240,
                    frameHeight=240,
                    frameRate = 90)
camera_start = Process(target=camera.run,
                            args=())

try:
    camera_start.start()
    while True:
        start = time.time()
        ctn = 0
        time.sleep(.1)
        data_ = camera.read_motion()
        data = camera.read_motion()
        if data_ is not None:
            while data is None and ctn < 5:
                data = camera.read_motion()
                ctn += 1
            if data is None:
                data = data_
            # print (data)
        # print ("b-",data['x'][10])
        else:
            continue
        # print("MAIN - Running at {}Hz".format(1/(time.time()-start)))
        

except KeyboardInterrupt:
    camera_start.terminate()
    camera_start.join()