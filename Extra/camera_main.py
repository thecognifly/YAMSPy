from camera import Camera
from filter import Filter

from multiprocessing import Pool, Process, Pipe

camera = Camera(frameWidth=240,
                    frameHeight=240,
                    frameRate = 60)
filter = Filter(vtl_threshold = 0.6, 
                    diff_threshold = 0.3,
                    data_threshold = 5, 
                    altitude = 100)
camera_start = Process(target=camera.run,
                            args=())

try:
    camera_start.start()
    while True:
        data = camera.read_motion()
        dz, dx, dy, gnd_x, gnd_y = filter.run(data) if data is not None else 0
        print (dx, dy)
        

except KeyboardInterrupt:
    camera_start.terminate()
    camera_start.join()