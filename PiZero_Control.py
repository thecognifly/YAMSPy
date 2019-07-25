import os
import io
import subprocess
import signal
from optical_flow import Filter, Video
from multiprocessing import Pool, Process
import time

os.nice(-10) # Main loop is highest proity 
video_process = None

def nice():
    os.nice(5)
try:
    os.mkfifo("/dev/shm/motion_stream")
    filter = Filter(vtl_threshold = 0.6, 
                        diff_threshold = 0.3,
                        data_threshold = 5, 
                        altitude = 100)
    video = Video(frameWidth=240, frameHeight=240, frameRate=10)
    # frameWidth, frameHeight, frameRate
    # video_process = subprocess.Popen(["python3", 'optical_flow.py', "VIDEOSTREAM", str(240), str(240), str(10)], preexec_fn = nice)
    video_process = Process(target = video.run, args=())
    print("start camera")
    video_process.start()
    video_fifo = io.open("/dev/shm/motion_stream", mode="rb")
    print("OK")
    
    while True:
        try:
            # Input the data into filter
            time.sleep(.01)
            dz, dx, dy, gnd_x, gnd_y = filter.run(video_fifo.read(filter.data_size))
            print(dx, dy)
        except KeyboardInterrupt:
            break

except FileExistsError:
    print("Run It Again !")

finally:  
    # .poll() return None when it is not terminated
    if video_process:
        if video_process.is_alive():
            # video_process.send_signal(signal.SIGINT) #Parent kill, all kill
            video_process.terminate()
            video_process.join()
        video_fifo.close()
    os.unlink("/dev/shm/motion_stream")
    print ("\nFINISH!!!!!!")
