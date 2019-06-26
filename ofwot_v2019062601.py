from of import GestureDetector
from NatNetClient_RigidBody import NatNetClient
from picamera.array import PiMotionAnalysis, PiRGBArray
import os
import numpy as np
import time
import cv2
import picamera

timesmp = pos = None

def receiveRigidBodyFrame(timestamp, id, position, rotation, rigidBodyDescriptor):
    if rigidBodyDescriptor:
        if 'RigidBody 1' in rigidBodyDescriptor:
            if id == rigidBodyDescriptor['RigidBody 1'][0]:
                # print("[{}],position: {}\n ".format(timestamp, position))
                global timesmp, pos
                timesmp = timestamp
                pos = position

streamingClient=NatNetClient(client_ip='192.168.2.113')
streamingClient.rigidBodyListener=receiveRigidBodyFrame
streamingClient.run()
with picamera.PiCamera(resolution='VGA', framerate=60) as camera:
    with GestureDetector(camera) as detector:
        camera.start_recording(
            os.devnull, format='h264', motion_output=detector)
        try:
            ctn = 0
            f= open("data/data.txt","w+")
            while timesmp is None:
                print("Not Starting")
            print ("Start")
            while True:
                try:
                    print (timesmp, detector.x_queue.mean(), detector.y_queue.mean(), pos)
                    camera.wait_recording(0.017) # maybe reduce this value
                    # camera.wait_recording(0.8)

                    f.write("%s,%s,%s,%s\n" % (timesmp, detector.x_queue.mean(), detector.y_queue.mean(), pos))
                    # if ctn == 100:
                    #     cv2.imwrite("/data/%s.jpg"%ctn, snap_shot(camera))
                    #     print ("SNAP!!!!")
                    #     break
                    # time.sleep(0.01)
                except KeyboardInterrupt:
                    streamingClient.is_alive=False
                    break

        finally:
            print("Closing")
            streamingClient.close()
            f.close()
            camera.stop_recording()
