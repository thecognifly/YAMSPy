import weakref
from threading import Lock
import time
from OptiTrackPython import NatNetClient
import math

to_fc = (1.1,2.2,3.4)

old_time = 0.0

class CogniFlyNatNetClient(object):

    def __init__(self,
                 client_ip="192.168.0.101",
                 server_ip="192.168.0.100",
                 multicast_address="239.255.42.99",
                 rigidbody_names2track=["Carbon"]):

        self.rigidbody_names2track = rigidbody_names2track

        self.lock_opti = Lock()

        self.optitrack_reading = {}

        # This will create a new NatNet client
        self.streamingClient = NatNetClient(client_ip,
                                            server_ip,
                                            multicast_address)

        # Configure the streaming client to call our rigid body handler on the emulator to send data out.
        self.streamingClient.rigidBodyListener = self.receiveRigidBodyFrame
        self.TO_FC = [0,0,0,0]
        self.last_reading = time.monotonic()

        # Start up the streaming client now that the callbacks are set up.
        # This will run perpetually, and operate on a separate thread.
        self.streamingClient.run()
        

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.exit()

    def exit(self):
        self.streamingClient.close()

    def read_new_data(self, key="Carbon"):
        if self.lock_opti.acquire(blocking=True, timeout=0.5): # blocks, but it has a timeout
            try:
                self.last_reading = self.optitrack_reading[key][0]
                to_fc = self.optitrack_reading[key][1]
                for ii in range(3):
                    self.TO_FC[ii] = to_fc[ii]

                to_fc = self.optitrack_reading[key][2]
                self.TO_FC[3] = to_fc[2]*180/math.pi
            except KeyError:
                pass
            finally:
                self.lock_opti.release()

        return self.TO_FC, self.last_reading # old value (the user decides if it's worth using or not)
        
        
    def receiveRigidBodyFrame(self, timestamp, id, position, rotation, rigidBodyDescriptor):
        if rigidBodyDescriptor:
            for rbname in self.rigidbody_names2track:
                if rbname in rigidBodyDescriptor:
                    if id == rigidBodyDescriptor[rbname][0]:
                        # skips this message if still lockedlast_mocap
                        if self.lock_opti.acquire(False):
                            try:
                                # rotation is a quaternion!
                                self.optitrack_reading[rbname] = [timestamp,
                                                                  position,
                                                                  rotation]
                            finally:
                                self.lock_opti.release()


if __name__ == '__main__':
    TT=CogniFlyNatNetClient()
    try:
      while True:
            t = time.time()
            if t - old_time > 0.2:
                last_mocap = TT.read_new_data()
                print(f"Last mocap value({last_mocap[1]}): {last_mocap[0]}")
                old_time = t
                time.sleep(.05)
    except KeyboardInterrupt:
        print('interrupted')
