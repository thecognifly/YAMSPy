import io
import time
from threading import Thread, Lock
from copy import deepcopy

import numpy as np

from PIL import Image, ImageDraw

import picamera


from aiy.vision.inference import ImageInference, ModelDescriptor
from aiy.vision.models import utils, object_detection




def tensors_info(tensors):
    return ', '.join('%s [%d elements]' % (name, len(tensor.data))
        for name, tensor in tensors.items())


def crop_center(image):
    width, height = image.size
    size = min(width, height)
    x, y = (width - size) / 2, (height - size) / 2
    return image.crop((x, y, x + size, y + size)), (x, y)

class ImgCap(io.IOBase):
    '''
    Capturing Image from a Raspicam (V2.1)
    '''
    def __init__(self, inference, lock, args = None, DEBUG = False):
        # Init the stuff we are inheriting from
        super(__class__, self).__init__()

        self.inference = inference

        self.lock = lock

        self.prev_time = time.time()

        self.output = None

        self.args = args

        self.DEBUG = DEBUG

        self.new_obj = False

    def writable(self):
        '''
        To be a nice file, you must have this method
        '''
        return True

    def write(self, b):
        '''
        Here is where the image data is received and made available at self.output
        '''

        try:
            # b is the numpy array of the image, 3 bytes of color depth
            self.output = np.reshape(np.frombuffer(b, dtype=np.uint8), (self.args.input_height, 
                                                                        self.args.input_width, 3))

            image_center, offset = crop_center(Image.fromarray(self.output))

            if self.args.sparse:
                result = self.inference.run(image_center,
                                    sparse_configs=object_detection.sparse_configs(self.args.threshold))
                objects = object_detection.get_objects_sparse(result, offset)
            else:
                result = self.inference.run(image_center)
                objects = object_detection.get_objects(result, self.args.threshold, offset)

            if self.DEBUG:
                for i, obj in enumerate(objects):
                    x, y, width, height = obj.bounding_box

                    x = x + int(width/2)
                    y = y + int(height/2)
                    print('ImgCap - Object #{}: kind={} score={:.3f} pos(x,y)=({},{})'.format(i, 
                                                                                     obj.kind,
                                                                                     obj.score,
                                                                                     x, y))
            if len(objects):
                with self.lock:
                    self.new_obj = True # indicates a new object is available
                    self.last_objects = deepcopy(objects)
                    self.last_image = np.copy(self.output)

            if self.DEBUG:
                print("ImgCap - Image.shape {}".format(self.output.shape))
                print("ImgCap - Running at {:2.2f} Hz".format(1/(time.time()-self.prev_time)))

            self.prev_time = time.time()

        except Exception as e:
            print("ImgCap error: {}".format(e))

        finally:
            return len(b)

class MyAIYInterface():

    def __init__(self, args = None, DEBUG = False):

        self.args = args
        self.DEBUG = DEBUG

        # See https://picamera.readthedocs.io/en/release-1.13/api_camera.html
        # for details about the parameters:
        # contrast = 40

        # Set the picamera parameters
        camera = picamera.PiCamera()
        camera.resolution = (args.input_width, args.input_height)
        camera.framerate = args.fps
        # camera.contrast = contrast

        camera.rotation = args.rotation

        self.camera = camera
        
        self.lock = Lock()

        self.shutdown = False

        self.inference_tread = None

    def do_inference(self):
        # Start the video process
        with ImageInference(object_detection.model()) as inference:
            with ImgCap(inference, self.lock, self.args, self.DEBUG) as self.img:
                self.camera.start_recording(self.img, format='rgb', splitter_port = 1)
                try:
                    while not self.shutdown:
                        self.camera.wait_recording(timeout=0) # using timeout=0, default, it'll return immediately  
                        time.sleep(1/self.args.fps)

                except KeyboardInterrupt:
                    pass
                
                finally:
                    self.camera.stop_recording(splitter_port = 1)
                    print("Camera closed!")

                    if hasattr(self.img, 'last_image'):
                        image = Image.fromarray(self.img.last_image)
                        draw = ImageDraw.Draw(image)
                        for obj in self.img.last_objects:
                            x, y, width, height = obj.bounding_box
                            draw.rectangle((x, y, x + width, y + height), outline='red')
                        image.save("last_image.jpg")

    def run(self):
        self.inference_tread = Thread(target=self.do_inference)
        self.inference_tread.start()
    
    def kill(self):
        self.shutdown = True
        if self.inference_tread:
            self.inference_tread.join()

if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--input_height', type=int, default=256, 
                        help='Input height.')
    parser.add_argument('--input_width', type=int, default=256, 
                        help='Input width.')
    parser.add_argument('--fps', type=int, default=5, 
                        help='Frames per second.')
    parser.add_argument('--rotation', type=int, default=180, 
                        help='Image rotation.')
    parser.add_argument('--sparse', '-s', action='store_true', default=True,
                        help='Use sparse tensors.')
    parser.add_argument('--threshold', '-t', type=float, default=0.3,
                        help='Detection probability threshold.')
    args = parser.parse_args()


    myaiy = MyAIYInterface(args, DEBUG = True)

    myaiy.run()
    
    try:
        while True:
            if hasattr(myaiy, 'img'):
                if hasattr(myaiy.img, 'last_image'):
                    with myaiy.lock:
                        for obj in myaiy.img.last_objects:
                            if obj.kind == 1:
                                x, y, width, height = obj.bounding_box
                                x = x + int(width/2)
                                y = y + int(height/2)
                                print(x,y)

            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        myaiy.kill()
        print("Bye!")