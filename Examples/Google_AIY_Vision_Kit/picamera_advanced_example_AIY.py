"""
Code for testing custom AIY Vision compiled models. Example of usage:
$ python3 picamera_advanced_example_AIY.py --model_path $(pwd)/compiled.binaryproto --input_width 256 --input_height 256

"""
import io
import time
import numpy as np

from PIL import Image


from aiy.vision.inference import ImageInference, ModelDescriptor
from aiy.vision.models import utils



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
    def __init__(self, model, frameWidth=240, frameHeight=240, DEBUG = False):
        # Init the stuff we are inheriting from
        super().__init__()

        self.DEBUG = DEBUG

        self.inference = ImageInference(model)

        # Set video frame parameters
        self.frameWidth = frameWidth
        self.frameHeight = frameHeight

        self.prev_time = time.time()

        self.output = None

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
            self.output = np.reshape(np.frombuffer(b, dtype=np.uint8), (self.frameHeight, self.frameWidth, 3))

            image_center, offset = crop_center(Image.fromarray(self.output))
            result = self.inference.run(image_center)
            if self.DEBUG:
                print(f"ImgCap - Inference result: {result}")
                print(f"ImgCap - Image.shape {self.output.shape}")
                
            print(f"ImgCap - Running at {1/(time.time()-self.prev_time):2.2f} Hz")

            self.prev_time = time.time()

        except Exception as e:
            print("ImgCap error: {}".format(e))

        finally:
            return len(b)


if __name__ == "__main__":

    import argparse

    import picamera

    # See https://picamera.readthedocs.io/en/release-1.10/api_camera.html
    # for details about the parameters:
    frameWidth = 256 
    frameHeight = 256
    frameRate = 20
    contrast = 40
    rotation = 180
        
    # Set the picamera parametertaob
    camera = picamera.PiCamera()
    camera.resolution = (frameWidth, frameHeight)
    camera.framerate = frameRate
    camera.contrast = contrast


    parser = argparse.ArgumentParser()
    parser.add_argument('--model_name', default='test_model', help='Model identifier.')
    parser.add_argument('--model_path', required=True, help='Path to model file.')
    parser.add_argument('--input_height', type=int, required=True, help='Input height.')
    parser.add_argument('--input_width', type=int, required=True, help='Input width.')
    parser.add_argument('--input_depth', type=int, default=3, help='Input depth.')
    parser.add_argument('--input_mean', type=float, default=128.0, help='Input mean.')
    parser.add_argument('--input_std', type=float, default=128.0, help='Input std.')
    parser.add_argument('--debug', default=False, action='store_true')
    args = parser.parse_args()

    DEBUG = args.debug

    model = ModelDescriptor(
        name=args.model_name,
        input_shape=(1, args.input_height, args.input_width, args.input_depth),
        input_normalizer=(args.input_mean, args.input_std),
        compute_graph=utils.load_compute_graph(args.model_path))

    # Start the video process
    with ImgCap(model, frameWidth, frameHeight, DEBUG) as img:
        camera.start_recording(img, format='rgb', splitter_port = 1)
        try:
            while True:
                camera.wait_recording(timeout=0) # using timeout=0, default, it'll return immediately  
                # if img.output is not None:
                    # print(img.output[0,0,0])

        except KeyboardInterrupt:
            pass
        finally:
            camera.stop_recording(splitter_port = 1)
            # camera.stop_recording(splitter_port = 2)
