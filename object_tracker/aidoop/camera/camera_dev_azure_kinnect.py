import numpy as np
import sys
import time
from enum import IntEnum

from aidoop.camera.camera_dev_abc import CameraDev
from aidoop.camera.




class AzureKinnectCapture(CameraDev):
    # Path to the module
    AK_MODULE_PATH = "/usr/lib/x86_64-linux-gnu/libk4a.so"


    def __init__(self, index=0):
        self.k4a = pyKinectAzure(AzureKinnectCapture.AK_MODULE_PATH)
        
        self.initialized = True
        try:
            # open a device
            self.k4a.device_open(index)
        except Exception as e:
            print("creation error: ", e)
            self.initialized = False

    def initialize(self, width, height, fps):
        # set frame width, frame height and frame per secods
        self.frameWidth = width
        self.frameHeight = height
        self.framePerSec = fps

        # prepare image handles
        self.color_image_handle = None
        self.depth_image_handle = None

        # Modify camera configuration
        self.device_config = self.k4a.config
        self.device_config.color_resolution = _k4a.K4A_COLOR_RESOLUTION_1080P if height == 1080 else 
        print(device_config)

    def prepare(self):
        # Start cameras using modified configuration
        self.k4a.device_start_cameras(self.device_config)

    def start_capture(self):
        self.k4a.device_get_capture()
        # Get the color image from the capture
        color_image_handle = self.k4a.capture_get_color_image()

    def stop_capture(self):
        # Release the image
        self.k4a.image_release(color_image_handle)
        self.k4a.capture_release()

    # get 3D position w.r.t an image pixel based on camera-based coordination
    def get_3D_pos(self, imageX, imageY):
        pass
