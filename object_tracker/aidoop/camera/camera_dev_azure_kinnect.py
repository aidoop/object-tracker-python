import platform
import open3d as o3d
import numpy as np
import sys
import time
from enum import IntEnum
import open3d as o3d


from aidoop.camera.camera_dev_abc import CameraDev


class AzureKinnectCapture(CameraDev):
    def __init__(self, device, config=None):
        self.device = device
        self.config = config if config is not None else o3d.io.AzureKinectSensorConfig()
        self.kinnect = None

    def initialize(self, width, height, fps):
        # set frame width, frame height and frame per secods
        self._frameWidth = width
        self._frameHeight = height
        self._framePerSec = fps

        self.kinnect = o3d.io.AzureKinectSensor(self.config)
        if not self.kinnect.connect(self.device):
            raise RuntimeError("failed to connect to azure kinnect")

    def prepare(self):
        # TODO: check if kinnect had something to do here before starting to capturesZ
        pass

    def start_capture(self):
        time.sleep(0.5)
        pass

    def stop_capture(self):
        pass

    def get_video_frame(self):
        # wait for a coherent pair of frames: depth an--d color
        rgbd = self.kinnect.capture_frame(True)

        # convert images to numpy arrays
        depth_image = np.asanyarray(rgbd.depth)
        color_image = np.asanyarray(rgbd.color)

        return color_image

    def get_frames(self):
        # wait for a coherent pair of frames: depth an--d color
        rgbd = self.kinnect.capture_frame(True)

        # convert images to numpy arrays
        depth_image = np.asanyarray(rgbd.depth)
        color_image = np.asanyarray(rgbd.color)

        return (color_image, depth_image)

    # get 3D position w.r.t an image pixel based on camera-based coordination
    def get_3D_pos(self, imageX, imageY):
        raise NotImplementedError


# # Using pyKinnectAzure

# import numpy as np
# import sys
# import time
# from enum import IntEnum

# from aidoop.camera.camera_dev_abc import CameraDev


# class AzureKinnectCapture(CameraDev):
#     # Path to the module
#     AK_MODULE_PATH = "/usr/lib/x86_64-linux-gnu/libk4a.so"


#     def __init__(self, index=0):
#         self.k4a = pyKinectAzure(AzureKinnectCapture.AK_MODULE_PATH)

#         self.initialized = True
#         try:
#             # open a device
#             self.k4a.device_open(index)
#         except Exception as e:
#             print("creation error: ", e)
#             self.initialized = False
#             raise RuntimeError("can't open kinnect camera")

#     def initialize(self, width, height, fps):
#         # set frame width, frame height and frame per secods
#         self.frameWidth = width
#         self.frameHeight = height
#         self.framePerSec = fps

#         # prepare image handles
#         self.color_image_handle = None
#         self.depth_image_handle = None

#         # Modify camera configuration
#         self.device_config = self.k4a.config
#         self.device_config.color_resolution = _k4a.K4A_COLOR_RESOLUTION_1080P if height == 1080 else
#         print(device_config)

#     def prepare(self):
#         # Start cameras using modified configuration
#         self.k4a.device_start_cameras(self.device_config)

#     def start_capture(self):
#         self.k4a.device_get_capture()
#         # Get the color image from the capture
#         color_image_handle = self.k4a.capture_get_color_image()

#     def stop_capture(self):
#         # Release the image
#         self.k4a.image_release(color_image_handle)
#         self.k4a.capture_release()

#     # get 3D position w.r.t an image pixel based on camera-based coordination
#     def get_3D_pos(self, imageX, imageY):
#         pass
