import platform
import pyrealsense2 as rs  # for realsense api
import numpy as np
import sys
import time
from enum import IntEnum

from aidoop.camera.camera_dev_abc import CameraDev


class RealSensePreset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5


class RealsenseCapture(CameraDev):
    def __init__(self, matchedSerialNumber):  # devIndex):

        # configure depth and color streams
        self.__pipeline = rs.pipeline()
        self.__config = rs.config()
        self.__config.enable_device(matchedSerialNumber)

        # check if the current device is availalbe
        self.__foundDevice = self.__config.can_resolve(self.__pipeline)

        #  prepare realsense modules
        self.__pipecfg = None
        self.__frames = None
        self.__aligned_frames = None
        self.aligned_depth_frame = None

        # clipping
        self.is_clipping = False
        self.clipping_distance = 2  # default = 2 meters

        # depth scalse
        self.depth_scale = 0.0

        # depth filters
        self.filters = None
        self.filters_is_applied = False

    def avaialbleDevice(self):
        return self.__foundDevice

    def initialize(self, width, height, fps):
        # set frame width, frame height and frame per secods
        self._frameWidth = width
        self._frameHeight = height
        self._framePerSec = fps

        # enable color and depth streams with fixed resolutions & fps of the depth frame
        self.__config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        self.__config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)

        # create an align object based on color frames
        self.__align = rs.align(rs.stream.color)

    def prepare(self):
        self.set_flag_filters(True)
        self.prepare_filters()

    def start_capture(self):
        # start streaming
        self.__pipecfg = self.__pipeline.start(self.__config)
        depth_sensor = self.__pipecfg.get_device().first_depth_sensor()

        # Using preset HighAccuracy for recording
        depth_sensor.set_option(rs.option.visual_preset, RealSensePreset.HighAccuracy)

        self.depth_scale = depth_sensor.get_depth_scale()

        # if clipping enabled, clippint distance should be converted to milimeters using depth scale
        if self.is_clipping == True:
            self.clipping_distance = self.clipping_distance / self.depth_scale

    def stop_capture(self):
        # stop streaming
        self.__pipeline.stop()

    def setClipping(self, clipping_distance):
        self.is_clipping = True if clipping_distance > 0.0 else False
        self.clipping_distance = clipping_distance

    def get_video_frame(self):
        # wait for a coherent pair of frames: depth an--d color
        self.__frames = self.__pipeline.wait_for_frames()

        # align the depth frame to color frame
        self.__aligned_frames = self.__align.process(self.__frames)

        # Get aligned frames
        # aligned_depth_frame is a 640x480 depth image
        self.aligned_depth_frame = self.__aligned_frames.get_depth_frame()
        color_frame = self.__aligned_frames.get_color_frame()
        if not self.aligned_depth_frame or not color_frame:
            return None

        self.aligned_depth_frame = self.apply_filters(self.aligned_depth_frame)

        # convert images to numpy arrays
        depth_image = np.asanyarray(self.aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return color_image
        # return (color_image, depth_image)

        # # Remove background - Set pixels further than clipping_distance to grey
        # color_image_outofdist = None
        # if self.is_clipping:
        #     outofdist_color = 150    # fixed color of 'out of distance'
        #     # depth image is 1 channel, color is 3 channels
        #     depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
        #     color_image_outofdist = np.where((depth_image_3d > self.clipping_distance) | (
        #         depth_image_3d <= 0), outofdist_color, color_image)

        # return (color_image_outofdist, depth_image) if self.is_clipping else (color_image, depth_image)

    def get_frames(self):
        # wait for a coherent pair of frames: depth an--d color
        self.__frames = self.__pipeline.wait_for_frames()

        # align the depth frame to color frame
        self.__aligned_frames = self.__align.process(self.__frames)

        # Get aligned frames
        # aligned_depth_frame is a 640x480 depth image
        self.aligned_depth_frame = self.__aligned_frames.get_depth_frame()
        color_frame = self.__aligned_frames.get_color_frame()
        if not self.aligned_depth_frame or not color_frame:
            return None

        self.aligned_depth_frame = self.apply_filters(self.aligned_depth_frame)

        # convert images to numpy arrays
        depth_image = np.asanyarray(self.aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return (color_image, depth_image)

    # TODO: arrange parameters for each filter as the inputs of this function
    def prepare_filters(self):
        # prepare post-processing filters
        # decimate = rs.decimation_filter()
        # decimate.set_option(rs.option.filter_magnitude, 2 ** 3)
        # spatial = rs.spatial_filter()
        # spatial.set_option(rs.option.filter_magnitude, 5)
        # spatial.set_option(rs.option.filter_smooth_alpha, 1)
        # spatial.set_option(rs.option.filter_smooth_delta, 50)
        # spatial.set_option(rs.option.holes_fill, 2)
        # hff = rs.hole_filling_filter(1)

        # colorizer = rs.colorizer()
        self.filters = [
            rs.disparity_transform(),
            rs.decimation_filter(),
            rs.spatial_filter(),
            rs.temporal_filter(),
            rs.disparity_transform(False),
        ]

    def set_flag_filters(self, flag):
        assert isinstance(flag, (int))
        self.filters_is_applied = flag

    def apply_filters(self, arg_depth_frame):
        if self.filters_is_applied == True:
            for f in self.filters:
                arg_depth_frame = f.process(arg_depth_frame)

        return arg_depth_frame

    # get 3D position w.r.t an image pixel based on camera-based coordination
    def get_3D_pos(self, imageX, imageY):
        aligned_depth_frame = self.__aligned_frames.get_depth_frame()
        depth = aligned_depth_frame.get_distance(imageX, imageY)

        depth_profile = self.__pipecfg.get_stream(rs.stream.depth)
        color_profile = self.__pipecfg.get_stream(rs.stream.color)
        depth_intrin = depth_profile.as_video_stream_profile().intrinsics
        color_intrin = color_profile.as_video_stream_profile().intrinsics

        depth_point = rs.rs2_deproject_pixel_to_point(
            color_intrin, [imageX, imageY], depth
        )
        return depth_point

    # covert realsense intrisic data to camera matrix
    def convertIntrinsicsMat(self, intrinsics):
        mtx = np.array(
            [
                [intrinsics.fx, 0, intrinsics.ppx],
                [0, intrinsics.fy, intrinsics.ppy],
                [0, 0, 1],
            ]
        )

        dist = np.array(intrinsics.coeffs[:4])
        return mtx, dist
