
import pyrealsense2 as rs   # for realsense api
import numpy as np
import sys
import time

# add src root directory to python path
import os, sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from packages.CameraDev import CameraDev
from packages.Util import PrintMsg

class RealsenseCapture(CameraDev):

    # def __new__(cls, devIndex):

    #     devFound = False

    #     ctx = rs.context()
    #     if devIndex < (len(ctx.devices)):
    #         devIter = 0
    #         for d in ctx.devices:
    #             if(devIter == devIndex):
    #                 PrintMsg.printStdErr('Serial Number: ' + d.get_info(rs.camera_info.serial_number))
    #                 devFound = True
    #             devIter += 1
    #     else:
    #         pass

    #     if(devFound == True):
    #         camdev = object.__new__(cls)
    #     else:
    #         camdev = None

    #     return camdev

    def __init__(self, matchedSerialNumber):    #devIndex):

        self.foundDevice = False

        # manage multiple devices
        # ctx = rs.context()
        # time.sleep(0.1) # to fix the internal uvc camera bug on linux
        # rsDevices = ctx.devices
        # time.sleep(0.1) # to fix the internal uvc camera bug on linux
        # if devIndex < (len(rsDevices)):
        #     devIter = 0
        #     for d in rsDevices:
        #         if(devIter == devIndex):
        #             matchedSerialNumber = d.get_info(rs.camera_info.serial_number)
        #             self.foundDevice = True
        #             break
        #         devIter += 1
        # else:
        #     self.foundDevice = False
        #     return

        # configure depth and color streams
        self.__pipeline = rs.pipeline()
        self.__config = rs.config()
        self.__config.enable_device(matchedSerialNumber)

        # check if the current device is availalbe 
        self.foundDevice = self.__config.can_resolve(self.__pipeline)

        # create an align object based on color frames
        self.__align = rs.align(rs.stream.color)

        #  declare align 
        self.__pipecfg = None
        self.__frames = None
        self.__aligned_frames = None

        # camera internal intrinsics
        self.depth_intrin = None
        self.color_intrin = None

    def avaialbleDevice(self):
        return self.foundDevice    
    
    def initialize(self, width, height, fps):
        # set frame width, frame height and frame per secods
        self._frameWidth = width
        self._frameHeight = height
        self._framePerSec = fps

        # enable color and depth streams 
        self.__config.enable_stream(rs.stream.depth, self._frameWidth, self._frameHeight, rs.format.z16, self._framePerSec)
        self.__config.enable_stream(rs.stream.color, self._frameWidth, self._frameHeight, rs.format.bgr8, self._framePerSec)

    def startCapture(self):
        # start streaming
        self.__pipecfg = self.__pipeline.start(self.__config)

    def stopCapture(self):
        # stop streaming
        self.__pipeline.stop()

    def getFrame(self):
        # wait for a coherent pair of frames: depth an--d color
        self.__frames = self.__pipeline.wait_for_frames()

        # align the depth frame to color frame
        self.__aligned_frames = self.__align.process(self.__frames)

        # Get aligned frames
        aligned_depth_frame = self.__aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = self.__aligned_frames.get_color_frame()
        if not aligned_depth_frame or not color_frame:
            return None

        # convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())

        return color_image

    # get 3D position w.r.t an image pixel based on camera-based coordination
    def get3DPosition(self, imageX, imageY):
        aligned_depth_frame = self.__aligned_frames.get_depth_frame()
        depth = aligned_depth_frame.get_distance(imageX, imageY)
        
        depth_profile = self.__pipecfg.get_stream(rs.stream.depth)
        self.depth_intrin = depth_profile.as_video_stream_profile().intrinsics        
        
        depth_point = rs.rs2_deproject_pixel_to_point(self.depth_intrin, [imageX, imageY], depth)
        return depth_point

    def getInternalIntrinsicsMat(self):
        # get internal intrinsics & extrinsics in D435
        profile = self.__pipecfg.get_stream(rs.stream.color)
        self.color_intrin = profile.as_video_stream_profile().intrinsics
        mtx, dist = self.convertIntrinsicsMat(self.color_intrin)
        return (mtx, dist)

    # covert realsense intrisic data to camera matrix
    def convertIntrinsicsMat(self, intrinsics):
        mtx = np.array([[intrinsics.fx,             0, intrinsics.ppx],
                        [            0, intrinsics.fy, intrinsics.ppy],
                        [            0,             0,              1]])
        
        dist = np.array(intrinsics.coeffs[:4])
        return mtx, dist        

###############################################################################
# test sample codes
###############################################################################
if __name__ == '__main__':

    rsCamDev = RealsenseCapture('001622071306')

    rsCamDev.initialize(640, 480, 30)

    rsCamDev.startCapture()

    frameIdx = 0
    for frmIdx in range(0, 10):
        rsCamDev.getFrame()
        print(frameIdx)
        frameIdx += 1

    rsCamDev.stopCapture()

