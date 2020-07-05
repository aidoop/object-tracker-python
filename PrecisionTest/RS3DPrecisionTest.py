
import numpy as np
import cv2
import cv2.aruco as aruco
import os
import time
import datetime
import glob
import sys
import argparse

# add src root directory to python path
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) )
import Config
from packages.CameraDevRealsense import RealsenseCapture
from packages.CameraDevOpencv import OpencvCapture
from packages.CameraVideoCapture import VideoCapture
import json
from packages.Aruco import ArucoDetect

import math

import pyrealsense2 as rs

###############################################################################
# Hand-eye calibration process 
#   -                                                                
###############################################################################

if __name__ == '__main__':

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    # Start streaming
    pipe_profile = pipeline.start(config)

    depth_sensor = pipe_profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()        

    # get instrinsics
    rsCamDev = RealsenseCapture(0)
    vcap = VideoCapture(rsCamDev, Config.VideoFrameWidth, Config.VideoFrameHeight, Config.VideoFramePerSec, 'camera01')
    mtx, dist = vcap.getIntrinsicsMat(0, False)

    # create an aruco detect object
    arucoDetect = ArucoDetect(Config.ArucoDict, Config.ArucoSize, mtx, dist)

    # align
    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        while(True):
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Intrinsics & Extrinsics
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
            color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
            #depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())            

            # operations on the frame
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            # lists of ids and the corners belonging to each id
            corners, ids = arucoDetect.detect(gray)

            # rvet and tvec-different from camera coefficients
            rvec, tvec = arucoDetect.estimatePose(corners)
            #print(tvec)

            

            if len(tvec) >= 2:
                centerPoints = list()
                for idx in range(0, ids.size):
                    if((rvec[idx].shape == (1,3)) or (rvec[idx].shape == (3,1))):
                        inputObjPts = np.float32([[0.0,0.0,0.0]]).reshape(-1,3)
                        imgpts, jac = cv2.projectPoints(inputObjPts, rvec[idx], tvec[idx], mtx, dist)
                        centerPoints.append(tuple(imgpts[0][0]))

                center3Ds = list()
                for centerPoint in centerPoints:
                    depth = depth_frame.get_distance(centerPoint[0], centerPoint[1])
                    depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [centerPoint[0], centerPoint[1]], depth)
                    center3Ds.append(depth_point)

                #tdiff = abs(center3Ds[0] - center3Ds[1])
                tdist = math.sqrt(math.pow(center3Ds[0][0] - center3Ds[1][0], 2.0)+math.pow(center3Ds[0][1] - center3Ds[1][1], 2.0)+math.pow(center3Ds[0][2] - center3Ds[1][2], 2.0))
                print(tdist)

            # # focusing on a (320, 120) pixel
            # depth = depth_frame.get_distance(640, 480)
            # depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [320, 120], depth)
            # text = "%.5lf, %.5lf, %.5lf\n" % (depth_point[0], depth_point[1], depth_point[2])
            # print(text)


            time.sleep(0.1)

            # display the captured image
            cv2.imshow('Prcision Test', color_image)

            # TODO: arrange these opencv key events based on other key event handler class
            # handle key inputs
            pressedKey = (cv2.waitKey(1) & 0xFF)
            if(pressedKey == ord('q')):
                break

    except Exception as ex:
        print("Error :", ex)

    finally:
        # Stop streaming
        vcap.stop()

    cv2.destroyAllWindows()

