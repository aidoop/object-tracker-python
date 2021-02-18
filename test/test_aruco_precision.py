import numpy as np
import cv2
import cv2.aruco as aruco
import os
import time
import datetime
import glob
import sys
import argparse
import json
import math

import config
from camera.camera_dev_realsense import RealsenseCapture
from camera.camera_dev_opencv import OpencvCapture
from camera.camera_videocapture import VideoCapture
from aruco.aruco_detect import ArucoDetect

###############################################################################
# Hand-eye calibration process
#   -
###############################################################################

if __name__ == "__main__":

    # camera index
    rsCamIndex = "4"
    # rsCamDev = RealsenseCapture(rsCamIndex)
    rsCamDev = OpencvCapture(int(rsCamIndex))
    vcap = VideoCapture(
        rsCamDev,
        config.VideoFrameWidth,
        config.VideoFrameHeight,
        config.VideoFramePerSec,
        "camera02",
    )

    # Start streamingq
    vcap.start()

    # get instrinsics
    mtx, dist = vcap.getIntrinsicsMat(
        int(rsCamIndex), config.UseRealSenseInternalMatrix
    )

    # create an aruco detect object
    arucoDetect = ArucoDetect(config.ArucoDict, config.ArucoSize, mtx, dist)
    # arucoDetect = ArucoDetect(config.ArucoDict, 0.075, mtx, dist)
    # arucoDetect = ArucoDetect(aruco.DICT_7X7_250, 0.05, mtx, dist)

    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            color_image = vcap.get_video_frame()

            # operations on the frame
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            # lists of ids and the corners belonging to each id
            corners, ids = arucoDetect.detect(gray)
            if (corners is not None) and (ids is not None):

                # rvet and tvec-different from camera coefficients
                rvec, tvec = arucoDetect.estimatePose(corners)
                # print(tvec)
                if len(tvec) < 2:
                    continue

                # draw axis
                arucoDetect.drawAx(color_image, rvec[0], tvec[0])
                arucoDetect.drawAx(color_image, rvec[1], tvec[1])

                # calculate the distance between two any aruco markers.
                tdiff = abs(tvec[0] - tvec[1])
                tdist = math.sqrt(
                    math.pow(tdiff[0][0], 2.0)
                    + math.pow(tdiff[0][1], 2.0)
                    + math.pow(tdiff[0][2], 2.0)
                )
                print(tdist)

            # displaqy the captured image
            cv2.imshow("Prcision Test", color_image)

            # TODO: arrange these opencv key events based on other key event handler class
            # handle key inputs
            pressedKey = cv2.waitKey(1) & 0xFF
            if pressedKey == ord("q"):
                break

    except Exception as ex:
        print("Error :", ex, file=sys.stderr)

    finally:
        # Stop streaming
        vcap.stop()

    cv2.destroyAllWindows()
