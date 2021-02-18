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

from aidobjtrack.config.appconfig import AppConfig
from aidobjtrack.camera.camera_dev_opencv import OpencvCapture
from aidobjtrack.camera.camera_dev_realsense import RealsenseCapture
from aidobjtrack.camera.camera_videocapture import VideoCapture
from aidobjtrack.aruco.aruco_detect import ArucoDetect
from aidobjtrack.aruco.aruco_advanced_pose import ArucoAdvPose
from aidobjtrack.util.hm_util import *
from aidobjtrack.handeye.calibhandeye_handeye import *


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
        AppConfig.VideoFrameWidth,
        AppConfig.VideoFrameHeight,
        AppConfig.VideoFramePerSec,
        "camera02",
    )

    # Start streamingq
    vcap.start()

    # get instrinsics
    mtx, dist = vcap.getIntrinsicsMat(
        int(rsCamIndex), AppConfig.UseRealSenseInternalMatrix
    )

    # create an aruco detect object
    arucoDetect = ArucoDetect(AppConfig.ArucoDict, AppConfig.ArucoSize, mtx, dist)
    # arucoDetect = ArucoDetect(AppConfig.ArucoDict, 0.075, mtx, dist)
    # arucoDetect = ArucoDetect(aruco.DICT_7X7_250, 0.05, mtx, dist)

    arucoAdvPose = ArucoAdvPose()

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

                # change a rotation vector to a rotation matrix
                rotMatrix = np.zeros(shape=(3, 3))
                cv2.Rodrigues(rvec, rotMatrix)

                # make a homogeneous matrix using a rotation matrix and a translation matrix a
                hmCal2Cam = HMUtil.makeHM(rotMatrix, tvec)

                # get a transformation matrix which was created by calibration process
                hmmtx = HandEyeCalibration.loadTransformMatrix()

                # calcaluate the specific position based on hmInput
                hmWanted = HMUtil.makeHM(
                    np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
                    np.array([0.0, 0.0, 0.0]).T,
                )
                hmInput = np.dot(hmCal2Cam, hmWanted)

                # get the last homogeneous matrix
                hmResult = np.dot(hmmtx, hmInput)

                # get a final xyzuvw for the last homogenous matrix
                xyzuvw = HMUtil.convertHMtoXYZABCDeg(hmResult)

                # print(hmmtx)
                print(xyzuvw)

                [x, y, z, u, v, w] = xyzuvw
                arucoAdvPose.setPose(x, y, z, u, v, w)

                # if(arucoAdvPose.stable() == True):
                #     print('Adv. Poses: ', arucoAdvPose.getPoses())

                arucoDetect.drawAx(color_image, rvec, tvec)

            # displaqy the captured image
            cv2.imshow("Prcision Fixing", color_image)

            time.sleep(0.1)

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
