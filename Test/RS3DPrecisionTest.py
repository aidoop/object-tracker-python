
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
from packages.Aruco import ArucoDetect

import math

import pyrealsense2 as rs

###############################################################################
# Hand-eye calibration process 
#   -                                                                
###############################################################################

if __name__ == '__main__':

    rsCamIndex = '001622072547'
    rsCamDev = RealsenseCapture(rsCamIndex)
    vcap = VideoCapture(rsCamDev, 1280, 720, 30, 'cameraName')
    vcap.start()
    mtx, dist = vcap.getIntrinsicsMat(0, True)

    # create an aruco detect object
    arucoDetect = ArucoDetect(Config.ArucoDict, Config.ArucoSize, mtx, dist)

    try:
        while(True):
            # Wait for a coherent pair of frames: depth and color
            (color_image, depth_image) = vcap.getFrame()

            # # operations on the frame
            # gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            # # # lists of ids and the corners belonging to each id
            # corners, ids = arucoDetect.detect(gray)
            # if (corners is None) or (ids is None):
            #     continue

            # # # rvet and tvec-different from camera coefficients
            # rvec, tvec = arucoDetect.estimatePose(corners)
            # print(tvec[0])

            # if len(tvec) >= 1:
            #     centerPoints = list()
            #     for idx in range(0, ids.size):
            #         if((rvec[idx].shape == (1,3)) or (rvec[idx].shape == (3,1))):
            #             inputObjPts = np.float32([[0.0,0.0,0.0]]).reshape(-1,3)
            #             imgpts, jac = cv2.projectPoints(inputObjPts, rvec[idx], tvec[idx], mtx, dist)
            #             centerPoints.append(tuple(imgpts[0][0]))
            # print(centerPoints[0])
            # print(vcap.get3DPosition(centerPoints[0][0], centerPoints[0][1]))
            # print('-------------------------------------')

            print(vcap.get3DPosition(421, 247))

            #     center3Ds = list()
            #     for centerPoint in centerPoints:
            #         depth = depth_frame.get_distance(centerPoint[0], centerPoint[1])
            #         depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [centerPoint[0], centerPoint[1]], depth)
            #         center3Ds.append(depth_point)

            #     #tdiff = abs(center3Ds[0] - center3Ds[1])
            #     tdist = math.sqrt(math.pow(center3Ds[0][0] - center3Ds[1][0], 2.0)+math.pow(center3Ds[0][1] - center3Ds[1][1], 2.0)+math.pow(center3Ds[0][2] - center3Ds[1][2], 2.0))
            #     print(tdist)

            # # # focusing on a (320, 120) pixel
            # # depth = depth_frame.get_distance(640, 480)
            # # depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [320, 120], depth)
            # # text = "%.5lf, %.5lf, %.5lf\n" % (depth_point[0], depth_point[1], depth_point[2])
            # # print(text)


            #time.sleep(0.5)

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


