
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
from packages.Util import ArucoTrackerErrMsg, PrintMsg
from ROIArucoManager import ROIAruco2DManager
from ROIKeyHandler import ROIKeyHandler

from ROIUpdateRegions import ROIUpdateRegions
import json

from packages.VisionGqlClient import VisonGqlDataClient

###############################################################################
# Hand-eye calibration process 
#   -                                                                
###############################################################################

if __name__ == '__main__':

    # # parse program parameters to get necessary aruments
    # argPar = argparse.ArgumentParser(description="HandEye Calibration")
    # argPar.add_argument('camType', type= str, default='rs', choices=['rs', 'uvc'], metavar='CameraType', help = 'rs: Intel Realsense, uvc: UVC-Supported')
    # argPar.add_argument('camIndex', type= int, metavar='CameraIndex', help = '0, 1, ...')
    # args = argPar.parse_args()

    # # create the camera device object
    # if(args.camType == 'rs'):
    #     rsCamDev = RealsenseCapture(args.camIndex)
    # elif(args.camType == 'uvc'):
    #     rsCamDev = OpencvCapture(args.camIndex)

    if len(sys.argv) < 2:
        print("Invalid paramters..")
        sys.exit()

    cameraName = sys.argv[1]    

    gqlDataClient = VisonGqlDataClient()
    if(gqlDataClient.connect('http://localhost:3000', 'system', 'admin@hatiolab.com', 'admin') is False):
        #print("Can't connect operato vision server.")
        sys.exit()

    #gqlDataClient.parseVisionWorkspaces()
    # process all elements here...
    gqlDataClient.fetchTrackingCameras()
    cameraObject = gqlDataClient.trackingCameras[cameraName]

    if cameraObject.type == 'realsense-camera':
        rsCamDev = RealsenseCapture(int(cameraObject.endpoint))
    elif cameraObject.type == 'camera-connector':
        rsCamDev = OpencvCapture(int(cameraObject.endpoint))    

    # create video capture object using realsense camera device object
    vcap = VideoCapture(rsCamDev, Config.VideoFrameWidth, Config.VideoFrameHeight, Config.VideoFramePerSec, cameraName)

    # Start streaming
    vcap.start()

    # get instrinsics
    #mtx, dist = vcap.getIntrinsicsMat(int(cameraObject.endpoint), Config.UseRealSenseInternalMatrix)
    mtx = cameraObject.cameraMatrix
    dist = cameraObject.distCoeff    

    # create aruco manager
    ROIMgr = ROIAruco2DManager(Config.ArucoDict, Config.ArucoSize, mtx, dist)

    # for arucoPair in arucoPairList:
    #     arucoPairValues = arucoPair.split(',')
    #     ROIMgr.setMarkIdPair((int(arucoPairValues[0]), int(arucoPairValues[1])))

    # create key handler for camera calibration1
    keyhander = ROIKeyHandler()    

    PrintMsg.printStdErr("press 'g' to save the current ROI regions")
    PrintMsg.printStdErr("press 'q' to exit...")
    try:
        while(True):
            # Wait for a coherent pair of frames: depth and color
            color_image = vcap.getFrame()

            # check core variables are available..
            if ArucoTrackerErrMsg.checkValueIsNone(mtx, "camera matrix") == False:
                break
            if ArucoTrackerErrMsg.checkValueIsNone(dist, "distortion coeff.") == False:
                break
            if ArucoTrackerErrMsg.checkValueIsNone(color_image, "video color frame") == False:
                break

            # find ROI region
            (ROIRegions, ROIRegionIds) = ROIMgr.findROIPair(color_image, mtx, dist)

            # draw ROI regions
            for ROIRegion in ROIRegions:
                cv2.rectangle(color_image, ROIRegion[0], ROIRegion[1], (255,0,0), 3)

            # display the captured image
            cv2.imshow('ROI Selection',color_image)

            time.sleep(0.1)

            # TODO: arrange these opencv key events based on other key event handler class
            # handle key inputs
            pressedKey = (cv2.waitKey(1) & 0xFF)
            if keyhander.processKeyHandler(pressedKey, int(cameraObject.endpoint), ROIRegions, ROIRegionIds):
                break

    except Exception as ex:
        print("Error :", ex)

    finally:
        # Stop streaming
        vcap.stop()

    cv2.destroyAllWindows()

