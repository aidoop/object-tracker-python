
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
from packages.CameraVideoCapture import VideoCapture
from ROIArucoManager import ROIAruco2DManager
from ROIKeyHandler import ROIKeyHandler

###############################################################################
# Hand-eye calibration process 
#   -                                                                
###############################################################################

if __name__ == '__main__':

    # parse program parameters to get necessary aruments
    argPar = argparse.ArgumentParser(description="HandEye Calibration")
    argPar.add_argument('-camType', type= str, default='rs', choices=['rs', 'uvc'], metavar='CameraType', help = 'Camera Type(rs: Intel Realsense, uvc: UVC-Supported')
    argPar.add_argument('camIndex', type= int, metavar='CameraIndex', help = 'Camera Index(zero-based)')
    args = argPar.parse_args()

    # create the camera device object
    if(args.camType == 'rs'):
        rsCamDev = RealsenseCapture(args.camIndex)
    elif(args.camType == 'uvc'):
        rsCamDev = OpencvCapture(args.camIndex)    

    # create video capture object using realsense camera device object
    vcap = VideoCapture(rsCamDev, Config.VideoFrameWidth, Config.VideoFrameHeight, Config.VideoFramePerSec)

    # Start streaming
    vcap.start()

    # get instrinsics
    mtx, dist = vcap.getIntrinsicsMat(Config.UseRealSenseInternalMatrix)

    # create aruco manager
    ROIMgr = ROIAruco2DManager(Config.ArucoDict, Config.ArucoSize, mtx, dist)

    # for arucoPair in arucoPairList:
    #     arucoPairValues = arucoPair.split(',')
    #     ROIMgr.setMarkIdPair((int(arucoPairValues[0]), int(arucoPairValues[1])))

    # create key handler for camera calibration1
    keyhander = ROIKeyHandler()    

    #print("press 'c' to capture an image or press 'q' to exit...")
    try:
        while(True):
            # Wait for a coherent pair of frames: depth and color
            color_image = vcap.getFrame()

            # find ROI region
            ROIRegions = ROIMgr.findROIPair(color_image, mtx, dist)

            # draw ROI regions
            for ROIRegion in ROIRegions:
                cv2.rectangle(color_image, ROIRegion[0], ROIRegion[1], (255,0,0), 3)

            # display the captured image
            cv2.imshow('ROI Selection',color_image)

            time.sleep(0.1)

            # TODO: arrange these opencv key events based on other key event handler class
            # handle key inputs
            pressedKey = (cv2.waitKey(1) & 0xFF)
            if keyhander.processKeyHandler(pressedKey, args.camIndex, ROIRegions):
                break

    finally:
        # Stop streaming
        vcap.stop()

    cv2.destroyAllWindows()

