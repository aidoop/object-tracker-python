
import numpy as np
import cv2
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
    argPar.add_argument('--camType', type= str, default='rs', choices=['rs', 'uvc'], metavar='CameraType', help = 'Camera Type(rs: Intel Realsense, uvc: UVC-Supported')
    argPar.add_argument('camIndex', type= int, metavar='CameraIndex', help = 'Camera Index(zero-based)')
    argPar.add_argument('frameWidth', type= int, metavar='FrameWidth', help = 'Camera Frame Width')
    argPar.add_argument('frameHeight', type= int, metavar='FrameHeight', help = 'Camera Frame Height')
    argPar.add_argument('fps', type= int, metavar='FPS', help = 'Camera Frame Per Seconds')
    argPar.add_argument('-l', '--list', action='append', metavar='ArucoPairList', help = 'Aruco Mark ID Pairs for ROI Regions ex) -l 9,10 -l 30, 40')
    args = argPar.parse_args()
    arucoPairList = args.list

    # create the camera device object
    if(args.camType == 'rs'):
        rsCamDev = RealsenseCapture(args.camIndex)
    elif(args.camType == 'uvc'):
        rsCamDev = OpencvCapture(args.camIndex)    

    # create video capture object using realsense camera device object
    vcap = VideoCapture(rsCamDev, args.frameWidth, args.frameHeight, args.fps)

    # Start streaming
    vcap.start()

    # create aruco manager
    ROIMgr = ROIAruco2DManager()

    for arucoPair in arucoPairList:
        arucoPairValues = arucoPair.split(',')
        ROIMgr.setMarkIdPair((int(arucoPairValues[0]), int(arucoPairValues[1])))
    
    # get instrinsics
    mtx, dist = vcap.getIntrinsicsMat(Config.UseRealSenseInternalMatrix)

    # create key handler for camera calibration1
    keyhander = ROIKeyHandler()    

    #print("press 'c' to capture an image or press 'q' to exit...")
    try:
        while(True):
            # Wait for a coherent pair of frames: depth and color
            color_image = vcap.getFrame()

            # find ROI region
            ROIRegions = ROIMgr.findROI(color_image, mtx, dist)

            # draw ROI regions
            for ROIRegion in ROIRegions:
                cv2.rectangle(color_image, ROIRegion[0], ROIRegion[1], (255,0,0), 3)

            # display the captured image
            cv2.imshow('ROI Selection',color_image)

            time.sleep(0.1)

            # TODO: arrange these opencv key events based on other key event handler class
            # handle key inputs
            pressedKey = (cv2.waitKey(1) & 0xFF)
            if keyhander.processKeyHandler(pressedKey, ROIRegions):
                break

    finally:
        # Stop streaming
        vcap.stop()

    cv2.destroyAllWindows()

