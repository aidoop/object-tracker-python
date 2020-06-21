
import numpy as np
import cv2
import os
import time
import datetime
import glob
import sys


# add src root directory to python path
print(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) )

import Config
from packages import *
from ROIArucoManager import ROIAruco2DManager
from ROIKeyHandler import ROIKeyHandler

###############################################################################
# Hand-eye calibration process 
#   -                                                                
###############################################################################

if __name__ == '__main__':

    # create the camera device object of intel realsense
    rsCamDev = CameraDevRealsense.RealsenseCapture(0)

    # create video capture object using realsense camera device object
    vcap = CameraVideoCapture.VideoCapture(rsCamDev, 1280, 720, 30)

    # Start streaming
    vcap.start()

    # create aruco manager
    ROIMgr = ROIAruco2DManager()

    ROIMgr.setMarkIdPair((9, 10))
    ROIMgr.setMarkIdPair((30, 40))

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

