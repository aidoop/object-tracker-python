
import numpy as np
import cv2
import os
import sys
import datetime
import argparse

# add src root directory to python path
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) )

import Config
from packages.CameraDevOpencv import OpencvCapture
from packages.CameraDevRealsense import RealsenseCapture
from packages.CameraVideoCapture import VideoCapture
from packages.ErrorMsg import ArucoTrackerErrMsg
from CalibCamera import CalibrationCamera
from CalibCameraKeyHandler import CalibCameraKeyHandler

# create a directory to save captured images 
def makeFrameImageDirectory():
    now = datetime.datetime.now()
    dirString = now.strftime("%Y%m%d%H%M%S")
    try:
        if not(os.path.isdir(dirString)):
            os.makedirs(os.path.join('../','Captured', dirString))
    except OSError as e:
        print("Can't make the directory: %s" % dirFrameImage)
        raise
    return os.path.join('../','Captured', dirString)

###############################################################################
# Hand-eye calibration process 
#   -                                                                
###############################################################################

if __name__ == '__main__':

    # parse program parameters to get necessary aruments
    argPar = argparse.ArgumentParser(description="Camera Calibration")
    argPar.add_argument('camType', type= str, default='rs', choices=['rs', 'uvc'], metavar='CameraType', help = 'rs: Intel Realsense, uvc: UVC-Supported')
    argPar.add_argument('camIndex', type= int, metavar='CameraIndex', help = '0, 1, ...')
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

    # create a camera calibration object
    calibcam = CalibrationCamera(Config.ChessWidth, Config.ChessHeight)

    # TODO: check where an image directory is created..
    dirFrameImage = makeFrameImageDirectory()

    # create key handler for camera calibration1
    keyhandler = CalibCameraKeyHandler()

    print("press 'c' to capture an image")
    print("press 'g' to process camera calibration using the captured images and save the result data")
    print("press 'q' to exit...")
    iteration = 0
    try: 
        while(True):
            # Wait for a coherent pair of frames: depth and color
            color_image = vcap.getFrame()

            if ArucoTrackerErrMsg.checkValueIsNone(color_image, "video color frame") == False:
                break

            # display the captured image
            cv2.imshow('Capture Images',color_image)
            pressedKey = (cv2.waitKey(1) & 0xFF)

            # TODO: arrange these opencv key events based on other key event handler class
            # handle key inputs
            if keyhandler.processKeyHandler(pressedKey, color_image, dirFrameImage, calibcam, args.camIndex):
                break

    except Exception as ex:
        print("Error :", ex)

    finally:
        # Stop streaming
        vcap.stop()

    cv2.destroyAllWindows()

