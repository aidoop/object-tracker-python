
import numpy as np
import cv2
import os
import sys
import datetime
import argparse
import time

# add src root directory to python path
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import Config
from packages.CameraDevOpencv import OpencvCapture
from packages.CameraDevRealsense import RealsenseCapture
from packages.CameraVideoCapture import VideoCapture
from packages.Util import ArucoTrackerErrMsg, DisplayInfoText
from CalibCamera import CalibrationCamera
from CalibCameraKeyHandler import CalibCameraKeyHandler
from packages.VisionGqlClient import VisonGqlDataClient

# create a directory to save captured images 
def makeFrameImageDirectory():
    now = datetime.datetime.now()
    dirString = now.strftime("%Y%m%d%H%M%S")
    try:
        if not(os.path.isdir(dirString)):
            os.makedirs(os.path.join('./','Captured', dirString))
    except OSError as e:
        print("Can't make the directory: %s" % dirFrameImage)
        raise
    return os.path.join('./','Captured', dirString)

###############################################################################
# Hand-eye calibration process 
#   -                                                                
###############################################################################

if __name__ == '__main__':

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
    cameraObject = gqlDataClient.trackingCameras[sys.argv[1]]

    if cameraObject.type == 'realsense-camera':
        rsCamDev = RealsenseCapture(cameraObject.endpoint)
    elif cameraObject.type == 'camera-connector':
        rsCamDev = OpencvCapture(int(cameraObject.endpoint))

    # create video capture object using realsense camera device object
    vcap = VideoCapture(rsCamDev, Config.VideoFrameWidth, Config.VideoFrameHeight, Config.VideoFramePerSec, cameraName)

    # Start streaming
    vcap.start()

    # create a camera calibration object
    calibcam = CalibrationCamera(Config.ChessWidth, Config.ChessHeight)

    # TODO: check where an image directory is created..
    dirFrameImage = makeFrameImageDirectory()

    # create key handler for camera calibration1
    keyhandler = CalibCameraKeyHandler()

    # create info text 
    infoText = DisplayInfoText(cv2.FONT_HERSHEY_PLAIN, (0, 20))    

    iteration = 0
    try: 
        while(True):
            # Wait for a coherent pair of frames: depth and color
            color_image = vcap.getFrame()

            if ArucoTrackerErrMsg.checkValueIsNone(color_image, "video color frame") == False:
                break

            # create info text 
            infoText.draw(color_image)

            # display the captured image
            cv2.imshow(cameraName,color_image)

            # TODO: arrange these opencv key events based on other key event handler class
            # handle key inputs
            pressedKey = (cv2.waitKey(1) & 0xFF)
            if keyhandler.processKeyHandler(pressedKey, color_image, dirFrameImage, calibcam, int(cameraObject.endpoint), infoText):
                break

    except Exception as ex:
        print("Error :", ex)    
            
    finally:
        # Stop streaming
        vcap.stop()
        cv2.destroyAllWindows()

