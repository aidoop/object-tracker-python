
import numpy as np
import cv2
import os
import sys
import datetime

# add src root directory to python path
print(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) )

from packages.CameraDevRealsense import RealsenseCapture
from packages.CameraVideoCapture import VideoCapture
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

    # create the camera device object of intel realsense
    rsCamDev = RealsenseCapture(0)

    # create video capture object using realsense camera device object
    vcap = VideoCapture(rsCamDev, 1280, 720, 30)

    # Start streaming
    vcap.start()

    # create a camera calibration object
    calibcam = CalibrationCamera(10, 7)

    # TODO: check where an image directory is created..
    dirFrameImage = makeFrameImageDirectory()

    # create key handler for camera calibration1
    keyhandler = CalibCameraKeyHandler()

    print("press 'c' to capture an image")
    print("press 'g' to calcuate the result using the captured images")
    print("press 'q' to exit...")
    iteration = 0
    try:
        while(True):
            # Wait for a coherent pair of frames: depth and color
            color_image = vcap.getFrame()

            # display the captured image
            cv2.imshow('Capture Images',color_image)
            pressedKey = (cv2.waitKey(1) & 0xFF)

            # TODO: arrange these opencv key events based on other key event handler class
            # handle key inputs
            if keyhandler.processKeyHandler(pressedKey, color_image, dirFrameImage, calibcam):
                break
    finally:
        # Stop streaming
        vcap.stop()

    cv2.destroyAllWindows()

