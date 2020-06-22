import cv2

from time import sleep
import os
import argparse

# add src root directory to python path
import sys 
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) )

import Config
from packages.CameraDevRealsense import RealsenseCapture
from packages.CameraVideoCapture import VideoCapture
from packages.RobotIndy7Dev import RobotIndy7Dev
from CalibHandEyeKeyHandler import CalibHandEyeKeyHandler
from HandEyeUtilSet import *
from HandEye import *


#####################################################################################
# Utility Functions
#####################################################################################
def drawText(img, text, imgpt):
    font = cv2.FONT_HERSHEY_PLAIN
    cv2.putText(img, text, imgpt, font, 1, (0,255,0),1,cv2.LINE_AA)

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
    argPar.add_argument('arucoID', type= int, metavar='ArucoMarkID', help = 'Aruco Mark ID for HandEye Calibration')
    args = argPar.parse_args()

    # create an indy7 object
    indy7 = RobotIndy7Dev()
    if(indy7.initalize(Config.INDY_SERVER_IP, Config.INDY_SERVER_NAME) == False):
        print("Can't connect the robot and exit this process..")
        sys.exit()

    # create a window to display video frames
    cv2.namedWindow('HandEye Calibration')

    # create a variable for frame indexing
    flagFindMainAruco = False

    # create a handeye calib. object
    handeye = HandEyeCalibration()

    # create the camera device object
    if(args.camType == 'rs'):
        rsCamDev = RealsenseCapture(args.camIndex)
    elif(args.camType == 'uvc'):
        rsCamDev = OpencvCapture(args.camIndex)

    # create video capture object using realsense camera device object
    vcap = VideoCapture(rsCamDev, args.frameWidth, args.frameHeight, args.fps)

    # Start streaming
    vcap.start()

    # get instrinsics
    mtx, dist = vcap.getIntrinsicsMat(Config.UseRealSenseInternalMatrix)

    # create key handler
    keyhandler = CalibHandEyeKeyHandler()

    # create handeye object
    handeyeAruco = HandEyeAruco()
    handeyeAruco.setCalibMarkerID(args.arucoID)

    # start indy7 as a direct-teaching mode as default
    indy7.setDirectTeachingMode(True)

    # get frames and process a key event
    try:
        while(True):
            # Wait for a coherent pair of frames: depth and color
            color_image = vcap.getFrame()

            (flagFindMainAruco, ids, rvec, tvec) =  handeyeAruco.processArucoMarker(color_image, mtx, dist)

            # display the captured image
            cv2.imshow('HandEye Calibration', color_image)
            
            # handle key inputs
            pressedKey = (cv2.waitKey(1) & 0xFF)
            if keyhandler.processKeyHandler(pressedKey, flagFindMainAruco, color_image, ids, tvec, rvec, mtx, dist, handeye, indy7):
                break
            
            # have a delay to make CPU usage lower...
            sleep(0.2)
    finally:
        # direct teaching mode is disalbe before exit
        if( indy7.getDirectTeachingMode() == True):
            indy7.setDirectTeachingMode(False)
        # Stop streaming
        vcap.stop()
    
    # arrange all to finitsh this application here
    cv2.destroyAllWindows()
    indy7.finalize()
        