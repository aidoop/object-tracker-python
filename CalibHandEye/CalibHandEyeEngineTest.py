import cv2

from time import sleep
import os
import argparse
import sys 

# add src root directory to python path
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) )
import Config
from packages.CameraDevRealsense import RealsenseCapture
from packages.CameraDevOpencv import OpencvCapture
from packages.CameraVideoCapture import VideoCapture
from packages.RobotIndy7Dev import RobotIndy7Dev
from packages.Util import ArucoTrackerErrMsg, DisplayInfoText
from CalibHandEyeKeyHandlerTest import CalibHandEyeKeyHandler
from HandEyeUtilSet import *
from HandEyeTest import *

from packages.VisionGqlClient import VisonGqlDataClient


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
    # argPar = argparse.ArgumentParser(description="HandEye Calibration")
    # argPar.add_argument('camType', type= str, default='rs', choices=['rs', 'uvc'], metavar='CameraType', help = 'rs: Intel Realsense, uvc: UVC-Supported')
    # argPar.add_argument('camIndex', type= int, metavar='CameraIndex', help = '0, 1, ...')
    # args = argPar.parse_args()

    # create an indy7 object
    indy7 = RobotIndy7Dev()
    if(indy7.initalize(Config.INDY_SERVER_IP, Config.INDY_SERVER_NAME) == False):
        print("Can't connect the robot and exit this process..")
        sys.exit()

    # create a variable for frame indexing
    flagFindMainAruco = False

    # create a handeye calib. object
    handeye = HandEyeCalibration()

    # camera index
    rsCamIndex = '10'

    # create the camera device object
    #rsCamDev = RealsenseCapture(rsCamIndex)
    rsCamDev = OpencvCapture(int(rsCamIndex))

    # create video capture object using realsense camera device object
    vcap = VideoCapture(rsCamDev, Config.VideoFrameWidth, Config.VideoFrameHeight, Config.VideoFramePerSec, 'camera04')     # fix camera name

    # Start streaming
    vcap.start()

    # get instrinsics
    mtx, dist = vcap.getIntrinsicsMat(int(rsCamIndex), Config.UseRealSenseInternalMatrix)

  # create key handler
    keyhandler = CalibHandEyeKeyHandler()

    # create handeye object
    handeyeAruco = HandEyeAruco(Config.ArucoDict, Config.ArucoSize, mtx, dist)
    handeyeAruco.setCalibMarkerID(Config.CalibMarkerID)

    # start indy7 as a direct-teaching mode as default
    indy7.setDirectTeachingMode(True)

    # create info text 
    infoText = DisplayInfoText(cv2.FONT_HERSHEY_PLAIN, (0, 20))

    # setup an opencv window
    # cv2.namedWindow('HandEye', cv2.WINDOW_NORMAL)
    # cv2.setWindowProperty('HandEye', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    # get frames and process a key event
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
            if ArucoTrackerErrMsg.checkValueIsNone(handeye, "hand eye matrix") == False:
                break
            if ArucoTrackerErrMsg.checkValueIsNone(indy7, "indy7 object") == False:
                break            

            (flagFindMainAruco, ids, rvec, tvec) =  handeyeAruco.processArucoMarker(color_image, mtx, dist, vcap)

            infoText.draw(color_image)

            # display the captured image
            cv2.imshow('HandEye', color_image)
            
            # handle key inputs
            pressedKey = (cv2.waitKey(1) & 0xFF)
            if keyhandler.processKeyHandler(pressedKey, flagFindMainAruco, color_image, ids, tvec, rvec, mtx, dist, handeye, indy7, infoText):
                break
            
            # have a delay to make CPU usage lower...
            #sleep(0.1)

    except Exception as ex:
        print("Error :", ex)

    finally:
        # direct teaching mode is disalbe before exit
        if( indy7.getDirectTeachingMode() == True):
            indy7.setDirectTeachingMode(False)
        # Stop streaming
        vcap.stop()
        
        # arrange all to finitsh this application here
        cv2.destroyAllWindows()
        indy7.finalize()
    

        