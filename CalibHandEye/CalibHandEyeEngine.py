import indydcp_client as indycli
import pyrealsense2 as rs
import cv2

import numpy as np 
import sys 
from time import sleep
import math
import os
import datetime

# add src root directory to python path
print(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) )

from packages.CameraDevRealsense import RealsenseCapture
from packages.CameraVideoCapture import VideoCapture

from CalibHandEyeKeyHandler import CalibHandEyeKeyHandler

import Config
from HandEyeUtilSet import *
from HandEye import *


#####################################################################################
# TODO: through Robot abstaraction class
# Robot API Wrapper
#####################################################################################
def indyConnect(servIP, connName):
    # Connect
    obj = indycli.IndyDCPClient(servIP, connName)
    conResult = obj.connect()
    if conResult == False:
        print("Connection Failed")
        obj = None
    return obj

def initializeIndy7(indy):
    indy.reset_robot()
    status = indy.get_robot_status()
    print("Resetting robot")
    print("is in resetting? ", status['resetting'])
    print("is robot ready? ", status['ready'])
    if( status['direct_teaching'] == True):
        indy.direct_teaching(False)
    if( status['emergency'] == True):
        indy.stop_emergency()
    sleep(5)
    status = indy.get_robot_status()
    print("Reset robot done")
    print("is in resetting? ", status['resetting'])
    print("is robot ready? ", status['ready'])

def indyPrintJointPosition():
    print('### Test: GetJointPos() ###')
    joint_pos = indy.get_joint_pos()
    print ("Joint Pos: ")
    print (joint_pos)    

def indyPrintTaskPosition():
    task_pos = indy.get_task_pos()
    task_pos_mm = [task_pos[0], task_pos[1], task_pos[2],task_pos[3], task_pos[4], task_pos[5]]
    print ("Task Pos: ")
    print (task_pos_mm) 

def indyGetCurrentHMPose():
    task_pos = indy.get_task_pos()
    hm = HMUtil.convertXYZABCtoHMDeg(task_pos)
    return hm

def indyGetTaskPose():
    task_pos = indy.get_task_pos()
    return task_pos    


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

    # connect to Indy
    indy = indyConnect(Config.INDY_SERVER_IP, Config.INDY_SERVER_NAME)
    if(indy == None):
        print("Can't connect the robot and exit this process..")
        sys.exit()

    # intialize the robot
    print("Intialize the robot...")
    initializeIndy7(indy)
    sleep(1)

    # create a window to display video frames
    cv2.namedWindow('HandEye Calibration')

    # create a variable for frame indexing
    flagFindMainAruco = False

    # create a handeye calib. object
    handeye = HandEyeCalibration()

    # create the camera device object of intel realsense
    rsCamDev = RealsenseCapture(0)

    # create video capture object using realsense camera device object
    vcap = VideoCapture(rsCamDev, 1280, 720, 30)

    # Start streaming
    vcap.start()

    # get instrinsics
    mtx, dist = vcap.getIntrinsicsMat(Config.UseRealSenseInternalMatrix)

    # create key handler
    keyhandler = CalibHandEyeKeyHandler()

    # create handeye object
    handeyeAruco = HandEyeAruco()
    handeyeAruco.setCalibMarkerID(2)

    # start indy7 as a direct-teaching mode as default
    indy.direct_teaching(True)

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
            if keyhandler.processKeyHandler(pressedKey, flagFindMainAruco, color_image, ids, tvec, rvec, mtx, dist, handeye, indy):
                break
    finally:
        # direct teaching mode is disalbe before exit
        robotStatus = indy.get_robot_status()
        if( robotStatus['direct_teaching'] == True):
            indy.direct_teaching(False)
        # Stop streaming
        vcap.stop()
    
    # arrange all to finitsh this application here
    cv2.destroyAllWindows()
    indy.disconnect()
        