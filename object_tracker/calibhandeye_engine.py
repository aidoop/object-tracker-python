import cv2

from time import sleep
import os
import argparse
import sys

from aidobjtrack.config.appconfig import AppConfig
from aidobjtrack.camera.camera_dev_opencv import OpencvCapture
from aidobjtrack.camera.camera_dev_realsense import RealsenseCapture
from aidobjtrack.camera.camera_videocapture import VideoCapture
#from aidobjtrack.robot.robot_dev_indydcp import RobotIndy7Dev
from aidobjtrack.util.util import ObjectTrackerErrMsg, DisplayInfoText
from aidobjtrack.keyhandler.calibhandeye_keyhandler import CalibHandEyeKeyHandler
from aidobjtrack.util.hm_util import *
from aidobjtrack.handeye.calibhandeye_handeye import *

from aidobjtrack.visiongql.visiongql_client import VisonGqlDataClient
from aidobjtrack.handeye.calibhandeye_auto_move import HandEyeAutoMove


#####################################################################################
# Utility Functions
#####################################################################################
def drawText(img, text, imgpt):
    font = cv2.FONT_HERSHEY_PLAIN
    cv2.putText(img, text, imgpt, font, 1, (0, 255, 0), 1, cv2.LINE_AA)

###############################################################################
# Hand-eye calibration process
#   -
###############################################################################


def calibhandeye_engine(camera_name, img_dict=None):

    cameraName = camera_name
    gqlDataClient = VisonGqlDataClient()
    if(gqlDataClient.connect('http://localhost:3000', 'system', 'admin@hatiolab.com', 'admin') is False):
        #print("Can't connect operato vision server.")
        sys.exit()

    gqlDataClient.fetchTrackingCamerasAll()
    gqlDataClient.fetchRobotArmsAll()

    cameraObject = gqlDataClient.trackingCameras[cameraName]

    AppConfig.VideoFrameWidth = cameraObject.width
    AppConfig.VideoFrameHeight = cameraObject.height

    robotName = ''
    if cameraObject.baseRobotArm is not None:
        robotName = cameraObject.baseRobotArm['name']
        robotObject = gqlDataClient.robotArms[robotName]
        robotIP = robotObject.endpoint
    else:
        robotIP = AppConfig.INDY_SERVER_IP

    # create an indy7 object
    # TODO: should check here if auto-mode will be used..
    # if cameraObject.handEyeAutoMode == True:
    #     indy7 = RobotIndy7Dev()
    #     if(indy7.initalize(robotIP, AppConfig.INDY_SERVER_NAME) == False):
    #         # print("Can't connect the robot and exit this process..")
    #         sys.exit()

    # create a window to display video frames
    # cv2.namedWindow(cameraName)

    # create a variable for frame indexing
    flagFindMainAruco = False

    # create a handeye calib. object
    handeye = HandEyeCalibration()

    # auto handeye calibration mode
    handeye_automove = HandEyeAutoMove()
    if cameraObject.handEyeAutoMode == True:
        handeye_automove.initialize()
        robot_ready_count = 0
    else:
        pass

    # # create the camera device object
    # if(args.camType == 'rs'):
    #     rsCamDev = RealsenseCapture(args.camIndex)
    # elif(args.camType == 'uvc'):
    #     rsCamDev = OpencvCapture(args.camIndex)
    if cameraObject.type == 'realsense-camera':
        rsCamDev = RealsenseCapture(cameraObject.endpoint)
        AppConfig.VideoFrameWidth = 1920
        AppConfig.VideoFrameHeight = 1080
    elif cameraObject.type == 'camera-connector':
        rsCamDev = OpencvCapture(int(cameraObject.endpoint))

    # create video capture object using realsense camera device object
    vcap = VideoCapture(rsCamDev, AppConfig.VideoFrameWidth,
                        AppConfig.VideoFrameHeight, AppConfig.VideoFramePerSec, cameraName)

    # Start streaming
    vcap.start()

    # get instrinsics
    #mtx, dist = vcap.getIntrinsicsMat(int(cameraObject.endpoint), AppConfig.UseRealSenseInternalMatrix)
    # get internal intrinsics & extrinsics in D435
    if(AppConfig.UseRealSenseInternalMatrix == True):
        mtx, dist = vcap.getInternalIntrinsicsMat()
    else:
        mtx = cameraObject.cameraMatrix
        dist = cameraObject.distCoeff

  # create key handler
    keyhandler = CalibHandEyeKeyHandler()

    # create handeye object
    handeyeAruco = HandEyeAruco(
        AppConfig.HandEyeArucoDict, AppConfig.HandEyeArucoSize, mtx, dist)
    handeyeAruco.setCalibMarkerID(AppConfig.CalibMarkerID)

    # start indy7 as a direct-teaching mode as default
    # indy7.set_teaching_mode(True)

    # create info text
    infoText = DisplayInfoText(cv2.FONT_HERSHEY_PLAIN, (0, 20))

    # setup an opencv window
    cv2.namedWindow(cameraName, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(
        cameraName, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    # get frames and process a key event
    try:
        while(True):
            # Wait for a coherent pair of frames: depth and color
            color_image = vcap.get_video_frame()

            # change the format to BGR format for opencv
            if cameraObject.type == 'realsense-camera':
                color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

            # check core variables are available..
            if ObjectTrackerErrMsg.checkValueIsNone(mtx, "camera matrix") == False:
                break
            if ObjectTrackerErrMsg.checkValueIsNone(dist, "distortion coeff.") == False:
                break
            if ObjectTrackerErrMsg.checkValueIsNone(color_image, "video color frame") == False:
                break
            if ObjectTrackerErrMsg.checkValueIsNone(handeye, "hand eye matrix") == False:
                break
            # if ObjectTrackerErrMsg.checkValueIsNone(indy7, "indy7 object") == False:
            #     break

            (flagFindMainAruco, ids, rvec, tvec) = handeyeAruco.processArucoMarker(
                color_image, mtx, dist, vcap)

            # draw info. text
            infoText.draw(color_image)

            ###########################################################################
            # automated handeye calibration...
            if cameraObject.handEyeAutoMode == True:
                if handeye_automove.isStarted():
                    if handeye_automove.get_stage() == HandEyeAutoMove.STAGE_GONEXT:
                        next_move = handeye_automove.get_next()
                        if next_move:
                            # indy7.move_task_by_async(next_move)
                            gqlDataClient.moveRobotTaskByNoWait(robotName, {
                                'x': next_move[0], 'y': next_move[1], 'z': next_move[2], 'u': next_move[3], 'v': next_move[4], 'w': next_move[5]})
                        handeye_automove.set_stage(
                            HandEyeAutoMove.STAGE_CAPTURE)
                        robot_ready_count = 0
                    elif handeye_automove.get_stage() == HandEyeAutoMove.STAGE_CAPTURE:
                        # robot_status = indy7.get_robot_status()
                        robot_status = gqlDataClient.get_robot_status(
                            robotName)
                        # if not robot_status['busy'] and robot_status['movedone']:
                        if not robot_status['busy'] and robot_status['moveFinished']:
                            robot_ready_count += 1

                            if robot_ready_count > 30:
                                # process the position capture operation(= keypress 'c')
                                keyhandler.processKeyHandler(
                                    99, flagFindMainAruco, color_image, ids, tvec, rvec, mtx, dist, handeye, infoText, gqlDataClient, robotName, handeye_automove)
                                handeye_automove.set_stage(
                                    HandEyeAutoMove.STAGE_GONEXT)
                                robot_ready_count = 0
                    else:
                        #print('unknown stage..')
                        pass

            # display the captured image
            cv2.imshow(cameraName, color_image)

            # handle key inputs
            pressedKey = (cv2.waitKey(1) & 0xFF)
            if keyhandler.processKeyHandler(pressedKey, flagFindMainAruco, color_image, ids, tvec, rvec, mtx, dist, handeye, infoText, gqlDataClient, robotName, handeye_automove):
                break

            # have a delay to make CPU usage lower...
            # qsleep(0.2)

    except Exception as ex:
        print("Error :", ex)

    finally:
        # direct teaching mode is disalbe before exit
        # if( 1.get_teaching_mode() == True):
        #     indy7.set_teaching_mode(False)
        # Stop streaming
        vcap.stop()

    # arrange all to finitsh this application here
    cv2.destroyAllWindows()
    # indy7.finalize()


if __name__ == '__main__':
    # parse program parameters to get necessary aruments
    # argPar = argparse.ArgumentParser(description="HandEye Calibration")
    # argPar.add_argument('camType', type= str, default='rs', choices=['rs', 'uvc'], metavar='CameraType', help = 'rs: Intel Realsense, uvc: UVC-Supported')
    # argPar.add_argument('camIndex', type= int, metavar='CameraIndex', help = '0, 1, ...')
    # args = argPar.parse_args()
    if len(sys.argv) < 2:
        print("Invalid paramters..")
        sys.exit()

    calibhandeye_engine(sys.argv[1])
