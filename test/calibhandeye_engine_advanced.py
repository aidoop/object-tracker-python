import cv2
import cv2.aruco as aruco

from time import sleep
import os
import argparse
import sys


from aidobjtrack.config.appconfig import AppConfig
from aidobjtrack.camera.camera_dev_opencv import OpencvCapture
from aidobjtrack.camera.camera_dev_realsense import RealsenseCapture
from aidobjtrack.camera.camera_videocapture import VideoCapture
from aidobjtrack.robot.robot_dev_indydcp import RobotIndy7Dev
from aidobjtrack.robot.robot_arm import RobotArm
from aidobjtrack.util.util import ObjectTrackerErrMsg, DisplayInfoText
from aidobjtrack.keyhandler.calibhandeye_keyhandler_test import CalibHandEyeKeyHandler
from aidobjtrack.util.hm_util import *
from aidobjtrack.handeye.calibhandeye_handeye import *

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


if __name__ == "__main__":

    # parse program parameters to get necessary aruments
    # argPar = argparse.ArgumentParser(description="HandEye Calibration")
    # argPar.add_argument('camType', type= str, default='rs', choices=['rs', 'uvc'], metavar='CameraType', help = 'rs: Intel Realsense, uvc: UVC-Supported')
    # argPar.add_argument('camIndex', type= int, metavar='CameraIndex', help = '0, 1, ...')
    # args = argPar.parse_args()

    # create an indy7 object
    indy7 = RobotIndy7Dev()

    # create an robot arm object
    robot_arm = RobotArm(indy7, "indy7")

    if robot_arm.start(AppConfig.INDY_SERVER_IP) == False:
        print("Can't connect the robot and exit this process..")
        sys.exit()

    # create a variable for frame indexing
    flagFindMainAruco = False

    # create a handeye calib. object
    handeye = HandEyeCalibration()

    # auto handeye calibration mode
    handeye_automove = HandEyeAutoMove()
    handeye_automove.initialize()
    robot_ready_count = 0

    # camera index
    rsCamIndex = "10"

    # create the camera device object
    # rsCamDev = RealsenseCapture(rsCamIndex)
    rsCamDev = OpencvCapture(int(rsCamIndex))

    # create video capture object using realsense camera device object
    AppConfig.VideoFrameWidth = 1920
    AppConfig.VideoFrameHeight = 1080
    vcap = VideoCapture(
        rsCamDev,
        AppConfig.VideoFrameWidth,
        AppConfig.VideoFrameHeight,
        AppConfig.VideoFramePerSec,
        "camera10",
    )  # fix camera name

    # Start streaming
    vcap.start()

    # get instrinsics
    mtx, dist = vcap.getIntrinsicsMat(int(rsCamIndex), False)

    # create key handler
    keyhandler = CalibHandEyeKeyHandler()

    # create handeye object
    handeyeAruco = HandEyeAruco(
        AppConfig.HandEyeArucoDict, AppConfig.HandEyeArucoSize, mtx, dist
    )
    handeyeAruco.setCalibMarkerID(19)

    # start robot_arm as a direct-teaching mode as default
    # robot_arm.set_teaching_mode(True)

    # create info text
    infoText = DisplayInfoText(cv2.FONT_HERSHEY_PLAIN, (0, 20))

    # setup an opencv window
    # cv2.namedWindow('HandEye', cv2.WINDOW_NORMAL)
    # cv2.setWindowProperty('HandEye', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    # get frames and process a key event
    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            color_image = vcap.get_video_frame()

            # check core variables are available..
            if ObjectTrackerErrMsg.checkValueIsNone(mtx, "camera matrix") == False:
                break
            if ObjectTrackerErrMsg.checkValueIsNone(dist, "distortion coeff.") == False:
                break
            if (
                ObjectTrackerErrMsg.checkValueIsNone(color_image, "video color frame")
                == False
            ):
                break
            if (
                ObjectTrackerErrMsg.checkValueIsNone(handeye, "hand eye matrix")
                == False
            ):
                break
            if (
                ObjectTrackerErrMsg.checkValueIsNone(robot_arm, "robot_arm object")
                == False
            ):
                break

            (flagFindMainAruco, ids, rvec, tvec) = handeyeAruco.processArucoMarker(
                color_image, mtx, dist, vcap
            )

            infoText.draw(color_image)

            ###########################################################################
            # automated handeye calibration...
            if handeye_automove.isStarted():
                if handeye_automove.get_stage() == HandEyeAutoMove.STAGE_GONEXT:
                    next_move = handeye_automove.get_next()
                    if next_move:
                        robot_arm.move_task_by_async(next_move)
                    handeye_automove.set_stage(HandEyeAutoMove.STAGE_CAPTURE)
                    robot_ready_count = 0
                elif handeye_automove.get_stage() == HandEyeAutoMove.STAGE_CAPTURE:
                    robot_status = robot_arm.get_robot_status()
                    if not robot_status["busy"] and robot_status["movedone"]:
                        robot_ready_count += 1

                        if robot_ready_count > 30:
                            # process the position capture operation(= keypress 'c')
                            keyhandler.processKeyHandler(
                                99,
                                flagFindMainAruco,
                                color_image,
                                ids,
                                tvec,
                                rvec,
                                mtx,
                                dist,
                                handeye,
                                robot_arm,
                                infoText,
                                handeye_automove,
                            )
                            handeye_automove.set_stage(HandEyeAutoMove.STAGE_GONEXT)
                            robot_ready_count = 0
                else:
                    print("unknown stage..")

            # display the captured image
            cv2.imshow("HandEye", color_image)

            # handle key inputs
            pressedKey = cv2.waitKey(1) & 0xFF
            if keyhandler.processKeyHandler(
                pressedKey,
                flagFindMainAruco,
                color_image,
                ids,
                tvec,
                rvec,
                mtx,
                dist,
                handeye,
                robot_arm,
                infoText,
                handeye_automove,
            ):
                break

            # have a delay to make CPU usage lower...
            # sleep(0.1)

    except Exception as ex:
        print("Error :", ex, file=sys.stderr)

    finally:
        # direct teaching mode is disalbe before exit-
        robot_arm.set_teaching_mode(False)
        # Stop streaming
        vcap.stop()

        # arrange all to finitsh this application here
        cv2.destroyAllWindows()
        robot_arm.stop()
