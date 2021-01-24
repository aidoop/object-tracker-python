import numpy as np
import cv2
import os
import sys
import datetime
import argparse
import time

import multiprocessing as mp
import queue

from aidobjtrack.config.appconfig import AppConfig
from aidobjtrack.camera.camera_dev_opencv import OpencvCapture
from aidobjtrack.camera.camera_dev_realsense import RealsenseCapture
from aidobjtrack.camera.camera_videocapture import VideoCapture
from aidobjtrack.util.util import ObjectTrackerErrMsg, DisplayInfoText, PrintMsg
from aidobjtrack.cameracalib.calibcamera import CalibrationCamera
from aidobjtrack.cameracalib.calibcamera_aruco import CalibrationCameraAruco
from aidobjtrack.keyhandler.calibcamera_keyhandler import CalibCameraKeyHandler
from aidobjtrack.visiongql.visiongql_client import VisonGqlDataClient

# create a directory to save captured images


def makeFrameImageDirectory():
    now = datetime.datetime.now()
    dirString = now.strftime("%Y%m%d%H%M%S")
    try:
        if not (os.path.isdir(dirString)):
            os.makedirs(os.path.join("./", "Captured", dirString))
    except OSError as e:
        print("Can't make the directory: %s" % dirFrameImage)
        raise
    return os.path.join("./", "Captured", dirString)


###############################################################################
# Hand-eye calibration process
#   -
###############################################################################


def calibcamera_engine(app_args, interproc_dict, ve=None, cq=None):

    cameraName = app_args
    if cameraName is "":
        PrintMsg.printStdErr("Input camera name is not available.")
        sys.exit()

    video_interproc_e = ve
    cmd_interproc_q = cq

    try:
        gqlDataClient = VisonGqlDataClient()
        if (
            gqlDataClient.connect(
                "http://localhost:3000", "system", "admin@hatiolab.com", "admin"
            )
            is False
        ):
            sys.exit()

        # get camera data from operato
        gqlDataClient.fetchTrackingCamerasAll()
        cameraObject = gqlDataClient.trackingCameras[cameraName]

        AppConfig.VideoFrameWidth = cameraObject.width or AppConfig.VideoFrameWidth
        AppConfig.VideoFrameHeight = cameraObject.height or AppConfig.VideoFrameHeight

        if cameraObject.type == "realsense-camera":
            rsCamDev = RealsenseCapture(cameraObject.endpoint)
            AppConfig.VideoFrameWidth = 1920
            AppConfig.VideoFrameHeight = 1080
        elif cameraObject.type == "camera-connector":
            rsCamDev = OpencvCapture(int(cameraObject.endpoint))

        # create video capture object using realsense camera device object
        vcap = VideoCapture(
            rsCamDev,
            AppConfig.VideoFrameWidth,
            AppConfig.VideoFrameHeight,
            AppConfig.VideoFramePerSec,
            cameraName,
        )

        # Start streaming
        vcap.start()

        # create a camera calqibration object
        if AppConfig.UseCalibChessBoard == True:
            calibcam = CalibrationCamera(
                AppConfig.ChessWidth, AppConfig.ChessHeight)
        else:
            calibcam = CalibrationCameraAruco()

        # TODO: check where an image directory is created..
        dirFrameImage = makeFrameImageDirectory()

        # create key handler for camera calibration1
        keyhandler = CalibCameraKeyHandler()

        # create info text
        infoText = DisplayInfoText(cv2.FONT_HERSHEY_PLAIN, (0, 20))
    except Exception as ex:
        print("Error :", ex)
        sys.exit(0)

    # setup an opencv window
    # cv2.namedWindow(cameraName, cv2.WINDOW_NORMAL)
    # cv2.setWindowProperty(
    #     cameraName, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            color_image = vcap.get_video_frame()

            # change the format to BGR format for opencv
            if cameraObject.type == "realsense-camera":
                color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

            if (
                ObjectTrackerErrMsg.checkValueIsNone(
                    color_image, "video color frame")
                == False
            ):
                break

            # create info text
            infoText.draw(color_image)

            # display the captured image
            # cv2.imshow(cameraName, color_image)
            if video_interproc_e is not None:
                color_image_resized = cv2.resize(
                    color_image, dsize=(640, 480), interpolation=cv2.INTER_AREA
                )
                interproc_dict["video"] = {
                    "device": "cameracalib",
                    "width": 640,
                    "height": 480,
                    "frame": color_image_resized,
                }
                interproc_dict["object"] = {}
                video_interproc_e.set()

            # TODO: arrange these opencv key events based on other key event handler class
            # handle key inputs
            # pressedKey = (cv2.waitKey(1) & 0xFF)
            try:
                (name, cmd) = cmd_interproc_q.get_nowait()
                if name != "cameracalib":
                    continue

                if cmd == "snapshot":
                    pressedKey = 0x63   # 'c' key
                elif cmd == "result":
                    pressedKey = 0x67   # 'g' key
                elif cmd == "exit":
                    pressedKey = 0x71   # 'q' key
            except queue.Empty:
                continue

            if keyhandler.processKeyHandler(
                pressedKey,
                color_image,
                dirFrameImage,
                calibcam,
                cameraObject.endpoint,
                infoText,
            ):
                break

    except Exception as ex:
        print("Error :", ex)

    finally:
        # Stop streaming
        vcap.stop()
        # cv2.destroyAllWindows()

        interproc_dict["app_exit"] = True
        video_interproc_e.set()


if __name__ == "__main__":

    if len(sys.argv) < 2:
        PrintMsg.printStdErr("Invalid paramters..")
        sys.exit()

    calibcamera_engine(sys.argv[1])
