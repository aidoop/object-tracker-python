import numpy as np
import cv2
import os
import sys
import datetime
import argparse
import time
import multiprocessing as mp
import queue

from aidoop.camera.camera_videocapture import VideoCaptureFactory
from aidoop.calibration.calibcamera import CalibrationCamera
from aidoop.calibration.calibcamera_aruco import CalibrationCameraAruco

from applications.config.appconfig import AppConfig
from applications.etc.util import ObjectTrackerErrMsg, DisplayInfoText, PrintMsg
from applications.keyhandler.calibcamera_keyhandler import CalibCameraKeyHandler
from applications.visiongql.visiongql_client import VisonGqlDataClient


def makeFrameImageDirectory():
    now = datetime.datetime.now()
    dirString = now.strftime("%Y%m%d%H%M%S")
    try:
        if not (os.path.isdir(dirString)):
            os.makedirs(os.path.join("./", "Captured", dirString))
    except OSError as e:
        print(f"Can't make the directory: {dirFrameImage}")
        raise
    return os.path.join("./", "Captured", dirString)


###############################################################################
# Hand-eye calibration process
#   -
###############################################################################


def calibcamera_engine(app_args, interproc_dict=None, ve=None, cq=None):

    cameraName = app_args
    if cameraName is "":
        PrintMsg.print_error("Input camera name is not available.")
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
        gqlDataClient.fetch_tracking_camera_all()
        cameraObject = gqlDataClient.trackingCameras[cameraName]

        AppConfig.VideoFrameWidth = cameraObject.width or AppConfig.VideoFrameWidth
        AppConfig.VideoFrameHeight = cameraObject.height or AppConfig.VideoFrameHeight
        (AppConfig.VideoFrameWidth, AppConfig.VideoFrameHeight) = (
            (1920, 1080)
            if cameraObject.type == "realsense-camera"
            else (AppConfig.VideoFrameWidth, AppConfig.VideoFrameHeight)
        )

        vcap = VideoCaptureFactory.create_video_capture(
            cameraObject.type,
            cameraObject.endpoint,
            AppConfig.VideoFrameWidth,
            AppConfig.VideoFrameHeight,
            AppConfig.VideoFramePerSec,
            cameraName,
        )

        # Start streaming
        vcap.start()

        # create a camera calqibration object
        if AppConfig.UseCalibChessBoard == True:
            calibcam = CalibrationCamera(AppConfig.ChessWidth, AppConfig.ChessHeight)
        else:
            calibcam = CalibrationCameraAruco(
                AppConfig.VideoFrameWidth, AppConfig.VideoFrameHeight
            )

        # TODO: check where an image directory is created..
        dirFrameImage = makeFrameImageDirectory()

        # create key handler for camera calibration1
        keyhandler = CalibCameraKeyHandler()

        # create info text
        infoText = DisplayInfoText(
            cv2.FONT_HERSHEY_PLAIN,
            (10, 60),
            AppConfig.VideoFrameWidth,
            AppConfig.VideoFrameHeight,
        )
    except Exception as ex:
        print("Preparation Exception:", ex, file=sys.stderr)
        sys.exit(0)

    # setup an opencv window
    if video_interproc_e is None:
        cv2.namedWindow(cameraName, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(
            cameraName, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
        )

    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            color_image = vcap.get_video_frame()

            # change the format to BGR format for opencv
            if cameraObject.type == "realsense-camera":
                color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

            if (
                ObjectTrackerErrMsg.check_value_none(color_image, "video color frame")
                == False
            ):
                break

            # create info text
            infoText.draw(color_image)

            # display the captured image
            if video_interproc_e is not None:
                color_image_resized = cv2.resize(
                    color_image, dsize=(640, 480), interpolation=cv2.INTER_AREA
                )
                if interproc_dict is not None:
                    interproc_dict["video"] = {
                        "name": "cameracalib" + ":" + cameraName,
                        "width": 640,
                        "height": 480,
                        "frame": color_image_resized,
                    }
                    video_interproc_e.set()
            else:
                cv2.imshow(cameraName, color_image)

            # TODO: arrange these opencv key events based on other key event handler class
            # handle key inputs
            # pressedKey = (cv2.waitKey(1) & 0xFF)
            try:
                if cmd_interproc_q is not None:
                    (name, cmd) = cmd_interproc_q.get_nowait()
                    if name != "cameracalib:" + cameraName:
                        continue

                    if cmd == "snapshot":
                        pressedKey = 0x63  # 'c' key
                    elif cmd == "result":
                        pressedKey = 0x67  # 'g' key
                    elif cmd == "exit":
                        pressedKey = 0x71  # 'q' key
                else:
                    pressedKey = cv2.waitKey(1) & 0xFF
            except queue.Empty:
                continue

            if keyhandler.processKeyHandler(
                pressedKey,
                color_image,
                dirFrameImage,
                calibcam,
                cameraObject.endpoint,
                infoText,
                cameraName,
                interproc_dict,
                video_interproc_e,
            ):
                break

    except Exception as ex:
        print("Main Loop Error :", ex, file=sys.stderr)

    finally:
        # Stop streaming
        vcap.stop()
        # cv2.destroyAllWindows()

        if interproc_dict is not None:
            interproc_dict["app_exit"] = True
            if video_interproc_e is not None:
                video_interproc_e.set()


if __name__ == "__main__":

    if len(sys.argv) < 2:
        PrintMsg.print_error("Invalid paramters..")
        sys.exit()

    calibcamera_engine(sys.argv[1])
