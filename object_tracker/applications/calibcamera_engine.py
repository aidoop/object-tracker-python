import cv2
import os
import sys
import datetime
import queue

from pyaidoop.camera.camera_videocapture import VideoCaptureFactory
from pyaidoop.calibration.calibcamera import CalibrationCamera
from pyaidoop.calibration.calibcamera_aruco import CalibrationCameraAruco
from pyaidoop.log import Logger

from applications.config.appconfig import AppConfig
from applications.etc.util import ObjectTrackerErrMsg, DisplayInfoText, PrintMsg
from applications.keyhandler.calibcamera_keyhandler import CalibCameraKeyHandler
from applications.visiongql.visiongql_client import VisonGqlDataClient

from applications.bridge.bridge_interprocess import BridgeInterprocess

camcalib_info = Logger.get("camcalib").info
camcalib_err = Logger.get("camcalib").error


def makeFrameImageDirectory():
    now = datetime.datetime.now()
    dirString = now.strftime("%Y%m%d%H%M%S")
    try:
        if not (os.path.isdir(dirString)):
            os.makedirs(os.path.join("./", "Captured", dirString))
    except OSError as e:
        print(f"Can't make new generated directory")
        raise
    return os.path.join("./", "Captured", dirString)


###############################################################################
# Hand-eye calibration process
#   -
###############################################################################


def calibcamera_engine(app_args, interproc_dict=None, ve=None, cq=None):

    camcalib_info("camera calibration started..")

    cameraName = app_args
    if cameraName is "":
        PrintMsg.print_error("Input camera name is not available.")
        sys.exit()

    bridge_ip = BridgeInterprocess(interproc_dict, ve, cq)

    try:
        camcalib_info("grpahql client parsing started..")
        gqlDataClient = VisonGqlDataClient()
        if (
            gqlDataClient.connect(
                f"http://{AppConfig.ServerIP}:3000",
                "system",
                "admin@hatiolab.com",
                "admin",
            )
            is False
        ):
            camcalib_err("grpahql client connection error..")
            sys.exit()

        # get camera data from operato
        gqlDataClient.fetch_tracking_camera_all()
        cameraObject = gqlDataClient.trackingCameras[cameraName]

        camcalib_info("video capture started..")
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

        camcalib_info("camera calibration configuration started..")
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
        camcalib_err(f"Preparation Exception: {ex}")
        sys.exit(0)

    # setup an opencv window
    if bridge_ip.isActive() is False:
        cv2.namedWindow(cameraName, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(
            cameraName, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
        )

    camcalib_info("video frame processing starts..")
    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            color_image = vcap.get_video_frame()

            if (
                ObjectTrackerErrMsg.check_value_none(color_image, "video color frame")
                == False
            ):
                bridge_ip.send_dict_data(
                    "error",
                    {
                        "name": "cameracalib:" + cameraName,
                        "message": "camera initialization failed",
                    },
                )
                raise Exception("camera initialization failed")
                break

            # change the format to BGR format for opencv
            if cameraObject.type == "realsense-camera":
                color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

            # create info text
            infoText.draw(color_image)

            # display the captured image
            if bridge_ip.isActive() is True:
                color_image_resized = cv2.resize(
                    color_image, dsize=(640, 480), interpolation=cv2.INTER_AREA
                )
                bridge_ip.send_dict_data(
                    "video",
                    {
                        "name": "cameracalib" + ":" + cameraName,
                        "width": 640,
                        "height": 480,
                        "frame": color_image_resized,
                    },
                )
            else:
                cv2.imshow(cameraName, color_image)

            # TODO: arrange these opencv key events based on other key event handler class
            # handle key inputs
            # pressedKey = (cv2.waitKey(1) & 0xFF)
            try:
                if bridge_ip.isActive() is True:
                    (name, cmd) = bridge_ip.get_cmd_queue_no_wait()
                    if name != "cameracalib:" + cameraName:
                        continue

                    if cmd == "snapshot":
                        pressedKey = 0x63  # 'c' key
                        camcalib_info("call snapshot handler")
                    elif cmd == "result":
                        pressedKey = 0x67  # 'g' key
                        camcalib_info("call get-reseult handler")
                    elif cmd == "exit":
                        camcalib_info("call exit handler")
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
                bridge_ip,
            ):
                break

    except Exception as ex:
        print("Main Loop Error :", ex, file=sys.stderr)
        camcalib_err(f"Main Loop Error : {ex}")
        bridge_ip.send_dict_data(
            "error",
            {
                "name": "cameracalib:" + cameraName,
                "message": f"Error: {ex}",
            },
        )

    finally:
        # Stop streaming
        vcap.stop()
        # cv2.destroyAllWindows()

        bridge_ip.send_dict_data("app_exit", True)
        camcalib_info("camera calibration ends..")


if __name__ == "__main__":

    if len(sys.argv) < 2:
        PrintMsg.print_error("Invalid paramters..")
        sys.exit()

    calibcamera_engine(sys.argv[1])
