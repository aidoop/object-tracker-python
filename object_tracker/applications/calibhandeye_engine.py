import cv2

from time import sleep
import sys

import queue

from pyaidoop.camera.camera_videocapture import VideoCaptureFactory
from pyaidoop.etc.hm_util import *
from pyaidoop.calibration.calibhandeye_handeye import *
from pyaidoop.calibration.calibhandeye_auto_move import HandEyeAutoMove
from pyaidoop.log import Logger

from applications.config.appconfig import AppConfig
from applications.etc.util import ObjectTrackerErrMsg, DisplayInfoText
from applications.keyhandler.calibhandeye_keyhandler import CalibHandEyeKeyHandler
from applications.visiongql.visiongql_client import VisonGqlDataClient

from applications.bridge.bridge_interprocess import BridgeInterprocess

handeyecalib_info = Logger.get("handeyecalib").info
handeyecalib_err = Logger.get("handeyecalib").error


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


def calibhandeye_engine(app_args, interproc_dict=None, ve=None, cq=None):

    handeyecalib_info("handeye calibration started..")

    cameraName = app_args
    if cameraName is "":
        PrintMsg.print_error("Input camera name is not available.")
        sys.exit()

    bridge_ip = BridgeInterprocess(interproc_dict, ve, cq)
    video_interproc_e = ve
    cmd_interproc_q = cq

    try:
        handeyecalib_info("grpahql client parsing started..")
        gqlDataClient = VisonGqlDataClient()
        if (
            gqlDataClient.connect(
                "http://localhost:3000", "system", "admin@hatiolab.com", "admin"
            )
            is False
        ):
            # print("Can't connect operato vision server.")
            handeyecalib_err("grpahql client connection error..")
            sys.exit()

        gqlDataClient.fetch_tracking_camera_all()
        gqlDataClient.fetch_robot_arm_all()

        cameraObject = gqlDataClient.trackingCameras[cameraName]

        AppConfig.VideoFrameWidth = cameraObject.width
        AppConfig.VideoFrameHeight = cameraObject.height

        robotName = ""
        if cameraObject.baseRobotArm is not None:
            robotName = cameraObject.baseRobotArm["name"]
            robotObject = gqlDataClient.robotArms[robotName]
            robotIP = robotObject.endpoint

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
        handeye = HandEyeCalibration(cameraObject.handEyeMode)

        # auto handeye calibration mode
        handeyecalib_info("handeye automove being configured..")
        handeye_automove = HandEyeAutoMove()
        if cameraObject.handEyeAutoMode == True:
            autohandeye_total_move = cameraObject.autoHandeyeTotalIterations
            xyz_move = cameraObject.autoHandeyeMoveXyz
            uvw_move = cameraObject.autoHandeyeMoveUvw

            handeye_automove.initialize(
                xyz_move,
                xyz_move,
                xyz_move,
                uvw_move,
                uvw_move,
                uvw_move,
                autohandeye_total_move,
            )
            robot_ready_count = 0
        else:
            pass

        # # create the camera device object
        # if(args.camType == 'rs'):
        #     rsCamDev = RealsenseCapture(args.camIndex)
        # elif(args.camType == 'uvc'):
        #     rsCamDev = OpencvCapture(args.camIndex)
        handeyecalib_info("video capture started..")
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

        # get instrinsics
        # mtx, dist = vcap.getIntrinsicsMat(int(cameraObject.endpoint), AppConfig.UseRealSenseInternalMatrix)
        # get internal intrinsics & extrinsics in D435
        if AppConfig.UseRealSenseInternalMatrix == True:
            mtx, dist = vcap.getInternalIntrinsicsMat()
        else:
            mtx = cameraObject.cameraMatrix
            dist = cameraObject.distCoeff

        # create key handler
        handeyecalib_info("handeye calibration configuration started..")
        keyhandler = CalibHandEyeKeyHandler()

        # create handeye object
        handeyeAruco = HandEyeAruco(
            AppConfig.HandEyeArucoDict, AppConfig.HandEyeArucoSize, mtx, dist
        )
        handeyeAruco.setCalibMarkerID(AppConfig.CalibMarkerID)

        # start indy7 as a direct-teaching mode as default
        # indy7.set_teaching_mode(True)

        # create info text
        infoText = DisplayInfoText(
            cv2.FONT_HERSHEY_PLAIN,
            (10, 60),
            AppConfig.VideoFrameWidth,
            AppConfig.VideoFrameHeight,
        )

        # setup an opencv window
        if bridge_ip.isActive() is False:
            cv2.namedWindow(cameraName, cv2.WINDOW_NORMAL)
            cv2.setWindowProperty(
                cameraName, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
            )
    except Exception as ex:
        print("Preparation Exception:", ex, file=sys.stderr)
        handeyecalib_err(f"Preparation Exception: {ex}")
        sys.exit(0)

    # get frames and process a key event
    handeyecalib_info("video frame processing starts..")
    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            color_image = vcap.get_video_frame()

            # check core variables are available..
            if ObjectTrackerErrMsg.check_value_none(mtx, "camera matrix") == False:
                break
            if ObjectTrackerErrMsg.check_value_none(dist, "distortion coeff.") == False:
                break
            if (
                ObjectTrackerErrMsg.check_value_none(color_image, "video color frame")
                == False
            ):
                bridge_ip.send_dict_data(
                    "error",
                    {
                        "name": "handeyecalib:" + cameraName,
                        "message": "camera initialization failed",
                    },
                )
                break
            if (
                ObjectTrackerErrMsg.check_value_none(handeye, "hand eye matrix")
                == False
            ):
                break
            # if ObjectTrackerErrMsg.check_value_none(indy7, "indy7 object") == False:
            #     break

            # change the format to BGR format for opencv
            if cameraObject.type == "realsense-camera":
                color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

            (flagFindMainAruco, ids, rvec, tvec) = handeyeAruco.processArucoMarker(
                color_image, mtx, dist, vcap
            )

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
                            gqlDataClient.move_robotarm_task_by_nowait(
                                robotName,
                                {
                                    "x": next_move[0],
                                    "y": next_move[1],
                                    "z": next_move[2],
                                    "u": next_move[3],
                                    "v": next_move[4],
                                    "w": next_move[5],
                                },
                            )
                            handeye_automove.set_stage(HandEyeAutoMove.STAGE_CAPTURE)
                            robot_ready_count = 0
                        else:
                            handeye_automove.stop()
                    elif handeye_automove.get_stage() == HandEyeAutoMove.STAGE_CAPTURE:
                        # robot_status = indy7.get_robot_status()
                        robot_status = gqlDataClient.get_robot_status(robotName)
                        # if not robot_status['busy'] and robot_status['movedone']:
                        if not robot_status["busy"] and robot_status["moveFinished"]:
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
                                    infoText,
                                    gqlDataClient,
                                    robotName,
                                    handeye_automove,
                                    bridge_ip,
                                    cameraName,
                                )
                                handeye_automove.set_stage(HandEyeAutoMove.STAGE_GONEXT)
                                robot_ready_count = 0
                    else:
                        # print('unknown stage..')
                        pass

            # display the captured image
            if bridge_ip.isActive() is True:
                color_image_resized = cv2.resize(
                    color_image, dsize=(640, 480), interpolation=cv2.INTER_AREA
                )
                bridge_ip.send_dict_data(
                    "video",
                    {
                        "name": "handeyecalib" + ":" + cameraName,
                        "width": 640,
                        "height": 480,
                        "frame": color_image_resized,
                    },
                )
            else:
                cv2.imshow(cameraName, color_image)

            # handle key inputs
            try:
                if bridge_ip.isActive() is True:
                    (name, cmd) = bridge_ip.get_cmd_queue_no_wait()
                    if name != "handeyecalib:" + cameraName:
                        continue

                    if cmd == "start":
                        pressedKey = 0x61  # 'a' key
                        handeyecalib_info("call start handler")
                    elif cmd == "result":
                        pressedKey = 0x67  # 'g' key
                        handeyecalib_info("call result handler")
                    elif cmd == "exit":
                        pressedKey = 0x71  # 'q' key
                        handeyecalib_info("call exit handler")
                else:
                    pressedKey = cv2.waitKey(1) & 0xFF
            except queue.Empty:
                continue

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
                infoText,
                gqlDataClient,
                robotName,
                handeye_automove,
                bridge_ip,
                cameraName,
            ):
                break
    except Exception as ex:
        print("Error :", ex, file=sys.stderr)
        handeyecalib_err(f"Main Loop Error : {ex}")
        bridge_ip.send_dict_data(
            "error",
            {
                "name": "handeyelib:" + cameraName,
                "message": f"Error: {ex}",
            },
        )

    finally:
        # direct teaching mode is disalbe before exit
        # if( 1.get_teaching_mode() == True):
        #     indy7.set_teaching_mode(False)
        # Stop streaming
        vcap.stop()

        bridge_ip.send_dict_data("app_exit", True)

        handeyecalib_info("handeye calibration ends..")
    # arrange all to finitsh this application here
    # cv2.destroyAllWindows()
    # indy7.finalize()


if __name__ == "__main__":
    # parse program parameters to get necessary aruments
    # argPar = argparse.ArgumentParser(description="HandEye Calibration")
    # argPar.add_argument('camType', type= str, default='rs', choices=['rs', 'uvc'], metavar='CameraType', help = 'rs: Intel Realsense, uvc: UVC-Supported')
    # argPar.add_argument('camIndex', type= int, metavar='CameraIndex', help = '0, 1, ...')
    # args = argPar.parse_args()
    if len(sys.argv) < 2:
        print("Invalid paramters..")
        sys.exit()

    calibhandeye_engine(sys.argv[1])
