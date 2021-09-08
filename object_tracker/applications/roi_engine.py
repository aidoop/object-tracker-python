import cv2
import time
import sys

from pyaidoop.camera.camera_videocapture import VideoCaptureFactory

from applications.config.appconfig import AppConfig
from applications.etc.util import ObjectTrackerErrMsg, DisplayInfoText
from applications.keyhandler.roi_keyhandler import ROIKeyHandler
from applications.visiongql.visiongql_client import VisonGqlDataClient
from applications.roi_arucomanager import ROIAruco2DManager

###############################################################################
# Hand-eye calibration process
#   -
###############################################################################

if __name__ == "__main__":

    # # parse program parameters to get necessary aruments
    # argPar = argparse.ArgumentParser(description="HandEye Calibration")
    # argPar.add_argument('camType', type= str, default='rs', choices=['rs', 'uvc'], metavar='CameraType', help = 'rs: Intel Realsense, uvc: UVC-Supported')
    # argPar.add_argument('camIndex', type= int, metavar='CameraIndex', help = '0, 1, ...')
    # args = argPar.parse_args()

    # # create the camera device object
    # if(args.camType == 'rs'):
    #     rsCamDev = RealsenseCapture(args.camIndex)
    # elif(args.camType == 'uvc'):
    #     rsCamDev = OpencvCapture(args.camIndex)

    if len(sys.argv) < 2:
        print("Invalid paramters..")
        sys.exit()

    cameraName = sys.argv[1]

    try:
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
            # print("Can't connect operato vision server.")
            sys.exit()

        # gqlDataClient.parseVisionWorkspaces()
        # process all elements here...
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

        # get instrinsics
        # mtx, dist = vcap.getIntrinsicsMat(int(cameraObject.endpoint), AppConfig.UseRealSenseInternalMatrix)
        if AppConfig.UseRealSenseInternalMatrix == True:
            mtx, dist = vcap.getInternalIntrinsicsMat()
        else:
            mtx = cameraObject.cameraMatrix
            dist = cameraObject.distCoeff

        # create aruco manager
        ROIMgr = ROIAruco2DManager(AppConfig.ArucoDict, AppConfig.ArucoSize, mtx, dist)

        # for arucoPair in arucoPairList:
        #     arucoPairValues = arucoPair.split(',')
        #     ROIMgr.setMarkIdPair((int(arucoPairValues[0]), int(arucoPairValues[1])))

        # create key handler for camera calibration1
        keyhander = ROIKeyHandler()

        # create info text
        infoText = DisplayInfoText(cv2.FONT_HERSHEY_PLAIN, (0, 20))
    except Exception as ex:
        print("Preparation Exception:", ex, file=sys.stderr)
        sys.exit(0)

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
                ObjectTrackerErrMsg.check_value_none(color_image, "vidqeo color frame")
                == False
            ):
                break

            # find ROI region
            (ROIRegions, ROIRegionIds) = ROIMgr.findROIPair(color_image, mtx, dist)

            # draw ROI regions
            for ROIRegion in ROIRegions:
                cv2.rectangle(color_image, ROIRegion[0], ROIRegion[1], (255, 0, 0), 3)

            # create info text
            infoText.draw(color_image)

            # display the captured image
            cv2.imshow("ROI Selection", color_image)

            time.sleep(0.2)

            # TODO: arrange these opencv key events based on other key event handler class
            # handle key inputs
            pressedKey = cv2.waitKey(1) & 0xFF
            if keyhander.processKeyHandler(
                pressedKey,
                int(cameraObject.endpoint),
                ROIRegions,
                ROIRegionIds,
                infoText,
            ):
                break

    except Exception as ex:
        print("Main Loop Error :", ex, file=sys.stderr)

    finally:
        # Stop streaming
        vcap.stop()

    cv2.destroyAllWindows()
