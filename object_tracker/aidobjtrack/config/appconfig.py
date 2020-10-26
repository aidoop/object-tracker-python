import cv2
import cv2.aruco as aruco
from enum import Enum, unique


@unique
class ObjectTrackingMethod(Enum):
    ARUCO = 1
    MRCNN = 2


class AppConfig:

    APP_TRACKING_METHOD = ObjectTrackingMethod.ARUCO

    # robot parameters
    #INDY_SERVER_IP = "192.168.0.104"
    INDY_SERVER_IP = "192.168.0.207"
    INDY_SERVER_NAME = "NRMK-Indy7"

    # realsense camera parameters
    UseRealSenseInternalMatrix = False

    VideoFrameWidth = 1920 if APP_TRACKING_METHOD is ObjectTrackingMethod.ARUCO else 848
    VideoFrameHeight = 1080 if APP_TRACKING_METHOD is ObjectTrackingMethod.ARUCO else 480

    VideoFramePerSec = 30

    # default aruco parameters
    ArucoDict = aruco.DICT_5X5_250
    ArucoSize = 0.05

    # camera calibration parameters
    UseCalibChessBoard = True
    ChessWidth = 10
    ChessHeight = 7

    # handeye calibration parameters
    UseHandEyePrecisionTest = False
    UseNewCameraMatrix = False
    HandEyeTargetZ = 0.20
    CalibMarkerID = 2
    TestMarkerID = 14
    UseArucoBoard = True

    # debug without @things-factory
    ObjTrackingDebugMode = True
