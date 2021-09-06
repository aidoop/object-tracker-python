import cv2.aruco as aruco
from enum import Enum, unique


@unique
class ObjectTrackingMethod(Enum):
    ARUCO = "Aruco"
    COUNTOUR = "Countour"
    ODAPI = "ODAPI"


class AppConfig:

    APP_TRACKING_METHOD = ObjectTrackingMethod.ARUCO

    # realsense camera parameters
    UseRealSenseInternalMatrix = False

    # default: 848x480 (for realsense)
    VideoFrameWidth = 848
    VideoFrameHeight = 480

    VideoFramePerSec = 30

    # default aruco parameters
    ArucoDict = aruco.DICT_5X5_250
    ArucoSize = 0.05

    # camera calibration parameters
    UseCalibChessBoard = False
    ChessWidth = 10
    ChessHeight = 7

    # handeye calibration parameters
    UseHandEyePrecisionTest = False
    UseNewCameraMatrix = False
    HandEyeTargetZ = 0.20
    CalibMarkerID = 19
    TestMarkerID = 14
    UseArucoBoard = True
    HandEyeArucoDict = aruco.DICT_7X7_1000
    HandEyeArucoSize = 0.075

    #################################################
    # Debug Parameters
    #################################################
    # debug without @things-factory (Default = False)
    ObjTrackingDebugMode = False
    # debug without robot (Default = False)
    ObjTrackingDebugWoRobot = False
