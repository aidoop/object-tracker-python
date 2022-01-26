import cv2.aruco as aruco
from enum import Enum, unique


@unique
class ObjectTrackingMethod(Enum):
    ARUCO = "Aruco"
    BOX = "Box"
    PACK = "Pack"


class AppConfig:

    # Server IP
    ServerIP = "localhost"

    APP_TRACKING_METHOD = ObjectTrackingMethod.PACK

    # realsense camera parameters
    UseRealSenseInternalMatrix = False

    # default: 848x480 (for realsense)
    VideoFrameWidth = 848
    VideoFrameHeight = 480
    VideoFramePerSec = 30

    #################################################
    # default aruco parameters
    #################################################
    ArucoDict = aruco.DICT_5X5_250
    ArucoSize = 0.05

    #################################################
    # camera calibration parameters
    #################################################
    UseCalibChessBoard = False
    ChessWidth = 10
    ChessHeight = 7

    #################################################
    # handeye calibration parameters
    #################################################
    UseHandEyePrecisionTest = False
    UseNewCameraMatrix = False
    HandEyeTargetZ = 0.20
    CalibMarkerID = 19
    TestMarkerID = 14
    UseArucoBoard = True
    HandEyeArucoDict = aruco.DICT_7X7_1000
    HandEyeArucoSize = 0.075

    #################################################
    # PACK Detection
    #################################################
    EnableBoxIOU = True
    BoxIOUCriteria = 0.90
    AvailableObectRegion = (400, 100, 900, 500)

    #################################################
    # Debug Parameters
    #################################################
    # debug without @things-factory (Default = False)
    ObjTrackingDebugMode = False
    # debug without robot (Default = False)
    ObjTrackingDebugWoRobot = False

    #################################################
    # Debug Parameters
    #################################################
    # Fixed Depth (default: 0)
    FixedDepthPose = 0.17
