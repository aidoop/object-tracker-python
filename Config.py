import cv2
import cv2.aruco as aruco

# robot parameters
INDY_SERVER_IP = "192.168.1.207"
INDY_SERVER_NAME = "NRMK-Indy7"

# realsense camera parameters
UseRealSenseInternalMatrix = False
VideoFrameWidth = 1280
VideoFrameHeight = 720
VideoFramePerSec = 30

# aruco parameters
ArucoDict = aruco.DICT_5X5_250
ArucoSize = 0.05

# camera calibration parameters
ChessWidth = 10
ChessHeight = 7

# handeye calibration parameters
UseHandEyePrecisionTest = False
UseNewCameraMatrix = False
HandEyeTargetZ = 0.30
CalibMarkerID = 2
TestMarkerID = 14

# Object Tracking parameters
ObjTrackingDebugMode = False