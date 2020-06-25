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

# handeye calibration parameters
UseHandEyeAlgorithmTest = False
UseHandEyePrint3DCoords = False
UseNewCameraMatrix = False
HandEyeTargetZ = 0.10
CalibMarkerID = 2
TestMarkerID = 14
