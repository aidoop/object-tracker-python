import cv2
import cv2.aruco as aruco

class ArucoDetect:
    def __init__(self, arucoDict, arucoSize, mtx, dist):
        # set aruco mark dictionary
        self.arucoDict = aruco.Dictionary_get(arucoDict)

        # set aruco size
        self.arucoSize = arucoSize

        # set camera matrix and distortion coeffs.
        self.cameraMat = mtx
        self.distCoeff = dist
        
        # set aruco detection parameters
        self.arucoParameters = aruco.DetectorParameters_create()
        self.arucoParameters.adaptiveThreshConstant = 10

    def detect(self, grayFrameImage):
        corners, ids, rejectedImgPoints = aruco.detectMarkers(grayFrameImage, self.arucoDict, parameters=self.arucoParameters)
        return (corners, ids)

    def estimatePose(self, corners, ):
        rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners, self.arucoSize, self.cameraMat, self.distCoeff)
        return (rvec, tvec)
