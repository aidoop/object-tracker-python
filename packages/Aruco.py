import cv2
import cv2.aruco as aruco

import collections

class ArucoDetect:
    def __init__(self, arucoDict, arucoSize, mtx, dist):
        # set aruco mark dictionary
        self.arucoDict = aruco.Dictionary_get(arucoDict)

        # set aruco detection parameters (see 'https://docs.opencv.org/trunk/d5/dae/tutorial_aruco_detection.html')
        # create aruco board 
        markerLength = 0.0375 # Here, measurement unit is centimetre.
        markerSeparation = 0.005   # Here, measurement unit is centimetre.
        self.arucoBoard = aruco.GridBoard_create(4, 5, markerLength, markerSeparation, self.arucoDict)

        # set aruco size
        self.arucoSize = arucoSize

        # set camera matrix and distortion coeffs.
        self.cameraMat = mtx
        self.distCoeff = dist

        self.sizeEstPoses = 10
        self.queueEstimatedPoses = collections.deque(maxlen=self.sizeEstPoses)
        
        ###########################################################
        # aruco parameters
        self.arucoParameters = aruco.DetectorParameters_create()
        self.arucoParameters.adaptiveThreshConstant = 7
        
        # Thresholding
        self.arucoParameters.adaptiveThreshWinSizeMin = 3
        self.arucoParameters.adaptiveThreshWinSizeMax = 23
        self.arucoParameters.adaptiveThreshWinSizeStep = 10
        self.arucoParameters.adaptiveThreshConstant = 7

        # Contour filtering        
        self.arucoParameters.minMarkerPerimeterRate = 0.03          # default: 0.03
        self.arucoParameters.maxMarkerPerimeterRate = 4.0
        self.arucoParameters.polygonalApproxAccuracyRate = 0.01
        self.arucoParameters.minCornerDistanceRate = 0.05           # default: 0.05
        self.arucoParameters.minMarkerDistanceRate = 0.05
        self.arucoParameters.minDistanceToBorder = 3

        # Bits Extraction
        self.arucoParameters.markerBorderBits = 1
        self.arucoParameters.minOtsuStdDev = 5.0
        self.arucoParameters.perspectiveRemovePixelPerCell = 4      # default: 4
        self.arucoParameters.perspectiveRemoveIgnoredMarginPerCell = 0.13

        # Marker identification
        self.arucoParameters.maxErroneousBitsInBorderRate = 0.35
        self.arucoParameters.errorCorrectionRate = 0.6

        # Corner Refinement
        self.arucoParameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX  #CORNER_REFINE_NONE(defalut)/CORNER_REFINE_SUBPIX/CORNER_REFINE_CONTOUR
        self.arucoParameters.cornerRefinementWinSize = 5            # default: 5
        self.arucoParameters.cornerRefinementMaxIterations = 30     # default: 30
        self.arucoParameters.cornerRefinementMinAccuracy = 0.1    # default: 0.1

        # use get optimal 
        # newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
        # self.newcameramtx = newcameramtx

    def undistort(self,grayFrameImage):
        dst = cv2.undistort(grayFrameImage, self.cameraMat, self.distCoeff, None, self.newcameramtx)
        return dst

    def detect(self, grayFrameImage):
        corners, ids, rejectedImgPoints = aruco.detectMarkers(grayFrameImage, self.arucoDict, parameters=self.arucoParameters)
        return (corners, ids)

    def estimatePose(self, corners):
        rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners, self.arucoSize, self.cameraMat, self.distCoeff)
        return (rvec, tvec)

    def estimatePose2(self, corners):
        rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners, self.arucoSize, self.cameraMat, self.distCoeff)

        # look through poses
        self.queueEstimatedPoses.append((rvec, tvec))

        for pose in self.queueEstimatedPoses:
            (rp, tp) = pose
            print("estimate pose: ", rp, " ", tp)
        

        return (rvec, tvec)

    def estimatePoseBoard(self, corners, ids):
        rvec = None
        tvec = None
        ret, rvec, tvec = aruco.estimatePoseBoard(corners, ids, self.arucoBoard, self.cameraMat, self.distCoeff, rvec, tvec) # For a board
        return (ret, rvec, tvec)

    def drawAx(self, color_image, rvec, tvec, defAxisLen=0.03):
        return aruco.drawAxis(color_image, self.cameraMat, self.distCoeff, rvec, tvec, defAxisLen)

    def calibrateCamera(self, corners, ids, counter, imageSize):
        ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraAruco(corners, ids, counter, self.arucoBoard, imageSize, None, None )
        return (ret, mtx, dist)

