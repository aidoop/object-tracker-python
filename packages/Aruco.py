import cv2
import cv2.aruco as aruco

class ArucoDetect:
    def __init__(self, arucoDict, arucoSize, mtx, dist):
        # set aruco mark dictionary
        self.arucoDict = aruco.Dictionary_get(arucoDict)

        # create aruco board 
        markerLength = 3.75  # Here, measurement unit is centimetre.
        markerSeparation = 0.5   # Here, measurement unit is centimetre.
        self.arucoBoard = aruco.GridBoard_create(4, 5, markerLength, markerSeparation, self.arucoDict)

        # set aruco size
        self.arucoSize = arucoSize

        # set camera matrix and distortion coeffs.
        self.cameraMat = mtx
        self.distCoeff = dist
        
        # set aruco detection parameters (see 'https://docs.opencv.org/trunk/d5/dae/tutorial_aruco_detection.html')
        self.arucoParameters = aruco.DetectorParameters_create()
        self.arucoParameters.adaptiveThreshConstant = 7
        
        # Thresholding
        self.arucoParameters.adaptiveThreshWinSizeMin = 3
        self.arucoParameters.adaptiveThreshWinSizeMax = 23
        self.arucoParameters.adaptiveThreshWinSizeStep = 10
        self.arucoParameters.adaptiveThreshConstant = 7

        # Contour filtering        
        self.arucoParameters.minMarkerPerimeterRate = 0.02          # default: 0.03
        self.arucoParameters.maxMarkerPerimeterRate = 4.0
        self.arucoParameters.polygonalApproxAccuracyRate = 0.01
        self.arucoParameters.minCornerDistanceRate = 0.055           # default: 0.05
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
        self.arucoParameters.cornerRefinementWinSize = 3            # default: 5
        self.arucoParameters.cornerRefinementMaxIterations = 100     # default: 30
        self.arucoParameters.cornerRefinementMinAccuracy = 0.001    # default: 0.1


    def detect(self, grayFrameImage):
        corners, ids, rejectedImgPoints = aruco.detectMarkers(grayFrameImage, self.arucoDict, parameters=self.arucoParameters)
        return (corners, ids)

    def estimatePose(self, corners):
        rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners, self.arucoSize, self.cameraMat, self.distCoeff)
        return (rvec, tvec)

    def estimatePoseBoard(self, corners, ids):
        ret, rvec, tvec = aruco.estimatePoseBoard(corners, ids, self.arucoBoard, self.cameraMat, self.distCoeff) # For a board
