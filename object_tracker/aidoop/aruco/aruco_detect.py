import cv2
import cv2.aruco as aruco

import collections


class ArucoDetect:
    def __init__(self, arucoDict, arucoSize, mtx, dist):
        # set aruco mark dictionary
        self.arucoDict = aruco.Dictionary_get(arucoDict)

        # set aruco detection parameters (see 'https://docs.opencv.org/trunk/d5/dae/tutorial_aruco_detection.html')
        # create aruco board
        markerLength = 0.0375  # Here, measurement unit is meter
        markerSeparation = 0.005  # Here, measurement unit is meter
        self.arucoBoard = aruco.GridBoard_create(
            4, 5, markerLength, markerSeparation, self.arucoDict
        )

        # set aruco size
        self.arucoSize = arucoSize

        # set camera matrix and distortion coeffs.
        self.cameraMat = mtx
        self.distCoeff = dist

        self.estimated_pose_size = 10
        self.queue_estimated_poses = collections.deque(maxlen=self.estimated_pose_size)

        ###########################################################
        # aruco parameters
        self.aruco_parameters = aruco.DetectorParameters_create()
        self.aruco_parameters.adaptiveThreshConstant = 7

        # Thresholding
        self.aruco_parameters.adaptiveThreshWinSizeMin = 3
        self.aruco_parameters.adaptiveThreshWinSizeMax = 23
        self.aruco_parameters.adaptiveThreshWinSizeStep = 10
        self.aruco_parameters.adaptiveThreshConstant = 7

        # Contour filtering
        self.aruco_parameters.minMarkerPerimeterRate = 0.03  # default: 0.03
        self.aruco_parameters.maxMarkerPerimeterRate = 4.0
        self.aruco_parameters.polygonalApproxAccuracyRate = 0.01
        self.aruco_parameters.minCornerDistanceRate = 0.05  # default: 0.05
        self.aruco_parameters.minMarkerDistanceRate = 0.05
        self.aruco_parameters.minDistanceToBorder = 3

        # Bits Extraction
        self.aruco_parameters.markerBorderBits = 1
        self.aruco_parameters.minOtsuStdDev = 5.0
        self.aruco_parameters.perspectiveRemovePixelPerCell = 4  # default: 4
        self.aruco_parameters.perspectiveRemoveIgnoredMarginPerCell = 0.13

        # Marker identification
        self.aruco_parameters.maxErroneousBitsInBorderRate = 0.35
        self.aruco_parameters.errorCorrectionRate = 0.6

        # Corner Refinement
        # CORNER_REFINE_NONE(defalut)/CORNER_REFINE_SUBPIX/CORNER_REFINE_CONTOUR
        self.aruco_parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.aruco_parameters.cornerRefinementWinSize = 5  # default: 5
        self.aruco_parameters.cornerRefinementMaxIterations = 30  # default: 30
        self.aruco_parameters.cornerRefinementMinAccuracy = 0.1  # default: 0.1

        # use get optimal
        # newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
        # self.newcameramtx = newcameramtx

    def undistort(self, grayFrameImage):
        dst = cv2.undistort(
            grayFrameImage, self.cameraMat, self.distCoeff, None, self.newcameramtx
        )
        return dst

    def detect(self, grayFrameImage):
        corners, ids, rejectedImgPoints = aruco.detectMarkers(
            grayFrameImage, self.arucoDict, parameters=self.aruco_parameters
        )
        return (corners, ids)

    def estimate_pose(self, corners):
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
            corners, self.arucoSize, self.cameraMat, self.distCoeff
        )
        return (rvec, tvec)

    def estimate_pose_queued(self, corners):
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
            corners, self.arucoSize, self.cameraMat, self.distCoeff
        )

        # look through poses
        self.queue_estimated_poses.append((rvec, tvec))

        for pose in self.queue_estimated_poses:
            (rp, tp) = pose
            print("estimate pose: ", rp, " ", tp)

        return (rvec, tvec)

    def estimate_board_pose(self, corners, ids):
        rvec = None
        tvec = None
        ret, rvec, tvec = aruco.estimatePoseBoard(
            corners, ids, self.arucoBoard, self.cameraMat, self.distCoeff, rvec, tvec
        )  # For a board
        return (ret, rvec, tvec)

    def drawAx(self, color_image, rvec, tvec, defAxisLen=0.03):
        return aruco.drawAxis(
            color_image, self.cameraMat, self.distCoeff, rvec, tvec, defAxisLen
        )

    def calibrate_camera(self, corners, ids, counter, imageSize):
        ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraAruco(
            corners, ids, counter, self.arucoBoard, imageSize, None, None
        )
        return (ret, mtx, dist)
