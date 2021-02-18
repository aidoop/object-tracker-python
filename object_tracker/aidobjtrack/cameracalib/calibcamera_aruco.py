import numpy as np
import cv2
import cv2.aruco as aruco
import os
import datetime
import glob
import sys

from aidobjtrack.config.appconfig import AppConfig
from aidobjtrack.aruco.aruco_detect import ArucoDetect


class CalibrationCameraAruco:
    def __init__(self):
        # prepare arrays to store object points and image points from all the images.
        # 3d point in real world space
        self.objpoints = []
        # 2d points in image plane.
        self.imgpoints = []

        # reprojection error criterion
        self.REPROERR_CRITERION = 1.0

        # create an aruco detect object
        self.arucoDetect = ArucoDetect(aruco.DICT_6X6_1000, 0.0375, None, None)

    def calcuateCameraMatrix(self, images):
        # opencv algorithm termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        counter, corners_list, id_list = [], [], []
        first = True
        for fname in images:
            print(fname, file=sys.stderr)
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            corners, ids = self.arucoDetect.detect(gray)
            if first == True:
                corners_list = corners
                id_list = ids
                first = False
            else:
                corners_list = np.vstack((corners_list, corners))
                id_list = np.vstack((id_list, ids))
            counter.append(len(ids))

        counter = np.array(counter)

        # start camera calibartion
        if len(id_list) > 0:
            ret, mtx, dist = self.arucoDetect.calibrateCamera(
                corners_list,
                id_list,
                counter,
                (AppConfig.VideoFrameHeight, AppConfig.VideoFrameWidth),
            )
            print("Reprojection Error: " + str(ret), file=sys.stderr)

            # TODO: should check this reprojection eror is available...
            calibResult = True if ret <= self.REPROERR_CRITERION else False
        else:
            calibResult = False
            mtx = None
            dist = None
            ret = -1

        # return reprojection error
        return (calibResult, mtx, dist, ret)

    def saveResults(self, fname, mtx, dist):
        # name, ext = os.path.splitext(fname)
        calibFile = cv2.FileStorage(fname, cv2.FILE_STORAGE_WRITE)
        calibFile.write("cameraMatrix", mtx)
        calibFile.write("distCoeff", dist)
        calibFile.release()

    def clearObjImgPoints(self):
        self.objpoints.clear()
        self.imgpoints.clear()
