import numpy as np
import cv2
import cv2.aruco as aruco
import os
import datetime
import glob
import sys

from aidoop.aruco.aruco_detect import ArucoDetect


class CalibrationCameraAruco:
    REPROJECTION_ERROR_CRITERION = 1.0
    ARUCO_SIZE = 0.0375
    OPENCV_TERMINATE_CRITERIA = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )

    def __init__(self, width=1920, height=1080):
        # prepare arrays to store object points and image points from all the images.
        # 3d point in real world space
        self.objpoints = []
        # 2d points in image plane.
        self.imgpoints = []

        # create an aruco detect objectCalibrationCameraAruco.ARUCO_SIZE
        self.arucoDetect = ArucoDetect(
            aruco.DICT_6X6_1000, CalibrationCameraAruco.ARUCO_SIZE, None, None
        )

        # set width & height
        self.width = width
        self.height = height

    def calculate_camera_matrix(self, images):
        # opencv algorithm termination criteria
        criteria = CalibrationCameraAruco.OPENCV_TERMINATE_CRITERIA

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
            ret, mtx, dist = self.arucoDetect.calibrate_camera(
                corners_list,
                id_list,
                counter,
                (self.height, self.width),
            )
            print("Reprojection Error: " + str(ret), file=sys.stderr)

            # TODO: should check this reprojection eror is available...
            calibResult = (
                True
                if ret <= CalibrationCameraAruco.REPROJECTION_ERROR_CRITERION
                else False
            )
        else:
            calibResult = False
            mtx = None
            dist = None
            ret = -1

        # return reprojection error
        return (calibResult, mtx, dist, ret)

    def save_result_to_file(self, fname, mtx, dist):
        # name, ext = os.path.splitext(fname)
        calibFile = cv2.FileStorage(fname, cv2.FILE_STORAGE_WRITE)
        calibFile.write("cameraMatrix", mtx)
        calibFile.write("distCoeff", dist)
        calibFile.release()

    def clear_all(self):
        self.objpoints.clear()
        self.imgpoints.clear()
