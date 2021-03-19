import numpy as np
import cv2
import cv2.aruco as aruco
import math
import sys
import os

from aidoop.etc.hm_util import *
from aidoop.aruco.aruco_detect import ArucoDetect
from applications.etc.util import PrintMsg


class HandEyeAruco:
    def __init__(self, markerSelectDict, markerSize, mtx, dist, use_board=True):

        # calibration aruco marker id
        self.calibMarkerID = 2

        # test aruco marker id
        self.testMarkerID = 14

        # use the specific aruco board?
        self.useArucoBoard = use_board

        self.flagFindMainAruco = False

        # create an aruco detect object
        if self.useArucoBoard is False:
            self.arucoDetect = ArucoDetect(markerSelectDict, markerSize, mtx, dist)
        else:
            # if use a board, fixed parameters would be used. (6x6, 3.75 size, 0.05 gap, ...)
            self.arucoDetect = ArucoDetect(aruco.DICT_6X6_1000, 0.0375, mtx, dist)

    def setCalibMarkerID(self, markerID):
        self.calibMarkerID = markerID

    def processArucoMarker(self, color_image, mtx, dist, vcap):

        # operations on the frame
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # lists of ids and the corners belonging to each id
        corners, ids = self.arucoDetect.detect(gray)

        # check if the ids list is not empty
        # if no check is added the code will crash
        if np.all(ids != None):

            if self.useArucoBoard is False:
                # estimate pose of each marker and return the values
                # rvet and tvec-different from camera coefficients
                rvec, tvec = self.arucoDetect.estimate_pose(corners)

                # should do something here using extracted aruco marker ids here
                self.flagFindMainAruco = False
                for idx in range(0, ids.size):
                    if (ids[idx] == self.calibMarkerID) or (
                        ids[idx] == self.testMarkerID
                    ):
                        if (rvec[idx].shape == (1, 3)) or (rvec[idx].shape == (3, 1)):
                            # # draw the point to check the center of marker
                            # inputObjPts = np.float32([[0.0,0.0,0.0]]).reshape(-1,3)
                            # imgpts, jac = cv2.projectPoints(inputObjPts, rvec[idx], tvec[idx], mtx, dist)
                            # centerPoint = tuple(imgpts[0][0])
                            # #cv2.circle(color_image,centerPoint,1,(0,0,255), -1)
                            # #print(str(rvec[idx]) + str(' ') + str(tvec[idx]))

                            # find the main aruco marker
                            self.flagFindMainAruco = True

                    aruco.drawAxis(color_image, mtx, dist, rvec[idx], tvec[idx], 0.03)

                # draw a square around the markers
                aruco.drawDetectedMarkers(color_image, corners)
            else:
                # rvet and tvec-different from camera coefficients
                poseret, rvec, tvec = self.arucoDetect.estimate_board_pose(corners, ids)

                if poseret >= 4:
                    # draw a cooordinate axis(x, y, z)
                    self.arucoDetect.drawAx(color_image, rvec, tvec, 0.165)

                    # find the main aruco board
                    self.flagFindMainAruco = True

        else:
            # code to show 'No Ids' when no markers are found
            # cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)
            self.flagFindMainAruco = False
            tvec = None
            rvec = None
            pass

        return (self.flagFindMainAruco, ids, rvec, tvec)


class HandEyeCalibrationMode:
    HAND_TO_EYE = "hand-to-eye"
    HAND_IN_EYE = "hand-in-eye"


class HandEyeCalibration:
    def __init__(self, calibration_mode=HandEyeCalibrationMode.HAND_TO_EYE):
        # input variables for handeye calibration
        self.R_gripper2base = []
        self.t_gripper2base = []
        self.R_target2cam = []
        self.t_target2cam = []
        self.R_cam2gripper = []
        self.t_cam2gripper = []

        # predefined hm
        self.predefined_hm = np.eye(4)

        # enable/disable algorthm test
        self.AlgorithmTest = True

        # input data count
        self.cntInputData = 0

        # distance
        self.distance = 0.0

        # mode setting
        self.calibration_mode = calibration_mode

    def captureHandEyeInputs(self, robotXYZABC, camRVec, camTVec):
        # prepare Gripper2Base inputs
        hmRobot = HMUtil.convert_xyzabc_to_hm_by_deg(robotXYZABC)
        self.R_gripper2base.append(hmRobot[0:3, 0:3])
        self.t_gripper2base.append(hmRobot[0:3, 3])

        # prepare Target2Cam inputs
        camRMatrix = np.zeros(shape=(3, 3))
        cv2.Rodrigues(camRVec, camRMatrix)
        hmCam = HMUtil.create_homogeneous_matrix(camRMatrix, camTVec)
        hmCam = (
            HMUtil.inverse_homogeneous_matrix(hmCam)
            if self.calibration_mode == HandEyeCalibrationMode.HAND_TO_EYE
            else hmCam
        )
        self.R_target2cam.append(hmCam[0:3, 0:3])
        self.t_target2cam.append(hmCam[0:3, 3])
        self.cntInputData += 1

    def resetHandEyeInputs(self):
        self.R_gripper2base.clear()
        self.t_gripper2base.clear()
        self.R_target2cam.clear()
        self.t_target2cam.clear()
        self.cntInputData = 0

    def calculateHandEyeMatrix(self):
        methodHE = [cv2.CALIB_HAND_EYE_HORAUD]
        # methodHE = [cv2.CALIB_HAND_EYE_TSAI, cv2.CALIB_HAND_EYE_PARK,
        #             cv2.CALIB_HAND_EYE_HORAUD, cv2.CALIB_HAND_EYE_ANDREFF, cv2.CALIB_HAND_EYE_DANIILIDIS]

        for mth in methodHE:
            self.R_cam2gripper, self.t_cam2gripper = cv2.calibrateHandEye(
                self.R_gripper2base,
                self.t_gripper2base,
                self.R_target2cam,
                self.t_target2cam,
                None,
                None,
                mth,
            )
            # output results
            # print("--------------------------------------", file=sys.stderr)
            # print("Method %d" % mth, file=sys.stderr)
            # print(self.R_cam2gripper, file=sys.stderr)
            # print(self.t_cam2gripper, file=sys.stderr)
            # print("--------------------------------------", file=sys.stderr)
            self.distance = math.sqrt(
                math.pow(self.t_cam2gripper[0], 2.0)
                + math.pow(self.t_cam2gripper[1], 2.0)
                + math.pow(self.t_cam2gripper[2], 2.0)
            )
            # print("Distance: %f" % self.distance, file=sys.stderr)
            # print("--------------------------------------", file=sys.stderr)

        # verify handeye calculation results
        hmTemp = HMUtil.create_homogeneous_matrix(
            self.R_cam2gripper, self.t_cam2gripper.T
        )

        xyzabc_temp = HMUtil.convert_hm_to_xyzabc_by_deg(hmTemp)

    def getHandEyeResultMatrixUsingOpenCV(self):

        if self.AlgorithmTest == True:
            fsHandEyeTest = cv2.FileStorage(
                "HandEyeResultsLog.xml", cv2.FILE_STORAGE_WRITE
            )

        self.calculateHandEyeMatrix()

        if self.AlgorithmTest == True:
            fsHandEyeTest.write(
                "Method",
                math.sqrt(
                    math.pow(self.t_cam2gripper[0], 2.0)
                    + math.pow(self.t_cam2gripper[1], 2.0)
                    + math.pow(self.t_cam2gripper[2], 2.0)
                ),
            )

        if self.calibration_mode == HandEyeCalibrationMode.HAND_TO_EYE:
            # select HORAUD algorithm on temporary
            # make a homogeneous matrix from Target(Calibration) to Gripper(TCP)
            hmT2G = HMUtil.create_homogeneous_matrix(
                self.R_cam2gripper, self.t_cam2gripper.T
            )
            # make a homogeneous matrix from Gripper(TCP) to Robot Base
            hmG2B = HMUtil.create_homogeneous_matrix(
                self.R_gripper2base[0], self.t_gripper2base[0].reshape(1, 3)
            )
            # make a homogeneous matrix from Camera to Target(Target)
            hmC2T = HMUtil.create_homogeneous_matrix(
                self.R_target2cam[0], self.t_target2cam[0].reshape(1, 3)
            )

            # Final HM(Camera to Robot Base)
            # H(C2B) = H(G2B)H(T2G)H(C2T)
            hmResultTransform = np.dot(hmG2B, hmT2G)
            hmResultTransform = np.dot(hmResultTransform, hmC2T)
        else:
            hmResultTransform = HMUtil.create_homogeneous_matrix(
                self.R_cam2gripper, self.t_cam2gripper.T
            )

        if self.AlgorithmTest == True:
            fsHandEyeTest.release()

        print("Result Transform: ", file=sys.stderr)
        print(hmResultTransform, file=sys.stderr)
        return hmResultTransform

    ################################################################33
    # Using Predefined Matrix(experimental)
    def getPredefinedHandeye(self):

        ############################################################
        # TODO: get predefined data from server
        prdefined_matrix = [0.001, 0.100, 0.001, 180, 0, 180]
        # prdefined_matrix = [0.00165360882730919, 0.10332931833889342,
        #                     0.001752356149939573, -179.52921594383568, 0.24227911479934125, 179.77335961667296]

        self.predefined_hm = HMUtil.convert_xyzabc_to_hm_by_deg(prdefined_matrix)

        return self.predefined_hm

    def getHandEyeMatUsingMarker(self, robotXYZABC, camRVec, camTVec):
        # get the predefined handeye matrix
        hmPredefined = self.getPredefinedHandeye()

        # prepare Gripper2Base inputs
        hmG2B = HMUtil.convert_xyzabc_to_hm_by_deg(robotXYZABC)

        # prepare Target2Cam inputs
        camRMatrix = np.zeros(shape=(3, 3))
        cv2.Rodrigues(camRVec, camRMatrix)
        hmT2C = HMUtil.create_homogeneous_matrix(camRMatrix, camTVec)
        hmC2T = HMUtil.inverse_homogeneous_matrix(hmT2C)

        xyzuvwC2T = HMUtil.convert_hm_to_xyzabc_by_deg(hmC2T)
        # print('camera position: ', xyzuvwC2T, file=sys.stderr)

        # Final HM(Camera to Robot Base)
        # H(C2B) = H(G2B)H(T2G)H(C2T)
        hmFinal = np.dot(hmG2B, hmPredefined)
        hmFinal = np.dot(hmFinal, hmC2T)

        return hmFinal

    # save a transform matrix to xml data as a file
    @staticmethod
    def saveTransformMatrix(resultMatrix):
        calibFile = cv2.FileStorage("HandEyeCalibResult.json", cv2.FILE_STORAGE_WRITE)
        calibFile.write("HEMatrix", resultMatrix)
        calibFile.release()

    # get a transformation matrix which was created by calibration process
    @staticmethod
    def loadTransformMatrix():
        calibFile = cv2.FileStorage("HandEyeCalibResult.json", cv2.FILE_STORAGE_READ)
        hmnode = calibFile.getNode("HEMatrix")
        hmmtx = hmnode.mat()
        return hmmtx
