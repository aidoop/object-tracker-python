
import numpy as np
import cv2
import cv2.aruco as aruco
import math
import sys
import os

from aidobjtrack.config.appconfig import AppConfig
from aidobjtrack.util.hm_util import *
from aidobjtrack.aruco.aruco_detect import ArucoDetect
from aidobjtrack.util.util import PrintMsg


class HandEyeAruco:

    def __init__(self, markerSelectDict, markerSize, mtx, dist):

        # calibration aruco marker id
        self.calibMarkerID = 2

        # test aruco marker id
        self.testMarkerID = 14

       # use the specific aruco board?
        self.useArucoBoard = AppConfig.UseArucoBoard

        self.flagFindMainAruco = False

        # create an aruco detect object
        if self.useArucoBoard is False:
            self.arucoDetect = ArucoDetect(
                markerSelectDict, markerSize, mtx, dist)
        else:
            # if use a board, fixed parameters would be used. (6x6, 3.75 size, 0.05 gap, ...)
            self.arucoDetect = ArucoDetect(
                aruco.DICT_6X6_1000, 0.0375, mtx, dist)

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
                rvec, tvec = self.arucoDetect.estimatePose(corners)

                # should do something here using extracted aruco marker ids here
                self.flagFindMainAruco = False
                for idx in range(0, ids.size):
                    if(ids[idx] == self.calibMarkerID) or (ids[idx] == self.testMarkerID):
                        if((rvec[idx].shape == (1, 3)) or (rvec[idx].shape == (3, 1))):
                            # inputObjPts = np.float32([[0.0,0.0,0.0]]).reshape(-1,3)
                            # imgpts, jac = cv2.projectPoints(inputObjPts, rvec[idx], tvec[idx], mtx, dist)
                            # centerPoint = tuple(imgpts[0][0])
                            # #cv2.circle(color_image,centerPoint,1,(0,0,255), -1)
                            # #print(str(rvec[idx]) + str(' ') + str(tvec[idx]))

                            # find the main aruco marker
                            self.flagFindMainAruco = True

                    aruco.drawAxis(color_image, mtx, dist,
                                   rvec[idx], tvec[idx], 0.03)

                # draw a square around the markers
                aruco.drawDetectedMarkers(color_image, corners)
            else:
                # rvet and tvec-different from camera coefficients
                poseret, rvec, tvec = self.arucoDetect.estimatePoseBoard(
                    corners, ids)

                if poseret >= 4:
                    # draw a cooordinate axis(x, y, z)
                    self.arucoDetect.drawAx(color_image, rvec, tvec, 0.165)

                    # find the main aruco board
                    self.flagFindMainAruco = True

        else:
            # code to show 'No Ids' when no markers are found
            #cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)
            self.flagFindMainAruco = False
            tvec = None
            rvec = None
            pass

        return (self.flagFindMainAruco, ids, rvec, tvec)


class HandEyeCalibration:

    def __init__(self):
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

    # deperecated
    # Ref) https://stackoverflow.com/questions/27546081/determining-a-homogeneous-affine-transformation-matrix-from-six-points-in-3d-usi
    #      https://math.stackexchange.com/questions/222113/given-3-points-of-a-rigid-body-in-space-how-do-i-find-the-corresponding-orienta/222170#222170

    def calculateTransformMatrixUsing3Points(self, p, p_prime):
        # construct intermediate matrix
        Q = p[1:] - p[0]
        Q_prime = p_prime[1:] - p_prime[0]

        # calculate rotation matrix
        R = np.dot(np.linalg.inv(np.row_stack((Q, np.cross(*Q)))),
                   np.row_stack((Q_prime, np.cross(*Q_prime))))

        # calculate translation vector
        t = p_prime[0] - np.dot(p[0], R)

        # calculate affine transformation matrix
        return np.column_stack((np.row_stack((R, t)),
                                (0, 0, 0, 1)))

    # deprecated...
    def calculateTransformMatrix(self, srcPoints, dstPoints):
        assert(len(srcPoints) == len(dstPoints))

        p = np.ones([len(srcPoints), 4])
        p_prime = np.ones([len(dstPoints), 4])
        for idx in range(len(srcPoints)):
            p[idx][0] = srcPoints[idx][0]
            p[idx][1] = srcPoints[idx][1]
            p[idx][2] = srcPoints[idx][2]

            p_prime[idx][0] = dstPoints[idx][0]
            p_prime[idx][1] = dstPoints[idx][1]
            p_prime[idx][2] = dstPoints[idx][2]

        trMatrix = cv2.solve(p, p_prime, flags=cv2.DECOMP_SVD)
        return trMatrix

    # handeye calibration test function
    def calibrateHandEyeTest(self, HMBase2TCPs, HMTarget2Cams):
        #assert (HMBase2TCPs.len() == HMTarget2Cams.len())
        for hmmat in HMBase2TCPs:
            rotataion = hmmat[0:3, 0:3]
            self.R_gripper2base.append(rotataion)
            translation = hmmat[0:3, 3]
            self.t_gripper2base.append(translation)

        for hmmat in HMTarget2Cams:
            rotataion = hmmat[0:3, 0:3]
            self.R_target2cam.append(rotataion)
            translation = hmmat[0:3, 3]
            self.t_target2cam.append(translation)

        methodHE = [cv2.CALIB_HAND_EYE_TSAI, cv2.CALIB_HAND_EYE_PARK,
                    cv2.CALIB_HAND_EYE_HORAUD, cv2.CALIB_HAND_EYE_ANDREFF, cv2.CALIB_HAND_EYE_DANIILIDIS]

        for mth in methodHE:
            self.R_cam2gripper, self.t_cam2gripper = cv2.calibrateHandEye(
                self.R_gripper2base, self.t_gripper2base, self.R_target2cam, self.t_target2cam, None, None, mth)
            cv2.calibrateHandEye(self.R_gripper2base, self.t_gripper2base,
                                 self.R_target2cam, self.t_target2cam, None, None, mth)
            # output results
            # PrintMsg.printStdErr("--------------------------------------")
            # PrintMsg.printStdErr("Method %d" % mth)
            # PrintMsg.printStdErr(self.R_cam2gripper)
            # PrintMsg.printStdErr(self.t_cam2gripper)
            # PrintMsg.printStdErr("--------------------------------------")

    def captureHandEyeInputs(self, robotXYZABC, camRVec, camTVec):
        # prepare Gripper2Base inputs
        hmRobot = HMUtil.convertXYZABCtoHMDeg(robotXYZABC)
        self.R_gripper2base.append(hmRobot[0:3, 0:3])
        self.t_gripper2base.append(hmRobot[0:3, 3])

        # prepare Target2Cam inputs
        camRMatrix = np.zeros(shape=(3, 3))
        cv2.Rodrigues(camRVec, camRMatrix)
        hmCam = HMUtil.makeHM(camRMatrix, camTVec)
        hmCam = HMUtil.inverseHM(hmCam)
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
                self.R_gripper2base, self.t_gripper2base, self.R_target2cam, self.t_target2cam, None, None, mth)
            # output results
            PrintMsg.printStdErr("--------------------------------------")
            PrintMsg.printStdErr("Method %d" % mth)
            PrintMsg.printStdErr(self.R_cam2gripper)
            PrintMsg.printStdErr(self.t_cam2gripper)
            PrintMsg.printStdErr("--------------------------------------")
            self.distance = math.sqrt(math.pow(self.t_cam2gripper[0], 2.0)+math.pow(
                self.t_cam2gripper[1], 2.0)+math.pow(self.t_cam2gripper[2], 2.0))
            PrintMsg.printStdErr("Distance: %f" % self.distance)
            PrintMsg.printStdErr("--------------------------------------")

        # verify handeye calculation results
        hmTemp = HMUtil.makeHM(self.R_cam2gripper, self.t_cam2gripper.T)

        xyzabc_temp = HMUtil.convertHMtoXYZABCDeg(hmTemp)
        # print(xyzabc_temp)

    def getPredefinedHandeye(self):

        ############################################################
        # TODO: get predefined data from server
        prdefined_matrix = [0.000, 0.102, 0.0000, 180, 0, 180]
        # prdefined_matrix = [0.00165360882730919, 0.10332931833889342,
        #                     0.001752356149939573, -179.52921594383568, 0.24227911479934125, 179.77335961667296]

        self.predefined_hm = HMUtil.convertXYZABCtoHMDeg(prdefined_matrix)

        return self.predefined_hm

    def getHandEyeResultMatrixUsingOpenCV(self):

        if(self.AlgorithmTest == True):
            fsHandEyeTest = cv2.FileStorage(
                "HandEyeResultsLog.xml", cv2.FILE_STORAGE_WRITE)

        self.calculateHandEyeMatrix()

        if(self.AlgorithmTest == True):
            fsHandEyeTest.write("Method", math.sqrt(math.pow(self.t_cam2gripper[0], 2.0)+math.pow(
                self.t_cam2gripper[1], 2.0)+math.pow(self.t_cam2gripper[2], 2.0)))

        # select HORAUD algorithm on temporary
        # make a homogeneous matrix from Target(Calibration) to Gripper(TCP)
        hmT2G = HMUtil.makeHM(self.R_cam2gripper, self.t_cam2gripper.T)
        # make a homogeneous matrix from Gripper(TCP) to Robot Base
        hmG2B = HMUtil.makeHM(
            self.R_gripper2base[0], self.t_gripper2base[0].reshape(1, 3))
        # make a homogeneous matrix from Camera to Target(Target)
        hmC2T = HMUtil.makeHM(
            self.R_target2cam[0], self.t_target2cam[0].reshape(1, 3))

        # Final HM(Camera to Robot Base)
        # H(C2B) = H(G2B)H(T2G)H(C2T)
        hmResultTransform = np.dot(hmG2B, hmT2G)
        hmResultTransform = np.dot(hmResultTransform, hmC2T)

        if(self.AlgorithmTest == True):
            fsHandEyeTest.release()

        PrintMsg.printStdErr("Result Transform: ")
        PrintMsg.printStdErr(hmResultTransform)
        return hmResultTransform

    def getHandEyeMatUsingMarker(self, robotXYZABC, camRVec, camTVec):
        # get the predefined handeye matrix
        hmPredefined = self.getPredefinedHandeye()

        # prepare Gripper2Base inputs
        hmG2B = HMUtil.convertXYZABCtoHMDeg(robotXYZABC)

        # prepare Target2Cam inputs
        camRMatrix = np.zeros(shape=(3, 3))
        cv2.Rodrigues(camRVec, camRMatrix)
        hmT2C = HMUtil.makeHM(camRMatrix, camTVec)
        hmC2T = HMUtil.inverseHM(hmT2C)

        xyzuvwC2T = HMUtil.convertHMtoXYZABCDeg(hmC2T)
        #print('camera position: ', xyzuvwC2T)

        # Final HM(Camera to Robot Base)
        # H(C2B) = H(G2B)H(T2G)H(C2T)
        hmFinal = np.dot(hmG2B, hmPredefined)
        hmFinal = np.dot(hmFinal, hmC2T)

        return hmFinal

    # save a transform matrix to xml data as a file
    @staticmethod
    def saveTransformMatrix(resultMatrix):
        calibFile = cv2.FileStorage(
            "HandEyeCalibResult.json", cv2.FILE_STORAGE_WRITE)
        calibFile.write("HEMatrix", resultMatrix)
        calibFile.release()

    # get a transformation matrix which was created by calibration process
    @staticmethod
    def loadTransformMatrix():
        calibFile = cv2.FileStorage(
            "HandEyeCalibResult.json", cv2.FILE_STORAGE_READ)
        hmnode = calibFile.getNode("HEMatrix")
        hmmtx = hmnode.mat()
        return hmmtx


###############################################################################
# Test Codes using sample data in yml format
###############################################################################
if __name__ == '__main__':

    YMLHMBase2TCPs = []
    YMLHMTarget2Cams = []

    cam3DTestPoints = []
    robot3DTestPoints = []

    # handEyeInput = cv2.FileStorage("./handEyeSample.yml", cv2.FILE_STORAGE_READ)
    # fileNode = handEyeInput.root()

    handeye = HandEyeCalibration()

    # for key in fileNode.keys():
    #     ymlnode = handEyeInput.getNode(key)
    #     ymlmtx = ymlnode.mat()

    #     if key.find("target2cam") >= 0:
    #         YMLHMTarget2Cams.append(ymlmtx)

    #     if key.find("gripper2base") >= 0:
    #         ymlmtx = HMUtil.inverseHM(ymlmtx)
    #         YMLHMBase2TCPs.append(ymlmtx)

    # handeye.calibrateHandEyeTest(YMLHMBase2TCPs, YMLHMTarget2Cams)

    # # calculateHM Test
    cam3DTestPoints.append([-0.10259, 0.07283, 0.40900])
    cam3DTestPoints.append([0.14604, 0.00431, 0.42700])
    cam3DTestPoints.append([-0.00145, 0.10705, 0.31100])
    cam3DTestPoints.append([-0.10259, 0.07283, 0.40900])
    cam3DTestPoints.append([0.14604, 0.00431, 0.42700])
    cam3DTestPoints.append([-0.00145, 0.10705, 0.31100])

    robot3DTestPoints.append([-0.18101, -0.52507, 0.01393])
    robot3DTestPoints.append([0.06137, -0.68306, 0.01546])
    robot3DTestPoints.append([-0.18807, -0.66342, 0.01510])
    robot3DTestPoints.append([-0.18101, -0.52507, 0.01393])
    robot3DTestPoints.append([0.06137, -0.68306, 0.01546])
    robot3DTestPoints.append([-0.18807, -0.66342, 0.01510])

    result = handeye.calculateTransformMatrixUsing3Points(np.array(((-0.10259, 0.07283, 0.40900), (0.14604, 0.00431, 0.42700), (-0.00145, 0.10705, 0.31100))),
                                                          np.array(
                                                              ((-0.18101, -0.52507, 0.01393), (0.06137, -0.68306, 0.01546), (-0.18807, -0.66342, 0.01510)))
                                                          )
    PrintMsg.printStdErr(result)
    PrintMsg.printStdErr(result.shape)

    camC = np.array(
        ((cam3DTestPoints[0]), (cam3DTestPoints[1]), (cam3DTestPoints[2])))
    PrintMsg.printStdErr(camC.shape)
    robotC = np.array(
        ((robot3DTestPoints[0]), (robot3DTestPoints[1]), (robot3DTestPoints[2])))
    result = handeye.calculateTransformMatrixUsing3Points(camC, robotC)
    PrintMsg.printStdErr(result)

    result = handeye.calculateTransformMatrix(
        cam3DTestPoints, robot3DTestPoints)
    PrintMsg.printStdErr(result)

    PrintMsg.printStdErr(
        np.dot(np.array([-0.10259, 0.07283, 0.40900, 1]).reshape(1, 4), result[1]))
