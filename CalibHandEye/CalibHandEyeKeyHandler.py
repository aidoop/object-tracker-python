import cv2
import os
import glob

import Config
from packages.KeyHandler import KeyHandler
from HandEyeUtilSet import *
from packages.RobotIndy7Dev import RobotIndy7Dev

class CalibHandEyeKeyHandler(KeyHandler):

    def __init__(self):
        super().__init__()
        super().setKeyHandler('q', self.processQ)
        super().setKeyHandler('d', self.processD)
        super().setKeyHandler('f', self.processF)
        super().setKeyHandler('c', self.processC)
        super().setKeyHandler('z', self.processZ)
        super().setKeyHandler('g', self.processG)

        # option: task move test..
        super().setKeyHandler('n', self.processN)

        self.interation = 0

    def processQ(self, *args):
        super().enableExitFlag()

    def processD(self, *args):
        indy = args[8]
        # set direct-teaching mode on
        print("direct teaching mode: On")
        indy.setDirectTeachingMode(True)

    def processF(self, *args):
        indy = args[8]
        # set direct-teaching mode off
        print("direct teaching mode: Off")
        indy.setDirectTeachingMode(False)        

    def processC(self, *args):
        tvec = args[3]
        rvec = args[4]
        ids = args[2]
        handeye = args[7]
        indy = args[8]
        print("---------------------------------------------------------------")
        for idx in range(0, ids.size):
            if(ids[idx] == Config.CalibMarkerID):
                # get the current robot position
                currTaskPose = indy.getCurrentPos()
                # capture additional matrices here
                handeye.captureHandEyeInputs(currTaskPose, rvec[idx], tvec[idx])
                print("Input Data Count: " + str(handeye.cntInputData))

    def processZ(self, *args):
        handeye = args[7]
        handeye.resetHandEyeInputs()

    def processG(self, *args):
        handeye = args[7]
        print("---------------------------------------------------------------")
        hmTransform = handeye.getHandEyeResultMatrixUsingOpenCV()
        print("Transform Matrix = ")
        print(hmTransform)
        HandEyeCalibration.saveTransformMatrix(hmTransform)

    def processN(self, *args):
        tvec = args[3]
        rvec = args[4]
        ids = args[2]
        handeye = args[7]
        indy = args[8]
        print("---------------------------------------------------------------")
        for idx in range(0, ids.size):
            if ids[idx] == Config.TestMarkerID:
                # change a rotation vector to a rotation matrix
                rotMatrix = np.zeros(shape=(3,3))
                cv2.Rodrigues(rvec[idx], rotMatrix)

                # make a homogeneous matrix using a rotation matrix and a translation matrix a
                hmCal2Cam = HMUtil.makeHM(rotMatrix, tvec[idx])

                # get a transformation matrix which was created by calibration process
                hmmtx = HandEyeCalibration.loadTransformMatrix()

                # calcaluate the specific position based on hmInput
                hmWanted = HMUtil.makeHM(np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]]), np.array([0.0, 0.0, Config.HandEyeTargetZ]).T)
                hmInput = np.dot(hmCal2Cam, hmWanted)

                # get the last homogeneous matrix
                hmResult = np.dot(hmmtx, hmInput)

                # get a final xyzuvw for the last homogenous matrix
                xyzuvw = HMUtil.convertHMtoXYZABCDeg(hmResult)
                print("Final XYZUVW: ")
                print(xyzuvw)

                ############################################################################################
                # test move to the destination
                [x,y,z,u,v,w] = xyzuvw

                # indy7 base position to gripper position
                xyzuvw = [x,y,z,u*(-1),v+180.0,w] 
                print("Modifed TCP XYZUVW: ")
                print(xyzuvw)
                indy.moveTaskPos(xyzuvw)
    

    