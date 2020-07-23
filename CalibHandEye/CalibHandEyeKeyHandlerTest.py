import cv2

# add src root directory to python path
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) )

import Config
from packages.KeyHandler import KeyHandler
from packages.RobotIndy7Dev import RobotIndy7Dev
from packages.Util import PrintMsg
from HandEyeUtilSet import *
from HandEye import *
from CalibHandEyeUpdate import CalibHandeyeUpdate

class CalibHandEyeKeyHandler(KeyHandler):

    def __init__(self):
        super().__init__()
        super().setKeyHandler('q', self.processQ)
        super().setKeyHandler('d', self.processD)
        super().setKeyHandler('f', self.processF)
        super().setKeyHandler('r', self.processR)        
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
        PrintMsg.printStdErr("direct teaching mode: On")
        indy.setDirectTeachingMode(True)

    def processF(self, *args):
        indy = args[8]
        # set direct-teaching mode off
        PrintMsg.printStdErr("direct teaching mode: Off")
        indy.setDirectTeachingMode(False)  

    def processR(self, *args):
        indy = args[8]
        indy.resetRobot()
        PrintMsg.printStdErr("resetting the robot")

    def processC(self, *args):
        colorImage = args[1]
        tvec = args[3]
        rvec = args[4]
        ids = args[2]
        handeye = args[7]
        indy = args[8]
        infoText = args[9]

        PrintMsg.printStdErr("---------------------------------------------------------------")
        if ids is None:
            return
        for idx in range(0, ids.size):
            if(ids[idx] == Config.CalibMarkerID):
                # get the current robot position
                currTaskPose = indy.getCurrentPos()

                # capture additional matrices here
                handeye.captureHandEyeInputs(currTaskPose, rvec[idx], tvec[idx])
                PrintMsg.printStdErr("Input Data Count: " + str(handeye.cntInputData))
                strText = "Input Data Count: " + str(handeye.cntInputData)
                infoText.setText(strText)

    def processZ(self, *args):
        handeye = args[7]
        handeye.resetHandEyeInputs()

    def processG(self, *args):
        handeye = args[7]
        infoText = args[9]

        if handeye.cntInputData < 3:
            return

        PrintMsg.printStdErr("---------------------------------------------------------------")
        hmTransform = handeye.getHandEyeResultMatrixUsingOpenCV()
        PrintMsg.printStdErr("Transform Matrix = ")
        PrintMsg.printStdErr(hmTransform)
        HandEyeCalibration.saveTransformMatrix(hmTransform)
        
        updateUI = CalibHandeyeUpdate()
        updateUI.updateData(hmTransform.reshape(1, 16)[0])      # TODO: [0] is available??

        infoText.setText('Succeeded to extract a handeye matrix.')

    def processN(self, *args):
        tvec = args[3]
        rvec = args[4]
        ids = args[2]
        handeye = args[7]
        indy = args[8]
        infoText = args[9]

        PrintMsg.printStdErr("---------------------------------------------------------------")
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
                hmWanted = HMUtil.makeHM(np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]]), np.array([-0.01, 0.005, Config.HandEyeTargetZ]).T)
                #hmWanted = HMUtil.makeHM(np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]]), np.array([0.08, 0.0, 0.0]).T)
                #hmWanted = HMUtil.makeHM(np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]]), np.array([0.0, 0.0, 0.0]).T)
                #hmWanted = HMUtil.makeHM(np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]]), np.array([0.0, 0.0, 0.0]).T)

                hmInput = np.dot(hmCal2Cam, hmWanted)

                # get the last homogeneous matrix
                hmResult = np.dot(hmmtx, hmInput)

                # get a final xyzuvw for the last homogenous matrix
                xyzuvw = HMUtil.convertHMtoXYZABCDeg(hmResult)
                PrintMsg.printStdErr("Final XYZUVW: ")
                PrintMsg.printStdErr(xyzuvw)

                ############################################################################################
                # test move to the destination
                [x,y,z,u,v,w] = xyzuvw

                # indy7 base position to gripper position
                xyzuvw = [x,y,z,u*(-1),v+180.0,w] 
                PrintMsg.printStdErr("Modifed TCP XYZUVW: ")
                PrintMsg.printStdErr(xyzuvw)
                indy.moveTaskPos(xyzuvw)

                # # get a HM from TCP to Base
                # hmRecal = HMUtil.convertXYZABCtoHMDeg(xyzuvw)
                # # 
                # hmWanted2 = HMUtil.makeHM(np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]]), np.array([0.0, 0.0, -0.3]).T)
                # hmResult2 = np.dot(hmRecal, hmWanted2)
                # xyzuvw2 = HMUtil.convertHMtoXYZABCDeg(hmResult2)
                # PrintMsg.printStdErr("Recalculated XYZUVW: ")
                # PrintMsg.printStdErr(xyzuvw2)     
                # #indy.moveTaskPos(xyzuvw2)




