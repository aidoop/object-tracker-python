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
        # super().setKeyHandler('d', self.processD)
        # super().setKeyHandler('f', self.processF)
        # super().setKeyHandler('r', self.processR)        
        super().setKeyHandler('c', self.processC)
        super().setKeyHandler('z', self.processZ)
        super().setKeyHandler('g', self.processG)

        # option: task move test..
        # super().setKeyHandler('n', self.processN)

        self.interation = 0

    def processQ(self, *args):
        super().enableExitFlag()

    # def processD(self, *args):
    #     indy = args[8]
    #     # set direct-teaching mode on
    #     PrintMsg.printStdErr("direct teaching mode: On")
    #     indy.setDirectTeachingMode(True)

    # def processF(self, *args):
    #     indy = args[8]
    #     # set direct-teaching mode off
    #     PrintMsg.printStdErr("direct teaching mode: Off")
    #     indy.setDirectTeachingMode(False)  

    # def processR(self, *args):
    #     indy = args[8]
    #     indy.resetRobot()
    #     PrintMsg.printStdErr("resetting the robot")

    def processC(self, *args):
        colorImage = args[1]
        tvec = args[3]
        rvec = args[4]
        ids = args[2]
        handeye = args[7]
        infoText = args[8]
        gqlDataClient = args[9]
        robotName = args[10]

        PrintMsg.printStdErr("---------------------------------------------------------------")
        if ids is None:
            return
        for idx in range(0, ids.size):
            if(ids[idx] == Config.CalibMarkerID):
                # get the current robot position
                # currTaskPose = indy.getCurrentPos()
                currTaskPose = gqlDataClient.getRobotPose(robotName)

                # convert dict. to list
                currTPList = [currTaskPose['x'], currTaskPose['y'], currTaskPose['z'], currTaskPose['u'], currTaskPose['v'], currTaskPose['w']]

                # capture additional matrices here
                handeye.captureHandEyeInputs(currTPList, rvec[idx], tvec[idx])

                if handeye.cntInputData >= 3:
                    handeye.calculateHandEyeMatrix()

                PrintMsg.printStdErr("Input Data Count: " + str(handeye.cntInputData))
                strText = "Input Data Count: " + str(handeye.cntInputData) +"(" + str(handeye.distance) + ")"
                infoText.setText(strText)

    def processZ(self, *args):
        handeye = args[7]
        handeye.resetHandEyeInputs()

    def processG(self, *args):
        handeye = args[7]
        infoText = args[8]

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





