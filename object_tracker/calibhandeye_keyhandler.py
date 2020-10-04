import cv2

import config
from util.keyhandler import KeyHandler
from robot.robot_dev_indydcp import RobotIndy7Dev
from util.util import PrintMsg
from util.hm_util import *
from calibhandeye_handeye import *
from calibhandeye_update import CalibHandeyeUpdate


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
        findAruco = args[0]
        colorImage = args[1]
        tvec = args[3]
        rvec = args[4]
        ids = args[2]
        handeye = args[7]
        infoText = args[8]
        gqlDataClient = args[9]
        robotName = args[10]

        PrintMsg.printStdErr(
            "---------------------------------------------------------------")
        if ids is None:
            return

        if config.UseArucoBoard == False:
            for idx in range(0, ids.size):
                if(ids[idx] == config.CalibMarkerID):
                    # get the current robot position
                    # currTaskPose = indy.getCurrentPos()
                    currTaskPose = gqlDataClient.getRobotPose(robotName)

                    # convert dict. to list
                    currTPList = [currTaskPose['x'], currTaskPose['y'], currTaskPose['z'],
                                  currTaskPose['u'], currTaskPose['v'], currTaskPose['w']]

                    # capture additional matrices here
                    handeye.captureHandEyeInputs(
                        currTPList, rvec[idx], tvec[idx])

                    if handeye.cntInputData >= 3:
                        handeye.calculateHandEyeMatrix()

                    PrintMsg.printStdErr(
                        "Input Data Count: " + str(handeye.cntInputData))
                    strText = "Input Data Count: " + \
                        str(handeye.cntInputData) + \
                        "(" + str(handeye.distance) + ")"
                    infoText.setText(strText)
        else:
            if findAruco is True:
                # get the current robot position
                # currTaskPose = indy.getCurrentPos()
                currTaskPose = gqlDataClient.getRobotPose(robotName)

                # convert dict. to list
                currTPList = [currTaskPose['x'], currTaskPose['y'], currTaskPose['z'],
                              currTaskPose['u'], currTaskPose['v'], currTaskPose['w']]

                # capture additional matrices here
                handeye.captureHandEyeInputs(currTPList, rvec.T, tvec.T)

                if handeye.cntInputData >= 3:
                    handeye.calculateHandEyeMatrix()

                PrintMsg.printStdErr(
                    "Input Data Count: " + str(handeye.cntInputData))
                strText = "Input Data Count: " + \
                    str(handeye.cntInputData) + \
                    "(" + str(handeye.distance) + ")"
                infoText.setText(strText)

    def processZ(self, *args):
        handeye = args[7]
        handeye.resetHandEyeInputs()
        handeye.distance = 0.0

    def processG(self, *args):
        handeye = args[7]
        infoText = args[8]

        if handeye.cntInputData < 3:
            return

        PrintMsg.printStdErr(
            "---------------------------------------------------------------")
        hmTransform = handeye.getHandEyeResultMatrixUsingOpenCV()
        PrintMsg.printStdErr("Transform Matrix = ")
        PrintMsg.printStdErr(hmTransform)
        HandEyeCalibration.saveTransformMatrix(hmTransform)

        updateUI = CalibHandeyeUpdate()
        # TODO: [0] is available??
        updateUI.updateData(hmTransform.reshape(1, 16)[0])

        infoText.setText('Succeeded to extract a handeye matrix.')
