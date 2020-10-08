import cv2

import config
from util.keyhandler import KeyHandler
from robot.robot_dev_indydcp import RobotIndy7Dev
from util.util import PrintMsg
from util.hm_util import *
from calibhandeye_handeye import *
from data_update.calibhandeye_update import CalibHandeyeUpdate


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

        # new api test
        super().setKeyHandler('l', self.processL)

        # option: task move test..
        super().setKeyHandler('n', self.processN)
        super().setKeyHandler('m', self.processM)
        # super().setKeyHandler('b', self.processB)

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
        findAruco = args[0]
        colorImage = args[1]
        tvec = args[3]
        rvec = args[4]
        ids = args[2]
        handeye = args[7]
        indy = args[8]
        infoText = args[9]

        PrintMsg.printStdErr(
            "---------------------------------------------------------------")
        if ids is None:
            return

        if config.UseArucoBoard == False:
            for idx in range(0, ids.size):
                if(ids[idx] == config.CalibMarkerID):
                    # get the current robot position
                    currTaskPose = indy.getCurrentPos()

                    # capture additional matrices here
                    handeye.captureHandEyeInputs(
                        currTaskPose, rvec[idx], tvec[idx])

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
                currTaskPose = indy.getCurrentPos()

                # capture additional matrices here
                handeye.captureHandEyeInputs(currTaskPose, rvec.T, tvec.T)

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

    def processG(self, *args):
        handeye = args[7]
        infoText = args[9]

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

    def processN(self, *args):
        tvec = args[3]
        rvec = args[4]
        ids = args[2]
        handeye = args[7]
        indy = args[8]
        infoText = args[9]

        PrintMsg.printStdErr(
            "---------------------------------------------------------------")
        for idx in range(0, ids.size):
            if ids[idx] == config.TestMarkerID:
                # change a rotation vector to a rotation matrix
                rotMatrix = np.zeros(shape=(3, 3))
                cv2.Rodrigues(rvec[idx], rotMatrix)

                # make a homogeneous matrix using a rotation matrix and a translation matrix a
                hmCal2Cam = HMUtil.makeHM(rotMatrix, tvec[idx])

                # get a transformation matrix which was created by calibration process
                hmmtx = HandEyeCalibration.loadTransformMatrix()

                # calcaluate the specific position based on hmInput
                hmWanted = HMUtil.makeHM(np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [
                                         0.0, 0.0, 1.0]]), np.array([0.0, 0.0, config.HandEyeTargetZ]).T)
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
                [x, y, z, u, v, w] = xyzuvw

                # indy7 base position to gripper position
                xyzuvw = [x, y, z, u*(-1), v+180.0, w]  # test w+180
                PrintMsg.printStdErr("Modifed TCP XYZUVW: ")
                PrintMsg.printStdErr(xyzuvw)
                # indy.moveTaskPos(xyzuvw)

                curjpos = indy.getCurrentJointPos()
                nextMoveAvail = indy.checkNextMove(xyzuvw, curjpos)
                print('nextMoveAvail: ', nextMoveAvail)
                print('currentTaskPos: ', indy.getCurrentPos())
                print('nextTaskMove: ', xyzuvw)

                if(nextMoveAvail != [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
                    indy.moveTaskPos(xyzuvw)
                else:
                    xyzuvw = [x, y, z, u*(-1), v+180.0, w+180]
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

    def processM(self, *args):
        tvec = args[3]
        rvec = args[4]
        ids = args[2]
        handeye = args[7]
        indy = args[8]
        infoText = args[9]

        PrintMsg.printStdErr(
            "---------------------------------------------------------------")
        for idx in range(0, ids.size):
            if ids[idx] == config.TestMarkerID:
                # change a rotation vector to a rotation matrix
                rotMatrix = np.zeros(shape=(3, 3))
                cv2.Rodrigues(rvec[idx], rotMatrix)

                # make a homogeneous matrix using a rotation matrix and a translation matrix a
                hmCal2Cam = HMUtil.makeHM(rotMatrix, tvec[idx])

                # get a transformation matrix which was created by calibration process
                hmmtx = HandEyeCalibration.loadTransformMatrix()

                # calcaluate the specific position based on hmInput
                hmWanted = HMUtil.makeHM(np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [
                                         0.0, 0.0, 1.0]]), np.array([0.0, 0.0, config.HandEyeTargetZ]).T)
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
                [x, y, z, u, v, w] = xyzuvw

                # indy7 base position to gripper position
                xyzuvw = [x, y, z, u*(-1), v+180.0, w+180]  # test w+180
                PrintMsg.printStdErr("Modifed TCP XYZUVW: ")
                PrintMsg.printStdErr(xyzuvw)
                # indy.moveTaskPos(xyzuvw)

                curjpos = indy.getCurrentJointPos()
                nextMoveAvail = indy.checkNextMove(xyzuvw, curjpos)
                print('nextMoveAvail: ', nextMoveAvail)

                print('nextMoveAvail: ', nextMoveAvail)
                print('currentTaskPos: ', indy.getCurrentPos())
                print('nextTaskMove: ', xyzuvw)

                if(nextMoveAvail != [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
                    indy.moveTaskPos(xyzuvw)
                else:
                    xyzuvw = [x, y, z, u*(-1), v+180.0, w-180]
                    indy.moveTaskPos(xyzuvw)

                # xyzuvw = [x,y,z,u*(-1),v+180.0,w+180] # test w+180
                # indy.moveTaskPos(xyzuvw)

                # # get a HM from TCP to Base
                # hmRecal = HMUtil.convertXYZABCtoHMDeg(xyzuvw)
                # #
                # hmWanted2 = HMUtil.makeHM(np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]]), np.array([0.0, 0.0, -0.3]).T)
                # hmResult2 = np.dot(hmRecal, hmWanted2)
                # xyzuvw2 = HMUtil.convertHMtoXYZABCDeg(hmResult2)
                # PrintMsg.printStdErr("Recalculated XYZUVW: ")
                # PrintMsg.printStdErr(xyzuvw2)
                # #indy.moveTaskPos(xyzuvw2)

    def processL(self, *args):
        tvec = args[3]
        rvec = args[4]
        ids = args[2]
        handeye = args[7]
        indy = args[8]
        infoText = args[9]

        PrintMsg.printStdErr(
            "---------------------------------------------------------------")
        for idx in range(0, ids.size):
            if ids[idx] == config.TestMarkerID:
                # change a rotation vector to a rotation matrix
                rotMatrix = np.zeros(shape=(3, 3))
                cv2.Rodrigues(rvec[idx], rotMatrix)

                # make a homogeneous matrix using a rotation matrix and a translation matrix a
                hmCal2Cam = HMUtil.makeHM(rotMatrix, tvec[idx])

                # get a transformation matrix which was created by calibration process
                hmmtx = HandEyeCalibration.loadTransformMatrix()

                # calcaluate the specific position based on hmInput
                hmWanted = HMUtil.makeHM(np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [
                                         0.0, 0.0, 1.0]]), np.array([-0.01, 0.005, config.HandEyeTargetZ]).T)
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
                [x, y, z, u, v, w] = xyzuvw

                # indy7 base position to gripper position
                xyzuvw = [x, y, z, u*(-1), v+180.0, w]
                PrintMsg.printStdErr("Modifed TCP XYZUVW: ")
                PrintMsg.printStdErr(xyzuvw)

                curjpos = indy.getCurrentJointPos()
                nextMoveAvail = indy.checkNextMove(xyzuvw, curjpos)

                print("------------------------------\n")
                print("Curr: ", indy.getCurrentPos())

                [xx, yy, zz, uu, vv, ww] = xyzuvw
                if(ww < 0):
                    ww = 180.0 + ww

                xyzuvw = [xx, yy, zz, uu, vv, ww]

                print("Next: ", xyzuvw)
                print(nextMoveAvail)

                indy.moveTaskPos(xyzuvw)

                # if(nextMoveAvail != [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
                #     indy.moveTaskPos(xyzuvw)
                # else:
                #     [xc, yc, zc, uc, vc, wc] = indy.getCurrentPos()
                #     [xx,yy,zz,uu,vv,ww] = xyzuvw

                #     if( (ww - wc) >= 0 ):
                #         ww = (ww-wc) - 360
                #     else:
                #         ww = (ww-wc) + 360

                #     xyzuvw = [xx,yy,zz,uu,vv,ww]
                #     print("target: ", xyzuvw)
                #     indy.moveTaskPos(xyzuvw)

                #print("Can't move next postion" + "\n")

                # # get a HM from TCP to Base
                # hmRecal = HMUtil.convertXYZABCtoHMDeg(xyzuvw)
                # #
                # hmWanted2 = HMUtil.makeHM(np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]]), np.array([0.0, 0.0, -0.3]).T)
                # hmResult2 = np.dot(hmRecal, hmWanted2)
                # xyzuvw2 = HMUtil.convertHMtoXYZABCDeg(hmResult2)
                # PrintMsg.printStdErr("Recalculated XYZUVW: ")
                # PrintMsg.printStdErr(xyzuvw2)
                # #indy.moveTaskPos(xyzuvw2)
