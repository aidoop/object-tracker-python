import cv2

from applications.config.appconfig import AppConfig
from applications.keyhandler.keyhandlerdev import KeyHandler
from applications.etc.util import PrintMsg
from pyaidoop.etc.hm_util import *
from pyaidoop.calibration.calibhandeye_handeye import *
from applications.data_update.calibhandeye_update import CalibHandeyeUpdate
from applications.bridge.bridge_interprocess import BridgeInterprocess


class CalibHandEyeKeyHandler(KeyHandler):
    def __init__(self):
        super().__init__()
        super().setKeyHandler("q", self.processQ)
        # super().setKeyHandler('d', self.processD)
        # super().setKeyHandler('f', self.processF)
        # super().setKeyHandler('r', self.processR)
        super().setKeyHandler("c", self.processC)
        super().setKeyHandler("z", self.processZ)
        super().setKeyHandler("g", self.processG)
        # super().setKeyHandler('r', self.processR)
        super().setKeyHandler("a", self.processA)
        super().setKeyHandler("s", self.processS)

        # option: task move test..
        # super().setKeyHandler('n', self.processN)

        self.interation = 0

    def processQ(self, *args):
        super().enableExitFlag()

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

        PrintMsg.print_error(
            "---------------------------------------------------------------"
        )
        if ids is None:
            return

        if AppConfig.UseArucoBoard == False:
            for idx in range(0, ids.size):
                if ids[idx] == AppConfig.CalibMarkerID:
                    # get the current robot position
                    # currTaskPose = indy.get_task_pos()
                    currTaskPose = gqlDataClient.get_robot_pose(robotName)

                    # convert dict. to list
                    currTPList = [
                        currTaskPose["x"],
                        currTaskPose["y"],
                        currTaskPose["z"],
                        currTaskPose["u"],
                        currTaskPose["v"],
                        currTaskPose["w"],
                    ]

                    # capture additional matrices here
                    handeye.captureHandEyeInputs(currTPList, rvec[idx], tvec[idx])

                    if handeye.cntInputData >= 3:
                        handeye.calculateHandEyeMatrix()

                    PrintMsg.print_error(
                        "Input Data Count: " + str(handeye.cntInputData)
                    )
                    strText = (
                        "Input Data Count: "
                        + str(handeye.cntInputData)
                        + "("
                        + str(handeye.distance)
                        + ")"
                    )
                    infoText.set_info_text(strText)
        else:
            if findAruco is True:
                # get the current robot position
                # currTaskPose = indy.get_task_pos()
                currTaskPose = gqlDataClient.get_robot_pose(robotName)

                # convert dict. to list
                currTPList = [
                    currTaskPose["x"],
                    currTaskPose["y"],
                    currTaskPose["z"],
                    currTaskPose["u"],
                    currTaskPose["v"],
                    currTaskPose["w"],
                ]

                # capture additional matrices here
                handeye.captureHandEyeInputs(currTPList, rvec.T, tvec.T)

                if handeye.cntInputData >= 3:
                    handeye.calculateHandEyeMatrix()

                PrintMsg.print_error(f"({str(handeye.cntInputData)}")
                strText = f"{handeye.cntInputData} ({handeye.distance})"
                infoText.set_info_text(strText)

    # def processR(self, *args):
    #     findAruco = args[0]
    #     colorImage = args[1]
    #     tvec = args[3]
    #     rvec = args[4]
    #     ids = args[2]
    #     handeye = args[7]
    #     infoText = args[8]
    #     gqlDataClient = args[9]
    #     robotName = args[10]
    #     indy = args[12]

    #     PrintMsg.print_error(
    #         "---------------------------------------------------------------")
    #     if ids is None:
    #         return

    #     if AppConfig.UseArucoBoard == False:
    #         for idx in range(0, ids.size):
    #             if(ids[idx] == AppConfig.CalibMarkerID):
    #                 # get the current robot position
    #                 currTPList = indy.get_task_pos()
    #                 # currTaskPose = gqlDataClient.get_robot_pose(robotName)

    #                 # # convert dict. to list
    #                 # currTPList = [currTaskPose['x'], currTaskPose['y'], currTaskPose['z'],
    #                 #               currTaskPose['u'], currTaskPose['v'], currTaskPose['w']]

    #                 # capture additional matrices here
    #                 handeye.captureHandEyeInputs(
    #                     currTPList, rvec[idx], tvec[idx])

    #                 if handeye.cntInputData >= 3:
    #                     handeye.calculateHandEyeMatrix()

    #                 PrintMsg.print_error(
    #                     "Input Data Count: " + str(handeye.cntInputData))
    #                 strText = "Input Data Count: " + \
    #                     str(handeye.cntInputData) + \
    #                     "(" + str(handeye.distance) + ")"
    #                 infoText.set_info_text(strText)
    #     else:
    #         if findAruco is True:
    #             # get the current robot position
    #             currTPList = indy.get_task_pos()
    #             # currTaskPose = gqlDataClient.get_robot_pose(robotName)

    #             # # convert dict. to list
    #             # currTPList = [currTaskPose['x'], currTaskPose['y'], currTaskPose['z'],
    #             #               currTaskPose['u'], currTaskPose['v'], currTaskPose['w']]

    #             # capture additional matrices here
    #             handeye.captureHandEyeInputs(currTPList, rvec.T, tvec.T)

    #             if handeye.cntInputData >= 3:
    #                 handeye.calculateHandEyeMatrix()

    #             PrintMsg.print_error(
    #                 "Input Data Count: " + str(handeye.cntInputData))
    #             strText = "Input Data Count: " + \
    #                 str(handeye.cntInputData) + \
    #                 "(" + str(handeye.distance) + ")"
    #             infoText.set_info_text(strText)

    def processZ(self, *args):
        handeye = args[7]
        handeye.resetHandEyeInputs()
        handeye.distance = 0.0

    def processG(self, *args):
        handeye = args[7]
        infoText = args[8]
        bridge_ip = args[12]
        cameraName = args[13]

        if handeye.cntInputData < 3:
            if interproc_dict is not None:
                super().enableExitFlag()
            return

        PrintMsg.print_error(
            "---------------------------------------------------------------"
        )
        hmTransform = handeye.getHandEyeResultMatrixUsingOpenCV()
        # PrintMsg.print_error("Transform Matrix = ")
        # PrintMsg.print_error(hmTransform)
        # HandEyeCalibration.saveTransformMatrix(hmTransform)

        updateUI = CalibHandeyeUpdate()
        # TODO: [0] is available??
        update_data = updateUI.update_result(hmTransform.reshape(1, 16)[0])

        infoText.set_info_text("Succeeded to extract a handeye matrix.")

        # get the result data and throw into the websocket process
        bridge_ip.send_dict_data(
            "object",
            {
                "name": "handeyecalib:" + cameraName,
                "objectData": update_data,
            },
        )

    # automated handeye calibration

    def processA(self, *args):
        handeye_automove = args[11]
        if handeye_automove.isInitialized():
            handeye_automove.start()

    def processS(self, *args):
        handeye_automove = args[11]
        if handeye_automove.isInitialized():
            handeye_automove.stop()
