import cv2
import os
import glob
import sys
import json

from aidobjtrack.data_update.calibcamera_update import CalibCameraUpdate
from aidobjtrack.util.util import PrintMsg
from aidobjtrack.abc.keyhandlerdev import KeyHandler


class CalibCameraKeyHandler(KeyHandler):
    def __init__(self):
        super().__init__()
        super().setKeyHandler("q", self.processQ)
        super().setKeyHandler("c", self.processC)
        super().setKeyHandler("z", self.processZ)
        super().setKeyHandler("g", self.processG)
        self.interation = 0

    def processQ(self, *args):
        super().enableExitFlag()

    def processC(self, *args):
        color_image = args[0]
        dirFrameImage = args[1]
        infoText = args[4]

        cv2.imwrite(
            os.path.join(dirFrameImage, str(self.interation) + ".jpg"), color_image
        )
        PrintMsg.printStdErr(
            "Image caputured - "
            + os.path.join(dirFrameImage, str(self.interation) + ".jpg")
        )
        self.interation += 1

        strInfoText = (
            "Image Captured("
            + str(self.interation)
            + ") - "
            + os.path.join(dirFrameImage, str(self.interation) + ".jpg")
        )
        infoText.setText(strInfoText)

    def processZ(self, *args):
        calibcam = args[2]
        calibcam.clearObjImgPoints()
        self.interation = 0

    def processG(self, *args):

        # cameraParameter = {
        #     "distortionCoefficient": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        #     "cameraMatrix": {
        #         "rows": 3,
        #         "columns": 3,
        #         "data": [0.0, 1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7,
        #                 8.8]
        #     }
        # }
        # print(json.dumps(cameraParameter))

        dirFrameImage = args[1]
        calibcam = args[2]
        camIndex = args[3]
        infoText = args[4]
        cameraName = args[5]
        interproc_dict = args[6]
        video_interproc_e = args[7]

        # get image file names
        images = glob.glob(dirFrameImage + "/*.jpg")
        ret, cammtx, distcoeff, reproerr = calibcam.calcuateCameraMatrix(images)

        strInfoText = ""

        if ret == True:
            strInfoText = "Calibration completed successfully... - " + str(reproerr)

            # save calibration data to the specific xml file
            savedFileName = "CalibCamResult" + str(camIndex) + ".json"
            calibcam.saveResults(savedFileName, cammtx, distcoeff)

            # update the result data
            calibResult = CalibCameraUpdate.updateData(
                distcoeff[0], cammtx.reshape(1, 9)[0]
            )

            # get the result data and throw into the websocket process
            interproc_dict["object"] = {
                "name": "cameracalib:" + cameraName,
                "object_type": "result",
                "object_data": calibResult,
            }
            video_interproc_e.set()

        else:
            strInfoText = "Calibration failed. - " + str(reproerr)

        PrintMsg.printStdErr(strInfoText)
        infoText.setText(strInfoText)

        super().enableExitFlag()
