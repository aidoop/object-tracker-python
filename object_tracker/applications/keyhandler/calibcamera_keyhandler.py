import cv2
import os
import glob
import sys
import json

from applications.keyhandler.keyhandlerdev import KeyHandler
from applications.data_update.calibcamera_update import CalibCameraUpdate
from applications.etc.util import PrintMsg


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
        PrintMsg.print_error(f"Image caputured - {self.interation}")
        self.interation += 1

        strInfoText = f"Image caputured - {self.interation}"
        infoText.set_info_text(strInfoText)

    def processZ(self, *args):
        calibcam = args[2]
        calibcam.clear_all()
        self.interation = 0

    def processG(self, *args):
        dirFrameImage = args[1]
        calibcam = args[2]
        camIndex = args[3]
        infoText = args[4]
        cameraName = args[5]
        bridge_ip = args[6]

        # get image file names
        images = glob.glob(dirFrameImage + "/*.jpg")
        _, cammtx, distcoeff, reproerr = calibcam.calculate_camera_matrix(images)

        strInfoText = ""

        # don't check results and make user decide if this calculated values can be used.
        # if ret == True:
        if (cammtx is not None) and (distcoeff is not None):
            strInfoText = "Calibration completed successfully... - " + str(reproerr)

            # save calibration data to the specific xml file
            # savedFileName = "CalibCamResult" + str(camIndex) + ".json"
            # calibcam.save_result_to_file(savedFileName, cammtx, distcoeff)

            # update the result data
            calibResult = CalibCameraUpdate.update_result(
                distcoeff[0], cammtx.reshape(1, 9)[0]
            )

            # get the result data and throw into the websocket process
            bridge_ip.send_dict_data(
                "object",
                {
                    "name": "cameracalib:" + cameraName,
                    "objectData": calibResult,
                },
            )
        else:
            strInfoText = "Calibration failed."

        # PrintMsg.print_error(strInfoText)
        infoText.set_info_text(strInfoText)
