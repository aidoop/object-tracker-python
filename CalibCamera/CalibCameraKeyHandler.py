
from packages.KeyHandler import KeyHandler
import cv2
import os
import glob

class CalibCameraKeyHandler(KeyHandler):

    def __init__(self):
        super().__init__()
        super().setKeyHandler('q', self.processQ)
        super().setKeyHandler('c', self.processC)
        super().setKeyHandler('z', self.processZ)
        super().setKeyHandler('g', self.processG)
        self.interation = 0

    def processQ(self, *args):
        super().enableExitFlag()

    def processC(self, *args):
        color_image = args[0]
        dirFrameImage = args[1]
        cv2.imwrite(os.path.join(dirFrameImage, str(self.interation) + '.jpg'), color_image)
        print('Image caputured - ' + os.path.join(dirFrameImage, str(self.interation) + '.jpg'))
        self.interation += 1

    def processZ(self, *args):
        calibcam = args[2]
        calibcam.clearObjImgPoints()
        self.interation = 0

    def processG(self, *args):
        dirFrameImage = args[1]
        calibcam = args[2]
        # get image file names
        images = glob.glob(dirFrameImage + '/*.jpg')
        ret, cammtx, distcoeff = calibcam.calcuateCameraMatrix(images)

        if ret == True:
            print('Calibration finished successfully...')
            # save calibration data to the specific xml file
            calibcam.saveResults("calibCamResult.json", cammtx, distcoeff)
        else:
            print('Calibration failed...')                
    

    