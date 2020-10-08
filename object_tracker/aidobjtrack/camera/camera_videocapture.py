
import sys
import cv2

from camera.camera_dev_realsense import RealsenseCapture  # for an example


class VideoCapture:

    def __new__(cls, camDev, width, height, fps, name):
        if camDev == None:
            print("Camera Device is not opened...")
            videoCap = None
        else:
            videoCap = object.__new__(cls)

        return videoCap

    def __init__(self, camDev, width, height, fps, name):

        # set camera device object
        self.camDev = camDev

        # initialize camera device object
        self.camDev.initialize(width, height, fps)

        # set camera name
        self.name = name

    def start(self):
        return self.camDev.startCapture()

    def stop(self):
        return self.camDev.stopCapture()

    def getName(self):
        return self.name

    def getFrame(self):
        return self.camDev.getFrame()

    def getInternalIntrinsicsMat(self):
        return self.camDev.getInternalIntrinsicsMat()

    def getIntrinsicsMat(self, camIndex, UseInternal=False):
        # get internal intrinsics & extrinsics in D435
        if(UseInternal == True):
            mtx, dist = self.getInternalIntrinsicsMat()
        else:
            loadFileName = "CalibCamResult"+str(camIndex)+".json"
            calibFile = cv2.FileStorage(loadFileName, cv2.FILE_STORAGE_READ)
            cmnode = calibFile.getNode("cameraMatrix")
            mtx = cmnode.mat()
            dcnode = calibFile.getNode("distCoeff")
            dist = dcnode.mat()
        return(mtx, dist)

    def get3DPosition(self, imageX, imageY):
        return self.camDev.get3DPosition(imageX, imageY)


###############################################################################
# test sample codes
###############################################################################
if __name__ == '__main__':

    # create the camera object of intel realsense
    rsCamDev = RealsenseCapture(0)
    if(rsCamDev is None):
        print("Realsense device can't be opened..")
        sys.exit()

    # create video capture object using realsense camera object
    vcap = VideoCapture(rsCamDev, 1280, 720, 30, 'camera01')

    vcap.start()

    frameIdx = 0
    for frmIdx in range(0, 10):
        vcap.getFrame()
        print(frameIdx)
        frameIdx += 1

    vcap.stop()
