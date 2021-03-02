import sys
import cv2


class VideoCapture(object):
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
        return self.camDev.start_capture()

    def stop(self):
        return self.camDev.stop_capture()

    def get_name(self):
        return self.name

    def get_video_frame(self):
        return self.camDev.get_video_frame()

    def get_frames(self):
        return self.camDev.get_frames()

    def getInternalIntrinsicsMat(self):
        return self.camDev.getInternalIntrinsicsMat()

    def getIntrinsicsMat(self, camIndex, UseInternal=False):
        # get internal intrinsics & extrinsics in D435
        if UseInternal == True:
            mtx, dist = self.getInternalIntrinsicsMat()
        else:
            loadFileName = "CalibCamResult" + str(camIndex) + ".json"
            calibFile = cv2.FileStorage(loadFileName, cv2.FILE_STORAGE_READ)
            cmnode = calibFile.getNode("cameraMatrix")
            mtx = cmnode.mat()
            dcnode = calibFile.getNode("distCoeff")
            dist = dcnode.mat()
        return (mtx, dist)

    def get_3D_pos(self, imageX, imageY):
        return self.camDev.get_3D_pos(imageX, imageY)
