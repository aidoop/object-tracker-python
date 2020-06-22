
import sys
import cv2

if __package__ == '':
    from CameraDevRealsense import RealsenseCapture
else:
    from packages.CameraDevRealsense import RealsenseCapture


class VideoCapture:

    def __new__(cls, camDev, width, height, fps):
        if camDev == None:
            print("Camera Device is not opened...")
            videoCap = None
        else:
            videoCap = object.__new__(cls)
        
        return videoCap
    
    def __init__(self, camDev, width, height, fps):

        # set camera device object
        self.camDev = camDev

        # initialize camera device object
        self.camDev.initialize(width, height, fps)

    def start(self):
        return self.camDev.startCapture()

    def stop(self):
        return self.camDev.stopCapture()

    def getFrame(self):
        return self.camDev.getFrame()

    def getInternalIntrinsicsMat(self):
        return self.camDev.getInternalIntrinsicsMat()

    def getIntrinsicsMat(self, UseInternal=False):
        # get internal intrinsics & extrinsics in D435
        if(UseInternal == True):    
            mtx, dist = self.getInternalIntrinsicsMat()
        else:
            calibFile = cv2.FileStorage("calibCamResult.json", cv2.FILE_STORAGE_READ)
            cmnode = calibFile.getNode("cameraMatrix")
            mtx = cmnode.mat()
            dcnode = calibFile.getNode("distCoeff")
            dist = dcnode.mat()
        return(mtx, dist)

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
    vcap = VideoCapture(rsCamDev, 1280, 720, 30)

    vcap.start()

    frameIdx = 0
    for frmIdx in range(0, 10):
        vcap.getFrame()
        print(frameIdx)
        frameIdx += 1

    vcap.stop()

