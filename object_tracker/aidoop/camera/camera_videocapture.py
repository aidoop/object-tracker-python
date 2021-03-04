import sys
import cv2

from aidoop.camera.camera_dev_opencv import OpencvCapture
from aidoop.camera.camera_dev_realsense import RealsenseCapture


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

        # prepare camera operatons
        self.camDev.prepare()

        # set camera name
        self.name = name

    def start(self):
        return self.camDev.start_capture()

    def stop(self):
        return self.camDev.stop_capture()

    def prepare(self):
        return self.camDev.prepare()

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


class VideoCaptureConnectorType:
    UVC = "camera-connector"
    REALSENSE = "realsense-camera"
    AZURE_KINNECT = "azure-kinnect-camera"


class VideoCaptureFactory:
    @staticmethod
    def create_video_capture(
        camera_device_type: str,
        cam_endpoint: str,
        width: int,
        height: int,
        fps: int,
        camera_name: str,
    ) -> VideoCapture:
        camDev = (
            RealsenseCapture(cam_endpoint)
            if camera_device_type == VideoCaptureConnectorType.REALSENSE
            else OpencvCapture(int(cam_endpoint))
            if camera_device_type == VideoCaptureConnectorType.UVC
            else None
        )

        return VideoCapture(
            camDev,
            width,
            height,
            fps,
            camera_name,
        )
