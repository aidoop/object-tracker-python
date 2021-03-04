import sys
import cv2

from aidoop.camera.camera_dev_abc import CameraDev


class OpencvCapture(CameraDev):
    def __init__(self, devIndex):
        # open VideoCapture
        self.__videoCapture = cv2.VideoCapture(devIndex)

    def initialize(self, width, height, fps):
        # set frame width, frame height and frame per secods
        self._frameWidth = width
        self._frameHeight = height
        self._framePerSec = fps

        # set opencv videocapture properties for width, height and fps
        self.__videoCapture.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.__videoCapture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.__videoCapture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.__videoCapture.set(cv2.CAP_PROP_FPS, fps)
        if width > 1920:
            fourcc = cv2.VideoWriter_fourcc("M", "J", "P", "G")
            self.__videoCapture.set(cv2.CAP_PROP_FOURCC, fourcc)

    def prepare(self):
        pass

    def start_capture(self):
        pass

    def stop_capture(self):
        pass

    # wait for a video color frame and return the frame
    def get_video_frame(self):
        # get a frame
        ret, frame = self.__videoCapture.read()
        if not ret:
            frame = None
        return frame

    def get_frames(self):
        pass

    # get 3D position w.r.t an image pixel based on camera-based coordination
    def get_3D_pos(self, imageX, imageY):
        raise NotImplementedError
