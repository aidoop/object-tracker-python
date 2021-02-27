from abc import *
import enum

# CameraPortLayer abstraction class


class CameraDev(metaclass=ABCMeta):

    ###################################
    # Properties
    ###################################
    # frame width
    _frameWidth = 0

    # frame height
    _frameHeight = 0

    # frame per sec
    _framePerSec = 0

    ###################################
    # Abstraction Functions
    ###################################
    # initialize parameters for any camera operation
    @abstractmethod
    def initialize(self):
        pass

    # do something before starting capture
    def prepare(self):
        pass

    # start to capture frames
    @abstractmethod
    def start_capture(self):
        pass

    # stop to capture frames
    @abstractmethod
    def stop_capture(self):
        pass

    # wait for a video color frame and return the frame
    @abstractmethod
    def get_video_frame(self):
        pass

    # wait for a video frame & and return color & depth frames
    @abstractmethod
    def get_frames(self):
        pass

    # get 3D position w.r.t an image pixel based on camera-based coordination
    @abstractmethod
    def get_3D_pos(self):
        pass
