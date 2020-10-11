from abc import *
import enum

# CameraPortLayer abstraction class


class ROIManager(metaclass=ABCMeta):

    ###################################
    # Abstraction Functions
    ###################################
    @abstractmethod
    def appendROI(self, roi):
        pass

    @abstractmethod
    def getROIList(self):
        pass

    @abstractclassmethod
    def isInsideROI(self, newRegion):
        pass

    @abstractclassmethod
    def clearROIAll(self):
        pass
