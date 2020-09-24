from abc import *
import enum

# CameraPortLayer abstraction class


class RobotDev(metaclass=ABCMeta):

    ###################################
    # Abstraction Functions
    ###################################
    # initialize parameters for any camera operation
    @abstractmethod
    def initalize(self):
        pass

    # finalize this object
    @abstractmethod
    def finalize(self):
        pass

    # start to capture frames
    @abstractmethod
    def moveTaskPos(self):
        pass

    # stop to capture frames
    @abstractmethod
    def getCurrentPos(self):
        pass

    # set the direct-teaching mode
    @abstractmethod
    def setDirectTeachingMode(self, flag):
        pass

    # get the direct-teaching mode
    @abstractmethod
    def getDirectTeachingMode(self):
        pass

    # reset
    @abstractmethod
    def resetRobot(self):
        pass
