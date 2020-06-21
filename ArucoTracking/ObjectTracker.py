from abc import *
import enum

## CameraPortLayer abstraction class
class ObjectTracker(metaclass=ABCMeta):
    
    ###################################
    # Properties
    ###################################

    ###################################
    # Abstraction Functions
    ###################################
    ## initialize parameters for any camera operation
    @abstractmethod
    def initialize(self, *args):
        pass
		
	# TODO: should define interface methods later

    # ## set detectable object like aruco marker
    # @abstractmethod
    # def setTrackingObject(self, object):
    #     pass

    # ## set detectable features and return the 2D or 3D positons in case that objects are detected..
    # @abstractmethod
    # def findObjects(self, *args):
    #     pass
