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

    # set an object to track
    @abstractmethod
    def setTrackingObject(self, object):
        pass

    # find objects in the registered object list
    @abstractmethod
    def findObjects(self, *args):
        pass
