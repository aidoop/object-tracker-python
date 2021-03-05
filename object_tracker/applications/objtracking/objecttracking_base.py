from abc import *
import enum


class ObjectTracker(metaclass=ABCMeta):

    ###################################
    # Properties
    ###################################

    ###################################
    # Abstraction Functions
    ###################################
    # initialize parameters for any camera operation
    @abstractmethod
    def initialize(self, *args):
        pass

    # set an object to track
    @abstractmethod
    def set_tracking_object(self, object):
        pass

    # find objects in the registered object list
    @abstractmethod
    def find_tracking_object(self, *args):
        pass
