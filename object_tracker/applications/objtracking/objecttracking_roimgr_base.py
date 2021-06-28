from abc import *

# CameraPortLayer abstraction class


class ROIManager(metaclass=ABCMeta):

    ###################################
    # Abstraction Functions
    ###################################
    @abstractmethod
    def append_roi(self, roi):
        pass

    @abstractmethod
    def get_roi_list(self):
        pass

    @abstractclassmethod
    def is_inside_roi(self, newRegion):
        pass

    @abstractclassmethod
    def clear_roi_all(self):
        pass
