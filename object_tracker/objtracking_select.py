from time import sleep
from enum import Enum, unique
from typing import NamedTuple

from calibcamera_engine import calibcamera_engine
from calibhandeye_engine import calibhandeye_engine
from objecttracking_engine import objecttracking_engine

from aidobjtrack.util.util import SingletonInstane


class ObjectTrackingAppType(Enum):
    NOSELECTED = "NOSELECTED"
    CAMERACALIB = "CAMERACALIB"
    HANDEYECALIB = "HANDEYECALIB"
    OBJTRACKING = "OBJTRACKING"


class ObjectTrackingAppData:
    @staticmethod
    def set(interproc_dict, app_type, app_args):
        if interproc_dict is not None:
            interproc_dict["app_type"] = app_type
            interproc_dict["app_args"] = app_args

    @staticmethod
    def get(interproc_dict):
        return (interproc_dict["app_type"], interproc_dict["app_args"])

    @staticmethod
    def reset(interproc_dict):
        if interproc_dict is not None:
            interproc_dict["app_type"] = ObjectTrackingAppType.NOSELECTED
            interproc_dict["app_args"] = None


def objtracking_select_start(interproc_dict, ve, cq):
    while True:
        if interproc_dict["app_type"] == ObjectTrackingAppType.NOSELECTED:
            sleep(1)
            continue

        if interproc_dict["app_type"] == ObjectTrackingAppType.CAMERACALIB.value:
            calibcamera_engine(interproc_dict["app_args"], interproc_dict, ve, cq)
        elif interproc_dict["app_type"] == ObjectTrackingAppType.HANDEYECALIB.value:
            calibhandeye_engine(interproc_dict["app_args"], interproc_dict)
        elif interproc_dict["app_type"] == ObjectTrackingAppType.OBJTRACKING.value:
            objecttracking_engine(interproc_dict["app_args"])
        else:
            pass

        ObjectTrackingAppData.reset(interproc_dict)
        break
