from time import sleep
from enum import Enum, unique
from typing import NamedTuple

from calibcamera_engine import calibcamera_engine
from calibhandeye_engine import calibhandeye_engine
from objecttracking_engine import objecttracking_engine

from aidobjtrack.util.util import SingletonInstane


class ObjectTrackingAppType(Enum):
    NOSELECTED = 'NOSELECTED'
    CAMERACALIB = 'CAMERACALIB'
    HANDEYECALIB = 'HANDEYECALIB'
    OBJTRACKING = 'OBJTRACKING'


class ObjectTrackingAppData(SingletonInstane):
    def __init__(self):
        self.global_dict = None

    def set_dict(self, dict):
        self.global_dict = dict

    def set(self, app_type, app_args):
        if global_dict is not None:
            self.global_dict['app_type'] = app_type
            self.global_dict['app_args'] = app_args

    def get(self):
        return (self.object_tracking_app_type, self.object_tracking_app_args)

    def reset(self):
        if global_dict is not None:
            self.global_dict['app_type'] = ObjectTrackingAppType.NOSELECTED
            self.global_dict['app_args'] = None


app_tracking_data = ObjectTrackingAppData.instance()


def objtracking_select(app_type, args):
    if isinstance(app_type, ObjectTrackingAppType):
        object_tracking_app_type = app_type
        object_tracking_app_args = args


def objtracking_select_start(share_dict):
    while True:
        if share_dict['app_type'] == ObjectTrackingAppType.NOSELECTED:
            sleep(1)
            continue

        if share_dict['app_type'] == ObjectTrackingAppType.CAMERACALIB:
            calibcamera_engine(share_dict['app_args'], share_dict)
        elif share_dict['app_type'] == ObjectTrackingAppType.HANDEYECALIB:
            calibhandeye_engine(share_dict['app_args'], share_dict)
        elif share_dict['app_type'] == ObjectTrackingAppType.OBJTRACKING:
            objecttracking_engine(share_dict['app_args'])
        else:
            pass

        app_tracking_data.reset()
